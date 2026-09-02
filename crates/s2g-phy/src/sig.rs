//! SIG (S1G_SHORT, Table 23-12) and SIG-A (S1G_LONG, Tables 23-14/23-15)
//! fields: bit layouts, CRC-4 [23.3.8.2.2.6], encoding [23.3.4.3.3] and
//! decoding, plus the preamble-format discriminator [23.3.8.2.3.2.5 NOTE].
//!
//! Both fields are 48 uncoded bits (symbol 1 B0..B23 then symbol 2 B0..B23)
//! → BCC R=1/2 (96 bits) → per-symbol 48-bit Clause-17 interleave → BPSK →
//! 48 data tones (±1..±26 minus pilots) + Clause-17 pilots, scaled 1/√52,
//! 8 µs GI. The difference is the constellation rotation:
//!
//! * S1G_SHORT SIG: **both** symbols rotated +90° (QBPSK).
//! * S1G_LONG SIG-A: symbol 1 rotated (QBPSK), symbol 2 plain BPSK — that
//!   asymmetry is how a receiver tells the two ≥ 2 MHz formats apart, and
//!   decoding SIG-A is mandatory for every S1G STA [4.3.14.1].
//!
//! All numeric fields are LSB-first [23.3.8.1].

use crate::bits::{bits_to_uint_lsb_first, push_uint_lsb_first};
use crate::error::PhyError;
use crate::ofdm::{self, SIG_SUBCARRIER_INDICES};
use crate::params::{self, N_GI_LONG, N_TONE_SIG};
use crate::vector::{data_field_geometry, Coding, GuardInterval, PreambleType, ResponseIndication, RxVector, TxVector};
use crate::{bcc, interleaver, mapping, pilots, Complex32};

/// Decoded/encodable S1G_SHORT SIG contents for a normal (non-NDP) PPDU
/// [Table 23-12].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SigFields {
    // SIG-1
    pub stbc: bool,
    pub uplink_indication: bool,
    /// BW field: 0=2 MHz, 1=4, 2=8, 3=16.
    pub bandwidth: u8,
    /// Nsts field: N_STS − 1 (0 ⇒ 1 stream).
    pub nsts: u8,
    /// Raw 9-bit ID field (COLOR + partial AID packing depends on
    /// `uplink_indication`, see accessors).
    pub id: u16,
    pub short_gi: bool,
    pub ldpc: bool,
    /// LDPC Extra OFDM Symbol when `ldpc`; must be 1 on transmit for BCC
    /// [Table 23-12 B18].
    pub ldpc_extra: bool,
    pub mcs: u8,
    pub smoothing: bool,
    // SIG-2
    pub aggregation: bool,
    /// 9-bit Length: octets when !aggregation, N_SYM when aggregation.
    pub length: u16,
    pub response_indication: ResponseIndication,
    pub traveling_pilots: bool,
}

/// S1G_LONG SIG-A contents for an SU PPDU [Table 23-14].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SigASu {
    pub stbc: bool,
    pub uplink_indication: bool,
    pub bandwidth: u8,
    /// N_STS − 1.
    pub nsts: u8,
    pub id: u16,
    pub short_gi: bool,
    pub ldpc: bool,
    pub ldpc_extra: bool,
    pub mcs: u8,
    /// Beam Change (1 STS) / Smoothing (> 1 STS) indication, B23.
    pub beam_change_or_smoothing: bool,
    pub aggregation: bool,
    pub length: u16,
    pub response_indication: ResponseIndication,
    pub traveling_pilots: bool,
}

/// S1G_LONG SIG-A contents for an MU PPDU [Table 23-15].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SigAMu {
    pub stbc: bool,
    /// MU[0..4] Nsts (0 = user position unused).
    pub nsts: [u8; 4],
    pub bandwidth: u8,
    pub group_id: u8,
    pub short_gi: bool,
    /// Coding-I: per-user LDPC flag (meaningful only where nsts[u] > 0).
    pub ldpc: [bool; 4],
    /// Coding-II: LDPC extra symbol.
    pub ldpc_extra: bool,
    /// N_SYM (MU PPDUs are always aggregated).
    pub length: u16,
    pub response_indication: ResponseIndication,
    pub traveling_pilots: bool,
}

/// Result of decoding a SIG / SIG-A field.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SigContent {
    /// S1G_SHORT data PPDU.
    Normal(SigFields),
    /// NDP CMAC PPDU: 37-bit body (B0 first = LSB) [23.3.11, Fig 23-21].
    Ndp { body: u64 },
    /// S1G_LONG SU PPDU.
    LongSu(SigASu),
    /// S1G_LONG MU PPDU.
    LongMu(SigAMu),
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SigError {
    /// CRC over bits 0..37 failed.
    CrcFailed,
    /// Tail bits nonzero, or a reserved bit transmitted as 0 ("Reserved SIG
    /// Indication" [23.3.20]). When the remaining fields still allow it,
    /// `duration_us` carries the PPDU duration so CCA can be held for it.
    Reserved { reason: &'static str, duration_us: Option<u32> },
}

/// What the receiver should do with a CRC-valid SIG/SIG-A.
#[derive(Debug, Clone, PartialEq)]
pub enum SigVerdict {
    /// A mode this receiver decodes: proceed to the Data field.
    Supported(RxVector),
    /// Valid but not decodable here (PHY-RXSTART then RXEND(UnsupportedRate));
    /// the RXVECTOR still carries everything needed for CCA/RID.
    Unsupported(RxVector, &'static str),
    /// Reserved SIG Indication → PHY-RXEND(FormatViolation). CCA is still
    /// held for `duration_us` when the fields allow computing it [23.3.20].
    Reserved { reason: &'static str, duration_us: Option<u32> },
}

/// PPDU duration for a (possibly invalid) field combination, or `None` when
/// the symbol count cannot be derived (unknown N_DBPS without aggregation).
#[allow(clippy::too_many_arguments)]
fn duration_from_fields(
    preamble: PreambleType,
    n_sts: u8,
    stbc: bool,
    short_gi: bool,
    mcs: u8,
    ldpc: bool,
    ldpc_extra: bool,
    aggregation: bool,
    length: u16,
) -> Option<u32> {
    let n_ss = if stbc { (n_sts / 2).max(1) } else { n_sts.max(1) };
    let n_sym = if aggregation {
        (length > 0).then_some(length as usize)?
    } else {
        let n_dbps = params::n_dbps_2mhz(mcs, n_ss)?;
        let coding = if ldpc { Coding::Ldpc } else { Coding::Bcc };
        data_field_geometry(length, false, coding, ldpc && ldpc_extra, stbc, n_dbps)?.0
    };
    let r = RxVector {
        preamble_type: preamble,
        num_sts: n_sts.max(1),
        gi: if short_gi { GuardInterval::Short } else { GuardInterval::Long },
        n_sym,
        ..Default::default()
    };
    Some(r.ppdu_duration_us())
}

/// CRC-4, G(D)=D⁴+D+1, register init all-ones, output ones-complemented,
/// c3 first [23.3.8.2.2.6; validated against the spec's worked example].
pub fn crc4(bits: &[u8]) -> [u8; 4] {
    let (mut c3, mut c2, mut c1, mut c0) = (1u8, 1, 1, 1);
    for &b in bits {
        let fb = (b & 1) ^ c3;
        c3 = c2;
        c2 = c1;
        c1 = c0 ^ fb;
        c0 = fb;
    }
    [1 ^ c3, 1 ^ c2, 1 ^ c1, 1 ^ c0]
}

/// Append CRC (over the 38 bits present) and 6 zero tail bits.
fn finish_48(mut v: Vec<u8>) -> [u8; 48] {
    debug_assert_eq!(v.len(), 38);
    let crc = crc4(&v);
    v.extend_from_slice(&crc);
    v.extend_from_slice(&[0; 6]);
    let mut out = [0u8; 48];
    out.copy_from_slice(&v);
    out
}

/// Check tail and CRC of 48 decoded bits.
fn check_48(bits: &[u8; 48]) -> Result<(), SigError> {
    if bits[42..48].iter().any(|&b| b != 0) {
        return Err(SigError::Reserved { reason: "nonzero tail bits", duration_us: None });
    }
    if bits[38..42] != crc4(&bits[..38]) {
        return Err(SigError::CrcFailed);
    }
    Ok(())
}

fn id_from_txvector(txv: &TxVector) -> Result<u16, PhyError> {
    if txv.color > 7 {
        return Err(PhyError::InvalidTxVector("color > 7"));
    }
    if txv.uplink_indication {
        if txv.partial_aid > 511 {
            return Err(PhyError::InvalidTxVector("partial_aid > 511"));
        }
        Ok(txv.partial_aid)
    } else {
        if txv.partial_aid > 63 {
            return Err(PhyError::InvalidTxVector("partial_aid > 63 with uplink_indication = false"));
        }
        Ok((txv.partial_aid << 3) | txv.color as u16)
    }
}

fn split_id(id: u16, uplink: bool) -> (u8, u16) {
    if uplink {
        (0, id & 0x1ff)
    } else {
        ((id & 0x7) as u8, (id >> 3) & 0x3f)
    }
}

impl SigFields {
    /// COLOR carried in the ID field (only meaningful when
    /// `uplink_indication` is false).
    pub fn color(&self) -> u8 {
        split_id(self.id, self.uplink_indication).0
    }

    /// Partial AID from the ID field.
    pub fn partial_aid(&self) -> u16 {
        split_id(self.id, self.uplink_indication).1
    }

    /// Serialize to the 48 SIG bits (computes CRC; B0 reserved = 1, tail 0).
    pub fn to_bits(&self) -> [u8; 48] {
        let mut v = Vec::with_capacity(48);
        // SIG-1
        v.push(1); // B0 reserved, set to 1 on transmit
        v.push(self.stbc as u8);
        v.push(self.uplink_indication as u8);
        push_uint_lsb_first(&mut v, self.bandwidth as u64, 2);
        push_uint_lsb_first(&mut v, self.nsts as u64, 2);
        push_uint_lsb_first(&mut v, self.id as u64, 9);
        v.push(self.short_gi as u8);
        v.push(self.ldpc as u8);
        v.push(self.ldpc_extra as u8);
        push_uint_lsb_first(&mut v, self.mcs as u64, 4);
        v.push(self.smoothing as u8);
        // SIG-2
        v.push(self.aggregation as u8);
        push_uint_lsb_first(&mut v, self.length as u64, 9);
        push_uint_lsb_first(&mut v, self.response_indication.to_bits() as u64, 2);
        v.push(self.traveling_pilots as u8);
        v.push(0); // NDP Indication = 0
        finish_48(v)
    }

    /// Build from a TXVECTOR plus the Length-field value (octets or N_SYM
    /// per aggregation) and the LDPC Extra flag — both computed by the TX
    /// chain.
    pub fn from_txvector(txv: &TxVector, length_field: u16, ldpc_extra: bool) -> Result<Self, PhyError> {
        params::mcs_params(txv.mcs)?;
        let id = id_from_txvector(txv)?;
        let ldpc = txv.fec_coding == Coding::Ldpc;
        Ok(SigFields {
            stbc: false,
            uplink_indication: txv.uplink_indication,
            bandwidth: 0,
            nsts: 0,
            id,
            short_gi: txv.gi == GuardInterval::Short,
            ldpc,
            // BCC: "this field is set to 1" [Table 23-12 B18].
            ldpc_extra: if ldpc { ldpc_extra } else { true },
            mcs: txv.mcs,
            smoothing: txv.smoothing,
            aggregation: txv.aggregation,
            length: length_field,
            response_indication: txv.response_indication,
            traveling_pilots: txv.traveling_pilots,
        })
    }

    /// PPDU duration implied by the fields even when the combination is
    /// reserved, if it can be computed.
    pub fn duration_if_computable(&self) -> Option<u32> {
        duration_from_fields(
            PreambleType::S1gShort,
            self.nsts + 1,
            self.stbc,
            self.short_gi,
            self.mcs,
            self.ldpc,
            self.ldpc_extra,
            self.aggregation,
            self.length,
        )
    }

    /// Classify and derive the RXVECTOR skeleton [23.3.20].
    pub fn verdict(&self) -> SigVerdict {
        let n_sts = self.nsts + 1;
        let n_ss = if self.stbc { n_sts / 2 } else { n_sts };
        let reserved = |reason: &'static str| SigVerdict::Reserved { reason, duration_us: self.duration_if_computable() };
        if self.stbc && !n_sts.is_multiple_of(2) {
            return reserved("STBC with odd N_STS");
        }
        if !self.ldpc && !self.ldpc_extra {
            return reserved("B18 must be 1 with BCC");
        }
        let Some(n_dbps) = params::n_dbps_2mhz(self.mcs, n_ss) else {
            return reserved("MCS/N_SS combination not in 23.5");
        };
        let coding = if self.ldpc { Coding::Ldpc } else { Coding::Bcc };
        let Some((n_sym, psdu_length)) =
            data_field_geometry(self.length, self.aggregation, coding, self.ldpc && self.ldpc_extra, self.stbc, n_dbps)
        else {
            return reserved("zero-length Data field");
        };
        let (color, partial_aid) = split_id(self.id, self.uplink_indication);
        let rxv = RxVector {
            preamble_type: PreambleType::S1gShort,
            mu: false,
            bandwidth_code: self.bandwidth,
            num_sts: n_sts,
            stbc: self.stbc,
            mcs: self.mcs,
            gi: if self.short_gi { GuardInterval::Short } else { GuardInterval::Long },
            fec_coding: coding,
            ldpc_extra: self.ldpc && self.ldpc_extra,
            aggregation: self.aggregation,
            response_indication: self.response_indication,
            smoothing: self.smoothing,
            traveling_pilots: self.traveling_pilots,
            uplink_indication: self.uplink_indication,
            color,
            partial_aid,
            group_id: 0,
            length: self.length,
            psdu_length,
            n_sym,
            ..Default::default()
        };
        let unsupported = if self.bandwidth != 0 {
            Some("bandwidth > 2 MHz")
        } else if self.stbc {
            Some("STBC")
        } else if n_sts != 1 {
            Some("multiple spatial streams")
        } else if params::mcs_params(self.mcs).is_err() {
            Some("MCS not supported")
        } else {
            None
        };
        match unsupported {
            Some(why) => SigVerdict::Unsupported(rxv, why),
            None => SigVerdict::Supported(rxv),
        }
    }

    /// Derive the RXVECTOR for a supported PPDU (legacy helper; see
    /// [`SigFields::verdict`]).
    pub fn to_rxvector(&self) -> Result<RxVector, PhyError> {
        match self.verdict() {
            SigVerdict::Supported(r) => Ok(r),
            SigVerdict::Unsupported(_, why) => Err(PhyError::Unsupported(why)),
            SigVerdict::Reserved { reason, .. } => Err(PhyError::InvalidLength { len: self.length as usize, reason }),
        }
    }
}

impl SigASu {
    /// PPDU duration implied by the fields even when the combination is
    /// reserved, if it can be computed.
    pub fn duration_if_computable(&self) -> Option<u32> {
        duration_from_fields(
            PreambleType::S1gLong,
            self.nsts + 1,
            self.stbc,
            self.short_gi,
            self.mcs,
            self.ldpc,
            self.ldpc_extra,
            self.aggregation,
            self.length,
        )
    }

    pub fn to_bits(&self) -> [u8; 48] {
        let mut v = Vec::with_capacity(48);
        // SIG-A1
        v.push(0); // MU/SU = 0
        v.push(self.stbc as u8);
        v.push(self.uplink_indication as u8);
        push_uint_lsb_first(&mut v, self.bandwidth as u64, 2);
        push_uint_lsb_first(&mut v, self.nsts as u64, 2);
        push_uint_lsb_first(&mut v, self.id as u64, 9);
        v.push(self.short_gi as u8);
        v.push(self.ldpc as u8);
        v.push(self.ldpc_extra as u8);
        push_uint_lsb_first(&mut v, self.mcs as u64, 4);
        v.push(self.beam_change_or_smoothing as u8);
        // SIG-A2
        v.push(self.aggregation as u8);
        push_uint_lsb_first(&mut v, self.length as u64, 9);
        push_uint_lsb_first(&mut v, self.response_indication.to_bits() as u64, 2);
        v.push(1); // B12 reserved, set to 1
        v.push(self.traveling_pilots as u8);
        finish_48(v)
    }

    /// Build from a TXVECTOR for an S1G_LONG SU transmission (1 STS).
    pub fn from_txvector(txv: &TxVector, length_field: u16, ldpc_extra: bool) -> Result<Self, PhyError> {
        params::mcs_params(txv.mcs)?;
        let id = id_from_txvector(txv)?;
        let ldpc = txv.fec_coding == Coding::Ldpc;
        Ok(SigASu {
            stbc: false,
            uplink_indication: txv.uplink_indication,
            bandwidth: 0,
            nsts: 0,
            id,
            short_gi: txv.gi == GuardInterval::Short,
            ldpc,
            ldpc_extra: if ldpc { ldpc_extra } else { true },
            mcs: txv.mcs,
            // With 1 STS, B23 = 0 means "no beam change, smoothing allowed"
            // [Table 23-14 NOTE 1].
            beam_change_or_smoothing: !txv.smoothing,
            aggregation: txv.aggregation,
            length: length_field,
            response_indication: txv.response_indication,
            traveling_pilots: txv.traveling_pilots,
        })
    }

    /// Classify and derive the RXVECTOR skeleton. SU PPDUs with one
    /// space-time stream at 2 MHz are decoded (an optional feature for a
    /// ≤ 2 MHz STA [4.3.14.1]); everything else is identified for CCA and
    /// RID only.
    pub fn verdict(&self) -> SigVerdict {
        let n_sts = self.nsts + 1;
        let n_ss = if self.stbc { n_sts / 2 } else { n_sts };
        let reserved = |reason: &'static str| SigVerdict::Reserved { reason, duration_us: self.duration_if_computable() };
        if self.stbc && !n_sts.is_multiple_of(2) {
            return reserved("STBC with odd N_STS");
        }
        if !self.ldpc && !self.ldpc_extra {
            return reserved("B18 must be 1 with BCC");
        }
        let Some(n_dbps) = params::n_dbps_2mhz(self.mcs, n_ss) else {
            return reserved("MCS/N_SS combination not in 23.5");
        };
        let coding = if self.ldpc { Coding::Ldpc } else { Coding::Bcc };
        let Some((n_sym, psdu_length)) =
            data_field_geometry(self.length, self.aggregation, coding, self.ldpc && self.ldpc_extra, self.stbc, n_dbps)
        else {
            return reserved("zero-length Data field");
        };
        let (color, partial_aid) = split_id(self.id, self.uplink_indication);
        let rxv = RxVector {
            preamble_type: PreambleType::S1gLong,
            mu: false,
            bandwidth_code: self.bandwidth,
            num_sts: n_sts,
            stbc: self.stbc,
            mcs: self.mcs,
            gi: if self.short_gi { GuardInterval::Short } else { GuardInterval::Long },
            fec_coding: coding,
            ldpc_extra: self.ldpc && self.ldpc_extra,
            aggregation: self.aggregation,
            response_indication: self.response_indication,
            // 1 STS: B23 = 0 ⇒ the beam-changeable portion is sent through
            // the same Q as the omni portion and smoothing is fine
            // [Table 23-14 NOTE 1]. With more streams B23 is the Beam
            // Change Indication and says nothing about smoothing.
            smoothing: n_sts == 1 && !self.beam_change_or_smoothing,
            traveling_pilots: self.traveling_pilots,
            uplink_indication: self.uplink_indication,
            color,
            partial_aid,
            group_id: 0,
            length: self.length,
            psdu_length,
            n_sym,
            ..Default::default()
        };
        let unsupported = if self.bandwidth != 0 {
            Some("bandwidth > 2 MHz")
        } else if self.stbc {
            Some("STBC")
        } else if n_sts != 1 {
            Some("multiple spatial streams")
        } else if params::mcs_params(self.mcs).is_err() {
            Some("MCS not supported")
        } else {
            None
        };
        match unsupported {
            Some(why) => SigVerdict::Unsupported(rxv, why),
            None => SigVerdict::Supported(rxv),
        }
    }
}

impl SigAMu {
    pub fn to_bits(&self) -> [u8; 48] {
        let mut v = Vec::with_capacity(48);
        // SIG-A1
        v.push(1); // MU/SU = 1
        v.push(self.stbc as u8);
        v.push(1); // B2 reserved
        for u in 0..4 {
            push_uint_lsb_first(&mut v, self.nsts[u] as u64, 2);
        }
        push_uint_lsb_first(&mut v, self.bandwidth as u64, 2);
        push_uint_lsb_first(&mut v, self.group_id as u64, 6);
        v.push(self.short_gi as u8);
        for u in 0..4 {
            // Reserved (1) where the user position is unused.
            v.push(if self.nsts[u] > 0 { self.ldpc[u] as u8 } else { 1 });
        }
        // SIG-A2
        v.push(self.ldpc_extra as u8);
        v.push(1); // B1 reserved
        push_uint_lsb_first(&mut v, self.length as u64, 9);
        push_uint_lsb_first(&mut v, self.response_indication.to_bits() as u64, 2);
        v.push(self.traveling_pilots as u8);
        finish_48(v)
    }

    /// PPDU duration (MU PPDUs are always aggregated, so N_SYM = Length).
    pub fn duration_if_computable(&self) -> Option<u32> {
        let total_sts: u8 = self.nsts.iter().sum();
        duration_from_fields(PreambleType::S1gLong, total_sts.max(1), self.stbc, self.short_gi, 0, false, false, true, self.length)
    }

    pub fn verdict(&self) -> SigVerdict {
        let total_sts: u8 = self.nsts.iter().sum();
        if total_sts == 0 {
            return SigVerdict::Reserved { reason: "MU PPDU with no space-time streams", duration_us: self.duration_if_computable() };
        }
        if self.length == 0 {
            return SigVerdict::Reserved { reason: "zero-length Data field", duration_us: None };
        }
        let rxv = RxVector {
            preamble_type: PreambleType::S1gLong,
            mu: true,
            bandwidth_code: self.bandwidth,
            num_sts: total_sts,
            stbc: self.stbc,
            mcs: 0,
            gi: if self.short_gi { GuardInterval::Short } else { GuardInterval::Long },
            fec_coding: if self.ldpc.iter().zip(&self.nsts).any(|(&l, &n)| l && n > 0) { Coding::Ldpc } else { Coding::Bcc },
            ldpc_extra: self.ldpc_extra,
            aggregation: true,
            response_indication: self.response_indication,
            traveling_pilots: self.traveling_pilots,
            group_id: self.group_id,
            length: self.length,
            psdu_length: 0,
            n_sym: self.length as usize,
            ..Default::default()
        };
        SigVerdict::Unsupported(rxv, "S1G MU PPDU")
    }
}

impl SigContent {
    /// Classification of any CRC-valid SIG/SIG-A. NDPs are handled by the
    /// caller (they have no Data field).
    pub fn verdict(&self) -> Option<SigVerdict> {
        match self {
            SigContent::Normal(f) => Some(f.verdict()),
            SigContent::LongSu(f) => Some(f.verdict()),
            SigContent::LongMu(f) => Some(f.verdict()),
            SigContent::Ndp { .. } => None,
        }
    }
}

/// Parse 48 decoded S1G_SHORT SIG bits.
fn parse_short(bits: &[u8; 48]) -> Result<SigContent, SigError> {
    check_48(bits)?;
    let ndp = bits[37] == 1;
    if ndp {
        let body = bits_to_uint_lsb_first(&bits[..37]);
        return Ok(SigContent::Ndp { body });
    }
    let fields = SigFields {
        stbc: bits[1] == 1,
        uplink_indication: bits[2] == 1,
        bandwidth: bits_to_uint_lsb_first(&bits[3..5]) as u8,
        nsts: bits_to_uint_lsb_first(&bits[5..7]) as u8,
        id: bits_to_uint_lsb_first(&bits[7..16]) as u16,
        short_gi: bits[16] == 1,
        ldpc: bits[17] == 1,
        ldpc_extra: bits[18] == 1,
        mcs: bits_to_uint_lsb_first(&bits[19..23]) as u8,
        smoothing: bits[23] == 1,
        aggregation: bits[24] == 1,
        length: bits_to_uint_lsb_first(&bits[25..34]) as u16,
        response_indication: ResponseIndication::from_bits(bits_to_uint_lsb_first(&bits[34..36]) as u8),
        traveling_pilots: bits[36] == 1,
    };
    if bits[0] != 1 {
        // Reserved bit is transmitted as 1 in S1G_SHORT; the other fields
        // still tell us how long to hold CCA.
        return Err(SigError::Reserved { reason: "SIG B0 = 0", duration_us: fields.duration_if_computable() });
    }
    Ok(SigContent::Normal(fields))
}

/// Parse 48 decoded S1G_LONG SIG-A bits.
fn parse_long(bits: &[u8; 48]) -> Result<SigContent, SigError> {
    check_48(bits)?;
    if bits[0] == 0 {
        // SU [Table 23-14]
        let fields = SigASu {
            stbc: bits[1] == 1,
            uplink_indication: bits[2] == 1,
            bandwidth: bits_to_uint_lsb_first(&bits[3..5]) as u8,
            nsts: bits_to_uint_lsb_first(&bits[5..7]) as u8,
            id: bits_to_uint_lsb_first(&bits[7..16]) as u16,
            short_gi: bits[16] == 1,
            ldpc: bits[17] == 1,
            ldpc_extra: bits[18] == 1,
            mcs: bits_to_uint_lsb_first(&bits[19..23]) as u8,
            beam_change_or_smoothing: bits[23] == 1,
            aggregation: bits[24] == 1,
            length: bits_to_uint_lsb_first(&bits[25..34]) as u16,
            response_indication: ResponseIndication::from_bits(bits_to_uint_lsb_first(&bits[34..36]) as u8),
            traveling_pilots: bits[37] == 1,
        };
        if bits[36] != 1 {
            return Err(SigError::Reserved { reason: "SIG-A2 B12 = 0", duration_us: fields.duration_if_computable() });
        }
        Ok(SigContent::LongSu(fields))
    } else {
        // MU [Table 23-15]
        let mut nsts = [0u8; 4];
        let mut ldpc = [false; 4];
        let mut coding_reserved = false;
        for u in 0..4 {
            nsts[u] = bits_to_uint_lsb_first(&bits[3 + 2 * u..5 + 2 * u]) as u8;
            ldpc[u] = bits[20 + u] == 1;
            if nsts[u] == 0 && !ldpc[u] {
                coding_reserved = true;
            }
        }
        let fields = SigAMu {
            stbc: bits[1] == 1,
            nsts,
            bandwidth: bits_to_uint_lsb_first(&bits[11..13]) as u8,
            group_id: bits_to_uint_lsb_first(&bits[13..19]) as u8,
            short_gi: bits[19] == 1,
            ldpc,
            ldpc_extra: bits[24] == 1,
            length: bits_to_uint_lsb_first(&bits[26..35]) as u16,
            response_indication: ResponseIndication::from_bits(bits_to_uint_lsb_first(&bits[35..37]) as u8),
            traveling_pilots: bits[37] == 1,
        };
        let reserved = |reason: &'static str| SigError::Reserved { reason, duration_us: fields.duration_if_computable() };
        if bits[2] != 1 {
            return Err(reserved("SIG-A1 B2 = 0"));
        }
        if bits[25] != 1 {
            return Err(reserved("SIG-A2 B1 = 0"));
        }
        if coding_reserved {
            return Err(reserved("Coding-I reserved bit = 0"));
        }
        Ok(SigContent::LongMu(fields))
    }
}

/// Encode 48 bits into the 160-sample field (2 symbols, 2 MS/s); `rotate`
/// selects QBPSK (+90°) per symbol.
fn encode_bits(bits: &[u8; 48], rotate: [bool; 2]) -> Vec<Complex32> {
    let coded = bcc::encode_r12(bits); // 96 bits, no puncturing
    let scale = 1.0 / (N_TONE_SIG as f32).sqrt();
    let mut out = Vec::with_capacity(160);
    for n in 0..2 {
        let ilv = interleaver::interleave_sig(&coded[n * 48..(n + 1) * 48]);
        let tones: Vec<Complex32> = ilv
            .iter()
            .map(|&b| {
                let v = if b == 1 { 1.0 } else { -1.0 };
                if rotate[n] {
                    Complex32::new(0.0, v) // BPSK rotated +90° (×j)
                } else {
                    Complex32::new(v, 0.0)
                }
            })
            .collect();
        let sym = ofdm::assemble_freq_symbol(&SIG_SUBCARRIER_INDICES, &tones, &pilots::PILOT_INDICES, &pilots::sig_pilots(n));
        out.extend(ofdm::to_time_domain(&sym, N_GI_LONG, scale));
    }
    out
}

/// TX: S1G_SHORT SIG field waveform for a normal PPDU.
pub fn encode(fields: &SigFields) -> Vec<Complex32> {
    encode_bits(&fields.to_bits(), [true, true])
}

/// TX: S1G_SHORT SIG field waveform for an NDP CMAC PPDU (37-bit body,
/// LSB = B0).
pub fn encode_ndp(body: u64) -> Vec<Complex32> {
    let mut v = Vec::with_capacity(48);
    push_uint_lsb_first(&mut v, body & ((1u64 << 37) - 1), 37);
    v.push(1); // NDP Indication
    encode_bits(&finish_48(v), [true, true])
}

/// TX: S1G_LONG SIG-A waveform (SU): SIG-A1 QBPSK, SIG-A2 BPSK
/// [23.3.8.2.3.2.5].
pub fn encode_sig_a_su(fields: &SigASu) -> Vec<Complex32> {
    encode_bits(&fields.to_bits(), [true, false])
}

/// TX: S1G_LONG SIG-A waveform (MU).
pub fn encode_sig_a_mu(fields: &SigAMu) -> Vec<Complex32> {
    encode_bits(&fields.to_bits(), [true, false])
}

/// Preamble-format discriminator from the equalized second SIG symbol
/// [23.3.8.2.3.2.5 NOTE]: S1G_SHORT puts its BPSK energy on the Q axis
/// (QBPSK), S1G_LONG on the I axis. Returns the format and a confidence in
/// 0..1 (|E_q − E_i| / (E_q + E_i)).
pub fn detect_preamble_type(sym2: &[Complex32]) -> (PreambleType, f32) {
    let (ei, eq) = sym2.iter().fold((0.0f32, 0.0f32), |(i, q), v| (i + v.re * v.re, q + v.im * v.im));
    let conf = (eq - ei).abs() / (eq + ei).max(1e-12);
    if eq >= ei {
        (PreambleType::S1gShort, conf)
    } else {
        (PreambleType::S1gLong, conf)
    }
}

/// RX: decode SIG or SIG-A from the two equalized symbols' 48 data tones
/// each (in `SIG_SUBCARRIER_INDICES` order, CPE already corrected) plus
/// per-tone CSI. `ptype` selects the rotation pattern and the bit layout.
pub fn decode(sym1: &[Complex32], sym2: &[Complex32], csi1: &[f32], csi2: &[f32], ptype: PreambleType) -> Result<SigContent, SigError> {
    debug_assert_eq!(sym1.len(), 48);
    debug_assert_eq!(sym2.len(), 48);
    let rotate = match ptype {
        PreambleType::S1gShort => [true, true],
        PreambleType::S1gLong => [true, false],
    };
    let mut llrs = Vec::with_capacity(96);
    for (n, (sym, csi)) in [(sym1, csi1), (sym2, csi2)].into_iter().enumerate() {
        let derot: Vec<Complex32> = if rotate[n] {
            // Undo the QBPSK rotation: multiply by −j.
            sym.iter().map(|v| Complex32::new(v.im, -v.re)).collect()
        } else {
            sym.to_vec()
        };
        let sym_llrs = mapping::demap_llrs(&derot, csi, params::Modulation::Bpsk);
        llrs.extend(interleaver::deinterleave_sig_llrs(&sym_llrs));
    }
    let bits_vec = bcc::viterbi_decode(&llrs, params::CodeRate::R1_2, 48);
    let mut bits = [0u8; 48];
    bits.copy_from_slice(&bits_vec);
    match ptype {
        PreambleType::S1gShort => parse_short(&bits),
        PreambleType::S1gLong => parse_long(&bits),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::params::N_SIG_SAMPLES;

    fn sample_fields() -> SigFields {
        SigFields {
            stbc: false,
            uplink_indication: false,
            bandwidth: 0,
            nsts: 0,
            id: (5 << 3) | 3, // partial_aid 5, color 3
            short_gi: false,
            ldpc: false,
            ldpc_extra: true,
            mcs: 4,
            smoothing: true,
            aggregation: false,
            length: 100,
            response_indication: ResponseIndication::Normal,
            traveling_pilots: false,
        }
    }

    fn sample_sig_a() -> SigASu {
        SigASu {
            stbc: false,
            uplink_indication: true,
            bandwidth: 0,
            nsts: 0,
            id: 0x155,
            short_gi: false,
            ldpc: true,
            ldpc_extra: true,
            mcs: 3,
            beam_change_or_smoothing: false,
            aggregation: true,
            length: 37,
            response_indication: ResponseIndication::Ndp,
            traveling_pilots: true,
        }
    }

    /// Spec worked example [23.3.8.2.2.6, p3776]: m0..m25 =
    /// 1101 1001 1101 1010 0111 1011 11 → c3..c0 = 0101.
    #[test]
    fn crc4_spec_example() {
        let m: Vec<u8> = "11011001110110100111101111".chars().map(|c| c.to_digit(2).unwrap() as u8).collect();
        assert_eq!(m.len(), 26);
        assert_eq!(crc4(&m), [0, 1, 0, 1]);
    }

    #[test]
    fn bits_roundtrip() {
        let f = sample_fields();
        let bits = f.to_bits();
        assert_eq!(bits.len(), 48);
        match parse_short(&bits).unwrap() {
            SigContent::Normal(g) => {
                assert_eq!(g, f);
                assert_eq!(g.color(), 3);
                assert_eq!(g.partial_aid(), 5);
            }
            _ => panic!("not normal"),
        }
    }

    #[test]
    fn corrupted_bits_fail_crc() {
        let mut bits = sample_fields().to_bits();
        bits[10] ^= 1;
        assert!(matches!(parse_short(&bits), Err(SigError::CrcFailed)));
    }

    /// Helper: run a field waveform through FFT + tone extraction (ideal
    /// channel, unit H) and decode with the given format hypothesis.
    fn decode_ideal(wave: &[Complex32], ptype: PreambleType) -> Result<SigContent, SigError> {
        let scale = (N_TONE_SIG as f32).sqrt(); // undo 1/sqrt(52)
        let mut syms = Vec::new();
        for n in 0..2 {
            let f = ofdm::fft_symbol(&wave[n * 80 + 16..n * 80 + 80]);
            let tones: Vec<Complex32> = SIG_SUBCARRIER_INDICES.iter().map(|&k| f[ofdm::bin(k)] * scale).collect();
            syms.push(tones);
        }
        let csi = vec![1.0f32; 48];
        decode(&syms[0], &syms[1], &csi, &csi, ptype)
    }

    fn detect_ideal(wave: &[Complex32]) -> (PreambleType, f32) {
        let f = ofdm::fft_symbol(&wave[96..160]);
        let tones: Vec<Complex32> = SIG_SUBCARRIER_INDICES.iter().map(|&k| f[ofdm::bin(k)]).collect();
        detect_preamble_type(&tones)
    }

    #[test]
    fn ndp_bits_roundtrip() {
        let body = 0x0012_3456_789A_u64 & ((1 << 37) - 1);
        let wave = encode_ndp(body);
        assert_eq!(wave.len(), N_SIG_SAMPLES);
        let content = decode_ideal(&wave, PreambleType::S1gShort);
        assert_eq!(content.unwrap(), SigContent::Ndp { body });
    }

    #[test]
    fn encode_decode_ideal_channel() {
        let f = sample_fields();
        let wave = encode(&f);
        assert_eq!(wave.len(), N_SIG_SAMPLES);
        match decode_ideal(&wave, PreambleType::S1gShort).unwrap() {
            SigContent::Normal(g) => assert_eq!(g, f),
            _ => panic!(),
        }
        // Unit average power.
        let p: f32 = wave.iter().map(|v| v.norm_sqr()).sum::<f32>() / wave.len() as f32;
        assert!((p - 1.0).abs() < 0.05, "SIG power {p}");
    }

    #[test]
    fn format_discrimination() {
        let short = encode(&sample_fields());
        let (t, conf) = detect_ideal(&short);
        assert_eq!(t, PreambleType::S1gShort);
        assert!(conf > 0.99, "conf {conf}");
        let long = encode_sig_a_su(&sample_sig_a());
        let (t, conf) = detect_ideal(&long);
        assert_eq!(t, PreambleType::S1gLong);
        assert!(conf > 0.99, "conf {conf}");
    }

    #[test]
    fn sig_a_su_roundtrip_and_verdict() {
        let f = sample_sig_a();
        let wave = encode_sig_a_su(&f);
        match decode_ideal(&wave, PreambleType::S1gLong).unwrap() {
            SigContent::LongSu(g) => {
                assert_eq!(g, f);
                match g.verdict() {
                    SigVerdict::Supported(rxv) => {
                        assert_eq!(rxv.preamble_type, PreambleType::S1gLong);
                        assert_eq!(rxv.fec_coding, Coding::Ldpc);
                        assert!(rxv.aggregation);
                        assert_eq!(rxv.n_sym, 37);
                        // N_SYM,init = 36 at MCS 3 (104): floor((3744−8)/8) = 467.
                        assert_eq!(rxv.psdu_length, 467);
                        assert_eq!(rxv.partial_aid, 0x155);
                        assert_eq!(rxv.response_indication, ResponseIndication::Ndp);
                        assert!(rxv.traveling_pilots);
                        assert_eq!(rxv.ppdu_duration_us(), 360 + 37 * 40);
                    }
                    other => panic!("{other:?}"),
                }
            }
            other => panic!("{other:?}"),
        }
        // Decoding it as S1G_SHORT must fail (wrong rotation on symbol 2).
        assert!(decode_ideal(&wave, PreambleType::S1gShort).is_err());
    }

    #[test]
    fn sig_a_mu_roundtrip() {
        let f = SigAMu {
            stbc: false,
            nsts: [1, 0, 2, 1],
            bandwidth: 0,
            group_id: 17,
            short_gi: true,
            ldpc: [false, false, true, false],
            ldpc_extra: true,
            length: 200,
            response_indication: ResponseIndication::Long,
            traveling_pilots: false,
        };
        let wave = encode_sig_a_mu(&f);
        match decode_ideal(&wave, PreambleType::S1gLong).unwrap() {
            SigContent::LongMu(g) => {
                assert_eq!(g.nsts, f.nsts);
                assert_eq!(g.group_id, 17);
                assert!(g.short_gi);
                assert!(g.ldpc[2]);
                assert_eq!(g.length, 200);
                match g.verdict() {
                    SigVerdict::Unsupported(rxv, _) => {
                        assert!(rxv.mu);
                        assert_eq!(rxv.num_sts, 4);
                        assert_eq!(rxv.gi, GuardInterval::Short);
                        // 240 + 40 + 4·40 + 40 = 480; data 40 + 199·36.
                        assert_eq!(rxv.ppdu_duration_us(), 480 + 40 + 199 * 36);
                    }
                    other => panic!("{other:?}"),
                }
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn rxvector_derivation() {
        // Non-aggregated, MCS 4 (N_DBPS 156), 100 octets:
        // N_SYM = ceil(814/156) = 6.
        let f = sample_fields();
        let rxv = f.to_rxvector().unwrap();
        assert_eq!(rxv.psdu_length, 100);
        assert_eq!(rxv.n_sym, 6);
        assert_eq!(rxv.length, 100);
        // Aggregated: length = N_SYM = 20 → PSDU = floor((20*156−14)/8) = 388.
        let mut fa = sample_fields();
        fa.aggregation = true;
        fa.length = 20;
        let rxa = fa.to_rxvector().unwrap();
        assert_eq!(rxa.n_sym, 20);
        assert_eq!(rxa.psdu_length, (20 * 156 - 14) / 8);
    }

    #[test]
    fn verdict_classes() {
        let mut f = sample_fields();
        f.ldpc = true;
        f.ldpc_extra = false;
        assert!(matches!(f.verdict(), SigVerdict::Supported(_)));
        let mut f2 = sample_fields();
        f2.nsts = 1;
        assert!(matches!(f2.verdict(), SigVerdict::Unsupported(_, "multiple spatial streams")));
        let mut f3 = sample_fields();
        f3.mcs = 9; // not valid at 1 SS: duration unknown without aggregation
        assert!(matches!(f3.verdict(), SigVerdict::Reserved { duration_us: None, .. }));
        f3.aggregation = true;
        f3.length = 20; // …but N_SYM = LENGTH when aggregated
        assert!(matches!(f3.verdict(), SigVerdict::Reserved { duration_us: Some(1040), .. }));
        let mut f4 = sample_fields();
        f4.stbc = true; // N_STS=1 with STBC is not a defined mode
        assert!(matches!(f4.verdict(), SigVerdict::Reserved { .. }));
        let mut f5 = sample_fields();
        f5.ldpc_extra = false; // BCC requires B18 = 1; duration still computable
        assert!(matches!(f5.verdict(), SigVerdict::Reserved { duration_us: Some(480), .. }));
        let mut f6 = sample_fields();
        f6.length = 0;
        assert!(matches!(f6.verdict(), SigVerdict::Reserved { duration_us: None, .. }));
        // Short GI is decoded; its first Data symbol keeps the long GI.
        let mut f7 = sample_fields();
        f7.short_gi = true;
        match f7.verdict() {
            SigVerdict::Supported(r) => {
                assert_eq!(r.gi, GuardInterval::Short);
                assert_eq!(r.data_duration_us(), 40 + 5 * 36);
            }
            other => panic!("{other:?}"),
        }
        // Unsupported modes still get a duration.
        let mut f8 = sample_fields();
        f8.bandwidth = 1;
        match f8.verdict() {
            SigVerdict::Unsupported(r, "bandwidth > 2 MHz") => {
                assert_eq!(r.data_duration_us(), 6 * 40)
            }
            other => panic!("{other:?}"),
        }
        // Reserved B0 is caught at parse time, with the duration preserved.
        let mut bits = sample_fields().to_bits();
        bits[0] = 0;
        let crc = crc4(&bits[..38]);
        bits[38..42].copy_from_slice(&crc);
        assert!(matches!(parse_short(&bits), Err(SigError::Reserved { duration_us: Some(480), .. })));
    }
}
