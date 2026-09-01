//! S1G_SHORT SIG field (2 MHz): bit layout [Table 23-12], CRC-4
//! [23.3.8.2.2.6], encoding [23.3.4.3.3] and decoding.
//!
//! 48 uncoded bits (SIG-1 B0..B23 then SIG-2 B0..B23) → BCC R=1/2 (96 bits)
//! → per-symbol 48-bit Clause-17 interleave → BPSK rotated +90° (QBPSK,
//! both symbols — the S1G_SHORT format discriminator) → 48 data tones
//! (±1..±26 minus pilots) + Clause-17 pilots, scaled 1/√52, 8 µs GI.
//! All numeric fields are LSB-first [23.3.8.1].

use crate::bits::{bits_to_uint_lsb_first, push_uint_lsb_first};
use crate::error::PhyError;
use crate::ofdm::{self, SIG_SUBCARRIER_INDICES};
use crate::params::{self, N_GI_LONG, N_TONE_SIG};
use crate::vector::{GuardInterval, ResponseIndication, RxVector, TxVector};
use crate::{bcc, interleaver, mapping, pilots, Complex32};

/// Decoded/encodable SIG contents for a normal (non-NDP) PPDU.
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
    /// Set to 1 on transmit when coding is BCC [Table 23-12 B18].
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

/// Result of decoding a SIG field.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SigContent {
    Normal(SigFields),
    /// NDP CMAC PPDU: 37-bit body (B0 first = LSB) [23.3.11, Fig 23-21].
    Ndp { body: u64 },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SigError {
    /// CRC over bits 0..37 failed.
    CrcFailed,
    /// Tail bits nonzero or reserved bit B0 not 1 — likely a false detect
    /// or a non-S1G_SHORT format.
    Malformed,
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

impl SigFields {
    /// COLOR carried in the ID field (only meaningful when
    /// `uplink_indication` is false).
    pub fn color(&self) -> u8 {
        (self.id & 0x7) as u8
    }

    /// Partial AID from the ID field.
    pub fn partial_aid(&self) -> u16 {
        if self.uplink_indication {
            self.id & 0x1ff
        } else {
            (self.id >> 3) & 0x3f
        }
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
        debug_assert_eq!(v.len(), 38);
        let crc = crc4(&v);
        v.extend_from_slice(&crc);
        v.extend_from_slice(&[0; 6]); // tail
        let mut out = [0u8; 48];
        out.copy_from_slice(&v);
        out
    }

    /// Build from a TXVECTOR plus the Length-field value (octets or N_SYM
    /// per aggregation — computed by the TX chain).
    pub fn from_txvector(txv: &TxVector, length_field: u16) -> Result<Self, PhyError> {
        params::mcs_params(txv.mcs)?;
        if txv.gi != GuardInterval::Long {
            return Err(PhyError::Unsupported("short GI"));
        }
        if txv.traveling_pilots {
            return Err(PhyError::Unsupported("traveling pilots"));
        }
        if txv.color > 7 {
            return Err(PhyError::InvalidTxVector("color > 7"));
        }
        let id = if txv.uplink_indication {
            if txv.partial_aid > 511 {
                return Err(PhyError::InvalidTxVector("partial_aid > 511"));
            }
            txv.partial_aid
        } else {
            if txv.partial_aid > 63 {
                return Err(PhyError::InvalidTxVector("partial_aid > 63 with uplink_indication = false"));
            }
            (txv.partial_aid << 3) | txv.color as u16
        };
        Ok(SigFields {
            stbc: false,
            uplink_indication: txv.uplink_indication,
            bandwidth: 0,
            nsts: 0,
            id,
            short_gi: false,
            ldpc: false,
            ldpc_extra: true,
            mcs: txv.mcs,
            smoothing: txv.smoothing,
            aggregation: txv.aggregation,
            length: length_field,
            response_indication: txv.response_indication,
            traveling_pilots: false,
        })
    }

    /// Derive the RXVECTOR skeleton (lengths from Eq 23-80 / Eq 23-81).
    /// Fails for configurations this implementation cannot receive.
    pub fn to_rxvector(&self) -> Result<RxVector, PhyError> {
        if self.short_gi {
            return Err(PhyError::Unsupported("short GI"));
        }
        if self.stbc {
            return Err(PhyError::Unsupported("STBC"));
        }
        if self.ldpc {
            return Err(PhyError::Unsupported("LDPC"));
        }
        if self.bandwidth != 0 {
            return Err(PhyError::Unsupported("bandwidth > 2 MHz"));
        }
        if self.nsts != 0 {
            return Err(PhyError::Unsupported("multiple spatial streams"));
        }
        if self.traveling_pilots {
            return Err(PhyError::Unsupported("traveling pilots"));
        }
        let p = params::mcs_params(self.mcs)?;
        let (psdu_length, n_sym) = if self.aggregation {
            let n_sym = self.length as usize;
            if n_sym == 0 {
                return Err(PhyError::InvalidLength { len: 0, reason: "aggregated PPDU with zero symbols" });
            }
            ((n_sym * p.n_dbps).saturating_sub(14) / 8, n_sym)
        } else {
            let len = self.length as usize;
            if len == 0 {
                // Length 0 without NDP indication: nothing to receive.
                return Err(PhyError::InvalidLength { len: 0, reason: "zero-length PSDU" });
            }
            (len, (8 * len + 14).div_ceil(p.n_dbps))
        };
        Ok(RxVector {
            mcs: self.mcs,
            gi: GuardInterval::Long,
            aggregation: self.aggregation,
            response_indication: self.response_indication,
            smoothing: self.smoothing,
            traveling_pilots: false,
            uplink_indication: self.uplink_indication,
            color: if self.uplink_indication { 0 } else { self.color() },
            partial_aid: self.partial_aid(),
            psdu_length,
            n_sym,
            scrambler_seed: 0,
            rssi_dbfs: 0.0,
        })
    }
}

/// Parse 48 decoded SIG bits into content (checks structure, not CRC —
/// callers use [`decode`], which checks both).
fn parse_bits(bits: &[u8; 48]) -> Result<SigContent, SigError> {
    if bits[42..48].iter().any(|&b| b != 0) {
        return Err(SigError::Malformed);
    }
    let crc_expect = crc4(&bits[..38]);
    if bits[38..42] != crc_expect {
        return Err(SigError::CrcFailed);
    }
    let ndp = bits[37] == 1;
    if ndp {
        let body = bits_to_uint_lsb_first(&bits[..37]);
        return Ok(SigContent::Ndp { body });
    }
    if bits[0] != 1 {
        // Reserved bit is transmitted as 1 in S1G_SHORT.
        return Err(SigError::Malformed);
    }
    Ok(SigContent::Normal(SigFields {
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
    }))
}

/// Encode 48 SIG bits into the 160-sample SIG field (2 symbols, 2 MS/s).
fn encode_bits(bits: &[u8; 48]) -> Vec<Complex32> {
    let coded = bcc::encode_r12(bits); // 96 bits, no puncturing
    let scale = 1.0 / (N_TONE_SIG as f32).sqrt();
    let mut out = Vec::with_capacity(160);
    for n in 0..2 {
        let ilv = interleaver::interleave_sig(&coded[n * 48..(n + 1) * 48]);
        // BPSK map then rotate +90° (×j): bit 0 → −j, bit 1 → +j.
        let tones: Vec<Complex32> = ilv
            .iter()
            .map(|&b| Complex32::new(0.0, if b == 1 { 1.0 } else { -1.0 }))
            .collect();
        let sym = ofdm::assemble_freq_symbol(&SIG_SUBCARRIER_INDICES, &tones, &pilots::sig_pilots(n));
        out.extend(ofdm::to_time_domain(&sym, N_GI_LONG, scale));
    }
    out
}

/// TX: SIG field waveform for a normal PPDU.
pub fn encode(fields: &SigFields) -> Vec<Complex32> {
    encode_bits(&fields.to_bits())
}

/// TX: SIG field waveform for an NDP CMAC PPDU (37-bit body, LSB = B0).
pub fn encode_ndp(body: u64) -> Vec<Complex32> {
    let mut v = Vec::with_capacity(48);
    push_uint_lsb_first(&mut v, body & ((1u64 << 37) - 1), 37);
    v.push(1); // NDP Indication
    let crc = crc4(&v);
    v.extend_from_slice(&crc);
    v.extend_from_slice(&[0; 6]);
    let mut bits = [0u8; 48];
    bits.copy_from_slice(&v);
    encode_bits(&bits)
}

/// RX: decode SIG from the two equalized symbols' 48 data tones each (in
/// `SIG_SUBCARRIER_INDICES` order, CPE already corrected) plus per-tone CSI.
pub fn decode(sym1: &[Complex32], sym2: &[Complex32], csi1: &[f32], csi2: &[f32]) -> Result<SigContent, SigError> {
    debug_assert_eq!(sym1.len(), 48);
    debug_assert_eq!(sym2.len(), 48);
    let mut llrs = Vec::with_capacity(96);
    for (sym, csi) in [(sym1, csi1), (sym2, csi2)] {
        // Undo the QBPSK rotation: multiply by −j, then plain BPSK demap.
        let derot: Vec<Complex32> = sym.iter().map(|v| Complex32::new(v.im, -v.re)).collect();
        let sym_llrs = mapping::demap_llrs(&derot, csi, params::Modulation::Bpsk);
        llrs.extend(interleaver::deinterleave_sig_llrs(&sym_llrs));
    }
    let bits_vec = bcc::viterbi_decode(&llrs, params::CodeRate::R1_2, 48);
    let mut bits = [0u8; 48];
    bits.copy_from_slice(&bits_vec);
    parse_bits(&bits)
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

    /// Spec worked example [23.3.8.2.2.6, p3776]: m0..m25 =
    /// 1101 1001 1101 1010 0111 1011 11 → c3..c0 = 0101.
    #[test]
    fn crc4_spec_example() {
        let m: Vec<u8> = "11011001110110100111101111"
            .chars()
            .map(|c| c.to_digit(2).unwrap() as u8)
            .collect();
        assert_eq!(m.len(), 26);
        assert_eq!(crc4(&m), [0, 1, 0, 1]);
    }

    #[test]
    fn bits_roundtrip() {
        let f = sample_fields();
        let bits = f.to_bits();
        assert_eq!(bits.len(), 48);
        match parse_bits(&bits).unwrap() {
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
        assert!(matches!(parse_bits(&bits), Err(SigError::CrcFailed)));
    }

    #[test]
    fn ndp_bits_roundtrip() {
        let body = 0x0123_4567_89Au64 & ((1 << 37) - 1);
        let wave = encode_ndp(body);
        assert_eq!(wave.len(), N_SIG_SAMPLES);
        // Round-trip through an ideal channel.
        let content = decode_ideal(&wave);
        assert_eq!(content.unwrap(), SigContent::Ndp { body });
    }

    /// Helper: run the TX waveform through FFT + tone extraction (ideal
    /// channel, unit H) and decode.
    fn decode_ideal(wave: &[Complex32]) -> Result<SigContent, SigError> {
        let scale = (N_TONE_SIG as f32).sqrt(); // undo 1/sqrt(52)
        let mut syms = Vec::new();
        for n in 0..2 {
            let f = ofdm::fft_symbol(&wave[n * 80 + 16..n * 80 + 80]);
            let tones: Vec<Complex32> = SIG_SUBCARRIER_INDICES.iter().map(|&k| f[ofdm::bin(k)] * scale).collect();
            syms.push(tones);
        }
        let csi = vec![1.0f32; 48];
        decode(&syms[0], &syms[1], &csi, &csi)
    }

    #[test]
    fn encode_decode_ideal_channel() {
        let f = sample_fields();
        let wave = encode(&f);
        assert_eq!(wave.len(), N_SIG_SAMPLES);
        match decode_ideal(&wave).unwrap() {
            SigContent::Normal(g) => assert_eq!(g, f),
            _ => panic!(),
        }
        // Unit average power.
        let p: f32 = wave.iter().map(|v| v.norm_sqr()).sum::<f32>() / wave.len() as f32;
        assert!((p - 1.0).abs() < 0.05, "SIG power {p}");
    }

    #[test]
    fn rxvector_derivation() {
        // Non-aggregated, MCS 4 (N_DBPS 156), 100 octets:
        // N_SYM = ceil(814/156) = 6.
        let f = sample_fields();
        let rxv = f.to_rxvector().unwrap();
        assert_eq!(rxv.psdu_length, 100);
        assert_eq!(rxv.n_sym, 6);
        // Aggregated: length = N_SYM = 20 → PSDU = floor((20*156−14)/8) = 388.
        let mut fa = sample_fields();
        fa.aggregation = true;
        fa.length = 20;
        let rxa = fa.to_rxvector().unwrap();
        assert_eq!(rxa.n_sym, 20);
        assert_eq!(rxa.psdu_length, (20 * 156 - 14) / 8);
    }

    #[test]
    fn unsupported_flags_rejected() {
        let mut f = sample_fields();
        f.ldpc = true;
        assert!(f.to_rxvector().is_err());
        let mut f2 = sample_fields();
        f2.nsts = 1;
        assert!(f2.to_rxvector().is_err());
    }
}
