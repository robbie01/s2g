//! TXVECTOR / RXVECTOR analogues (IEEE 802.11-2024 23.2.2, Table 23-1) for
//! 2 MHz operation.
//!
//! These are the PHY↔MAC contract. The MAC constructs a [`TxVector`] per PPDU
//! and receives an [`RxVector`] with `PHY-RXSTART` / each decoded PSDU.
//! Transmission is restricted to 1 spatial stream, S1G_SHORT, long GI, BCC or
//! LDPC. Reception decodes the same set; every other mode the SIG / SIG-A can
//! signal is still *identified* (for CCA and RID) and reported as an
//! unsupported rate.

use crate::params::{self, N_SERVICE, N_TAIL, T_DLTF_US, T_DSTF_US, T_LTF_US, T_PREAMBLE_US, T_SIGB_US, T_SYML_US, T_SYMS_US};

/// Guard interval selection. Only `Long` (8 µs) is transmitted/decoded;
/// `Short` PPDUs are identified for duration prediction only.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum GuardInterval {
    #[default]
    Long,
    Short,
}

/// FEC_CODING [Table 23-1].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Coding {
    #[default]
    Bcc,
    Ldpc,
}

/// PREAMBLE_TYPE [Table 23-1]. S1G_1M is out of scope (1 MHz).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PreambleType {
    #[default]
    S1gShort,
    S1gLong,
}

/// SIG-2 Response Indication field (what the transmitter expects SIFS after
/// this PPDU; used by third parties for RID deferral) [Table 23-12; 10.3.2.5].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ResponseIndication {
    #[default]
    None,
    Ndp,
    Normal,
    Long,
}

impl ResponseIndication {
    pub fn to_bits(self) -> u8 {
        match self {
            ResponseIndication::None => 0,
            ResponseIndication::Ndp => 1,
            ResponseIndication::Normal => 2,
            ResponseIndication::Long => 3,
        }
    }
    pub fn from_bits(v: u8) -> Self {
        match v & 3 {
            0 => ResponseIndication::None,
            1 => ResponseIndication::Ndp,
            2 => ResponseIndication::Normal,
            _ => ResponseIndication::Long,
        }
    }
}

/// Per-PPDU transmit parameters (TXVECTOR analogue).
#[derive(Debug, Clone, PartialEq)]
pub struct TxVector {
    /// S1G MCS index; valid for 2 MHz / 1 SS: 0..=8 or 11.
    pub mcs: u8,
    /// Guard interval (must be `Long`).
    pub gi: GuardInterval,
    /// FEC_CODING: BCC (mandatory) or LDPC (optional, 23.3.9.4.4).
    pub fec_coding: Coding,
    /// True when the PSDU is an A-MPDU. Mandatory for PSDUs > 511 octets
    /// (SIG Length is 9 bits). Changes SIG Length semantics to symbols.
    pub aggregation: bool,
    /// SIG-2 Response Indication.
    pub response_indication: ResponseIndication,
    /// Scrambler initial state (1..=127). `None` → pseudo-random per PPDU.
    /// A MAC that expects an NDP Ack / NDP BlockAck must choose the seed
    /// itself, because the Ack ID is derived from it [23.3.12.2.4].
    pub scrambler_seed: Option<u8>,
    /// SIG-1 Smoothing bit (channel-smoothing recommendation to receivers).
    pub smoothing: bool,
    /// Traveling pilots (Doppler mode) [23.3.9.10]. Optional feature; only
    /// send to a peer that advertised support (10.55).
    pub traveling_pilots: bool,
    /// SIG-1 Uplink Indication: 1 = PPDU addressed to an AP [Table 23-12].
    /// For OCB/non-BSS operation leave false.
    pub uplink_indication: bool,
    /// BSS COLOR (0..=7), carried in SIG-1 ID bits when
    /// `uplink_indication` is false.
    pub color: u8,
    /// Partial AID: 0..=511 when `uplink_indication`, else 0..=63.
    pub partial_aid: u16,
}

impl Default for TxVector {
    fn default() -> Self {
        Self {
            mcs: 0,
            gi: GuardInterval::Long,
            fec_coding: Coding::Bcc,
            aggregation: false,
            response_indication: ResponseIndication::None,
            scrambler_seed: None,
            smoothing: true,
            traveling_pilots: false,
            uplink_indication: false,
            color: 0,
            partial_aid: 0,
        }
    }
}

/// Per-PPDU receive parameters and measurements (RXVECTOR analogue).
#[derive(Debug, Clone, PartialEq)]
pub struct RxVector {
    /// FORMAT is always S1G at 2 MHz; PREAMBLE_TYPE distinguishes the two
    /// ≥ 2 MHz preambles.
    pub preamble_type: PreambleType,
    /// MU/SU subfield of an S1G_LONG SIG-A (always false for S1G_SHORT).
    pub mu: bool,
    /// CH_BANDWIDTH as the SIG BW field code: 0 = 2 MHz, 1 = 4, 2 = 8, 3 = 16.
    pub bandwidth_code: u8,
    /// NUM_STS (total space-time streams; per-user for MU is not tracked).
    pub num_sts: u8,
    pub stbc: bool,
    pub mcs: u8,
    pub gi: GuardInterval,
    pub fec_coding: Coding,
    /// LDPC Extra OFDM Symbol bit.
    pub ldpc_extra: bool,
    pub aggregation: bool,
    pub response_indication: ResponseIndication,
    pub smoothing: bool,
    pub traveling_pilots: bool,
    pub uplink_indication: bool,
    /// COLOR when `uplink_indication` is false (0 otherwise).
    pub color: u8,
    pub partial_aid: u16,
    /// Group ID of an MU PPDU (S1G_LONG only).
    pub group_id: u8,
    /// Raw 9-bit SIG LENGTH field (octets or symbols per `aggregation`).
    pub length: u16,
    /// PSDU length in octets derived from the SIG (Eq 23-71 / 23-72 when
    /// aggregated). 0 for an NDP.
    pub psdu_length: usize,
    /// Number of Data-field OFDM symbols.
    pub n_sym: usize,
    /// Recovered scrambler seed (RXVECTOR SCRAMBLER_OR_CRC); 0 until the
    /// SERVICE field has been decoded.
    pub scrambler_seed: u8,
    /// RSSI, 0–255, measured over LTF1, monotonically increasing with
    /// received power [23.3.18.6]. Implementation: 2·(dBFS + 127.5).
    pub rssi: u8,
    /// The same measurement in dB relative to full scale.
    pub rssi_dbfs: f32,
    /// RCPI code [Table 9-215] for the LTF1 power converted to dBm through
    /// the receiver's calibration offset [23.3.18.7].
    pub rcpi: u8,
    /// RCPI in dBm (before encoding).
    pub rcpi_dbm: f32,
    /// SNR: mean over the used tones of the per-tone SNR in dB [Table 23-1].
    pub snr_db: f32,
}

impl Default for RxVector {
    fn default() -> Self {
        Self {
            preamble_type: PreambleType::S1gShort,
            mu: false,
            bandwidth_code: 0,
            num_sts: 1,
            stbc: false,
            mcs: 0,
            gi: GuardInterval::Long,
            fec_coding: Coding::Bcc,
            ldpc_extra: false,
            aggregation: false,
            response_indication: ResponseIndication::None,
            smoothing: false,
            traveling_pilots: false,
            uplink_indication: false,
            color: 0,
            partial_aid: 0,
            group_id: 0,
            length: 0,
            psdu_length: 0,
            n_sym: 0,
            scrambler_seed: 0,
            rssi: 0,
            rssi_dbfs: -100.0,
            rcpi: 255,
            rcpi_dbm: f32::NAN,
            snr_db: 0.0,
        }
    }
}

impl RxVector {
    /// Number of spatial streams (N_STS halved under STBC).
    pub fn num_ss(&self) -> u8 {
        if self.stbc {
            (self.num_sts / 2).max(1)
        } else {
            self.num_sts.max(1)
        }
    }

    /// Duration of everything before the Data field, µs: STF + LTF1 + SIG
    /// (+ LTF2..N for S1G_SHORT; + D-STF + D-LTFs + SIG-B for S1G_LONG)
    /// [Figure 23-5; Table 23-5].
    pub fn preamble_duration_us(&self) -> u32 {
        let n_ltf = params::n_ltf(self.num_sts) as u32;
        match self.preamble_type {
            PreambleType::S1gShort => T_PREAMBLE_US + (n_ltf - 1) * T_LTF_US,
            PreambleType::S1gLong => T_PREAMBLE_US + T_DSTF_US + n_ltf * T_DLTF_US + T_SIGB_US,
        }
    }

    /// Data-field duration, µs (short GI: first symbol long, rest short)
    /// [Eq 23-69].
    pub fn data_duration_us(&self) -> u32 {
        let n = self.n_sym as u32;
        match self.gi {
            GuardInterval::Long => n * T_SYML_US,
            GuardInterval::Short => {
                if n == 0 {
                    0
                } else {
                    T_SYML_US + (n - 1) * T_SYMS_US
                }
            }
        }
    }

    /// Total PPDU duration (TXTIME of the received PPDU), µs.
    pub fn ppdu_duration_us(&self) -> u32 {
        self.preamble_duration_us() + self.data_duration_us()
    }

    /// RXTIME as literally defined by Eq 23-69 (S1G_SHORT) / Eq 23-70
    /// (S1G_LONG): the duration after SIG/SIG-A plus one 40 µs margin.
    pub fn rxtime_us(&self) -> u32 {
        self.ppdu_duration_us() - T_PREAMBLE_US + T_DSTF_US
            - if self.preamble_type == PreambleType::S1gLong { T_DSTF_US } else { 0 }
    }
}

/// N_SYM and PSDU_LENGTH from the SIG/SIG-A fields, for any coding and STBC
/// setting [23.3.20, Eqs 23-65..23-68, 23-71, 23-72]. `n_dbps` is the
/// (possibly multi-stream) N_DBPS. Returns `None` for an empty data field.
pub fn data_field_geometry(
    length: u16,
    aggregation: bool,
    coding: Coding,
    ldpc_extra: bool,
    stbc: bool,
    n_dbps: usize,
) -> Option<(usize, usize)> {
    let m_stbc = if stbc { 2 } else { 1 };
    let len = length as usize;
    match (coding, aggregation) {
        (Coding::Bcc, true) => {
            let n_sym = len;
            (n_sym > 0).then(|| (n_sym, (n_sym * n_dbps).saturating_sub(N_SERVICE + N_TAIL) / 8))
        }
        (Coding::Bcc, false) => {
            if len == 0 {
                return None;
            }
            let n_sym = m_stbc * (8 * len + N_SERVICE + N_TAIL).div_ceil(m_stbc * n_dbps);
            Some((n_sym, len))
        }
        (Coding::Ldpc, true) => {
            // LENGTH counts the transmitted symbols including any LDPC extra
            // symbol(s); the PSDU capacity is that of N_SYM,init (Eq 23-82).
            let n_sym = len;
            if n_sym == 0 {
                return None;
            }
            let extra = if ldpc_extra { m_stbc } else { 0 };
            let n_init = n_sym.checked_sub(extra)?.max(1);
            Some((n_sym, (n_init * n_dbps).saturating_sub(N_SERVICE) / 8))
        }
        (Coding::Ldpc, false) => {
            if len == 0 {
                return None;
            }
            let n_init = m_stbc * (8 * len + N_SERVICE).div_ceil(m_stbc * n_dbps);
            let n_sym = n_init + if ldpc_extra { m_stbc } else { 0 };
            Some((n_sym, len))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn short_preamble_durations() {
        let r = RxVector { n_sym: 32, ..Default::default() };
        assert_eq!(r.preamble_duration_us(), 240);
        assert_eq!(r.ppdu_duration_us(), 240 + 32 * 40); // 1520 [digest sanity]
        assert_eq!(r.rxtime_us(), 40 + 32 * 40);
        let sgi = RxVector { n_sym: 32, gi: GuardInterval::Short, ..Default::default() };
        assert_eq!(sgi.data_duration_us(), 40 + 31 * 36);
        let two_sts = RxVector { n_sym: 10, num_sts: 2, ..Default::default() };
        assert_eq!(two_sts.preamble_duration_us(), 280);
    }

    #[test]
    fn long_preamble_durations() {
        let r = RxVector { preamble_type: PreambleType::S1gLong, n_sym: 5, num_sts: 1, ..Default::default() };
        // STF 80 + LTF1 80 + SIG-A 80 + D-STF 40 + D-LTF 40 + SIG-B 40 = 360.
        assert_eq!(r.preamble_duration_us(), 360);
        assert_eq!(r.ppdu_duration_us(), 360 + 200);
        // Eq 23-70: T_DSTF + N_LTF·T_DLTF + T_SIG-B + N_SYM·T_SYML.
        assert_eq!(r.rxtime_us(), 40 + 40 + 40 + 200);
        let r3 = RxVector { preamble_type: PreambleType::S1gLong, n_sym: 5, num_sts: 3, ..Default::default() };
        assert_eq!(r3.preamble_duration_us(), 240 + 40 + 4 * 40 + 40);
    }

    #[test]
    fn geometry_bcc() {
        // MCS 4 (156), 100 octets → ceil(814/156) = 6.
        assert_eq!(data_field_geometry(100, false, Coding::Bcc, false, false, 156), Some((6, 100)));
        // Aggregated 20 symbols → floor((3120−14)/8) = 388.
        assert_eq!(data_field_geometry(20, true, Coding::Bcc, false, false, 156), Some((20, 388)));
        // STBC doubles the symbol granularity.
        assert_eq!(data_field_geometry(100, false, Coding::Bcc, false, true, 156), Some((6, 100)));
        assert_eq!(data_field_geometry(101, false, Coding::Bcc, false, true, 156), Some((6, 101)));
        assert_eq!(data_field_geometry(0, false, Coding::Bcc, false, false, 156), None);
    }

    #[test]
    fn geometry_ldpc() {
        // No tail bits: 100 octets at MCS 4 → ceil(808/156) = 6 (+1 extra).
        assert_eq!(data_field_geometry(100, false, Coding::Ldpc, false, false, 156), Some((6, 100)));
        assert_eq!(data_field_geometry(100, false, Coding::Ldpc, true, false, 156), Some((7, 100)));
        // Aggregated: LENGTH = 21 with extra ⇒ N_SYM,init = 20 ⇒ (3120−8)/8 = 389.
        assert_eq!(data_field_geometry(21, true, Coding::Ldpc, true, false, 156), Some((21, 389)));
        assert_eq!(data_field_geometry(20, true, Coding::Ldpc, false, false, 156), Some((20, 389)));
    }
}
