//! S1G PHY constants and MCS tables for 2 MHz.
//! All values from IEEE 802.11-2024; citations are [clause, PDF page] against
//! docs/spec-digest/.

use crate::error::PhyError;

/// FFT length for 2 MHz S1G [23.3.7, p3761].
pub const N_FFT: usize = 64;
/// Native complex baseband sample rate, Hz (64 × 31.25 kHz).
pub const SAMPLE_RATE_HZ: f64 = 2.0e6;
/// Subcarrier spacing, Hz [Table 23-5].
pub const DELTA_F_HZ: f64 = 31_250.0;
/// Long guard interval, samples (8 µs) [Table 23-5].
pub const N_GI_LONG: usize = 16;
/// Short guard interval, samples (4 µs) [Table 23-5].
pub const N_GI_SHORT: usize = 8;
/// Double guard interval, samples (16 µs, LTF1 only) [Table 23-5].
pub const N_GI2: usize = 32;
/// Long-GI OFDM symbol, samples (40 µs).
pub const N_SYM_SAMPLES_LGI: usize = N_FFT + N_GI_LONG;

/// Data-field tone plan (2 MHz) [Table 23-5, p3758].
pub const N_SD: usize = 52;
pub const N_SP: usize = 4;
pub const N_ST: usize = N_SD + N_SP;
/// Highest occupied subcarrier index in the Data field / LTF.
pub const N_SR: i32 = 28;

/// SIG-field tone plan (2 MHz S1G_SHORT; 11a-like) [Table 23-6, p3759].
pub const SIG_N_SD: usize = 48;
pub const SIG_N_ST: usize = 52;
/// Highest occupied subcarrier index in the SIG field.
pub const SIG_N_SR: i32 = 26;

/// Field durations in samples at 2 MS/s [Table 23-5; digest timing-math §7].
pub const N_STF_SAMPLES: usize = 160; // 80 µs, 10 × 16-sample period
pub const N_LTF1_SAMPLES: usize = 160; // 80 µs: [GI2 32][LTS 64][LTS 64]
pub const N_SIG_SAMPLES: usize = 160; // 80 µs: 2 × (16 GI + 64)
/// Preamble total (STF+LTF1+SIG) for 1 STS: 240 µs.
pub const N_PREAMBLE_SAMPLES: usize = N_STF_SAMPLES + N_LTF1_SAMPLES + N_SIG_SAMPLES;

/// Field durations in µs [Table 23-5].
pub const T_STF_US: u32 = 80;
pub const T_LTF1_US: u32 = 80;
pub const T_SIG_US: u32 = 80;
/// Second and later LTFs of an S1G_SHORT PPDU (N_STS > 1), µs.
pub const T_LTF_US: u32 = 40;
/// S1G_LONG-only fields: D-STF, each D-LTF, SIG-B [Table 23-5].
pub const T_DSTF_US: u32 = 40;
pub const T_DLTF_US: u32 = 40;
pub const T_SIGB_US: u32 = 40;
/// Long-GI / short-GI OFDM symbol durations, µs.
pub const T_SYML_US: u32 = 40;
pub const T_SYMS_US: u32 = 36;
/// STF + LTF1 + SIG (or SIG-A): the omnidirectional part shared by
/// S1G_SHORT and S1G_LONG, µs.
pub const T_PREAMBLE_US: u32 = T_STF_US + T_LTF1_US + T_SIG_US;

/// Per-field tone-scaling counts N_Tone [Table 23-8, pp3764–3765].
pub const N_TONE_STF: usize = 12;
pub const N_TONE_LTF: usize = 56;
pub const N_TONE_SIG: usize = 52;
pub const N_TONE_DATA: usize = 56;

/// SERVICE field bits (8 in S1G, 16 in other OFDM PHYs) [Table 23-5; 23.3.9.2].
pub const N_SERVICE: usize = 8;
/// BCC tail bits [Table 23-5].
pub const N_TAIL: usize = 6;

/// Max PSDU octets without aggregation (9-bit SIG Length) [Table 23-41].
pub const PSDU_MAX_NO_AGG: usize = 511;
/// Max Data-field symbol count (9-bit SIG Length, aggregated) [23.4.3].
pub const N_SYM_MAX: usize = 511;

/// Number of LTF / D-LTF symbols for N_STS space-time streams
/// [Table 23-11, p3769].
pub fn n_ltf(n_sts: u8) -> u8 {
    match n_sts {
        0 | 1 => 1,
        2 => 2,
        _ => 4,
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Modulation {
    Bpsk,
    Qpsk,
    Qam16,
    Qam64,
    Qam256,
    Qam1024,
}

impl Modulation {
    /// Coded bits per subcarrier (N_BPSCS).
    pub fn n_bpscs(self) -> usize {
        match self {
            Modulation::Bpsk => 1,
            Modulation::Qpsk => 2,
            Modulation::Qam16 => 4,
            Modulation::Qam64 => 6,
            Modulation::Qam256 => 8,
            Modulation::Qam1024 => 10,
        }
    }

    /// K_MOD normalization [Eq 17-20; 21.3.10.9.1; 27.3.12.9].
    pub fn k_mod(self) -> f32 {
        match self {
            Modulation::Bpsk => 1.0,
            Modulation::Qpsk => 1.0 / (2.0f32).sqrt(),
            Modulation::Qam16 => 1.0 / (10.0f32).sqrt(),
            Modulation::Qam64 => 1.0 / (42.0f32).sqrt(),
            Modulation::Qam256 => 1.0 / (170.0f32).sqrt(),
            Modulation::Qam1024 => 1.0 / (682.0f32).sqrt(),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CodeRate {
    R1_2,
    R2_3,
    R3_4,
    R5_6,
}

impl CodeRate {
    /// (numerator, denominator).
    pub fn as_fraction(self) -> (usize, usize) {
        match self {
            CodeRate::R1_2 => (1, 2),
            CodeRate::R2_3 => (2, 3),
            CodeRate::R3_4 => (3, 4),
            CodeRate::R5_6 => (5, 6),
        }
    }

    pub fn as_f64(self) -> f64 {
        let (n, d) = self.as_fraction();
        n as f64 / d as f64
    }
}

/// Modulation and code rate for every S1G-MCS index defined for ≥ 2 MHz
/// [23.3.5, Tables 23-46..23-49]. Validity for a given N_SS is a separate
/// question (see [`n_dbps_2mhz`]); MCS 10 exists only at 1 MHz.
pub fn mcs_modulation_rate(mcs: u8) -> Option<(Modulation, CodeRate)> {
    Some(match mcs {
        0 => (Modulation::Bpsk, CodeRate::R1_2),
        1 => (Modulation::Qpsk, CodeRate::R1_2),
        2 => (Modulation::Qpsk, CodeRate::R3_4),
        3 => (Modulation::Qam16, CodeRate::R1_2),
        4 => (Modulation::Qam16, CodeRate::R3_4),
        5 => (Modulation::Qam64, CodeRate::R2_3),
        6 => (Modulation::Qam64, CodeRate::R3_4),
        7 => (Modulation::Qam64, CodeRate::R5_6),
        8 => (Modulation::Qam256, CodeRate::R3_4),
        9 => (Modulation::Qam256, CodeRate::R5_6),
        11 => (Modulation::Qam1024, CodeRate::R3_4),
        12 => (Modulation::Qam1024, CodeRate::R5_6),
        _ => return None,
    })
}

/// N_DBPS for (MCS, N_SS) at 2 MHz, or `None` when the combination is "Not
/// valid" in Tables 23-46..23-49 (N_DBPS = 52·N_BPSCS·N_SS·R must be an
/// integer — e.g. MCS 9 and 12 exist only for N_SS = 3). Used for PPDU
/// duration prediction of PPDUs this receiver cannot decode.
pub fn n_dbps_2mhz(mcs: u8, n_ss: u8) -> Option<usize> {
    if !(1..=4).contains(&n_ss) {
        return None;
    }
    let (m, r) = mcs_modulation_rate(mcs)?;
    let n_cbps = N_SD * m.n_bpscs() * n_ss as usize;
    let (num, den) = r.as_fraction();
    (n_cbps * num).is_multiple_of(den).then_some(n_cbps * num / den)
}

/// Per-MCS derived parameters (2 MHz, 1 SS) [Table 23-46, pp3858–3859].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct McsParams {
    pub mcs: u8,
    pub modulation: Modulation,
    pub rate: CodeRate,
    pub n_bpscs: usize,
    /// Coded bits per OFDM symbol.
    pub n_cbps: usize,
    /// Data bits per OFDM symbol.
    pub n_dbps: usize,
}

// Table 23-46, S1G-MCSs for 2 MHz, N_SS = 1. Valid indices: 0–8 and 11
// (9, 10, 12 are "Not valid"; 10 exists only at 1 MHz).
static MCS_TABLE: [McsParams; 10] = [
    McsParams { mcs: 0, modulation: Modulation::Bpsk, rate: CodeRate::R1_2, n_bpscs: 1, n_cbps: 52, n_dbps: 26 },
    McsParams { mcs: 1, modulation: Modulation::Qpsk, rate: CodeRate::R1_2, n_bpscs: 2, n_cbps: 104, n_dbps: 52 },
    McsParams { mcs: 2, modulation: Modulation::Qpsk, rate: CodeRate::R3_4, n_bpscs: 2, n_cbps: 104, n_dbps: 78 },
    McsParams { mcs: 3, modulation: Modulation::Qam16, rate: CodeRate::R1_2, n_bpscs: 4, n_cbps: 208, n_dbps: 104 },
    McsParams { mcs: 4, modulation: Modulation::Qam16, rate: CodeRate::R3_4, n_bpscs: 4, n_cbps: 208, n_dbps: 156 },
    McsParams { mcs: 5, modulation: Modulation::Qam64, rate: CodeRate::R2_3, n_bpscs: 6, n_cbps: 312, n_dbps: 208 },
    McsParams { mcs: 6, modulation: Modulation::Qam64, rate: CodeRate::R3_4, n_bpscs: 6, n_cbps: 312, n_dbps: 234 },
    McsParams { mcs: 7, modulation: Modulation::Qam64, rate: CodeRate::R5_6, n_bpscs: 6, n_cbps: 312, n_dbps: 260 },
    McsParams { mcs: 8, modulation: Modulation::Qam256, rate: CodeRate::R3_4, n_bpscs: 8, n_cbps: 416, n_dbps: 312 },
    McsParams { mcs: 11, modulation: Modulation::Qam1024, rate: CodeRate::R3_4, n_bpscs: 10, n_cbps: 520, n_dbps: 390 },
];

/// Look up MCS parameters. Valid for 2 MHz / 1 SS: 0..=8 and 11.
pub fn mcs_params(mcs: u8) -> Result<&'static McsParams, PhyError> {
    MCS_TABLE.iter().find(|m| m.mcs == mcs).ok_or(PhyError::InvalidMcs(mcs))
}

/// All valid MCS indices for 2 MHz / 1 SS.
pub fn valid_mcs() -> impl Iterator<Item = u8> {
    MCS_TABLE.iter().map(|m| m.mcs)
}

/// PHY characteristics a MAC needs [Table 23-41, 23.4.4, p3856].
pub mod characteristics {
    /// aSIFSTime.
    pub const A_SIFS_TIME_US: u32 = 160;
    /// aSlotTime [23.3.15].
    pub const A_SLOT_TIME_US: u32 = 52;
    /// aCCATime (upper bound; "< 40 µs").
    pub const A_CCA_TIME_US: u32 = 40;
    /// aCCAMidTime.
    pub const A_CCA_MID_TIME_US: u32 = 212;
    /// aRxPHYStartDelay for S1G_SHORT.
    pub const A_RX_PHY_START_DELAY_US: u32 = 280;
    /// aAirPropagationTime.
    pub const A_AIR_PROPAGATION_TIME_US: u32 = 6;
    /// aPPDUMaxTime.
    pub const A_PPDU_MAX_TIME_US: u32 = 27_920;
    /// TXTIME of an NDP_2M CMAC PPDU (STF+LTF1+SIG, no Data): NDPTxTime
    /// for a 2 MHz preamble [10.3.2.5.2; 23.3.11].
    pub const NDP_TX_TIME_US: u32 = 240;
}

/// Receiver RF requirements [23.3.18].
pub mod rf {
    /// CCA channel classification [23.3.18.5.2]: type 2 thresholds are 3 dB
    /// higher (more spatial reuse).
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
    pub enum CcaType {
        #[default]
        Type1,
        Type2,
    }

    /// Energy-detect threshold for the primary 2 MHz channel, dBm (both
    /// types) [23.3.18.5.3.1, Tables 23-37/38].
    pub const ED_THRESHOLD_2MHZ_DBM: f32 = -72.0;

    /// Mid-packet detection threshold (aCCAMidTime window), dBm.
    pub fn mid_packet_threshold_2mhz_dbm(t: CcaType) -> f32 {
        match t {
            CcaType::Type1 => -89.0,
            CcaType::Type2 => -86.0,
        }
    }

    /// Receive level below which CCA is released after a SIG CRC failure:
    /// minimum-MCS sensitivity + 20 dB [23.3.20, Fig 23-53].
    pub const CRC_FAIL_RELEASE_2MHZ_DBM: f32 = -92.0 + 20.0;

    /// RCPI encoding [Table 9-215]: 0 ⇒ P < −109.5 dBm; 1..=219 ⇒
    /// 2·(P + 110); 220 ⇒ P ≥ 0 dBm; 255 ⇒ not available.
    pub fn rcpi_encode(power_dbm: f32) -> u8 {
        if !power_dbm.is_finite() {
            255
        } else if power_dbm < -109.5 {
            0
        } else if power_dbm >= 0.0 {
            220
        } else {
            ((2.0 * (power_dbm + 110.0)).floor() as i32).clamp(1, 219) as u8
        }
    }

}

/// Transmitter conformance limits [23.3.17].
pub mod tx_limits {
    use super::{CodeRate, Modulation};

    /// Allowed relative constellation error (EVM) per modulation/rate, dB
    /// [Table 23-34, p3829]. 1024-QAM: −35 dB with amplitude-drift
    /// compensation in the test instrument, −32 dB without.
    pub fn evm_limit_db(m: Modulation, r: CodeRate) -> f32 {
        match (m, r) {
            (Modulation::Bpsk, _) => -5.0,
            (Modulation::Qpsk, CodeRate::R1_2) => -10.0,
            (Modulation::Qpsk, _) => -13.0,
            (Modulation::Qam16, CodeRate::R1_2) => -16.0,
            (Modulation::Qam16, _) => -19.0,
            (Modulation::Qam64, CodeRate::R2_3) => -22.0,
            (Modulation::Qam64, CodeRate::R3_4) => -25.0,
            (Modulation::Qam64, _) => -27.0,
            (Modulation::Qam256, CodeRate::R3_4) => -30.0,
            (Modulation::Qam256, _) => -32.0,
            (Modulation::Qam1024, _) => -35.0,
        }
    }

    /// 2 MHz interim transmit spectral mask, dBr versus |frequency offset|
    /// in MHz [23.3.17.1, Fig 23-40]: 0 dBr to 0.9 MHz, −20 dBr at 1.1,
    /// −28 dBr at 2, −40 dBr at ≥ 3, linear in dB between the corners.
    pub fn spectral_mask_2mhz_dbr(offset_mhz: f32) -> f32 {
        let f = offset_mhz.abs();
        let seg = |f0: f32, f1: f32, a: f32, b: f32| a + (b - a) * (f - f0) / (f1 - f0);
        if f <= 0.9 {
            0.0
        } else if f <= 1.1 {
            seg(0.9, 1.1, 0.0, -20.0)
        } else if f <= 2.0 {
            seg(1.1, 2.0, -20.0, -28.0)
        } else if f <= 3.0 {
            seg(2.0, 3.0, -28.0, -40.0)
        } else {
            -40.0
        }
    }

    /// Spectral flatness limits [23.3.17.2, Table 23-33]: inner tones
    /// (|k| ≤ 16) within ±4 dB of the inner-tone average; outer tones
    /// (17 ≤ |k| ≤ 28) within +4 / −6 dB.
    pub const FLATNESS_INNER_MAX_K: i32 = 16;
    pub const FLATNESS_INNER_DB: (f32, f32) = (-4.0, 4.0);
    pub const FLATNESS_OUTER_DB: (f32, f32) = (-6.0, 4.0);

    /// Symbol-clock and center-frequency tolerance [23.3.17.3].
    pub const CLOCK_TOLERANCE_PPM: f32 = 20.0;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mcs_table_identities() {
        for m in valid_mcs() {
            let p = mcs_params(m).unwrap();
            assert_eq!(p.n_bpscs, p.modulation.n_bpscs());
            assert_eq!(p.n_cbps, N_SD * p.n_bpscs, "N_CBPS = 52*N_BPSCS for MCS {m}");
            let (num, den) = p.rate.as_fraction();
            assert_eq!(p.n_dbps, p.n_cbps * num / den, "N_DBPS = N_CBPS*R for MCS {m}");
            assert_eq!(p.n_cbps * num % den, 0);
            assert_eq!(n_dbps_2mhz(m, 1), Some(p.n_dbps));
        }
    }

    #[test]
    fn data_rates_match_table() {
        // rate_kbps = N_DBPS * 25 at LGI (T_SYML = 40 µs) [digest pilots-mcs §7].
        let expect = [(0, 650), (1, 1300), (2, 1950), (3, 2600), (4, 3900), (5, 5200), (6, 5850), (7, 6500), (8, 7800), (11, 9750)];
        for (m, kbps) in expect {
            assert_eq!(mcs_params(m).unwrap().n_dbps * 25, kbps);
        }
    }

    #[test]
    fn invalid_mcs_rejected() {
        for m in [9u8, 10, 12, 13, 255] {
            assert!(mcs_params(m).is_err(), "MCS {m} must be invalid");
        }
    }

    #[test]
    fn multi_stream_validity_matches_tables() {
        // Tables 23-47..49: MCS 9 / 12 valid only for N_SS = 3 at 2 MHz.
        assert_eq!(n_dbps_2mhz(9, 1), None);
        assert_eq!(n_dbps_2mhz(9, 2), None);
        assert_eq!(n_dbps_2mhz(9, 3), Some(1040));
        assert_eq!(n_dbps_2mhz(9, 4), None);
        assert_eq!(n_dbps_2mhz(12, 3), Some(1300));
        assert_eq!(n_dbps_2mhz(7, 2), Some(520));
        assert_eq!(n_dbps_2mhz(10, 1), None);
        assert_eq!(n_dbps_2mhz(0, 5), None);
    }

    #[test]
    fn timing_identities() {
        assert_eq!(N_FFT as f64 * DELTA_F_HZ, SAMPLE_RATE_HZ);
        assert_eq!(N_SYM_SAMPLES_LGI, 80);
        assert_eq!(N_PREAMBLE_SAMPLES, 480);
        assert_eq!(T_PREAMBLE_US, 240);
        assert_eq!(N_ST, 56);
        assert_eq!(SIG_N_ST, 52);
        assert_eq!(n_ltf(1), 1);
        assert_eq!(n_ltf(3), 4);
    }

    #[test]
    fn rcpi_codes() {
        assert_eq!(rf::rcpi_encode(-120.0), 0);
        assert_eq!(rf::rcpi_encode(-109.5), 1);
        assert_eq!(rf::rcpi_encode(-60.0), 100);
        assert_eq!(rf::rcpi_encode(5.0), 220);
        assert_eq!(rf::rcpi_encode(f32::NAN), 255);
    }

    #[test]
    fn spectral_mask_corners() {
        use tx_limits::spectral_mask_2mhz_dbr as m;
        assert_eq!(m(0.5), 0.0);
        assert_eq!(m(0.9), 0.0);
        assert!((m(1.0) + 10.0).abs() < 1e-5);
        assert!((m(1.1) + 20.0).abs() < 1e-5);
        assert!((m(2.0) + 28.0).abs() < 1e-5);
        assert!((m(3.0) + 40.0).abs() < 1e-5);
        assert_eq!(m(-5.0), -40.0);
    }
}
