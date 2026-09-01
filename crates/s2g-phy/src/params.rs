//! S1G PHY constants and MCS tables for 2 MHz / 1 spatial stream.
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

/// Per-field tone-scaling counts N_Tone [Table 23-8, pp3764–3765].
pub const N_TONE_STF: usize = 12;
pub const N_TONE_LTF: usize = 56;
pub const N_TONE_SIG: usize = 52;
pub const N_TONE_DATA: usize = 56;

/// SERVICE field bits (S1G is 8, not 16!) [Table 23-5; 23.3.9.2].
pub const N_SERVICE: usize = 8;
/// BCC tail bits [Table 23-5].
pub const N_TAIL: usize = 6;

/// Max PSDU octets without aggregation (9-bit SIG Length) [Table 23-41].
pub const PSDU_MAX_NO_AGG: usize = 511;
/// Max Data-field symbol count (9-bit SIG Length, aggregated) [23.4.3].
pub const N_SYM_MAX: usize = 511;

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
    fn timing_identities() {
        assert_eq!(N_FFT as f64 * DELTA_F_HZ, SAMPLE_RATE_HZ);
        assert_eq!(N_SYM_SAMPLES_LGI, 80);
        assert_eq!(N_PREAMBLE_SAMPLES, 480);
        assert_eq!(N_ST, 56);
        assert_eq!(SIG_N_ST, 52);
    }
}
