//! S1G_SHORT preamble fields for 2 MHz, 1 STS [23.3.8.2.2; digest
//! preamble.md]. STF = 20 MHz L-STF pattern (Eq 19-8) scaled 1/√12;
//! LTF = VHT-LTF sequence (Eq 21-36) scaled 1/√56.
//!
//! Time-domain output convention: 2 MS/s, unit average power per field,
//! rectangular windowing (see `crate::ofdm`).

use crate::ofdm::{self, FreqSymbol};
use crate::params::{N_LTF1_SAMPLES, N_STF_SAMPLES, N_TONE_LTF, N_TONE_STF};
use crate::Complex32;

/// Nonzero STF tones: (subcarrier, sign of (1+j)·√½) [Eq 19-8].
const STF_TONES: [(i32, f32); 12] = [
    (-24, 1.0),
    (-20, -1.0),
    (-16, 1.0),
    (-12, -1.0),
    (-8, -1.0),
    (-4, 1.0),
    (4, -1.0),
    (8, -1.0),
    (12, 1.0),
    (16, 1.0),
    (20, 1.0),
    (24, 1.0),
];

/// LTF sequence values for k = −28..+28 (57 entries incl. DC=0) [Eq 21-36].
pub const LTF_SEQ: [i8; 57] = [
    1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1, 1, 1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1, 1, 1, 1, // −28..−1
    0, // DC
    1, -1, -1, 1, 1, -1, 1, -1, 1, -1, -1, -1, -1, -1, 1, 1, -1, -1, 1, -1, 1, -1, 1, 1, 1, 1, -1, -1, // +1..+28
];

/// STF frequency-domain sequence in logical bin order.
pub fn stf_freq() -> FreqSymbol {
    let mut sym = [Complex32::new(0.0, 0.0); 64];
    let a = (0.5f32).sqrt();
    for &(k, s) in &STF_TONES {
        sym[ofdm::bin(k)] = Complex32::new(s * a, s * a);
    }
    sym
}

/// LTF frequency-domain sequence in logical bin order.
pub fn ltf_freq() -> FreqSymbol {
    let mut sym = [Complex32::new(0.0, 0.0); 64];
    for (i, &v) in LTF_SEQ.iter().enumerate() {
        let k = i as i32 - 28;
        sym[ofdm::bin(k)] = Complex32::new(v as f32, 0.0);
    }
    sym
}

/// One 16-sample period of the time-domain STF (the waveform is periodic
/// because every occupied tone index is a multiple of 4).
pub fn stf_period() -> [Complex32; 16] {
    let x = ofdm::idft(&stf_freq(), 1.0 / (N_TONE_STF as f32).sqrt());
    core::array::from_fn(|i| x[i])
}

/// Full STF field: 160 samples = 10 periods [T_STF = 80 µs].
pub fn stf_time() -> Vec<Complex32> {
    let p = stf_period();
    let mut out = Vec::with_capacity(N_STF_SAMPLES);
    for _ in 0..N_STF_SAMPLES / 16 {
        out.extend_from_slice(&p);
    }
    out
}

/// One 64-sample period of the time-domain long training symbol.
pub fn ltf_period() -> [Complex32; 64] {
    ofdm::idft(&ltf_freq(), 1.0 / (N_TONE_LTF as f32).sqrt())
}

/// Full LTF1 field: [GI2 = last 32 samples][LTS 64][LTS 64] = 160 samples.
pub fn ltf1_time() -> Vec<Complex32> {
    let x = ltf_period();
    let mut out = Vec::with_capacity(N_LTF1_SAMPLES);
    out.extend_from_slice(&x[32..]);
    out.extend_from_slice(&x);
    out.extend_from_slice(&x);
    out
}

/// D-STF of the S1G_LONG beam-changeable portion: the same STF sequence
/// over one 40 µs symbol = 5 periods, 80 samples [23.3.8.2.3.3.3].
pub fn dstf_time() -> Vec<Complex32> {
    let p = stf_period();
    let mut out = Vec::with_capacity(80);
    for _ in 0..5 {
        out.extend_from_slice(&p);
    }
    out
}

/// One D-LTF symbol: [GI 16 = last 16 samples of the LTS][LTS 64], 80
/// samples [23.3.8.2.3.3.4, Eq 23-27]. For an SU PPDU the SIG-B symbol is
/// an identical copy [23.3.8.2.3.3.5].
pub fn dltf_time() -> Vec<Complex32> {
    let x = ltf_period();
    let mut out = Vec::with_capacity(80);
    out.extend_from_slice(&x[48..]);
    out.extend_from_slice(&x);
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn beam_changeable_fields() {
        let dstf = dstf_time();
        let dltf = dltf_time();
        assert_eq!(dstf.len(), 80);
        assert_eq!(dltf.len(), 80);
        assert_eq!(&dstf[..64], &stf_time()[..64]);
        let x = ltf_period();
        for i in 0..16 {
            assert!((dltf[i] - x[48 + i]).norm() < 1e-6);
        }
        assert_eq!(&dltf[16..], &x[..]);
    }

    #[test]
    fn field_lengths_and_power() {
        let stf = stf_time();
        let ltf = ltf1_time();
        assert_eq!(stf.len(), 160);
        assert_eq!(ltf.len(), 160);
        let p_stf: f32 = stf.iter().map(|v| v.norm_sqr()).sum::<f32>() / 160.0;
        let p_ltf: f32 = ltf.iter().map(|v| v.norm_sqr()).sum::<f32>() / 160.0;
        assert!((p_stf - 1.0).abs() < 1e-3, "STF power {p_stf}");
        assert!((p_ltf - 1.0).abs() < 1e-3, "LTF power {p_ltf}");
    }

    #[test]
    fn stf_periodicity() {
        let stf = stf_time();
        for i in 0..stf.len() - 16 {
            assert!((stf[i] - stf[i + 16]).norm() < 1e-5);
        }
    }

    #[test]
    fn ltf1_structure() {
        let ltf = ltf1_time();
        let x = ltf_period();
        // GI2 = last 32 samples of the period.
        for i in 0..32 {
            assert!((ltf[i] - x[32 + i]).norm() < 1e-6);
        }
        // Two identical periods.
        for i in 0..64 {
            assert!((ltf[32 + i] - ltf[96 + i]).norm() < 1e-6);
        }
    }

    #[test]
    fn tone_counts() {
        let nz_stf = stf_freq().iter().filter(|v| v.norm() > 0.0).count();
        let nz_ltf = ltf_freq().iter().filter(|v| v.norm() > 0.0).count();
        assert_eq!(nz_stf, 12);
        assert_eq!(nz_ltf, 56);
        // Pilot-position values of the LTF sequence [digest preamble §3.1]:
        // {−21: +1, −7: −1, +7: +1, +21: +1}.
        let f = ltf_freq();
        assert_eq!(f[ofdm::bin(-21)].re, 1.0);
        assert_eq!(f[ofdm::bin(-7)].re, -1.0);
        assert_eq!(f[ofdm::bin(7)].re, 1.0);
        assert_eq!(f[ofdm::bin(21)].re, 1.0);
    }
}
