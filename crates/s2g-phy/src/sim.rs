//! Deterministic simulation helpers shared by tests and tools: a
//! splitmix64 generator with uniform and Gaussian draws, and channel
//! impairments (AWGN, carrier offset, fractional delay, sampling-clock
//! offset, echo) at the native 2 MS/s.

use crate::params::SAMPLE_RATE_HZ;
use crate::Complex32;

/// splitmix64. A bare LCG has lattice correlation at the STF
/// autocorrelation lag and falsely triggers the detector; this does not.
#[derive(Debug, Clone)]
pub struct Rng(pub u64);

impl Rng {
    pub fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Uniform in [0, 1).
    pub fn uniform(&mut self) -> f32 {
        (self.next_u64() >> 40) as f32 / (1u64 << 24) as f32
    }

    /// Standard normal (sum of 12 uniforms).
    pub fn gauss(&mut self) -> f32 {
        (0..12).map(|_| self.uniform()).sum::<f32>() - 6.0
    }

    pub fn bytes(&mut self, n: usize) -> Vec<u8> {
        (0..n).map(|_| (self.next_u64() >> 40) as u8).collect()
    }
}

/// `n` samples of complex white Gaussian noise with total variance
/// `sigma_sq` per sample.
pub fn noise(n: usize, sigma_sq: f32, rng: &mut Rng) -> Vec<Complex32> {
    let s = (sigma_sq / 2.0).sqrt();
    (0..n).map(|_| Complex32::new(rng.gauss() * s, rng.gauss() * s)).collect()
}

/// `sig` plus white Gaussian noise at `snr_db` relative to the mean power
/// of `sig`.
pub fn awgn(sig: &[Complex32], snr_db: f32, rng: &mut Rng) -> Vec<Complex32> {
    let p: f32 = sig.iter().map(|v| v.norm_sqr()).sum::<f32>() / sig.len().max(1) as f32;
    let s = (p / 10f32.powf(snr_db / 10.0) / 2.0).sqrt();
    sig.iter().map(|&v| v + Complex32::new(rng.gauss() * s, rng.gauss() * s)).collect()
}

/// `sig` rotated by `cfo_hz`, phase zero at index 0.
pub fn apply_cfo(sig: &[Complex32], cfo_hz: f32) -> Vec<Complex32> {
    let w = 2.0 * std::f64::consts::PI * cfo_hz as f64 / SAMPLE_RATE_HZ;
    sig.iter().enumerate().map(|(i, &v)| v * Complex32::from_polar(1.0, (w * i as f64) as f32)).collect()
}

/// Fractional delay of `mu` samples (0..1) by linear interpolation; one
/// sample shorter than the input.
pub fn frac_delay(sig: &[Complex32], mu: f32) -> Vec<Complex32> {
    (0..sig.len().saturating_sub(1)).map(|i| sig[i] * (1.0 - mu) + sig[i + 1] * mu).collect()
}

/// `sig` plus a copy delayed by `delay` samples with complex `gain`.
pub fn echo(sig: &[Complex32], delay: usize, gain: Complex32) -> Vec<Complex32> {
    (0..sig.len()).map(|i| sig[i] + if i >= delay { sig[i - delay] * gain } else { Complex32::new(0.0, 0.0) }).collect()
}

/// Channel impairments, applied in the order amplitude, echo,
/// sampling-clock offset, carrier offset, fractional delay, noise.
#[derive(Debug, Clone, Copy)]
pub struct Impairments {
    /// SNR relative to the mean power of the impaired signal, dB.
    pub snr_db: Option<f32>,
    pub cfo_hz: f32,
    /// Fractional-sample delay 0..1.
    pub frac_delay: f32,
    pub amplitude: f32,
    /// Sampling-clock offset between transmitter and receiver, ppm
    /// (positive: receiver clock fast).
    pub sfo_ppm: f64,
    /// Static echo: (delay in samples, complex gain).
    pub echo: Option<(usize, Complex32)>,
}

impl Default for Impairments {
    fn default() -> Self {
        Self { snr_db: None, cfo_hz: 0.0, frac_delay: 0.0, amplitude: 1.0, sfo_ppm: 0.0, echo: None }
    }
}

impl Impairments {
    pub fn apply(&self, sig: &[Complex32], rng: &mut Rng) -> Vec<Complex32> {
        let mut v: Vec<Complex32> = sig.iter().map(|&s| s * self.amplitude).collect();
        if let Some((delay, gain)) = self.echo {
            v = echo(&v, delay, gain);
        }
        if self.sfo_ppm != 0.0 {
            v = s2g_dsp::apply_sfo_ppm(&v, self.sfo_ppm);
        }
        if self.cfo_hz != 0.0 {
            v = apply_cfo(&v, self.cfo_hz);
        }
        if self.frac_delay > 0.0 {
            v = frac_delay(&v, self.frac_delay);
        }
        if let Some(snr) = self.snr_db {
            v = awgn(&v, snr, rng);
        }
        v
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gauss_is_standard_normal() {
        let mut rng = Rng(1);
        let n = 20_000;
        let (mut sum, mut sq) = (0.0f64, 0.0f64);
        for _ in 0..n {
            let g = rng.gauss() as f64;
            sum += g;
            sq += g * g;
        }
        let mean = sum / n as f64;
        let var = sq / n as f64 - mean * mean;
        assert!(mean.abs() < 0.03, "mean {mean}");
        assert!((var - 1.0).abs() < 0.05, "variance {var}");
    }

    #[test]
    fn awgn_hits_the_requested_snr() {
        let mut rng = Rng(2);
        let sig = vec![Complex32::new(1.0, 0.0); 20_000];
        let noisy = awgn(&sig, 10.0, &mut rng);
        let np: f32 = noisy.iter().zip(&sig).map(|(a, b)| (a - b).norm_sqr()).sum::<f32>() / sig.len() as f32;
        let snr = -10.0 * np.log10();
        assert!((snr - 10.0).abs() < 0.3, "snr {snr}");
    }
}
