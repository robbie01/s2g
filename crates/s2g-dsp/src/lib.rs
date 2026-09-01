//! Rate conversion between the PHY's native 2 MS/s and the SDR device rate
//! (Pluto runs at 4 MS/s; see ARCHITECTURE.md).
//!
//! Both converters use the same 47-tap Blackman-windowed halfband FIR
//! (≈ −70 dB stopband, exact linear phase). Group delay: 23 samples at the
//! filter's running rate (4 MS/s), i.e. 11.5 input samples for the
//! interpolator and 11.5 output samples for the decimator.

use num_complex::Complex;

pub type Complex32 = Complex<f32>;

const NTAPS: usize = 47;
const CENTER: usize = (NTAPS - 1) / 2; // 23

/// Blackman-windowed halfband lowpass, cutoff 0.25 of the running rate.
fn halfband_taps() -> [f32; NTAPS] {
    let mut h = [0.0f32; NTAPS];
    for (n, t) in h.iter_mut().enumerate() {
        let x = n as f64 - CENTER as f64;
        let sinc = if x == 0.0 {
            0.5
        } else {
            (std::f64::consts::PI * x / 2.0).sin() / (std::f64::consts::PI * x)
        };
        let w = 0.42 - 0.5 * (2.0 * std::f64::consts::PI * n as f64 / (NTAPS - 1) as f64).cos()
            + 0.08 * (4.0 * std::f64::consts::PI * n as f64 / (NTAPS - 1) as f64).cos();
        *t = (sinc * w) as f32;
    }
    // Normalize DC gain to exactly 1.
    let sum: f32 = h.iter().sum();
    for t in &mut h {
        *t /= sum;
    }
    h
}

/// ×2 halfband interpolator (2 → 4 MS/s), streaming.
/// Output length = 2 × input length; overall gain 1 (×2 applied to
/// compensate the zero-stuffing).
pub struct HalfbandInterp2 {
    taps: [f32; NTAPS],
    hist: Vec<Complex32>,
}

impl HalfbandInterp2 {
    pub fn new() -> Self {
        Self { taps: halfband_taps(), hist: vec![Complex32::new(0.0, 0.0); NTAPS] }
    }

    pub fn process(&mut self, input: &[Complex32], out: &mut Vec<Complex32>) {
        // Zero-stuffed convolution at 4 MS/s, evaluated per input sample:
        // for input x[n] arriving, produce y[2n] and y[2n+1].
        for &x in input {
            self.hist.push(x);
            let base = self.hist.len() - 1;
            // In upsampled index space u[2j] = x[j], u[odd] = 0; producing
            // y[m] = 2·Σ_tap h[tap]·u[m−tap] for m = 2·base and 2·base+1.
            // Only taps with tap ≡ m (mod 2) hit nonzero u.
            for phase in 0..2usize {
                let m = 2 * base + phase;
                let mut acc = Complex32::new(0.0, 0.0);
                let mut tap = phase;
                while tap < NTAPS {
                    let j = (m - tap) / 2;
                    acc += self.hist[j] * self.taps[tap];
                    tap += 2;
                }
                out.push(acc * 2.0);
            }
            if self.hist.len() > 4096 {
                let cut = self.hist.len() - NTAPS;
                self.hist.drain(..cut);
            }
        }
    }
}

impl Default for HalfbandInterp2 {
    fn default() -> Self {
        Self::new()
    }
}

/// ÷2 halfband decimator (4 → 2 MS/s), streaming. Output length =
/// input length / 2 (odd leftovers carried across calls).
pub struct HalfbandDecim2 {
    taps: [f32; NTAPS],
    hist: Vec<Complex32>,
    /// Parity of the next input sample in the running stream.
    parity: usize,
}

impl HalfbandDecim2 {
    pub fn new() -> Self {
        Self { taps: halfband_taps(), hist: vec![Complex32::new(0.0, 0.0); NTAPS], parity: 0 }
    }

    pub fn process(&mut self, input: &[Complex32], out: &mut Vec<Complex32>) {
        for &x in input {
            self.hist.push(x);
            self.parity ^= 1;
            // Emit on even stream indices (first, third, … pushes) so a
            // 2→4→2 cascade lands on the interpolator's original-sample
            // phase (integer, not half-sample, group delay).
            if self.parity == 1 {
                // Emit y = Σ h[k]·v[end−k].
                let end = self.hist.len();
                let mut acc = Complex32::new(0.0, 0.0);
                for (k, &t) in self.taps.iter().enumerate() {
                    if t != 0.0 && k < end {
                        acc += self.hist[end - 1 - k] * t;
                    }
                }
                out.push(acc);
            }
            if self.hist.len() > 4096 {
                let cut = self.hist.len() - NTAPS;
                self.hist.drain(..cut);
            }
        }
    }
}

impl Default for HalfbandDecim2 {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rustfft::FftPlanner;

    fn tone(n: usize, f_rel: f32) -> Vec<Complex32> {
        (0..n)
            .map(|i| Complex32::from_polar(1.0, 2.0 * std::f32::consts::PI * f_rel * i as f32))
            .collect()
    }

    fn spectrum(x: &[Complex32]) -> Vec<f32> {
        let mut buf: Vec<Complex32> = x.to_vec();
        FftPlanner::new().plan_fft_forward(buf.len()).process(&mut buf);
        buf.iter().map(|v| v.norm()).collect()
    }

    #[test]
    fn taps_symmetric_halfband() {
        let h = halfband_taps();
        for k in 0..NTAPS {
            assert!((h[k] - h[NTAPS - 1 - k]).abs() < 1e-7, "symmetry at {k}");
        }
        // Halfband property: odd offsets from center are zero except center.
        for k in (2..=CENTER).step_by(2) {
            assert!(h[CENTER - k].abs() < 1e-7, "halfband zero at −{k}");
        }
        assert!((h[CENTER] - 0.5).abs() < 0.01);
    }

    #[test]
    fn interp_passband_tone() {
        // 0.1 of 2 MS/s tone → interpolated tone at 0.05 of 4 MS/s, ~unit amp.
        // f chosen so the analysis FFT hits an integer bin (no scalloping):
        // input 102/1024 of 2 MS/s → output bin 102 of a 2048-point FFT.
        let f_in = 102.0 / 1024.0;
        let mut it = HalfbandInterp2::new();
        let mut out = Vec::new();
        it.process(&tone(2048, f_in), &mut out);
        assert_eq!(out.len(), 4096);
        let s = spectrum(&out[256..256 + 2048]);
        let peak_bin = 102usize;
        let peak = s[peak_bin - 1..=peak_bin + 1].iter().cloned().fold(0.0f32, f32::max);
        assert!((peak / 2048.0 - 1.0).abs() < 0.02, "passband gain {}", peak / 2048.0);
        // Image at 0.5−0.05 = 0.45 of 4 MS/s suppressed ≥ 55 dB.
        let img_bin = (0.45 * 2048.0) as usize;
        let img = s[img_bin - 2..=img_bin + 2].iter().cloned().fold(0.0f32, f32::max);
        assert!(20.0 * (img / peak).log10() < -55.0, "image {} dB", 20.0 * (img / peak).log10());
    }

    #[test]
    fn decim_alias_rejection() {
        // A tone at 0.4 of 4 MS/s (stopband) must be strongly attenuated.
        let mut dc = HalfbandDecim2::new();
        let mut out = Vec::new();
        dc.process(&tone(4096, 0.4), &mut out);
        let p: f32 = out[128..].iter().map(|v| v.norm_sqr()).sum::<f32>() / (out.len() - 128) as f32;
        assert!(10.0 * p.log10() < -55.0, "stopband power {} dB", 10.0 * p.log10());
        // And a passband tone at 0.1 passes with unit gain.
        let mut dc2 = HalfbandDecim2::new();
        let mut out2 = Vec::new();
        dc2.process(&tone(4096, 0.1), &mut out2);
        let p2: f32 = out2[128..].iter().map(|v| v.norm_sqr()).sum::<f32>() / (out2.len() - 128) as f32;
        assert!((10.0 * p2.log10()).abs() < 0.2, "passband {} dB", 10.0 * p2.log10());
    }

    #[test]
    fn roundtrip_2_4_2() {
        // 2→4→2 MS/s roundtrip preserves a 0.9 MHz-equivalent tone (0.45 of
        // 2 MS/s is beyond cutoff; use 0.2 = 400 kHz which is in-band).
        let x = tone(4096, 0.2);
        let mut it = HalfbandInterp2::new();
        let mut up = Vec::new();
        it.process(&x, &mut up);
        let mut dc = HalfbandDecim2::new();
        let mut down = Vec::new();
        dc.process(&up, &mut down);
        // Compare against the input, allowing for the combined group delay
        // (23 samples at 4 MS/s per filter → 23 samples at 2 MS/s total).
        let delay = 23usize;
        let mut err = 0.0f32;
        let mut sig = 0.0f32;
        for i in 200..3800 {
            err += (down[i + delay] - x[i]).norm_sqr();
            sig += x[i].norm_sqr();
        }
        assert!(10.0 * (err / sig).log10() < -40.0, "roundtrip error {} dB", 10.0 * (err / sig).log10());
    }

    #[test]
    fn chunked_equals_oneshot() {
        let x = tone(1000, 0.13);
        let mut a = Vec::new();
        HalfbandInterp2::new().process(&x, &mut a);
        let mut b = Vec::new();
        let mut it = HalfbandInterp2::new();
        for c in x.chunks(7) {
            it.process(c, &mut b);
        }
        assert_eq!(a.len(), b.len());
        for (u, v) in a.iter().zip(&b) {
            assert!((u - v).norm() < 1e-6);
        }
        let mut d1 = Vec::new();
        HalfbandDecim2::new().process(&a, &mut d1);
        let mut d2 = Vec::new();
        let mut dc = HalfbandDecim2::new();
        for c in a.chunks(13) {
            dc.process(c, &mut d2);
        }
        assert_eq!(d1.len(), d2.len());
        for (u, v) in d1.iter().zip(&d2) {
            assert!((u - v).norm() < 1e-6);
        }
    }
}

pub mod resample;
pub use resample::{apply_sfo_ppm, resample};
