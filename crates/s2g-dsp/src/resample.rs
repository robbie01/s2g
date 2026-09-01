//! Arbitrary-ratio resampling by windowed-sinc interpolation. Used to
//! simulate sample-clock offset (SFO) between transmitter and receiver: a
//! receiver whose ADC clock is `ppm` parts-per-million fast sees the
//! transmitted waveform stretched by that ratio.

use num_complex::Complex;

type C32 = Complex<f32>;

/// Half-width of the interpolation kernel in input samples.
const HALF_TAPS: usize = 16;

fn kernel(frac: f64) -> [f32; 2 * HALF_TAPS] {
    // Hann-windowed sinc, cutoff at Nyquist, centered at `frac` in
    // (−HALF_TAPS, HALF_TAPS].
    let mut k = [0.0f32; 2 * HALF_TAPS];
    let mut sum = 0.0f64;
    for (i, slot) in k.iter_mut().enumerate() {
        let t = i as f64 - (HALF_TAPS as f64 - 1.0) - frac;
        let x = std::f64::consts::PI * t;
        let sinc = if x.abs() < 1e-9 { 1.0 } else { x.sin() / x };
        let w = 0.5 * (1.0 + (std::f64::consts::PI * t / HALF_TAPS as f64).cos());
        let v = sinc * w;
        *slot = v as f32;
        sum += v;
    }
    for v in &mut k {
        *v /= sum as f32;
    }
    k
}

/// Resample `sig` so that output sample n is the input evaluated at time
/// n·`step` (in input samples). `step` = 1 / (1 + ppm·1e−6) models a receiver
/// clock that runs `ppm` fast (more output samples than input).
pub fn resample(sig: &[C32], step: f64) -> Vec<C32> {
    assert!(step > 0.0);
    let n_out = ((sig.len() as f64 - 1.0) / step).floor() as usize + 1;
    let mut out = Vec::with_capacity(n_out);
    for n in 0..n_out {
        let t = n as f64 * step;
        let i0 = t.floor() as i64;
        let frac = t - i0 as f64;
        let k = kernel(frac);
        let mut acc = C32::new(0.0, 0.0);
        for (j, &w) in k.iter().enumerate() {
            let idx = i0 - (HALF_TAPS as i64 - 1) + j as i64;
            if idx >= 0 && (idx as usize) < sig.len() {
                acc += sig[idx as usize] * w;
            }
        }
        out.push(acc);
    }
    out
}

/// Apply a sample-clock offset of `ppm` parts per million (receiver fast
/// for positive `ppm`).
pub fn apply_sfo_ppm(sig: &[C32], ppm: f64) -> Vec<C32> {
    resample(sig, 1.0 / (1.0 + ppm * 1e-6))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identity_step_reproduces_input() {
        let sig: Vec<C32> = (0..200).map(|i| C32::from_polar(1.0, 0.1 * i as f32)).collect();
        let out = resample(&sig, 1.0);
        assert_eq!(out.len(), sig.len());
        for i in HALF_TAPS..sig.len() - HALF_TAPS {
            assert!((out[i] - sig[i]).norm() < 1e-5, "{i}");
        }
    }

    #[test]
    fn stretches_a_tone_correctly() {
        // A tone at 0.3 cycles/sample resampled with step 0.999 must appear
        // at 0.3·0.999 cycles/sample: check phase progression.
        let f = 0.3f64;
        let sig: Vec<C32> = (0..4000)
            .map(|i| C32::from_polar(1.0, (2.0 * std::f64::consts::PI * f * i as f64) as f32))
            .collect();
        let step = 0.999;
        let out = resample(&sig, step);
        let expect_len = ((sig.len() as f64 - 1.0) / step).floor() as usize + 1;
        assert_eq!(out.len(), expect_len);
        for (n, &o) in out.iter().enumerate().take(out.len() - 100).skip(100) {
            let expect = C32::from_polar(1.0, (2.0 * std::f64::consts::PI * f * n as f64 * step) as f32);
            assert!((o - expect).norm() < 2e-3, "n {n}: {o} vs {expect}");
        }
    }

    #[test]
    fn sfo_changes_length_by_ppm() {
        let sig = vec![C32::new(1.0, 0.0); 100_000];
        let out = apply_sfo_ppm(&sig, 40.0);
        assert!((out.len() as i64 - 100_004).abs() <= 1, "{}", out.len());
    }
}
