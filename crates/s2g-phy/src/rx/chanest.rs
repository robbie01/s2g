//! Channel estimation, one-tap equalization, and pilot-based tracking.
//!
//! H is least-squares estimated from the two repeated LTS periods of LTF1
//! against `preamble::ltf_freq()`; the noise variance comes from their
//! difference. Per-symbol common phase error (CPE) and residual linear phase
//! slope (timing offset / sampling-clock drift) are estimated from the 4
//! pilots (fixed or traveling) and removed before the data tones are
//! handed to the demapper. CSI weight per tone is |H|²/σ² (for LLR
//! scaling). With traveling pilots the estimate at each visited tone is
//! refreshed, so a slowly varying channel is tracked through the PPDU.

use crate::ofdm::{self, FreqSymbol};
use crate::preamble;
use crate::Complex32;

/// Per-subcarrier channel estimate over the 64 logical bins.
#[derive(Debug, Clone)]
pub struct ChannelEstimate {
    pub h: [Complex32; 64],
    /// Per-tone noise variance in the FFT domain.
    pub noise_var: f32,
    /// Mean received signal power per used tone (FFT domain).
    pub signal_power: f32,
}

impl ChannelEstimate {
    /// SNR estimate in dB (mean signal power over mean noise power).
    pub fn snr_db(&self) -> f32 {
        10.0 * (self.signal_power / self.noise_var.max(1e-12)).log10()
    }

    /// Average two independent estimates of the same channel (e.g. LTF1
    /// and the D-LTF/SIG-B pair of an S1G_LONG SU PPDU whose SIG-A says the
    /// beam did not change): halves the estimation noise.
    pub fn merge(&self, other: &ChannelEstimate) -> ChannelEstimate {
        let mut h = self.h;
        for (a, o) in h.iter_mut().zip(&other.h) {
            *a = (*a + *o) * 0.5;
        }
        ChannelEstimate {
            h,
            noise_var: 0.5 * (self.noise_var + other.noise_var),
            signal_power: 0.5 * (self.signal_power + other.signal_power),
        }
    }

    /// Normalized correlation of two estimates over the used tones (1 =
    /// identical channel up to a common scale).
    pub fn similarity(&self, other: &ChannelEstimate) -> f32 {
        let ltf = preamble::ltf_freq();
        let (mut x, mut p1, mut p2) = (Complex32::new(0.0, 0.0), 0.0f32, 0.0f32);
        for a in (0..64).filter(|&a| ltf[a].norm_sqr() > 0.0) {
            x += self.h[a] * other.h[a].conj();
            p1 += self.h[a].norm_sqr();
            p2 += other.h[a].norm_sqr();
        }
        x.norm() / (p1 * p2).sqrt().max(1e-12)
    }

    /// RXVECTOR SNR [Table 23-1]: the mean over the used tones of the
    /// per-tone SNR in dB.
    pub fn mean_tone_snr_db(&self) -> f32 {
        let ltf = preamble::ltf_freq();
        let nv = self.noise_var.max(1e-12);
        let (sum, n) = (0..64)
            .filter(|&a| ltf[a].norm_sqr() > 0.0)
            .fold((0.0f32, 0usize), |(s, n), a| (s + 10.0 * (self.h[a].norm_sqr() / nv).max(1e-12).log10(), n + 1));
        sum / n.max(1) as f32
    }
}

/// Estimate the channel from the two repeated LTS periods (freq domain,
/// from `ofdm::fft_symbol`).
pub fn estimate(lts1: &FreqSymbol, lts2: &FreqSymbol) -> ChannelEstimate {
    let ltf = preamble::ltf_freq();
    let mut h = [Complex32::new(0.0, 0.0); 64];
    let mut noise = 0.0f32;
    let mut sig = 0.0f32;
    let mut used = 0usize;
    for a in 0..64 {
        let r = ltf[a];
        if r.norm_sqr() > 0.0 {
            let avg = (lts1[a] + lts2[a]) * 0.5;
            // The reference has unit magnitude (±1): dividing by r is exact.
            h[a] = avg / r;
            let d = lts1[a] - lts2[a];
            // Var of (n1−n2) is 2σ²; the averaged estimate sees σ²/2, but
            // the reported value is the per-tone, per-symbol σ².
            noise += d.norm_sqr() * 0.5;
            sig += avg.norm_sqr();
            used += 1;
        }
    }
    ChannelEstimate { h, noise_var: noise / used as f32, signal_power: sig / used as f32 }
}

/// Mean power per used tone (±1..±28) of an FFT-domain symbol, compared
/// with `ChannelEstimate::signal_power` to detect loss of carrier.
pub fn used_tone_power(sym: &FreqSymbol) -> f32 {
    let mut p = 0.0f32;
    for k in -28..=28i32 {
        if k != 0 {
            p += sym[ofdm::bin(k)].norm_sqr();
        }
    }
    p / 56.0
}

/// Raw pilot measurement for one symbol.
#[derive(Debug, Clone, Copy)]
pub struct PilotMeasurement {
    /// Common phase error, radians.
    pub cpe: f32,
    /// Linear phase slope, radians per subcarrier (see
    /// [`EqualizedSymbol::slope`]).
    pub slope: f32,
    /// Coherence of the 4 pilots after CPE/slope removal (1 = perfect).
    pub quality: f32,
}

impl PilotMeasurement {
    /// Timing offset implied by `slope`, in samples (positive = signal late
    /// relative to the FFT window).
    pub fn timing_offset_samples(&self) -> f32 {
        -self.slope * 64.0 / (2.0 * core::f32::consts::PI)
    }
}

/// Phase slope (radians per subcarrier) equivalent to a timing offset of
/// `samples`.
pub fn slope_for_timing_offset(samples: f32) -> f32 {
    -samples * 2.0 * core::f32::consts::PI / 64.0
}

/// Equalized output for one OFDM symbol.
pub struct EqualizedSymbol {
    /// Equalized data tones in the order of the `indices` passed in.
    pub data: Vec<Complex32>,
    /// Per-tone CSI weight (|H|²/σ²) for LLR scaling (same order).
    pub csi: Vec<f32>,
    /// Common phase error removed from this symbol (radians).
    pub cpe: f32,
    /// Residual linear phase slope removed, radians per subcarrier. A
    /// signal arriving `d` samples later than the FFT window assumes shows
    /// slope −2π·d/64.
    pub slope: f32,
    /// Coherence of the 4 pilots after CPE removal (1 = perfect).
    pub pilot_quality: f32,
}

impl EqualizedSymbol {
    /// Timing offset implied by `slope`, in samples (positive = signal late
    /// relative to the FFT window).
    pub fn timing_offset_samples(&self) -> f32 {
        -self.slope * 64.0 / (2.0 * core::f32::consts::PI)
    }
}

pub struct Equalizer {
    est: ChannelEstimate,
    inv_h: [Complex32; 64],
    csi_all: [f32; 64],
}

fn wrap(mut r: f32) -> f32 {
    while r > core::f32::consts::PI {
        r -= 2.0 * core::f32::consts::PI;
    }
    while r < -core::f32::consts::PI {
        r += 2.0 * core::f32::consts::PI;
    }
    r
}

impl Equalizer {
    pub fn new(est: ChannelEstimate) -> Self {
        let mut eq = Self { est, inv_h: [Complex32::new(0.0, 0.0); 64], csi_all: [0.0; 64] };
        for a in 0..64 {
            eq.refresh_bin(a);
        }
        eq
    }

    fn refresh_bin(&mut self, a: usize) {
        let nv = self.est.noise_var.max(1e-12);
        let hp = self.est.h[a].norm_sqr();
        if hp > 1e-12 {
            self.inv_h[a] = self.est.h[a].conj() / hp;
            self.csi_all[a] = hp / nv;
        } else {
            self.inv_h[a] = Complex32::new(0.0, 0.0);
            self.csi_all[a] = 0.0;
        }
    }

    pub fn estimate(&self) -> &ChannelEstimate {
        &self.est
    }

    /// Equalize one symbol: extract `indices` tones, remove CPE and linear
    /// phase slope estimated from the pilots at `pilot_positions` (expected
    /// values in the same order). Uses the raw per-symbol pilot measurement;
    /// the Data-field path smooths measurements across symbols via
    /// [`Equalizer::measure_pilots`] + [`Equalizer::apply`].
    pub fn equalize(
        &self,
        sym: &FreqSymbol,
        indices: &[i32],
        pilot_positions: &[i32; 4],
        expected_pilots: &[Complex32; 4],
    ) -> EqualizedSymbol {
        let m = self.measure_pilots(sym, pilot_positions, expected_pilots, None);
        let mut e = self.apply(sym, indices, m.cpe, m.slope);
        e.pilot_quality = m.quality;
        e
    }

    /// Smooth the channel estimate across adjacent used tones with a
    /// [1 2 1]/4 kernel (the "Smoothing recommended" SIG bit means the
    /// channel is benign enough for this): a little bias in
    /// frequency-selective channels for ~2 dB less estimation noise.
    /// `known_slope` is a linear phase (radians per subcarrier) that the
    /// estimate is known to contain, e.g. from the FFT window backoff; it is
    /// removed before averaging so it does not bias the result.
    pub fn smooth(&mut self, known_slope: f32) {
        let ltf = preamble::ltf_freq();
        let used: Vec<usize> = (0..64).filter(|&a| ltf[a].norm_sqr() > 0.0).collect();
        let derot = |a: usize| self.est.h[a] * Complex32::from_polar(1.0, -known_slope * (a as f32 - 32.0));
        let flat: Vec<Complex32> = (0..64).map(derot).collect();
        for (i, &a) in used.iter().enumerate() {
            let prev = if i > 0 && used[i - 1] + 1 == a { Some(used[i - 1]) } else { None };
            let next = if i + 1 < used.len() && used[i + 1] == a + 1 { Some(used[i + 1]) } else { None };
            let (mut acc, mut w) = (flat[a] * 2.0, 2.0f32);
            if let Some(p) = prev {
                acc += flat[p];
                w += 1.0;
            }
            if let Some(n) = next {
                acc += flat[n];
                w += 1.0;
            }
            self.est.h[a] = acc / w * Complex32::from_polar(1.0, known_slope * (a as f32 - 32.0));
        }
        for a in used {
            self.refresh_bin(a);
        }
    }

    /// Equalize `indices` with an externally supplied CPE and slope (e.g.
    /// smoothed across symbols).
    pub fn apply(&self, sym: &FreqSymbol, indices: &[i32], cpe: f32, slope: f32) -> EqualizedSymbol {
        let mut data = Vec::with_capacity(indices.len());
        let mut csi = Vec::with_capacity(indices.len());
        for &k in indices {
            let a = ofdm::bin(k);
            let corr = Complex32::from_polar(1.0, -(cpe + slope * k as f32));
            data.push(sym[a] * self.inv_h[a] * corr);
            csi.push(self.csi_all[a]);
        }
        EqualizedSymbol { data, csi, cpe, slope, pilot_quality: 1.0 }
    }

    /// Measure common phase error and linear phase slope from the four
    /// pilots of one symbol. With a `slope_hint` (radians per subcarrier,
    /// e.g. the tracked timing drift) the expected slope is removed first
    /// and the pilot phase differences are unwrapped around it, which is
    /// reliable at any SNR while the residual stays below 0.76 samples.
    /// Without a hint the short-baseline pilot pair resolves the wrap of
    /// the long-baseline pair (unambiguous to ±2.3 samples, but the short
    /// pair is noisy at low SNR).
    pub fn measure_pilots(
        &self,
        sym: &FreqSymbol,
        pilot_positions: &[i32; 4],
        expected_pilots: &[Complex32; 4],
        slope_hint: Option<f32>,
    ) -> PilotMeasurement {
        let hint = slope_hint.unwrap_or(0.0);
        // Per-pilot rotation after equalization, reference removal and
        // hint removal, weighted by |H|².
        let mut rot = [Complex32::new(0.0, 0.0); 4];
        let mut weights = [0.0f32; 4];
        for (l, &k) in pilot_positions.iter().enumerate() {
            let a = ofdm::bin(k);
            rot[l] = sym[a] * self.inv_h[a] * expected_pilots[l].conj() * Complex32::from_polar(1.0, -hint * k as f32);
            weights[l] = self.est.h[a].norm_sqr();
        }
        // Residual slope from the two pilot pairs (independent of the
        // CPE), weighted least squares through the origin. Positions are
        // ascending, so the pairs are (0,3), the precise one, and (1,2).
        let pair = |i: usize, j: usize| -> (f32, f32, f32) {
            let span = (pilot_positions[j] - pilot_positions[i]) as f32;
            ((rot[j] * rot[i].conj()).arg(), span, (weights[i] * weights[j]).sqrt())
        };
        let (dphi_in, span_in, w_in) = pair(1, 2);
        let (mut dphi_out, span_out, w_out) = pair(0, 3);
        if slope_hint.is_none() && span_in > 0.0 {
            // Unwrap the long baseline around the short one's estimate.
            let expect = dphi_in / span_in * span_out;
            dphi_out = expect + wrap(dphi_out - expect);
        }
        let num = w_in * dphi_in * span_in + w_out * dphi_out * span_out;
        let den = w_in * span_in * span_in + w_out * span_out * span_out;
        let slope = hint + if den > 0.0 { num / den } else { 0.0 };
        let slope_hint = hint;
        // Undo the hint removal for the CPE computation below.
        for (l, &k) in pilot_positions.iter().enumerate() {
            rot[l] *= Complex32::from_polar(1.0, slope_hint * k as f32);
        }
        // CPE from the slope-corrected pilots; quality = their coherence.
        let mut acc = Complex32::new(0.0, 0.0);
        let mut mags = 0.0f32;
        for (l, &k) in pilot_positions.iter().enumerate() {
            let r = rot[l] * Complex32::from_polar(1.0, -slope * k as f32);
            acc += r * weights[l];
            mags += r.norm() * weights[l];
        }
        let cpe = acc.arg();
        let quality = if mags > 0.0 { acc.norm() / mags } else { 0.0 };
        PilotMeasurement { cpe, slope, quality }
    }

    /// Traveling-pilot channel tracking: refresh H at the pilot tones of
    /// this symbol from the received pilots (after removing the CPE/slope
    /// found by [`Equalizer::equalize`]), blending with weight `beta`.
    pub fn track_pilots(
        &mut self,
        sym: &FreqSymbol,
        pilot_positions: &[i32; 4],
        expected_pilots: &[Complex32; 4],
        cpe: f32,
        slope: f32,
        beta: f32,
    ) {
        for (l, &k) in pilot_positions.iter().enumerate() {
            let a = ofdm::bin(k);
            let corr = Complex32::from_polar(1.0, -(cpe + slope * k as f32));
            let observed = sym[a] * corr / expected_pilots[l];
            self.est.h[a] = self.est.h[a] * (1.0 - beta) + observed * beta;
            self.refresh_bin(a);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ofdm::DATA_SUBCARRIER_INDICES;
    use crate::pilots::{self, PILOT_INDICES};

    fn ltf_fft_pair(chan: impl Fn(i32) -> Complex32) -> (FreqSymbol, FreqSymbol) {
        let ltf = preamble::ltf_freq();
        let mut s = [Complex32::new(0.0, 0.0); 64];
        for a in 0..64 {
            let k = a as i32 - 32;
            s[a] = ltf[a] * chan(k);
        }
        (s, s)
    }

    #[test]
    fn flat_channel_recovered() {
        let h0 = Complex32::new(0.8, -0.3);
        let (l1, l2) = ltf_fft_pair(|_| h0);
        let est = estimate(&l1, &l2);
        for a in 0..64 {
            if preamble::ltf_freq()[a].norm_sqr() > 0.0 {
                assert!((est.h[a] - h0).norm() < 1e-5);
            }
        }
        assert!(est.noise_var < 1e-10);
        assert!((used_tone_power(&l1) - h0.norm_sqr()).abs() < 1e-5);
    }

    #[test]
    fn selective_channel_and_equalization() {
        // Frequency-selective phase ramp channel.
        let chan = |k: i32| Complex32::from_polar(1.0 + 0.2 * (k as f32 / 28.0), 0.05 * k as f32);
        let (l1, l2) = ltf_fft_pair(chan);
        let est = estimate(&l1, &l2);
        let eq = Equalizer::new(est);
        // Build a data symbol through the same channel with a CPE of 0.3 rad.
        let vals: Vec<Complex32> = (0..52).map(|i| Complex32::new(if i % 2 == 0 { 1.0 } else { -1.0 }, 0.0)).collect();
        let p = pilots::data_pilots(3, false);
        let sym_tx = ofdm::assemble_freq_symbol(&DATA_SUBCARRIER_INDICES, &vals, &PILOT_INDICES, &p);
        let cpe = Complex32::from_polar(1.0, 0.3);
        let mut sym_rx = [Complex32::new(0.0, 0.0); 64];
        for a in 0..64 {
            let k = a as i32 - 32;
            sym_rx[a] = sym_tx[a] * chan(k) * cpe;
        }
        let out = eq.equalize(&sym_rx, &DATA_SUBCARRIER_INDICES, &PILOT_INDICES, &p);
        assert!((out.cpe - 0.3).abs() < 1e-3, "cpe {}", out.cpe);
        assert!(out.pilot_quality > 0.999);
        for (o, v) in out.data.iter().zip(&vals) {
            assert!((o - v).norm() < 1e-3);
        }
    }

    #[test]
    fn timing_shift_absorbed_as_slope() {
        // A 1-sample late signal = linear phase e^{-j2πk/64} on every tone.
        let shift = |k: i32| Complex32::from_polar(1.0, -2.0 * core::f32::consts::PI * k as f32 / 64.0);
        let (l1, l2) = ltf_fft_pair(|_| Complex32::new(1.0, 0.0));
        let eq = Equalizer::new(estimate(&l1, &l2));
        let vals: Vec<Complex32> = (0..52).map(|_| Complex32::new(1.0, 0.0)).collect();
        let p = pilots::data_pilots(0, false);
        let sym_tx = ofdm::assemble_freq_symbol(&DATA_SUBCARRIER_INDICES, &vals, &PILOT_INDICES, &p);
        let mut sym_rx = [Complex32::new(0.0, 0.0); 64];
        for a in 0..64 {
            let k = a as i32 - 32;
            sym_rx[a] = sym_tx[a] * shift(k);
        }
        // Slope correction from pilots should mostly restore the tones.
        let out = eq.equalize(&sym_rx, &DATA_SUBCARRIER_INDICES, &PILOT_INDICES, &p);
        for (i, o) in out.data.iter().enumerate() {
            assert!((o - vals[i]).norm() < 0.05, "tone {i}: {o}");
        }
        assert!((out.timing_offset_samples() - 1.0).abs() < 0.02, "{}", out.timing_offset_samples());
    }

    #[test]
    fn traveling_pilots_track_a_changing_channel() {
        // Start from a flat estimate; the true channel has a gain step on
        // one tone. After one 14-symbol traveling-pilot period every tone
        // has been refreshed.
        let (l1, l2) = ltf_fft_pair(|_| Complex32::new(1.0, 0.0));
        let mut eq = Equalizer::new(estimate(&l1, &l2));
        let chan = |k: i32| {
            if k == 5 {
                Complex32::new(0.5, 0.5)
            } else {
                Complex32::new(1.0, 0.0)
            }
        };
        for n in 0..14 {
            let pos = pilots::pilot_positions(n, true);
            let vals = pilots::data_pilots(n, true);
            let d = pilots::data_subcarriers(n, true);
            let dvals: Vec<Complex32> = (0..52).map(|_| Complex32::new(1.0, 0.0)).collect();
            let tx = ofdm::assemble_freq_symbol(&d, &dvals, &pos, &vals);
            let mut rx = [Complex32::new(0.0, 0.0); 64];
            for a in 0..64 {
                rx[a] = tx[a] * chan(a as i32 - 32);
            }
            let out = eq.equalize(&rx, &d, &pos, &vals);
            eq.track_pilots(&rx, &pos, &vals, out.cpe, out.slope, 1.0);
        }
        // The single-tone step is partly absorbed into that symbol's CPE
        // estimate (a per-tone change is not separable from a common
        // rotation with only 4 pilots), so the magnitude is exact and the
        // phase is close.
        let h5 = eq.estimate().h[ofdm::bin(5)];
        assert!((h5.norm() - 0.5f32.hypot(0.5)).abs() < 1e-3, "{h5}");
        assert!((h5.arg() - core::f32::consts::FRAC_PI_4).abs() < 0.25, "{h5}");
        assert!((eq.estimate().h[ofdm::bin(-5)] - Complex32::new(1.0, 0.0)).norm() < 0.05);
    }

    #[test]
    fn slope_hint_resolves_large_timing_offsets() {
        // A 1.5-sample offset wraps the outer pilot pair (1.5·2π·42/64 >
        // π); with a hint near the truth the measurement is exact.
        let (l1, l2) = ltf_fft_pair(|_| Complex32::new(1.0, 0.0));
        let eq = Equalizer::new(estimate(&l1, &l2));
        let d = 1.5f32;
        let shift = |k: i32| Complex32::from_polar(1.0, -2.0 * core::f32::consts::PI * k as f32 * d / 64.0);
        let vals: Vec<Complex32> = (0..52).map(|_| Complex32::new(1.0, 0.0)).collect();
        let p = pilots::data_pilots(0, false);
        let sym_tx = ofdm::assemble_freq_symbol(&DATA_SUBCARRIER_INDICES, &vals, &PILOT_INDICES, &p);
        let mut sym_rx = [Complex32::new(0.0, 0.0); 64];
        for a in 0..64 {
            sym_rx[a] = sym_tx[a] * shift(a as i32 - 32) * Complex32::from_polar(1.0, 0.4);
        }
        let m = eq.measure_pilots(&sym_rx, &PILOT_INDICES, &p, Some(slope_for_timing_offset(1.2)));
        assert!((m.timing_offset_samples() - d).abs() < 0.02, "{}", m.timing_offset_samples());
        assert!((m.cpe - 0.4).abs() < 0.02, "{}", m.cpe);
        assert!(m.quality > 0.999);
        // Without a hint the short pair still resolves it.
        let m2 = eq.measure_pilots(&sym_rx, &PILOT_INDICES, &p, None);
        assert!((m2.timing_offset_samples() - d).abs() < 0.02, "{}", m2.timing_offset_samples());
    }

    #[test]
    fn smoothing_reduces_estimate_noise() {
        let ltf = preamble::ltf_freq();
        let mut s = 7u64;
        let mut noise = || {
            s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((s >> 33) as f32 / (1u64 << 31) as f32 - 0.5) * 0.4
        };
        let mut l1 = [Complex32::new(0.0, 0.0); 64];
        let mut l2 = l1;
        for a in 0..64 {
            l1[a] = ltf[a] + Complex32::new(noise(), noise());
            l2[a] = ltf[a] + Complex32::new(noise(), noise());
        }
        let mut eq = Equalizer::new(estimate(&l1, &l2));
        let err = |eq: &Equalizer| -> f32 {
            (0..64).filter(|&a| ltf[a].norm_sqr() > 0.0).map(|a| (eq.estimate().h[a] - Complex32::new(1.0, 0.0)).norm_sqr()).sum()
        };
        let before = err(&eq);
        eq.smooth(0.0);
        let after = err(&eq);
        assert!(after < before * 0.7, "before {before} after {after}");
        // A known linear phase (window backoff) is not smeared by smoothing.
        let slope = 2.0 * core::f32::consts::PI * 6.0 / 64.0;
        let mut l3 = [Complex32::new(0.0, 0.0); 64];
        for a in 0..64 {
            l3[a] = ltf[a] * Complex32::from_polar(1.0, slope * (a as f32 - 32.0));
        }
        let mut eq2 = Equalizer::new(estimate(&l3, &l3));
        let before = eq2.estimate().h;
        eq2.smooth(slope);
        for a in 0..64 {
            if ltf[a].norm_sqr() > 0.0 {
                assert!((eq2.estimate().h[a] - before[a]).norm() < 1e-4, "bin {a}");
            }
        }
    }
}
