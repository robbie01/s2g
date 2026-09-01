//! Channel estimation, one-tap equalization, and pilot-based phase tracking.
//!
//! H is least-squares estimated from the two repeated LTS periods of LTF1
//! against `preamble::ltf_freq()`; the noise variance comes from their
//! difference. Per-symbol common phase error (CPE) and residual linear phase
//! slope (timing drift / sampling offset) are estimated from the 4 pilots
//! and removed before the data tones are handed to the demapper. CSI weight
//! per tone is |H|²/σ² (for LLR scaling).

use crate::ofdm::{self, FreqSymbol};
use crate::pilots::PILOT_INDICES;
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
    /// SNR estimate in dB.
    pub fn snr_db(&self) -> f32 {
        10.0 * (self.signal_power / self.noise_var.max(1e-12)).log10()
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
            // Var of (n1−n2)/1 is 2σ²; the averaged estimate sees σ²/2 but we
            // report the per-tone, per-symbol σ².
            noise += d.norm_sqr() * 0.5;
            sig += avg.norm_sqr();
            used += 1;
        }
    }
    ChannelEstimate {
        h,
        noise_var: noise / used as f32,
        signal_power: sig / used as f32,
    }
}

/// Equalized output for one OFDM symbol.
pub struct EqualizedSymbol {
    /// Equalized data tones in the order of the `indices` passed in.
    pub data: Vec<Complex32>,
    /// Per-tone CSI weight (|H|²/σ²) for LLR scaling (same order).
    pub csi: Vec<f32>,
    /// Common phase error removed from this symbol (radians).
    pub cpe: f32,
}

pub struct Equalizer {
    est: ChannelEstimate,
    inv_h: [Complex32; 64],
    csi_all: [f32; 64],
}

impl Equalizer {
    pub fn new(est: ChannelEstimate) -> Self {
        let mut inv_h = [Complex32::new(0.0, 0.0); 64];
        let mut csi_all = [0.0f32; 64];
        let nv = est.noise_var.max(1e-12);
        for a in 0..64 {
            let hp = est.h[a].norm_sqr();
            if hp > 1e-12 {
                inv_h[a] = est.h[a].conj() / hp;
                csi_all[a] = hp / nv;
            }
        }
        Self { est, inv_h, csi_all }
    }

    pub fn estimate(&self) -> &ChannelEstimate {
        &self.est
    }

    /// Equalize one symbol: extract `indices` tones, remove CPE and linear
    /// phase slope estimated from the pilots (`expected_pilots` in
    /// `PILOT_INDICES` order).
    pub fn equalize(&self, sym: &FreqSymbol, indices: &[i32], expected_pilots: &[Complex32; 4]) -> EqualizedSymbol {
        // Pilot phase measurement, weighted by |H|².
        let mut acc = Complex32::new(0.0, 0.0);
        let mut num = 0.0f32; // Σ w·φ·k
        let mut den = 0.0f32; // Σ w·k²
        let mut phases = [0.0f32; 4];
        let mut weights = [0.0f32; 4];
        for (l, &k) in PILOT_INDICES.iter().enumerate() {
            let a = ofdm::bin(k);
            let eq = sym[a] * self.inv_h[a];
            let rot = eq * expected_pilots[l].conj();
            let w = self.est.h[a].norm_sqr();
            acc += rot * w;
            phases[l] = rot.arg();
            weights[l] = w;
        }
        let cpe = acc.arg();
        // Residual per-pilot phase after CPE removal → weighted LS slope
        // over k (pilot positions are symmetric, so intercept ≈ cpe).
        for (l, &k) in PILOT_INDICES.iter().enumerate() {
            let mut r = phases[l] - cpe;
            // wrap to [-π, π]
            while r > core::f32::consts::PI {
                r -= 2.0 * core::f32::consts::PI;
            }
            while r < -core::f32::consts::PI {
                r += 2.0 * core::f32::consts::PI;
            }
            num += weights[l] * r * k as f32;
            den += weights[l] * (k * k) as f32;
        }
        let slope = if den > 0.0 { num / den } else { 0.0 };

        let mut data = Vec::with_capacity(indices.len());
        let mut csi = Vec::with_capacity(indices.len());
        for &k in indices {
            let a = ofdm::bin(k);
            let corr = Complex32::from_polar(1.0, -(cpe + slope * k as f32));
            data.push(sym[a] * self.inv_h[a] * corr);
            csi.push(self.csi_all[a]);
        }
        EqualizedSymbol { data, csi, cpe }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ofdm::DATA_SUBCARRIER_INDICES;
    use crate::pilots;

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
        let p = pilots::data_pilots(3);
        let sym_tx = ofdm::assemble_freq_symbol(&DATA_SUBCARRIER_INDICES, &vals, &p);
        let cpe = Complex32::from_polar(1.0, 0.3);
        let mut sym_rx = [Complex32::new(0.0, 0.0); 64];
        for a in 0..64 {
            let k = a as i32 - 32;
            sym_rx[a] = sym_tx[a] * chan(k) * cpe;
        }
        let out = eq.equalize(&sym_rx, &DATA_SUBCARRIER_INDICES, &p);
        assert!((out.cpe - 0.3).abs() < 1e-3, "cpe {}", out.cpe);
        for (o, v) in out.data.iter().zip(&vals) {
            assert!((o - v).norm() < 1e-3);
        }
    }

    #[test]
    fn timing_shift_absorbed_as_slope() {
        // A 1-sample timing shift = linear phase e^{-j2πk/64} on every tone.
        let shift = |k: i32| Complex32::from_polar(1.0, -2.0 * core::f32::consts::PI * k as f32 / 64.0);
        let (l1, l2) = ltf_fft_pair(|_| Complex32::new(1.0, 0.0));
        let eq = Equalizer::new(estimate(&l1, &l2));
        let vals: Vec<Complex32> = (0..52).map(|_| Complex32::new(1.0, 0.0)).collect();
        let p = pilots::data_pilots(0);
        let sym_tx = ofdm::assemble_freq_symbol(&DATA_SUBCARRIER_INDICES, &vals, &p);
        let mut sym_rx = [Complex32::new(0.0, 0.0); 64];
        for a in 0..64 {
            let k = a as i32 - 32;
            sym_rx[a] = sym_tx[a] * shift(k);
        }
        // Slope correction from pilots should mostly restore the tones.
        let out = eq.equalize(&sym_rx, &DATA_SUBCARRIER_INDICES, &p);
        for (i, o) in out.data.iter().enumerate() {
            assert!((o - vals[i]).norm() < 0.05, "tone {i}: {o}");
        }
    }
}
