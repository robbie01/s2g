//! Transmitter conformance measurements [23.3.17]: spectral flatness
//! (23.3.17.2), constellation error / EVM (23.3.17.4.3), transmit spectral
//! mask (23.3.17.1) and center-frequency leakage (23.3.17.4.2).
//!
//! These operate on baseband sample streams, so they verify the digital
//! waveform (and, fed with a captured loopback, the whole radio). The
//! measurement recipe follows 23.3.17.4.4: the EVM path literally runs the
//! receiver (sync, LTF channel estimate, pilot tracking) and compares each
//! equalized data tone with the nearest constellation point.

use crate::ofdm;
use crate::params::{self, tx_limits};
use crate::rx::{Receiver, RxConfig, RxEvent};
use crate::Complex32;

/// Spectral flatness result [Table 23-33].
#[derive(Debug, Clone)]
pub struct FlatnessReport {
    /// Per-tone average energy relative to the inner-tone average, dB.
    pub deviation_db: Vec<(i32, f32)>,
    pub worst_inner_db: f32,
    pub worst_outer_db: f32,
    pub pass: bool,
}

/// Average constellation energy per data-field tone over `n_sym` OFDM
/// symbols of an S1G_SHORT PPDU waveform at 2 MS/s (Data field starts at
/// sample 480; symbol = 16 GI + 64).
pub fn tone_energies(wave: &[Complex32], n_sym: usize) -> [f32; 64] {
    let mut e = [0.0f32; 64];
    for n in 0..n_sym {
        let start = 480 + n * 80 + 16;
        if start + 64 > wave.len() {
            break;
        }
        let f = ofdm::fft_symbol(&wave[start..start + 64]);
        for a in 0..64 {
            e[a] += f[a].norm_sqr();
        }
    }
    for v in &mut e {
        *v /= n_sym.max(1) as f32;
    }
    e
}

/// Spectral flatness of a PPDU waveform per 23.3.17.2: average linear
/// energy over |k| ≤ 16; inner tones within ±4 dB, outer (17..28) within
/// +4/−6 dB.
pub fn spectral_flatness(wave: &[Complex32], n_sym: usize) -> FlatnessReport {
    let e = tone_energies(wave, n_sym);
    let inner: Vec<f32> = (-16..=16).filter(|&k| k != 0).map(|k| e[ofdm::bin(k)]).collect();
    let avg = inner.iter().sum::<f32>() / inner.len() as f32;
    let mut deviation_db = Vec::with_capacity(56);
    let (mut worst_inner, mut worst_outer) = (0.0f32, 0.0f32);
    let mut pass = true;
    for k in -28..=28i32 {
        if k == 0 {
            continue;
        }
        let d = 10.0 * (e[ofdm::bin(k)] / avg.max(1e-30)).max(1e-30).log10();
        deviation_db.push((k, d));
        let (lo, hi) = if k.abs() <= tx_limits::FLATNESS_INNER_MAX_K {
            if d.abs() > worst_inner.abs() {
                worst_inner = d;
            }
            tx_limits::FLATNESS_INNER_DB
        } else {
            if d.abs() > worst_outer.abs() {
                worst_outer = d;
            }
            tx_limits::FLATNESS_OUTER_DB
        };
        if d < lo || d > hi {
            pass = false;
        }
    }
    FlatnessReport { deviation_db, worst_inner_db: worst_inner, worst_outer_db: worst_outer, pass }
}

/// Center-frequency (DC) leakage relative to the average per-subcarrier
/// power, dB [23.3.17.4.2]: must be ≤ 0 dB (P − 10·log10(N_ST)).
pub fn dc_leakage_db(wave: &[Complex32], n_sym: usize) -> f32 {
    let e = tone_energies(wave, n_sym);
    let used: f32 = (-28..=28).filter(|&k| k != 0).map(|k| e[ofdm::bin(k)]).sum::<f32>() / 56.0;
    10.0 * (e[ofdm::bin(0)] / used.max(1e-30)).max(1e-30).log10()
}

/// EVM measurement result [Table 23-34].
#[derive(Debug, Clone)]
pub struct EvmReport {
    pub mcs: u8,
    pub evm_db: f32,
    pub limit_db: f32,
    pub pass: bool,
}

/// Relative constellation error of a PPDU waveform measured the way a
/// receiver would (23.3.17.4.4 a–i). `None` if the PPDU does not decode.
pub fn tx_evm(wave: &[Complex32]) -> Option<EvmReport> {
    let mut rx = Receiver::new(RxConfig { emit_cca: false, ..Default::default() });
    let mut ev = Vec::new();
    // Lead-in / lead-out silence so detection and flushing work.
    let pad = vec![Complex32::new(0.0, 0.0); 400];
    rx.process(&pad, &mut ev);
    rx.process(wave, &mut ev);
    rx.process(&pad, &mut ev);
    rx.finish(&mut ev);
    ev.into_iter().find_map(|e| match e {
        RxEvent::PsduReceived { rxvector, metrics, .. } => {
            let (m, r) = params::mcs_modulation_rate(rxvector.mcs)?;
            let limit_db = tx_limits::evm_limit_db(m, r);
            Some(EvmReport { mcs: rxvector.mcs, evm_db: metrics.evm_db, limit_db, pass: metrics.evm_db <= limit_db })
        }
        _ => None,
    })
}

/// Transmit spectral mask result.
#[derive(Debug, Clone)]
pub struct MaskReport {
    /// (offset MHz, PSD dBr, mask dBr) for every analysed bin.
    pub bins: Vec<(f32, f32, f32)>,
    /// Smallest (mask − PSD) over all bins; negative = violation.
    pub worst_margin_db: f32,
    pub pass: bool,
}

/// Welch power spectral density (Hann windowed, 50 % overlap) with a
/// resolution bandwidth of about `rbw_hz`, returned as (offset Hz, power)
/// pairs ordered from −fs/2 to +fs/2.
pub fn psd(wave: &[Complex32], sample_rate_hz: f64, rbw_hz: f64) -> Vec<(f64, f64)> {
    let n = ((sample_rate_hz / rbw_hz).round() as usize).max(16);
    let hop = n / 2;
    let mut planner = rustfft::FftPlanner::<f32>::new();
    let fft = planner.plan_fft_forward(n);
    let win: Vec<f32> = (0..n).map(|i| 0.5 - 0.5 * (2.0 * std::f32::consts::PI * i as f32 / n as f32).cos()).collect();
    let mut acc = vec![0.0f64; n];
    let mut segs = 0usize;
    let mut start = 0;
    while start + n <= wave.len() {
        let mut buf: Vec<Complex32> = wave[start..start + n].iter().zip(&win).map(|(&s, &w)| s * w).collect();
        fft.process(&mut buf);
        for (a, v) in acc.iter_mut().zip(&buf) {
            *a += v.norm_sqr() as f64;
        }
        segs += 1;
        start += hop;
    }
    let segs = segs.max(1) as f64;
    (0..n)
        .map(|i| {
            // Reorder to −fs/2..fs/2.
            let bin = (i + n / 2) % n;
            let k = bin as i64 - if bin >= n / 2 { n as i64 } else { 0 };
            (k as f64 * sample_rate_hz / n as f64, acc[bin] / segs)
        })
        .collect()
}

/// Check a waveform (at any sample rate) against the 2 MHz transmit
/// spectral mask [23.3.17.1], 10 kHz RBW. Only offsets within ±fs/2 can be
/// examined — feed an interpolated (≥ 6 MS/s) stream to cover the full
/// ±3 MHz mask.
pub fn spectral_mask(wave: &[Complex32], sample_rate_hz: f64) -> MaskReport {
    let spectrum = psd(wave, sample_rate_hz, 10_000.0);
    let peak = spectrum.iter().map(|&(_, p)| p).fold(0.0f64, f64::max).max(1e-30);
    let mut bins = Vec::with_capacity(spectrum.len());
    let mut worst = f32::INFINITY;
    for (f, p) in spectrum {
        let dbr = (10.0 * (p / peak).max(1e-30).log10()) as f32;
        let off = (f / 1e6) as f32;
        let mask = tx_limits::spectral_mask_2mhz_dbr(off);
        worst = worst.min(mask - dbr);
        bins.push((off, dbr, mask));
    }
    MaskReport { bins, worst_margin_db: worst, pass: worst >= 0.0 }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::params::valid_mcs;
    use crate::vector::{Coding, TxVector};
    use crate::Transmitter;

    #[test]
    fn our_waveform_is_flat_and_dc_free() {
        let tx = Transmitter::new();
        let psdu: Vec<u8> = (0..300u32).map(|i| (i * 17) as u8).collect();
        for coding in [Coding::Bcc, Coding::Ldpc] {
            let txv = TxVector { mcs: 0, fec_coding: coding, scrambler_seed: Some(7), ..Default::default() };
            let w = tx.generate(&txv, &psdu).unwrap();
            let n_sym = (w.len() - 480) / 80;
            let r = spectral_flatness(&w, n_sym);
            assert!(r.pass, "{r:?}");
            assert!(r.worst_inner_db.abs() < 0.5, "{}", r.worst_inner_db);
            assert!(dc_leakage_db(&w, n_sym) < -40.0);
        }
        // Traveling pilots boost 4 of 56 tones by 1.5² every symbol but the
        // average over a period is still flat.
        let txv = TxVector { mcs: 1, traveling_pilots: true, scrambler_seed: Some(7), ..Default::default() };
        let w = tx.generate(&txv, &psdu).unwrap();
        let r = spectral_flatness(&w, (w.len() - 480) / 80);
        assert!(r.pass, "{r:?}");
    }

    #[test]
    fn evm_meets_table_23_34_for_every_mcs() {
        let tx = Transmitter::new();
        let psdu: Vec<u8> = (0..120u32).map(|i| (i * 29 + 3) as u8).collect();
        for mcs in valid_mcs() {
            let txv = TxVector { mcs, scrambler_seed: Some(11), ..Default::default() };
            let w = tx.generate(&txv, &psdu).unwrap();
            let r = tx_evm(&w).expect("decodes");
            assert_eq!(r.mcs, mcs);
            assert!(r.pass, "{r:?}");
            assert!(r.evm_db < -40.0, "MCS {mcs}: {}", r.evm_db);
        }
    }

    #[test]
    fn mask_within_native_nyquist() {
        // At 2 MS/s only |f| ≤ 1 MHz is observable: the 0 dBr shoulder and
        // the 0.9..1.0 MHz roll-off.
        let tx = Transmitter::new();
        let psdu: Vec<u8> = (0..400u32).map(|i| (i * 7) as u8).collect();
        let txv = TxVector { mcs: 3, aggregation: false, scrambler_seed: Some(3), ..Default::default() };
        let w = tx.generate(&txv, &psdu).unwrap();
        let r = spectral_mask(&w, 2.0e6);
        assert!(r.pass, "worst margin {} dB", r.worst_margin_db);
    }
}
