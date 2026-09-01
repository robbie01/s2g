//! OFDM symbol assembly for 2 MHz: subcarrier layout, IFFT/FFT, GI insertion,
//! LDPC tone mapping.
//!
//! Subcarrier convention: logical indices −32..31 (DC = 0); a `FreqSymbol`
//! array index a holds subcarrier k = a − 32 (fftshift order). TX waveform
//! per field: x[n] = scale · Σ_k X_k e^{j2πkn/64} with scale = 1/√N_Tone
//! [Eq 23-4], giving unit average power when the occupied tones have unit
//! magnitude. RX `fft_symbol` normalizes by 1/64 so it returns scale · X_k
//! for a clean TX symbol — constant factors are absorbed by the channel
//! estimate (both LTF and Data use 1/√56; the SIG field's 1/√52 makes its
//! equalized amplitude √(56/52) ≈ 1.04, harmless for BPSK decisions).
//!
//! Windowing: fields/symbols are concatenated rectangularly (the T_TR → 0
//! degenerate case of Eq 17-4, explicitly spec-conformant; spectral shaping
//! is left to the SDR-side interpolation filtering).

use crate::Complex32;
use rustfft::{Fft, FftPlanner};
use std::sync::{Arc, OnceLock};

/// The 52 Data-field subcarrier indices with fixed pilots, ascending
/// (±1..±28 minus pilots at ±7, ±21). Ascending order matches M'_2(k)
/// [Eq 23-30]: data symbol i goes on the i-th of these tones. With traveling
/// pilots the set varies per symbol — see `pilots::data_subcarriers`.
pub const DATA_SUBCARRIER_INDICES: [i32; 52] = data_indices();

const fn data_indices() -> [i32; 52] {
    let mut out = [0i32; 52];
    let mut n = 0;
    let mut k = -28i32;
    while k <= 28 {
        if k != 0 && k != -21 && k != -7 && k != 7 && k != 21 {
            out[n] = k;
            n += 1;
        }
        k += 1;
    }
    out
}

/// The 48 SIG-field subcarrier indices, ascending (±1..±26 minus pilots)
/// [Table 23-6; Eq 23-20/23-21 M'2 for SIG].
pub const SIG_SUBCARRIER_INDICES: [i32; 48] = sig_indices();

const fn sig_indices() -> [i32; 48] {
    let mut out = [0i32; 48];
    let mut n = 0;
    let mut k = -26i32;
    while k <= 26 {
        if k != 0 && k != -21 && k != -7 && k != 7 && k != 21 {
            out[n] = k;
            n += 1;
        }
        k += 1;
    }
    out
}

/// LDPC tone-mapping distance D_TM for 2 MHz (= VHT 20 MHz) [Table 21-19;
/// 23.3.9.9.2].
pub const LDPC_D_TM: usize = 4;

/// LDPC tone mapper permutation t(k) [Eq 21-86] for N_SD = 52, D_TM = 4:
/// t(k) = D_TM·(k mod (N_SD/D_TM)) + ⌊k·D_TM/N_SD⌋. Constellation point k
/// is transmitted on data tone t(k), so consecutive points land ≥ D_TM − 1
/// data tones apart.
pub fn ldpc_tone_map_index(k: usize) -> usize {
    let n_sd = 52;
    LDPC_D_TM * (k % (n_sd / LDPC_D_TM)) + (k * LDPC_D_TM) / n_sd
}

/// Apply the LDPC tone mapper to 52 constellation points (output index =
/// data-tone index).
pub fn ldpc_tone_map(points: &[Complex32]) -> Vec<Complex32> {
    debug_assert_eq!(points.len(), 52);
    let mut out = vec![Complex32::new(0.0, 0.0); 52];
    for (k, &p) in points.iter().enumerate() {
        out[ldpc_tone_map_index(k)] = p;
    }
    out
}

/// Undo the LDPC tone mapper on per-bit LLRs (`n_bpscs` LLRs per data tone,
/// in data-tone order) → LLRs in constellation-point order.
pub fn ldpc_tone_demap_llrs(llrs: &[f32], n_bpscs: usize) -> Vec<f32> {
    debug_assert_eq!(llrs.len(), 52 * n_bpscs);
    let mut out = vec![0.0f32; llrs.len()];
    for k in 0..52 {
        let t = ldpc_tone_map_index(k);
        out[k * n_bpscs..(k + 1) * n_bpscs].copy_from_slice(&llrs[t * n_bpscs..(t + 1) * n_bpscs]);
    }
    out
}

/// Freq-domain symbol: 64 bins, array index a = subcarrier (a − 32).
pub type FreqSymbol = [Complex32; 64];

/// Array index for logical subcarrier k.
#[inline]
pub fn bin(k: i32) -> usize {
    (k + 32) as usize
}

/// Place data values onto `indices` and pilot values onto `pilot_indices`
/// of a zeroed symbol.
pub fn assemble_freq_symbol(
    indices: &[i32],
    values: &[Complex32],
    pilot_indices: &[i32; 4],
    pilots: &[Complex32; 4],
) -> FreqSymbol {
    debug_assert_eq!(indices.len(), values.len());
    let mut sym = [Complex32::new(0.0, 0.0); 64];
    for (&k, &v) in indices.iter().zip(values) {
        sym[bin(k)] = v;
    }
    for (&k, &p) in pilot_indices.iter().zip(pilots) {
        sym[bin(k)] = p;
    }
    sym
}

fn ifft64() -> &'static Arc<dyn Fft<f32>> {
    static F: OnceLock<Arc<dyn Fft<f32>>> = OnceLock::new();
    F.get_or_init(|| FftPlanner::new().plan_fft_inverse(64))
}

fn fft64() -> &'static Arc<dyn Fft<f32>> {
    static F: OnceLock<Arc<dyn Fft<f32>>> = OnceLock::new();
    F.get_or_init(|| FftPlanner::new().plan_fft_forward(64))
}

/// Unnormalized 64-point IDFT of a logical-order symbol: returns the 64 time
/// samples x[n] = Σ_k X_k e^{j2πkn/64}, scaled by `scale`.
pub fn idft(freq: &FreqSymbol, scale: f32) -> [Complex32; 64] {
    let mut buf = [Complex32::new(0.0, 0.0); 64];
    for (a, &v) in freq.iter().enumerate() {
        let k = a as i32 - 32;
        buf[((k + 64) % 64) as usize] = v;
    }
    ifft64().process(&mut buf);
    for v in &mut buf {
        *v *= scale;
    }
    buf
}

/// TX: IDFT + prepend `gi_len` samples of cyclic prefix, scaled by `scale`.
pub fn to_time_domain(freq: &FreqSymbol, gi_len: usize, scale: f32) -> Vec<Complex32> {
    let x = idft(freq, scale);
    let mut out = Vec::with_capacity(64 + gi_len);
    out.extend_from_slice(&x[64 - gi_len..]);
    out.extend_from_slice(&x);
    out
}

/// RX: forward DFT of exactly 64 time samples (GI already stripped),
/// normalized by 1/64, returned in logical order −32..31.
pub fn fft_symbol(time: &[Complex32]) -> FreqSymbol {
    debug_assert_eq!(time.len(), 64);
    let mut buf: [Complex32; 64] = core::array::from_fn(|i| time[i]);
    fft64().process(&mut buf);
    let mut out = [Complex32::new(0.0, 0.0); 64];
    for a in 0..64usize {
        let k = a as i32 - 32;
        out[a] = buf[((k + 64) % 64) as usize] / 64.0;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pilots::PILOT_INDICES;

    #[test]
    fn index_tables() {
        assert_eq!(DATA_SUBCARRIER_INDICES.len(), 52);
        assert_eq!(DATA_SUBCARRIER_INDICES[0], -28);
        assert_eq!(DATA_SUBCARRIER_INDICES[51], 28);
        for k in [0i32, -21, -7, 7, 21] {
            assert!(!DATA_SUBCARRIER_INDICES.contains(&k));
        }
        assert_eq!(SIG_SUBCARRIER_INDICES.len(), 48);
        assert_eq!(SIG_SUBCARRIER_INDICES[0], -26);
        assert_eq!(SIG_SUBCARRIER_INDICES[47], 26);
        // M'_2 endpoint checks [digest timing-math sanity #4]: data index 0 →
        // k=−28, 6 → −22, 7 → −20, 51 → 28.
        assert_eq!(DATA_SUBCARRIER_INDICES[6], -22);
        assert_eq!(DATA_SUBCARRIER_INDICES[7], -20);
    }

    #[test]
    fn idft_fft_roundtrip() {
        let vals: Vec<Complex32> = (0..52)
            .map(|i| Complex32::new(((i % 3) as f32) - 1.0, ((i % 5) as f32) / 2.0 - 1.0))
            .collect();
        let pilots = [Complex32::new(1.0, 0.0); 4];
        let sym = assemble_freq_symbol(&DATA_SUBCARRIER_INDICES, &vals, &PILOT_INDICES, &pilots);
        let scale = 1.0 / (56.0f32).sqrt();
        let t = to_time_domain(&sym, 16, scale);
        assert_eq!(t.len(), 80);
        // GI is cyclic: first 16 samples == last 16.
        for i in 0..16 {
            let d = t[i] - t[64 + i];
            assert!(d.norm() < 1e-5);
        }
        // FFT of the payload recovers scale * X_k.
        let rec = fft_symbol(&t[16..]);
        for (&k, &v) in DATA_SUBCARRIER_INDICES.iter().zip(&vals) {
            let d = rec[bin(k)] - v * scale;
            assert!(d.norm() < 1e-4, "tone {k}");
        }
        for (&k, &p) in PILOT_INDICES.iter().zip(&pilots) {
            let d = rec[bin(k)] - p * scale;
            assert!(d.norm() < 1e-4);
        }
        // Nulls stay null.
        for k in [0i32, -30, 31, -32] {
            assert!(rec[bin(k)].norm() < 1e-4);
        }
    }

    #[test]
    fn unit_power_symbol() {
        // 56 unit tones scaled 1/sqrt(56) → unit average time-domain power.
        let vals: Vec<Complex32> = (0..52).map(|i| Complex32::new(if i % 2 == 0 { 1.0 } else { -1.0 }, 0.0)).collect();
        let pilots = [Complex32::new(1.0, 0.0), Complex32::new(1.0, 0.0), Complex32::new(1.0, 0.0), Complex32::new(-1.0, 0.0)];
        let sym = assemble_freq_symbol(&DATA_SUBCARRIER_INDICES, &vals, &PILOT_INDICES, &pilots);
        let x = idft(&sym, 1.0 / (56.0f32).sqrt());
        let p: f32 = x.iter().map(|v| v.norm_sqr()).sum::<f32>() / 64.0;
        assert!((p - 1.0).abs() < 1e-4, "power {p}");
    }

    #[test]
    fn ldpc_tone_mapper_is_a_spread_bijection() {
        let mut seen = [false; 52];
        for k in 0..52 {
            let t = ldpc_tone_map_index(k);
            assert!(!seen[t]);
            seen[t] = true;
        }
        // Consecutive points ≥ D_TM − 1 tones apart (except the row wrap).
        for k in 0..51 {
            let (a, b) = (ldpc_tone_map_index(k) as i64, ldpc_tone_map_index(k + 1) as i64);
            if k % 13 != 12 {
                assert!((a - b).abs() >= LDPC_D_TM as i64, "k {k}: {a} {b}");
            }
        }
        assert_eq!(ldpc_tone_map_index(0), 0);
        assert_eq!(ldpc_tone_map_index(1), 4);
        assert_eq!(ldpc_tone_map_index(13), 1);
        // LLR demap inverts the point permutation.
        let pts: Vec<Complex32> = (0..52).map(|i| Complex32::new(i as f32, 0.0)).collect();
        let mapped = ldpc_tone_map(&pts);
        let llrs: Vec<f32> = mapped.iter().flat_map(|p| [p.re, p.re + 0.5]).collect();
        let back = ldpc_tone_demap_llrs(&llrs, 2);
        for k in 0..52 {
            assert_eq!(back[2 * k], k as f32);
            assert_eq!(back[2 * k + 1], k as f32 + 0.5);
        }
    }
}
