//! Constellation mapping and soft demapping [23.3.9.9.1 → 21.3.10.9.1 →
//! 17.3.5.8; 1024-QAM → 27.3.12.9].
//!
//! Within each group of N_BPSCS interleaved bits, the first half selects the
//! I axis and the second half the Q axis; the earliest bit of each half is
//! the most significant of the axis' Gray index [17.3.5.8]. Axis level for
//! Gray index n is 2n − (M−1); axis bit pattern is gray(n) = n ^ (n >> 1)
//! (verified bit-for-bit against Tables 17-15..17-18 and Figs 21-24..27,
//! 27-38..41 in the digest). Output d = (I + jQ) · K_MOD [Eq 17-20].
//!
//! LLR convention: LLR > 0 ⇒ bit = 0; max-log; scaled by per-subcarrier
//! `csi` (which should include |H|²/σ² from the equalizer).

use crate::params::Modulation;
use crate::Complex32;

/// Gray index → axis bit pattern (MSB = earliest bit).
#[inline]
fn gray(n: u32) -> u32 {
    n ^ (n >> 1)
}

/// Axis bits (earliest first) → amplitude level (odd integer).
fn axis_level(bits: &[u8]) -> i32 {
    let m = bits.len() as u32;
    let g: u32 = bits.iter().fold(0, |acc, &b| (acc << 1) | (b as u32 & 1));
    // Inverse Gray code: prefix XOR from the MSB down.
    let mut n = g;
    let mut sh = 1;
    while sh < m {
        n ^= n >> sh;
        sh <<= 1;
    }
    2 * (n as i32) - ((1i32 << m) - 1)
}

/// Map one symbol's interleaved bits (`52 * m.n_bpscs()` bits) to 52 points
/// in data-subcarrier order.
pub fn map(bits: &[u8], m: Modulation) -> Vec<Complex32> {
    let nb = m.n_bpscs();
    debug_assert_eq!(bits.len() % nb, 0);
    let k_mod = m.k_mod();
    bits.chunks(nb)
        .map(|g| {
            if nb == 1 {
                Complex32::new(axis_level(g) as f32 * k_mod, 0.0)
            } else {
                let half = nb / 2;
                let i = axis_level(&g[..half]) as f32;
                let q = axis_level(&g[half..]) as f32;
                Complex32::new(i, q) * k_mod
            }
        })
        .collect()
}

/// Per-axis max-log LLRs for coordinate `y` against `m_axis`-bit Gray levels.
fn axis_llrs(y: f32, m_axis: usize, k_mod: f32, out: &mut [f32]) {
    let levels = 1u32 << m_axis;
    // min squared distance per (bit index, bit value)
    let mut best = [[f32::INFINITY; 2]; 5];
    for n in 0..levels {
        let level = (2 * n as i32 - (levels as i32 - 1)) as f32 * k_mod;
        let d2 = (y - level) * (y - level);
        let pat = gray(n);
        for (bit, b) in best.iter_mut().enumerate().take(m_axis) {
            // bit 0 = earliest = MSB of the pattern
            let v = ((pat >> (m_axis - 1 - bit)) & 1) as usize;
            if d2 < b[v] {
                b[v] = d2;
            }
        }
    }
    for (o, b) in out.iter_mut().zip(&best).take(m_axis) {
        *o = b[1] - b[0]; // >0 ⇒ bit 0 closer
    }
}

/// Soft-demap 52 equalized points into `52 * m.n_bpscs()` LLRs.
/// `csi[k]` scales the LLRs of point `k`.
pub fn demap_llrs(points: &[Complex32], csi: &[f32], m: Modulation) -> Vec<f32> {
    debug_assert_eq!(points.len(), csi.len());
    let nb = m.n_bpscs();
    let k_mod = m.k_mod();
    let mut out = Vec::with_capacity(points.len() * nb);
    let mut buf = [0.0f32; 5];
    for (p, &w) in points.iter().zip(csi) {
        if nb == 1 {
            axis_llrs(p.re, 1, k_mod, &mut buf);
            out.push(buf[0] * w);
        } else {
            let half = nb / 2;
            axis_llrs(p.re, half, k_mod, &mut buf);
            for &l in &buf[..half] {
                out.push(l * w);
            }
            axis_llrs(p.im, half, k_mod, &mut buf);
            for &l in &buf[..half] {
                out.push(l * w);
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::params::Modulation::*;

    const ALL: [Modulation; 6] = [Bpsk, Qpsk, Qam16, Qam64, Qam256, Qam1024];

    #[test]
    fn spot_check_tables() {
        // BPSK [Table 17-15]: 0 → −1, 1 → +1.
        assert_eq!(axis_level(&[0]), -1);
        assert_eq!(axis_level(&[1]), 1);
        // 16-QAM [Table 17-17]: 00→−3, 01→−1, 11→+1, 10→+3.
        assert_eq!(axis_level(&[0, 0]), -3);
        assert_eq!(axis_level(&[0, 1]), -1);
        assert_eq!(axis_level(&[1, 1]), 1);
        assert_eq!(axis_level(&[1, 0]), 3);
        // 64-QAM [Table 17-18]: 000→−7, 010→−1, 110→+1, 100→+7, 011→−3, 101→+5.
        assert_eq!(axis_level(&[0, 0, 0]), -7);
        assert_eq!(axis_level(&[0, 1, 0]), -1);
        assert_eq!(axis_level(&[1, 1, 0]), 1);
        assert_eq!(axis_level(&[1, 0, 0]), 7);
        assert_eq!(axis_level(&[0, 1, 1]), -3);
        assert_eq!(axis_level(&[1, 0, 1]), 5);
        // 256-QAM [Figs 21-24..27]: 1100→+1, 1000→+15, 0000→−15, 0100→−1.
        assert_eq!(axis_level(&[1, 1, 0, 0]), 1);
        assert_eq!(axis_level(&[1, 0, 0, 0]), 15);
        assert_eq!(axis_level(&[0, 0, 0, 0]), -15);
        assert_eq!(axis_level(&[0, 1, 0, 0]), -1);
        // 1024-QAM [Figs 27-38..41]: 11000→+1 (n=16 → gray 11000).
        assert_eq!(axis_level(&[1, 1, 0, 0, 0]), 1);
    }

    #[test]
    fn unit_average_power() {
        for m in ALL {
            let nb = m.n_bpscs();
            // Enumerate all bit patterns per point via a counter.
            let n_pat = 1usize << nb.min(20);
            let mut acc = 0.0f64;
            for pat in 0..n_pat {
                let bits: Vec<u8> = (0..nb).map(|i| ((pat >> (nb - 1 - i)) & 1) as u8).collect();
                let p = map(&bits, m)[0];
                acc += (p.norm_sqr()) as f64;
            }
            let avg = acc / n_pat as f64;
            assert!((avg - 1.0).abs() < 1e-5, "{m:?} avg power {avg}");
        }
    }

    #[test]
    fn map_demap_roundtrip() {
        for m in ALL {
            let nb = m.n_bpscs();
            let bits: Vec<u8> = (0..52 * nb).map(|i| ((i * 29 + 5) % 2) as u8).collect();
            let pts = map(&bits, m);
            assert_eq!(pts.len(), 52);
            let csi = vec![1.0f32; 52];
            let llrs = demap_llrs(&pts, &csi, m);
            let hard: Vec<u8> = llrs.iter().map(|&l| if l > 0.0 { 0 } else { 1 }).collect();
            assert_eq!(hard, bits, "{m:?}");
        }
    }

    #[test]
    fn csi_scales_llrs() {
        let bits = [1u8, 0, 1, 1, 0, 0, 1, 0];
        let pts = map(&bits, Qam256);
        let l1 = demap_llrs(&pts, &[1.0], Qam256);
        let l2 = demap_llrs(&pts, &[2.5], Qam256);
        for (a, b) in l1.iter().zip(&l2) {
            assert!((b - a * 2.5).abs() < 1e-6);
        }
    }
}
