//! BCC interleavers.
//!
//! Data field: S1G 2 MHz uses the VHT 20 MHz interleaver [23.3.9.8 →
//! 21.3.10.8]: N_COL = 13, N_ROW = 4·N_BPSCS, no stream rotation for 1 SS.
//! Two permutations (Eq 21-76, 21-77) with s = max(1, N_BPSCS/2).
//!
//! SIG field: the Clause-17 48-bit BPSK interleaver [23.3.4.3.3 → 17.3.5.7]:
//! j = 3·(k mod 16) + ⌊k/16⌋ (second permutation is identity for BPSK).
//!
//! Deinterleaving of soft values applies the exact inverse permutation.

/// Forward permutation for one Data symbol: input index k → output index.
fn data_perm(n_bpscs: usize) -> Vec<usize> {
    let n_col = 13;
    let n_row = 4 * n_bpscs;
    let n_cbps = n_col * n_row;
    let s = 1.max(n_bpscs / 2);
    let mut perm = vec![0usize; n_cbps];
    for k in 0..n_cbps {
        let i = n_row * (k % n_col) + k / n_col; // Eq 21-76
        let j = s * (i / s) + (i + n_cbps - (n_col * i) / n_cbps) % s; // Eq 21-77
        perm[k] = j;
    }
    perm
}

/// Interleave one Data OFDM symbol. `bits.len() == 52 * n_bpscs`.
pub fn interleave(bits: &[u8], n_bpscs: usize) -> Vec<u8> {
    let perm = data_perm(n_bpscs);
    debug_assert_eq!(bits.len(), perm.len());
    let mut out = vec![0u8; bits.len()];
    for (k, &j) in perm.iter().enumerate() {
        out[j] = bits[k];
    }
    out
}

/// Inverse permutation over per-bit LLRs for one Data symbol.
pub fn deinterleave_llrs(llrs: &[f32], n_bpscs: usize) -> Vec<f32> {
    let perm = data_perm(n_bpscs);
    debug_assert_eq!(llrs.len(), perm.len());
    let mut out = vec![0.0f32; llrs.len()];
    for (k, &j) in perm.iter().enumerate() {
        out[k] = llrs[j];
    }
    out
}

/// SIG-field forward permutation (48 bits, BPSK): k → 3·(k mod 16) + ⌊k/16⌋.
fn sig_perm() -> [usize; 48] {
    let mut p = [0usize; 48];
    for (k, slot) in p.iter_mut().enumerate() {
        *slot = 3 * (k % 16) + k / 16;
    }
    p
}

/// Interleave one 48-bit SIG symbol block.
pub fn interleave_sig(bits: &[u8]) -> Vec<u8> {
    debug_assert_eq!(bits.len(), 48);
    let mut out = vec![0u8; 48];
    for (k, &j) in sig_perm().iter().enumerate() {
        out[j] = bits[k];
    }
    out
}

/// Inverse permutation over LLRs for one 48-bit SIG symbol block.
pub fn deinterleave_sig_llrs(llrs: &[f32]) -> Vec<f32> {
    debug_assert_eq!(llrs.len(), 48);
    let mut out = vec![0.0f32; 48];
    for (k, &j) in sig_perm().iter().enumerate() {
        out[k] = llrs[j];
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    const ALL_NBPSCS: [usize; 6] = [1, 2, 4, 6, 8, 10];

    #[test]
    fn perm_is_bijection() {
        for n in ALL_NBPSCS {
            let perm = data_perm(n);
            let mut seen = vec![false; perm.len()];
            for &j in &perm {
                assert!(!seen[j]);
                seen[j] = true;
            }
        }
        let mut seen = [false; 48];
        for &j in sig_perm().iter() {
            assert!(!seen[j]);
            seen[j] = true;
        }
    }

    #[test]
    fn roundtrip() {
        for n in ALL_NBPSCS {
            let n_cbps = 52 * n;
            let bits: Vec<u8> = (0..n_cbps).map(|i| ((i * 37 + 11) % 2) as u8).collect();
            let ilv = interleave(&bits, n);
            let llrs: Vec<f32> = ilv.iter().map(|&b| if b == 0 { 1.0 } else { -1.0 }).collect();
            let de = deinterleave_llrs(&llrs, n);
            let back: Vec<u8> = de.iter().map(|&l| if l > 0.0 { 0 } else { 1 }).collect();
            assert_eq!(back, bits, "n_bpscs {n}");
        }
    }

    #[test]
    fn sig_roundtrip_and_known_positions() {
        // j = 3*(k mod 16) + k/16: k=0→0, k=1→3, k=16→1, k=47→46.
        let p = sig_perm();
        assert_eq!(p[0], 0);
        assert_eq!(p[1], 3);
        assert_eq!(p[16], 1);
        assert_eq!(p[47], 3 * 15 + 2);
        let bits: Vec<u8> = (0..48).map(|i| ((i * 5 + 1) % 2) as u8).collect();
        let ilv = interleave_sig(&bits);
        let llrs: Vec<f32> = ilv.iter().map(|&b| if b == 0 { 1.0 } else { -1.0 }).collect();
        let back: Vec<u8> = deinterleave_sig_llrs(&llrs).iter().map(|&l| if l > 0.0 { 0 } else { 1 }).collect();
        assert_eq!(back, bits);
    }

    #[test]
    fn adjacent_coded_bits_spread_in_frequency() {
        // First permutation guarantees adjacent input bits land ~N_ROW apart
        // (different columns) — spot-check for QPSK.
        let perm = data_perm(2);
        // Consecutive k map to output positions differing by N_ROW = 8 until wrap.
        assert_eq!(perm[1] as i64 - perm[0] as i64, 8);
    }
}
