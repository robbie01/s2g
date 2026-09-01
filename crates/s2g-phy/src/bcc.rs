//! Binary convolutional coding: rate-1/2 K=7 encoder (g0=133o, g1=171o)
//! [17.3.5.6, p3361], puncturing to 2/3, 3/4 [Fig 17-9, p3362] and 5/6
//! [Fig 19-11, p3444], and a soft-decision Viterbi decoder.
//!
//! Encoder output order is A then B per input bit. LLR convention:
//! LLR > 0 ⇒ bit 0; punctured (stolen) positions get zero LLR at the
//! decoder [17.3.5.6].

use crate::params::CodeRate;

const G0: u8 = 0b101_1011; // 133 octal; bit 6 = current input, bit 0 = x[n-6]
const G1: u8 = 0b111_1001; // 171 octal

#[inline]
fn parity(x: u8) -> u8 {
    (x.count_ones() & 1) as u8
}

/// Puncturing pattern: (period, keep flags interleaved [A0,B0,A1,B1,...]).
fn puncture_pattern(rate: CodeRate) -> (usize, &'static [u8]) {
    match rate {
        CodeRate::R1_2 => (1, &[1, 1]),
        CodeRate::R2_3 => (2, &[1, 1, 1, 0]),
        CodeRate::R3_4 => (3, &[1, 1, 1, 0, 0, 1]),
        CodeRate::R5_6 => (5, &[1, 1, 1, 0, 0, 1, 1, 0, 0, 1]),
    }
}

/// Convolutionally encode at rate 1/2 (no puncturing). The encoder starts in
/// the all-zero state; the caller's zero tail bits return it to zero.
pub fn encode_r12(bits: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(bits.len() * 2);
    let mut sh: u8 = 0; // bits [x-1 .. x-6] in bit positions 5..0
    for &x in bits {
        let v = ((x & 1) << 6) | sh;
        out.push(parity(v & G0));
        out.push(parity(v & G1));
        sh = v >> 1;
    }
    out
}

/// Encode then puncture to `rate`.
pub fn encode(bits: &[u8], rate: CodeRate) -> Vec<u8> {
    let mother = encode_r12(bits);
    let (period, keep) = puncture_pattern(rate);
    let mut out = Vec::with_capacity(mother.len() * 2 / 3);
    for (k, pair) in mother.chunks(2).enumerate() {
        let m = k % period;
        if keep[2 * m] == 1 {
            out.push(pair[0]);
        }
        if keep[2 * m + 1] == 1 {
            out.push(pair[1]);
        }
    }
    out
}

/// Number of punctured (transmitted) bits for `n_info_bits` input bits.
/// Only valid when the puncture period divides cleanly; S1G symbol sizes
/// always satisfy this per-field.
pub fn punctured_len(n_info_bits: usize, rate: CodeRate) -> usize {
    let (num, den) = rate.as_fraction();
    debug_assert!(n_info_bits % num == 0);
    n_info_bits * den / num
}

/// Soft Viterbi decode of `llrs` (punctured stream at `rate`) into exactly
/// `n_info_bits` bits (including any tail bits the caller appended).
/// The trellis is assumed terminated: start and end state are zero.
pub fn viterbi_decode(llrs: &[f32], rate: CodeRate, n_info_bits: usize) -> Vec<u8> {
    let (period, keep) = puncture_pattern(rate);

    // Depuncture into (la, lb) per trellis step; stolen bits get 0.0.
    let mut pairs = vec![(0.0f32, 0.0f32); n_info_bits];
    let mut idx = 0;
    for (k, p) in pairs.iter_mut().enumerate() {
        let m = k % period;
        if keep[2 * m] == 1 {
            p.0 = llrs.get(idx).copied().unwrap_or(0.0);
            idx += 1;
        }
        if keep[2 * m + 1] == 1 {
            p.1 = llrs.get(idx).copied().unwrap_or(0.0);
            idx += 1;
        }
    }
    debug_assert_eq!(idx, llrs.len(), "LLR count mismatch for n_info_bits/rate");

    const NEG: f32 = -1.0e30;
    let mut metric = [NEG; 64];
    metric[0] = 0.0;
    // decisions[step] bit ns = chosen predecessor's oldest bit for next-state ns.
    let mut decisions = vec![0u64; n_info_bits];

    // Precompute branch outputs: out[s][x] = (a, b) as signs.
    // (Cheap enough to compute inline.)
    for (step, &(la, lb)) in pairs.iter().enumerate() {
        let mut next = [NEG; 64];
        let mut dec = 0u64;
        for s in 0..64u8 {
            let m = metric[s as usize];
            if m <= NEG {
                continue;
            }
            for x in 0..2u8 {
                let v = (x << 6) | s;
                let a = parity(v & G0);
                let b = parity(v & G1);
                let bm = (if a == 0 { la } else { -la }) + (if b == 0 { lb } else { -lb });
                let ns = ((x << 5) | (s >> 1)) as usize;
                let cand = m + bm;
                if cand > next[ns] {
                    next[ns] = cand;
                    // predecessor s = ((ns & 0x1f) << 1) | oldest_bit; store oldest bit of s.
                    let oldest = s & 1;
                    dec = (dec & !(1u64 << ns)) | ((oldest as u64) << ns);
                }
            }
        }
        metric = next;
        decisions[step] = dec;
    }

    // Traceback from state 0 (terminated trellis).
    let mut bits = vec![0u8; n_info_bits];
    let mut state: u8 = 0;
    for step in (0..n_info_bits).rev() {
        let x = (state >> 5) & 1;
        bits[step] = x;
        let oldest = ((decisions[step] >> state) & 1) as u8;
        state = ((state & 0x1f) << 1) | oldest;
    }
    bits
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rand_bits(n: usize, seed: u64) -> Vec<u8> {
        // Small deterministic LCG; avoids dev-dep use in unit tests.
        let mut s = seed;
        (0..n)
            .map(|_| {
                s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                ((s >> 33) & 1) as u8
            })
            .collect()
    }

    fn with_tail(mut v: Vec<u8>) -> Vec<u8> {
        v.extend_from_slice(&[0; 6]);
        v
    }

    #[test]
    fn encoder_first_steps() {
        // From state 0, input 1: v = 1000000 → A = parity(1000000 & 1011011) = 1,
        // B = 1. Then input 0: sh = 100000, v = 0100000 → A = 0, B = 1.
        assert_eq!(encode_r12(&[1, 0]), vec![1, 1, 0, 1]);
        // All-zero input stays all-zero.
        assert_eq!(encode_r12(&[0, 0, 0]), vec![0; 6]);
    }

    #[test]
    fn puncture_lengths() {
        for (rate, n, expect) in [
            (CodeRate::R1_2, 12, 24),
            (CodeRate::R2_3, 12, 18),
            (CodeRate::R3_4, 12, 16),
            (CodeRate::R5_6, 10, 12),
        ] {
            let bits = rand_bits(n, 7);
            assert_eq!(encode(&bits, rate).len(), expect);
            assert_eq!(punctured_len(n, rate), expect);
        }
    }

    #[test]
    fn puncture_pattern_r34_steals_a2_b1() {
        // Period 3: sent A0 B0 A1 B2 (steal A2, B1) [Fig 17-9].
        let bits = [1, 0, 0, 0, 0, 0, 0, 0, 0];
        let mother = encode_r12(&bits);
        let sent = encode(&bits, CodeRate::R3_4);
        assert_eq!(sent[0], mother[0]); // A0
        assert_eq!(sent[1], mother[1]); // B0
        assert_eq!(sent[2], mother[2]); // A1
        assert_eq!(sent[3], mother[5]); // B2
    }

    fn hard_llrs(bits: &[u8]) -> Vec<f32> {
        bits.iter().map(|&b| if b == 0 { 1.0 } else { -1.0 }).collect()
    }

    #[test]
    fn roundtrip_all_rates() {
        for rate in [CodeRate::R1_2, CodeRate::R2_3, CodeRate::R3_4, CodeRate::R5_6] {
            for n in [30usize, 114, 253] {
                // Info sized so total (with tail) is fine for any period.
                let info = with_tail(rand_bits(n, n as u64 * 31 + 7));
                let coded = encode(&info, rate);
                let decoded = viterbi_decode(&hard_llrs(&coded), rate, info.len());
                assert_eq!(decoded, info, "rate {rate:?} n {n}");
            }
        }
    }

    #[test]
    fn corrects_bit_errors_r12() {
        let info = with_tail(rand_bits(200, 99));
        let coded = encode(&info, CodeRate::R1_2);
        let mut llrs = hard_llrs(&coded);
        // Flip 8 well-separated coded bits.
        for i in [3usize, 60, 111, 170, 231, 290, 344, 401] {
            llrs[i] = -llrs[i];
        }
        let decoded = viterbi_decode(&llrs, CodeRate::R1_2, info.len());
        assert_eq!(decoded, info);
    }

    #[test]
    fn soft_weighting_beats_hard_ties() {
        // A weak (low |LLR|) wrong bit among strong right bits still decodes.
        let info = with_tail(rand_bits(64, 5));
        let coded = encode(&info, CodeRate::R1_2);
        let mut llrs = hard_llrs(&coded).iter().map(|x| x * 4.0).collect::<Vec<_>>();
        llrs[10] = 0.3 * -llrs[10].signum(); // weak wrong evidence
        let decoded = viterbi_decode(&llrs, CodeRate::R1_2, info.len());
        assert_eq!(decoded, info);
    }
}
