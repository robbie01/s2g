//! LDPC coding for S1G [23.3.9.4.4 → 19.3.11.7; matrices from Annex F].
//!
//! The codes are the 802.11n/ac quasi-cyclic LDPC codes: three block lengths
//! (648 / 1296 / 1944) × four rates (1/2, 2/3, 3/4, 5/6), each parity-check
//! matrix built from Z×Z cyclic-permutation blocks (Z = 27 / 54 / 81). The
//! parity part has the dual-diagonal structure that allows linear-time
//! systematic encoding. Decoding is normalized min-sum with layered
//! (block-row) scheduling and early termination.
//!
//! The PPDU-level process (codeword length/count selection, shortening,
//! puncturing, repetition: 19.3.11.7.5 with the S1G N_pld/N_avbits
//! substitutions of 23.3.9.4.4) lives in [`PpduParams`]; `encode_data` /
//! `decode_data` turn N_pld scrambled data bits into N_avbits coded bits and
//! back. No BCC interleaver is used for LDPC; the constellation points are
//! LDPC tone-mapped instead (`ofdm::ldpc_tone_map`).
//!
//! Support for LDPC is optional in the standard [4.3.14.1].

use crate::params::CodeRate;
use std::sync::OnceLock;

/// Codeword block lengths [Table 19-15].
pub const BLOCK_LENGTHS: [usize; 3] = [648, 1296, 1944];

// Annex F matrix prototypes, transcribed from IEEE 802.11-2024 Tables
// F-1..F-3 (raw text extraction). "-" = null Z×Z block, integer i = cyclic
// permutation P_i (identity with columns shifted right by i).
const F1_R12: &str = "\
0 - - - 0 0 - - 0 - - 0 1 0 - - - - - - - - - -
22 0 - - 17 - 0 0 12 - - - - 0 0 - - - - - - - - -
6 - 0 - 10 - - - 24 - 0 - - - 0 0 - - - - - - - -
2 - - 0 20 - - - 25 0 - - - - - 0 0 - - - - - - -
23 - - - 3 - - - 0 - 9 11 - - - - 0 0 - - - - - -
24 - 23 1 17 - 3 - 10 - - - - - - - - 0 0 - - - - -
25 - - - 8 - - - 7 18 - - 0 - - - - - 0 0 - - - -
13 24 - - 0 - 8 - 6 - - - - - - - - - - 0 0 - - -
7 20 - 16 22 10 - - 23 - - - - - - - - - - - 0 0 - -
11 - - - 19 - - - 13 - 3 17 - - - - - - - - - 0 0 -
25 - 8 - 23 18 - 14 9 - - - - - - - - - - - - - 0 0
3 - - - 16 - - 2 25 5 - - 1 - - - - - - - - - - 0";

const F1_R23: &str = "\
25 26 14 - 20 - 2 - 4 - - 8 - 16 - 18 1 0 - - - - - -
10 9 15 11 - 0 - 1 - - 18 - 8 - 10 - - 0 0 - - - - -
16 2 20 26 21 - 6 - 1 26 - 7 - - - - - - 0 0 - - - -
10 13 5 0 - 3 - 7 - - 26 - - 13 - 16 - - - 0 0 - - -
23 14 24 - 12 - 19 - 17 - - - 20 - 21 - 0 - - - 0 0 - -
6 22 9 20 - 25 - 17 - 8 - 14 - 18 - - - - - - - 0 0 -
14 23 21 11 20 - 24 - 18 - 19 - - - - 22 - - - - - - 0 0
17 11 11 20 - 21 - 26 - 3 - - 18 - 26 - 1 - - - - - - 0";

const F1_R34: &str = "\
16 17 22 24 9 3 14 - 4 2 7 - 26 - 2 - 21 - 1 0 - - - -
25 12 12 3 3 26 6 21 - 15 22 - 15 - 4 - - 16 - 0 0 - - -
25 18 26 16 22 23 9 - 0 - 4 - 4 - 8 23 11 - - - 0 0 - -
9 7 0 1 17 - - 7 3 - 3 23 - 16 - - 21 - 0 - - 0 0 -
24 5 26 7 1 - - 15 24 15 - 8 - 13 - 13 - 11 - - - - 0 0
2 2 19 14 24 1 15 19 - 21 - 2 - 24 - 3 - 2 1 - - - - 0";

const F1_R56: &str = "\
17 13 8 21 9 3 18 12 10 0 4 15 19 2 5 10 26 19 13 13 1 0 - -
3 12 11 14 11 25 5 18 0 9 2 26 26 10 24 7 14 20 4 2 - 0 0 -
22 16 4 3 10 21 12 5 21 14 19 5 - 8 5 18 11 5 5 15 0 - 0 0
7 7 14 14 4 16 16 24 24 10 1 7 15 6 10 26 8 18 21 14 1 - - 0";

const F2_R12: &str = "\
40 - - - 22 - 49 23 43 - - - 1 0 - - - - - - - - - -
50 1 - - 48 35 - - 13 - 30 - - 0 0 - - - - - - - - -
39 50 - - 4 - 2 - - - - 49 - - 0 0 - - - - - - - -
33 - - 38 37 - - 4 1 - - - - - - 0 0 - - - - - - -
45 - - - 0 22 - - 20 42 - - - - - - 0 0 - - - - - -
51 - - 48 35 - - - 44 - 18 - - - - - - 0 0 - - - - -
47 11 - - - 17 - - 51 - - - 0 - - - - - 0 0 - - - -
5 - 25 - 6 - 45 - 13 40 - - - - - - - - - 0 0 - - -
33 - - 34 24 - - - 23 - - 46 - - - - - - - - 0 0 - -
1 - 27 - 1 - - - 38 - 44 - - - - - - - - - - 0 0 -
- 18 - - 23 - - 8 0 35 - - - - - - - - - - - - 0 0
49 - 17 - 30 - - - 34 - - 19 1 - - - - - - - - - - 0";

const F2_R23: &str = "\
39 31 22 43 - 40 4 - 11 - - 50 - - - 6 1 0 - - - - - -
25 52 41 2 6 - 14 - 34 - - - 24 - 37 - - 0 0 - - - - -
43 31 29 0 21 - 28 - - 2 - - 7 - 17 - - - 0 0 - - - -
20 33 48 - 4 13 - 26 - - 22 - - 46 42 - - - - 0 0 - - -
45 7 18 51 12 25 - - - 50 - - 5 - - - 0 - - - 0 0 - -
35 40 32 16 5 - - 18 - - 43 51 - 32 - - - - - - - 0 0 -
9 24 13 22 28 - - 37 - - 25 - - 52 - 13 - - - - - - 0 0
32 22 4 21 16 - - - 27 28 - 38 - - - 8 1 - - - - - - 0";

const F2_R34: &str = "\
39 40 51 41 3 29 8 36 - 14 - 6 - 33 - 11 - 4 1 0 - - - -
48 21 47 9 48 35 51 - 38 - 28 - 34 - 50 - 50 - - 0 0 - - -
30 39 28 42 50 39 5 17 - 6 - 18 - 20 - 15 - 40 - - 0 0 - -
29 0 1 43 36 30 47 - 49 - 47 - 3 - 35 - 34 - 0 - - 0 0 -
1 32 11 23 10 44 12 7 - 48 - 4 - 9 - 17 - 16 - - - - 0 0
13 7 15 47 23 16 47 - 43 - 29 - 52 - 2 - 53 - 1 - - - - 0";

const F2_R56: &str = "\
48 29 37 52 2 16 6 14 53 31 34 5 18 42 53 31 45 - 46 52 1 0 - -
17 4 30 7 43 11 24 6 14 21 6 39 17 40 47 7 15 41 19 - - 0 0 -
7 2 51 31 46 23 16 11 53 40 10 7 46 53 33 35 - 25 35 38 0 - 0 0
19 48 41 1 10 7 36 47 5 29 52 52 31 10 26 6 3 2 - 51 1 - - 0";

const F3_R12: &str = "\
57 - - - 50 - 11 - 50 - 79 - 1 0 - - - - - - - - - -
3 - 28 - 0 - - - 55 7 - - - 0 0 - - - - - - - - -
30 - - - 24 37 - - 56 14 - - - - 0 0 - - - - - - - -
62 53 - - 53 - - 3 35 - - - - - - 0 0 - - - - - - -
40 - - 20 66 - - 22 28 - - - - - - - 0 0 - - - - - -
0 - - - 8 - 42 - 50 - - 8 - - - - - 0 0 - - - - -
69 79 79 - - - 56 - 52 - - - 0 - - - - - 0 0 - - - -
65 - - - 38 57 - - 72 - 27 - - - - - - - - 0 0 - - -
64 - - - 14 52 - - 30 - - 32 - - - - - - - - 0 0 - -
- 45 - 70 0 - - - 77 9 - - - - - - - - - - - 0 0 -
2 56 - 57 35 - - - - - 12 - - - - - - - - - - - 0 0
24 - 61 - 60 - - 27 51 - - 16 1 - - - - - - - - - - 0";

const F3_R23: &str = "\
61 75 4 63 56 - - - - - - 8 - 2 17 25 1 0 - - - - - -
56 74 77 20 - - - 64 24 4 67 - 7 - - - - 0 0 - - - - -
28 21 68 10 7 14 65 - - - 23 - - - 75 - - - 0 0 - - - -
48 38 43 78 76 - - - - 5 36 - 15 72 - - - - - 0 0 - - -
40 2 53 25 - 52 62 - 20 - - 44 - - - - 0 - - - 0 0 - -
69 23 64 10 22 - 21 - - - - - 68 23 29 - - - - - - 0 0 -
12 0 68 20 55 61 - 40 - - - 52 - - - 44 - - - - - - 0 0
58 8 34 64 78 - - 11 78 24 - - - - - 58 1 - - - - - - 0";

const F3_R34: &str = "\
48 29 28 39 9 61 - - - 63 45 80 - - - 37 32 22 1 0 - - - -
4 49 42 48 11 30 - - - 49 17 41 37 15 - 54 - - - 0 0 - - -
35 76 78 51 37 35 21 - 17 64 - - - 59 7 - - 32 - - 0 0 - -
9 65 44 9 54 56 73 34 42 - - - 35 - - - 46 39 0 - - 0 0 -
3 62 7 80 68 26 - 80 55 - 36 - 26 - 9 - 72 - - - - - 0 0
26 75 33 21 69 59 3 38 - - - 35 - 62 36 26 - - 1 - - - - 0";

const F3_R56: &str = "\
13 48 80 66 4 74 7 30 76 52 37 60 - 49 73 31 74 73 23 - 1 0 - -
69 63 74 56 64 77 57 65 6 16 51 - 64 - 68 9 48 62 54 27 - 0 0 -
51 15 0 80 24 25 42 54 44 71 71 9 67 35 - 58 - 29 - 53 0 - 0 0
16 29 36 41 44 56 59 37 50 24 - 65 4 65 52 - 4 - 73 52 1 - - 0";

fn prototype_text(n: usize, rate: CodeRate) -> &'static str {
    match (n, rate) {
        (648, CodeRate::R1_2) => F1_R12,
        (648, CodeRate::R2_3) => F1_R23,
        (648, CodeRate::R3_4) => F1_R34,
        (648, CodeRate::R5_6) => F1_R56,
        (1296, CodeRate::R1_2) => F2_R12,
        (1296, CodeRate::R2_3) => F2_R23,
        (1296, CodeRate::R3_4) => F2_R34,
        (1296, CodeRate::R5_6) => F2_R56,
        (1944, CodeRate::R1_2) => F3_R12,
        (1944, CodeRate::R2_3) => F3_R23,
        (1944, CodeRate::R3_4) => F3_R34,
        (1944, CodeRate::R5_6) => F3_R56,
        _ => panic!("no LDPC code for n={n} {rate:?}"),
    }
}

/// One of the twelve codes: expanded parity-check structure plus the block
/// prototype used by the encoder.
pub struct Code {
    pub n: usize,
    pub k: usize,
    pub z: usize,
    pub rate: CodeRate,
    /// Prototype: `proto[block_row][block_col]` = shift or −1.
    proto: Vec<Vec<i16>>,
    /// Expanded check-node adjacency: `checks[c]` = variable indices.
    checks: Vec<Vec<u32>>,
}

impl Code {
    fn build(n: usize, rate: CodeRate) -> Code {
        let z = n / 24;
        let proto: Vec<Vec<i16>> = prototype_text(n, rate)
            .lines()
            .map(|l| {
                let row: Vec<i16> = l
                    .split_whitespace()
                    .map(|t| if t == "-" { -1 } else { t.parse::<i16>().expect("prototype entry") })
                    .collect();
                assert_eq!(row.len(), 24, "prototype row width");
                row
            })
            .collect();
        let m_blocks = proto.len();
        let k = n - m_blocks * z;
        let (num, den) = rate.as_fraction();
        assert_eq!(k * den, n * num, "prototype row count vs rate");
        let mut checks = vec![Vec::new(); m_blocks * z];
        for (bi, row) in proto.iter().enumerate() {
            for (bj, &s) in row.iter().enumerate() {
                if s < 0 {
                    continue;
                }
                for r in 0..z {
                    // P_s has a 1 at (r, (r + s) mod z).
                    let col = bj * z + (r + s as usize) % z;
                    checks[bi * z + r].push(col as u32);
                }
            }
        }
        Code { n, k, z, rate, proto, checks }
    }

    /// Shared instance for (n, rate).
    pub fn get(n: usize, rate: CodeRate) -> &'static Code {
        static CODES: OnceLock<Vec<Code>> = OnceLock::new();
        let all = CODES.get_or_init(|| {
            let mut v = Vec::new();
            for &n in &BLOCK_LENGTHS {
                for r in [CodeRate::R1_2, CodeRate::R2_3, CodeRate::R3_4, CodeRate::R5_6] {
                    v.push(Code::build(n, r));
                }
            }
            v
        });
        all.iter().find(|c| c.n == n && c.rate == rate).expect("valid LDPC code")
    }

    /// Number of parity-check equations.
    pub fn m(&self) -> usize {
        self.n - self.k
    }

    /// Systematic encoding of `k` information bits → `n` codeword bits
    /// (information first), using the dual-diagonal parity structure:
    /// p0 = Σ_i A_i·s, then p_{i+1} = A_i·s + Σ_{j≤i} B_{i,j}·p_j.
    pub fn encode(&self, info: &[u8]) -> Vec<u8> {
        assert_eq!(info.len(), self.k);
        let z = self.z;
        let kb = self.k / z;
        let mb = self.m() / z;
        // A_i · s for every block row, as z-bit vectors.
        let a_s: Vec<Vec<u8>> = (0..mb)
            .map(|bi| {
                let mut acc = vec![0u8; z];
                for bj in 0..kb {
                    let s = self.proto[bi][bj];
                    if s < 0 {
                        continue;
                    }
                    let blk = &info[bj * z..(bj + 1) * z];
                    for r in 0..z {
                        acc[r] ^= blk[(r + s as usize) % z];
                    }
                }
                acc
            })
            .collect();
        let mut parity: Vec<Vec<u8>> = vec![vec![0u8; z]; mb];
        // p0 = Σ_i A_i s (column 0 of the parity part sums to the identity,
        // the dual-diagonal columns sum to zero).
        for a in &a_s {
            for r in 0..z {
                parity[0][r] ^= a[r];
            }
        }
        for bi in 0..mb - 1 {
            let mut acc = a_s[bi].clone();
            for (bj, p) in parity.iter().enumerate().take(bi + 1) {
                let s = self.proto[bi][kb + bj];
                if s < 0 {
                    continue;
                }
                for r in 0..z {
                    acc[r] ^= p[(r + s as usize) % z];
                }
            }
            parity[bi + 1] = acc;
        }
        let mut out = Vec::with_capacity(self.n);
        out.extend_from_slice(info);
        for p in parity {
            out.extend_from_slice(&p);
        }
        out
    }

    /// Syndrome check: true if every parity equation holds.
    pub fn is_codeword(&self, bits: &[u8]) -> bool {
        self.checks.iter().all(|c| c.iter().fold(0u8, |acc, &v| acc ^ bits[v as usize]) == 0)
    }

    /// Normalized min-sum decoding with layered scheduling. `llrs.len() ==
    /// n`, convention LLR > 0 ⇒ bit 0. Returns hard decisions and whether
    /// the syndrome converged to zero.
    pub fn decode(&self, llrs: &[f32], max_iter: usize) -> (Vec<u8>, bool) {
        assert_eq!(llrs.len(), self.n);
        const ALPHA: f32 = 0.8;
        let mut post: Vec<f32> = llrs.to_vec();
        // Check-to-variable messages, flat per check in adjacency order.
        let offsets: Vec<usize> = {
            let mut o = Vec::with_capacity(self.checks.len() + 1);
            let mut acc = 0;
            for c in &self.checks {
                o.push(acc);
                acc += c.len();
            }
            o.push(acc);
            o
        };
        let mut r = vec![0.0f32; *offsets.last().unwrap()];
        let mut q = Vec::with_capacity(32);
        let hard = |post: &[f32]| -> Vec<u8> { post.iter().map(|&l| if l < 0.0 { 1 } else { 0 }).collect() };
        for _ in 0..max_iter {
            for (c, vars) in self.checks.iter().enumerate() {
                let base = offsets[c];
                q.clear();
                let mut min1 = f32::INFINITY;
                let mut min2 = f32::INFINITY;
                let mut min_idx = 0usize;
                let mut sign = 1.0f32;
                for (i, &v) in vars.iter().enumerate() {
                    let qi = post[v as usize] - r[base + i];
                    q.push(qi);
                    let a = qi.abs();
                    if a < min1 {
                        min2 = min1;
                        min1 = a;
                        min_idx = i;
                    } else if a < min2 {
                        min2 = a;
                    }
                    if qi < 0.0 {
                        sign = -sign;
                    }
                }
                for (i, &v) in vars.iter().enumerate() {
                    let qi = q[i];
                    let mag = if i == min_idx { min2 } else { min1 };
                    let s = if qi < 0.0 { -sign } else { sign };
                    let new_r = ALPHA * s * mag;
                    r[base + i] = new_r;
                    post[v as usize] = qi + new_r;
                }
            }
            let h = hard(&post);
            if self.is_codeword(&h) {
                return (h, true);
            }
        }
        let h = hard(&post);
        let ok = self.is_codeword(&h);
        (h, ok)
    }
}

/// PPDU-level LDPC parameters [19.3.11.7.5 with 23.3.9.4.4 substitutions].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PpduParams {
    pub rate: CodeRate,
    /// Bits to encode: N_SYM,init · N_DBPS (SERVICE + PSDU + pad, no tail).
    pub n_pld: usize,
    /// Coded bits available: N_SYM · N_CBPS (after any extra symbol).
    pub n_avbits: usize,
    pub n_cw: usize,
    pub l_ldpc: usize,
    pub n_shrt: usize,
    pub n_punc: usize,
    pub n_rep: usize,
    /// Final symbol count (N_SYM,init or N_SYM,init + 1).
    pub n_sym: usize,
    /// LDPC Extra OFDM Symbol flag.
    pub extra_symbol: bool,
}

impl PpduParams {
    /// Compute the encoding parameters for an SU PPDU with `n_sym_init`
    /// initial symbols (Eq 23-46/47), no STBC.
    pub fn new(n_sym_init: usize, n_dbps: usize, n_cbps: usize, rate: CodeRate) -> PpduParams {
        let r = rate.as_f64();
        let n_pld = n_sym_init * n_dbps; // Eq 23-45
        let mut n_avbits = n_sym_init * n_cbps; // Eq 23-48
        // Table 19-16.
        let (n_cw, l_ldpc) = if n_avbits <= 648 {
            (1, if n_avbits as f64 >= n_pld as f64 + 912.0 * (1.0 - r) { 1296 } else { 648 })
        } else if n_avbits <= 1296 {
            (1, if n_avbits as f64 >= n_pld as f64 + 1464.0 * (1.0 - r) { 1944 } else { 1296 })
        } else if n_avbits <= 1944 {
            (1, 1944)
        } else if n_avbits <= 2592 {
            (2, if n_avbits as f64 >= n_pld as f64 + 2916.0 * (1.0 - r) { 1944 } else { 1296 })
        } else {
            ((n_pld as f64 / (1944.0 * r)).ceil() as usize, 1944)
        };
        let k_total = (n_cw * l_ldpc) as f64 * r;
        let n_shrt = (k_total - n_pld as f64).max(0.0).round() as usize; // Eq 19-37
        let mut n_punc = (n_cw * l_ldpc).saturating_sub(n_avbits + n_shrt); // Eq 19-38
        let parity_total = (n_cw * l_ldpc) as f64 * (1.0 - r);
        let mut extra_symbol = false;
        if (n_punc as f64 > 0.1 * parity_total && (n_shrt as f64) < 1.2 * n_punc as f64 * r / (1.0 - r))
            || n_punc as f64 > 0.3 * parity_total
        {
            n_avbits += n_cbps; // Eq 19-39 (m_STBC = 1)
            n_punc = (n_cw * l_ldpc).saturating_sub(n_avbits + n_shrt); // Eq 19-40
            extra_symbol = true;
        }
        let n_sym = n_avbits / n_cbps; // Eq 19-41
        let n_rep = (n_avbits as f64 - parity_total - n_pld as f64).max(0.0).round() as usize; // Eq 19-42
        PpduParams { rate, n_pld, n_avbits, n_cw, l_ldpc, n_shrt, n_punc, n_rep, n_sym, extra_symbol }
    }

    fn code(&self) -> &'static Code {
        Code::get(self.l_ldpc, self.rate)
    }

    /// Per-codeword (shortening, puncturing, repetition) counts: the first
    /// `N mod N_CW` codewords take one more.
    fn per_codeword(&self, j: usize) -> (usize, usize, usize) {
        let split = |total: usize| total / self.n_cw + usize::from(j < total % self.n_cw);
        (split(self.n_shrt), split(self.n_punc), split(self.n_rep))
    }

    /// Number of transmitted bits of codeword `j`.
    fn tx_len(&self, j: usize) -> usize {
        let (s, p, r) = self.per_codeword(j);
        self.l_ldpc - s - p + r
    }

    /// Encode `n_pld` scrambled data bits into `n_avbits` coded bits
    /// (codewords concatenated, i0 first) [19.3.11.7.5 steps c–g].
    pub fn encode_data(&self, data: &[u8]) -> Vec<u8> {
        assert_eq!(data.len(), self.n_pld);
        let code = self.code();
        let k = code.k;
        let mut out = Vec::with_capacity(self.n_avbits);
        let mut pos = 0;
        for j in 0..self.n_cw {
            let (shrt, punc, rep) = self.per_codeword(j);
            let n_info = k - shrt;
            let mut info = data[pos..pos + n_info].to_vec();
            pos += n_info;
            info.resize(k, 0); // shortening: last info bits are zero
            let cw = code.encode(&info);
            let mut tx: Vec<u8> = Vec::with_capacity(self.tx_len(j));
            tx.extend_from_slice(&cw[..n_info]); // shortened bits discarded
            let n_parity = code.m() - punc; // last parity bits punctured
            tx.extend_from_slice(&cw[k..k + n_parity]);
            // Repetition: copy from the shortened codeword from i0 onward,
            // wrapping if necessary.
            let base_len = tx.len();
            for i in 0..rep {
                let b = tx[i % base_len];
                tx.push(b);
            }
            out.extend_from_slice(&tx);
        }
        debug_assert_eq!(pos, self.n_pld);
        debug_assert_eq!(out.len(), self.n_avbits);
        out
    }

    /// Decode `n_avbits` LLRs (LLR > 0 ⇒ 0) into `n_pld` data bits. Returns
    /// the bits and the number of codewords whose syndrome did not converge.
    pub fn decode_data(&self, llrs: &[f32], max_iter: usize) -> (Vec<u8>, usize) {
        assert_eq!(llrs.len(), self.n_avbits);
        let code = self.code();
        let k = code.k;
        let mut out = Vec::with_capacity(self.n_pld);
        let mut failures = 0;
        let mut pos = 0;
        for j in 0..self.n_cw {
            let (shrt, punc, rep) = self.per_codeword(j);
            let n_info = k - shrt;
            let n_parity = code.m() - punc;
            let base_len = n_info + n_parity;
            let rx = &llrs[pos..pos + base_len + rep];
            pos += base_len + rep;
            // Shortened bits are known zeros: give them an LLR that dominates
            // every received one (the LLR scale is arbitrary, since it
            // depends on the CSI weighting, so derive it from the data).
            let max_abs = rx.iter().fold(0.0f32, |m, &v| m.max(v.abs()));
            let known_zero = (16.0 * max_abs).max(1.0e3);
            let mut l = vec![0.0f32; code.n];
            l[..n_info].copy_from_slice(&rx[..n_info]);
            l[n_info..k].fill(known_zero);
            l[k..k + n_parity].copy_from_slice(&rx[n_info..base_len]);
            // Punctured parity: no information (already 0).
            // Repeated bits: combine with their source positions.
            for i in 0..rep {
                let src = i % base_len;
                let idx = if src < n_info { src } else { k + (src - n_info) };
                l[idx] += rx[base_len + i];
            }
            let (bits, ok) = code.decode(&l, max_iter);
            if !ok {
                failures += 1;
            }
            out.extend_from_slice(&bits[..n_info]);
        }
        debug_assert_eq!(out.len(), self.n_pld);
        (out, failures)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const RATES: [CodeRate; 4] = [CodeRate::R1_2, CodeRate::R2_3, CodeRate::R3_4, CodeRate::R5_6];

    fn rand_bits(n: usize, seed: u64) -> Vec<u8> {
        let mut s = seed;
        (0..n)
            .map(|_| {
                s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                ((s >> 33) & 1) as u8
            })
            .collect()
    }

    #[test]
    fn all_twelve_matrices_build_and_encode_valid_codewords() {
        for &n in &BLOCK_LENGTHS {
            for r in RATES {
                let c = Code::get(n, r);
                assert_eq!(c.n, n);
                assert_eq!(c.k * r.as_fraction().1, n * r.as_fraction().0);
                for seed in 1..4u64 {
                    let info = rand_bits(c.k, seed * 7 + n as u64);
                    let cw = c.encode(&info);
                    assert_eq!(cw.len(), n);
                    assert_eq!(&cw[..c.k], &info[..]);
                    assert!(c.is_codeword(&cw), "n {n} {r:?} seed {seed}: H·c ≠ 0");
                    // Flip a bit → not a codeword (min distance > 1).
                    let mut bad = cw.clone();
                    bad[5] ^= 1;
                    assert!(!c.is_codeword(&bad));
                }
            }
        }
    }

    #[test]
    fn decoder_corrects_errors() {
        for &n in &BLOCK_LENGTHS {
            for r in RATES {
                let c = Code::get(n, r);
                let info = rand_bits(c.k, 99 + n as u64);
                let cw = c.encode(&info);
                // Flip a fraction of the bits scaled with the parity budget
                // (hard-decision-like LLRs are the decoder's worst case).
                let flips = (n - c.k) / 16;
                let mut llrs: Vec<f32> = cw.iter().map(|&b| if b == 0 { 2.0 } else { -2.0 }).collect();
                let idx = rand_bits(n * 4, 5 + n as u64);
                let mut done = 0;
                let mut i = 0;
                while done < flips {
                    let pos = (i * 7919 + idx[i % idx.len()] as usize * 31) % n;
                    llrs[pos] = -llrs[pos];
                    done += 1;
                    i += 1;
                }
                let (bits, ok) = c.decode(&llrs, 40);
                assert!(ok, "n {n} {r:?} did not converge");
                assert_eq!(&bits[..c.k], &info[..], "n {n} {r:?}");
            }
        }
    }

    #[test]
    fn ppdu_params_examples() {
        // MCS 0 (26/52), 100 octets: N_SYM,init = ceil(808/26) = 32,
        // N_pld = 832, N_avbits = 1664 → one 1944 codeword, R=1/2:
        // N_shrt = 972−832 = 140, N_punc = 1944−1664−140 = 140;
        // 140 > 0.1·972 = 97.2 and N_shrt 140 < 1.2·140·1 = 168 → extra
        // symbol: N_avbits = 1716, N_punc = 88.
        let p = PpduParams::new(32, 26, 52, CodeRate::R1_2);
        assert_eq!(p.n_pld, 832);
        assert_eq!(p.n_cw, 1);
        assert_eq!(p.l_ldpc, 1944);
        assert_eq!(p.n_shrt, 140);
        assert!(p.extra_symbol);
        assert_eq!(p.n_sym, 33);
        assert_eq!(p.n_avbits, 1716);
        assert_eq!(p.n_punc, 88);
        assert_eq!(p.n_rep, 0);
        // Small PPDU: MCS 7 (260/312), 20 octets → N_SYM,init = 1, N_pld =
        // 260, N_avbits = 312 ≤ 648; 312 ≥ 260 + 912/6 = 412? no → L=648.
        // N_shrt = 540−260 = 280, N_punc = 648−312−280 = 56 ≤ 0.1·108 = 10.8?
        // no: 56 > 10.8 and N_shrt 280 < 1.2·56·5 = 336 → extra symbol.
        let p = PpduParams::new(1, 260, 312, CodeRate::R5_6);
        assert_eq!(p.l_ldpc, 648);
        assert!(p.extra_symbol);
        assert_eq!(p.n_sym, 2);
        assert_eq!(p.n_avbits, 624);
        assert_eq!(p.n_punc, 0);
        assert_eq!(p.n_rep, 624 - 108 - 260);
        // Large aggregated PPDU: 200 symbols at MCS 4 (156/208).
        let p = PpduParams::new(200, 156, 208, CodeRate::R3_4);
        assert_eq!(p.n_pld, 31200);
        assert_eq!(p.n_cw, (31200.0f64 / 1458.0).ceil() as usize);
        assert_eq!(p.l_ldpc, 1944);
    }

    #[test]
    fn ppdu_encode_decode_roundtrip_all_regimes() {
        let cases = [
            (1usize, 26usize, 52usize, CodeRate::R1_2),
            (5, 26, 52, CodeRate::R1_2),
            (32, 26, 52, CodeRate::R1_2),
            (1, 260, 312, CodeRate::R5_6),
            (3, 208, 312, CodeRate::R2_3),
            (7, 78, 104, CodeRate::R3_4),
            (40, 156, 208, CodeRate::R3_4),
            (200, 312, 416, CodeRate::R3_4),
            (511, 390, 520, CodeRate::R3_4),
        ];
        for (n_init, n_dbps, n_cbps, rate) in cases {
            let p = PpduParams::new(n_init, n_dbps, n_cbps, rate);
            let data = rand_bits(p.n_pld, n_init as u64 * 13 + n_dbps as u64);
            let coded = p.encode_data(&data);
            assert_eq!(coded.len(), p.n_avbits);
            assert_eq!(p.n_avbits, p.n_sym * n_cbps);
            // Clean channel plus a sprinkle of errors.
            let mut llrs: Vec<f32> = coded.iter().map(|&b| if b == 0 { 3.0 } else { -3.0 }).collect();
            for i in (0..llrs.len()).step_by(97) {
                llrs[i] = -llrs[i] * 0.5;
            }
            let (back, fails) = p.decode_data(&llrs, 40);
            assert_eq!(fails, 0, "{p:?}");
            assert_eq!(back, data, "{p:?}");
        }
    }
}
