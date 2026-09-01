//! Pilot subcarriers for 2 MHz [23.3.9.10; digest pilots-mcs.md].
//!
//! **Fixed pilots** at k ∈ {−21, −7, +7, +21}. Data symbol n (0-based within
//! the Data field) carries value p_{n+2} · Ψ_{(n+l) mod 4} on pilot l, with
//! Ψ = [1, 1, 1, −1] [Eq 21-91; Table 19-19 N_STS=1] and p the 127-element
//! polarity sequence [Eq 17-25]. SIG symbol n ∈ {0, 1} carries p_n · P_k
//! with P = [1, 1, 1, −1] (Clause-17 fixed values) [Eq 23-18 → 17.3.5.10].
//!
//! **Traveling pilots** (TXVECTOR TRAVELING_PILOTS = 1, SU / N_STS = 1): the
//! four pilots move every symbol through the positions of Table 23-23
//! (pattern index m = n mod 14, Eq 23-52), each carrying 1.5× the value it
//! would have carried on its fixed tone [Eq 23-51]. Over one 14-symbol
//! period every one of the 56 used tones is visited exactly once, which is
//! what lets a receiver track a time-varying (Doppler) channel. The data
//! tone mapping skips whichever tones are pilots *this* symbol [Eq 23-60].

use crate::Complex32;

/// Fixed pilot subcarrier indices for 2 MHz, ascending (pilot l = 0..3).
pub const PILOT_INDICES: [i32; 4] = [-21, -7, 7, 21];

/// Ψ row for N_STS = 1 [Table 19-19].
pub const PSI: [f32; 4] = [1.0, 1.0, 1.0, -1.0];

/// Clause-17 fixed pilot values P_k at {−21,−7,+7,+21} (SIG field) [Eq 17-24].
pub const SIG_PILOT_VALUES: [f32; 4] = [1.0, 1.0, 1.0, -1.0];

/// Traveling-pilot pattern period for 2 MHz [Eq 23-52].
pub const N_TP_2MHZ: usize = 14;

/// Traveling-pilot amplitude relative to a fixed pilot [Eq 23-51].
pub const TRAVELING_PILOT_GAIN: f32 = 1.5;

/// Traveling pilot positions K_Pilot_Travel^(l)(n) for N_STS = 1, 2 MHz
/// [Table 23-23, p3799]: row = pilot index l, column = pattern index m.
pub const TRAVELING_POSITIONS_2MHZ: [[i32; N_TP_2MHZ]; 4] = [
    [-28, -24, -20, -16, -26, -22, -18, -27, -23, -19, -15, -25, -21, -17],
    [-12, -8, -4, -2, -14, -10, -6, -11, -7, -3, 1, -13, -9, -5],
    [4, 8, 12, 16, 2, 6, 10, 5, 9, 13, 17, -1, 3, 7],
    [20, 24, 28, 26, 14, 18, 22, 21, 25, 23, 27, 11, 15, 19],
];

/// 127-element pilot polarity sequence [Eq 17-25, p3366].
pub const POLARITY: [i8; 127] = [
    1, 1, 1, 1, -1, -1, -1, 1, -1, -1, -1, -1, 1, 1, -1, 1,
    -1, -1, 1, 1, -1, 1, 1, -1, 1, 1, 1, 1, 1, 1, -1, 1,
    1, 1, -1, 1, 1, -1, -1, 1, 1, 1, -1, 1, -1, -1, -1, 1,
    -1, 1, -1, -1, 1, -1, -1, 1, 1, 1, 1, 1, -1, -1, 1, 1,
    -1, -1, 1, -1, 1, -1, 1, 1, -1, -1, -1, 1, 1, -1, -1, -1,
    -1, 1, -1, -1, 1, -1, 1, 1, 1, 1, -1, 1, -1, 1, -1, 1,
    -1, -1, -1, -1, -1, 1, -1, 1, 1, -1, 1, -1, 1, 1, 1, -1,
    -1, 1, -1, -1, -1, 1, 1, 1, -1, -1, -1, -1, -1, -1, -1,
];

/// Pilot polarity p_n (cyclic).
pub fn polarity(n: usize) -> f32 {
    POLARITY[n % 127] as f32
}

/// Pilot subcarrier positions for Data symbol `n` (ascending, pilot index
/// order) — fixed, or traveling per Table 23-23.
pub fn pilot_positions(n: usize, traveling: bool) -> [i32; 4] {
    if traveling {
        let m = n % N_TP_2MHZ;
        core::array::from_fn(|l| TRAVELING_POSITIONS_2MHZ[l][m])
    } else {
        PILOT_INDICES
    }
}

/// Pilot values for Data symbol `n` (0-based within the Data field), in
/// pilot-index order. Includes the p_{n+2} polarity (the two SIG symbols
/// consume p_0 and p_1) [Eq 23-55] and the 1.5× traveling-pilot gain
/// [Eq 23-51].
pub fn data_pilots(n: usize, traveling: bool) -> [Complex32; 4] {
    let pol = polarity(n + 2);
    let gain = if traveling { TRAVELING_PILOT_GAIN } else { 1.0 };
    core::array::from_fn(|l| Complex32::new(gain * pol * PSI[(n + l) % 4], 0.0))
}

/// The 52 data subcarriers of Data symbol `n`, ascending: every used tone
/// (±1..±28) that is not a pilot this symbol [Eq 23-60, M'_2(k)].
pub fn data_subcarriers(n: usize, traveling: bool) -> [i32; 52] {
    let p = pilot_positions(n, traveling);
    let mut out = [0i32; 52];
    let mut i = 0;
    for k in -28..=28 {
        if k != 0 && !p.contains(&k) {
            out[i] = k;
            i += 1;
        }
    }
    debug_assert_eq!(i, 52);
    out
}

/// Pilot values for SIG symbol `n` (0 or 1), in `PILOT_INDICES` order.
pub fn sig_pilots(n: usize) -> [Complex32; 4] {
    debug_assert!(n < 2);
    let pol = polarity(n);
    core::array::from_fn(|l| Complex32::new(pol * SIG_PILOT_VALUES[l], 0.0))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn polarity_sequence_checks() {
        assert_eq!(POLARITY.len(), 127);
        // First 8 and last 3 per digest.
        assert_eq!(&POLARITY[..8], &[1, 1, 1, 1, -1, -1, -1, 1]);
        assert_eq!(&POLARITY[124..], &[-1, -1, -1]);
        // p is the scrambler output for the all-ones *register* state with
        // 1 → −1, 0 → +1; in our seed convention (first 7 output bits, LSB
        // first) that register state produces the seed-112 sequence.
        let seq = crate::scrambler::sequence(112, 127);
        for (i, &s) in seq.iter().enumerate() {
            let expect = if s == 1 { -1 } else { 1 };
            assert_eq!(POLARITY[i], expect, "index {i}");
        }
        // Cyclic.
        assert_eq!(polarity(127), POLARITY[0] as f32);
    }

    #[test]
    fn sig_pilots_are_constant_positive_polarity() {
        // p_0 = p_1 = +1 ⇒ both SIG symbols carry {1,1,1,−1}.
        for n in 0..2 {
            let p = sig_pilots(n);
            assert_eq!(p.map(|c| c.re), [1.0, 1.0, 1.0, -1.0]);
        }
    }

    #[test]
    fn data_pilots_rotate_psi() {
        // n = 0: p_2 = +1, Ψ rotation offset 0 ⇒ {1,1,1,−1} (matches SIG P_k).
        assert_eq!(data_pilots(0, false).map(|c| c.re), [1.0, 1.0, 1.0, -1.0]);
        // n = 1: p_3 = +1, offset 1 ⇒ {Ψ1,Ψ2,Ψ3,Ψ0} = {1,1,−1,1}.
        assert_eq!(data_pilots(1, false).map(|c| c.re), [1.0, 1.0, -1.0, 1.0]);
        // n = 2: p_4 = −1, offset 2 ⇒ −{1,−1,1,1} = {−1,1,−1,−1}.
        assert_eq!(data_pilots(2, false).map(|c| c.re), [-1.0, 1.0, -1.0, -1.0]);
        // Traveling: same values × 1.5.
        assert_eq!(data_pilots(2, true).map(|c| c.re), [-1.5, 1.5, -1.5, -1.5]);
    }

    #[test]
    fn traveling_positions_cover_every_tone_once_per_period() {
        let mut seen = std::collections::HashSet::new();
        for m in 0..N_TP_2MHZ {
            let p = pilot_positions(m, true);
            // Ascending, distinct, never DC, within ±28.
            for w in p.windows(2) {
                assert!(w[0] < w[1], "m {m}: {p:?}");
            }
            for &k in &p {
                assert!(k != 0 && (-28..=28).contains(&k));
                assert!(seen.insert(k), "tone {k} repeated within one period");
            }
        }
        assert_eq!(seen.len(), 56);
        // Pattern index wraps.
        assert_eq!(pilot_positions(14, true), pilot_positions(0, true));
        assert_eq!(pilot_positions(12, true), [-21, -9, 3, 15]);
        assert_eq!(pilot_positions(5, false), PILOT_INDICES);
    }

    #[test]
    fn data_subcarriers_complement_pilots() {
        for n in 0..20 {
            for trav in [false, true] {
                let d = data_subcarriers(n, trav);
                let p = pilot_positions(n, trav);
                let mut all: Vec<i32> = d.to_vec();
                all.extend_from_slice(&p);
                all.sort();
                let expect: Vec<i32> = (-28..=28).filter(|&k| k != 0).collect();
                assert_eq!(all, expect, "n {n} trav {trav}");
            }
        }
        assert_eq!(data_subcarriers(0, false), crate::ofdm::DATA_SUBCARRIER_INDICES);
    }
}
