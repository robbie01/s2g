//! Pilot subcarriers for 2 MHz [23.3.9.10; digest pilots-mcs.md].
//!
//! Fixed pilots at k ∈ {−21, −7, +7, +21}. Data symbol n (0-based within the
//! Data field) carries value p_{n+2} · Ψ_{(n+l) mod 4} on pilot l, with
//! Ψ = [1, 1, 1, −1] [Eq 21-91; Table 19-19 N_STS=1] and p the 127-element
//! polarity sequence [Eq 17-25]. SIG symbol n ∈ {0, 1} carries p_n · P_k
//! with P = [1, 1, 1, −1] (Clause-17 fixed values) [Eq 23-18 → 17.3.5.10].

use crate::Complex32;

/// Pilot subcarrier indices for 2 MHz, ascending (pilot l = 0..3).
pub const PILOT_INDICES: [i32; 4] = [-21, -7, 7, 21];

/// Ψ row for N_STS = 1 [Table 19-19].
pub const PSI: [f32; 4] = [1.0, 1.0, 1.0, -1.0];

/// Clause-17 fixed pilot values P_k at {−21,−7,+7,+21} (SIG field) [Eq 17-24].
pub const SIG_PILOT_VALUES: [f32; 4] = [1.0, 1.0, 1.0, -1.0];

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

/// Pilot values for Data symbol `n` (0-based within the Data field), in
/// `PILOT_INDICES` order. Includes the p_{n+2} polarity (the two SIG symbols
/// consume p_0 and p_1) [Eq 23-55].
pub fn data_pilots(n: usize) -> [Complex32; 4] {
    let pol = polarity(n + 2);
    core::array::from_fn(|l| Complex32::new(pol * PSI[(n + l) % 4], 0.0))
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
        assert_eq!(data_pilots(0).map(|c| c.re), [1.0, 1.0, 1.0, -1.0]);
        // n = 1: p_3 = +1, offset 1 ⇒ {Ψ1,Ψ2,Ψ3,Ψ0} = {1,1,−1,1}.
        assert_eq!(data_pilots(1).map(|c| c.re), [1.0, 1.0, -1.0, 1.0]);
        // n = 2: p_4 = −1, offset 2 ⇒ −{1,−1,1,1} = {−1,1,−1,−1}.
        assert_eq!(data_pilots(2).map(|c| c.re), [-1.0, 1.0, -1.0, -1.0]);
    }
}
