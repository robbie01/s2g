//! Frame-synchronous data scrambler, x^7 + x^4 + 1 [23.3.9.3 → 17.3.5.5].
//!
//! The scrambling sequence's first 7 bits equal the seed's bits LSB-first;
//! thereafter s[i] = s[i-4] ^ s[i-7]. Scrambling and descrambling are the
//! same XOR. Because the S1G SERVICE field's first 7 bits are transmitted as
//! zero [23.3.9.2], the first 7 *scrambled* bits on air are exactly the
//! sequence bits; that is how a receiver recovers the seed.

/// Generate `len` bits of the scrambling sequence for `seed` (1..=127; only
/// the low 7 bits are used). A zero seed is forbidden on transmit
/// [17.3.5.5], but some chips send it; it yields the all-zero
/// sequence (no scrambling), which is what a receiver must apply.
pub fn sequence(seed: u8, len: usize) -> Vec<u8> {
    let mut s = Vec::with_capacity(len.max(7));
    for i in 0..7.min(len) {
        s.push((seed >> i) & 1);
    }
    for i in 7..len {
        let b = s[i - 4] ^ s[i - 7];
        s.push(b);
    }
    s.truncate(len);
    s
}

/// Scramble/descramble `bits` in place with the sequence for `seed`.
pub fn scramble_in_place(seed: u8, bits: &mut [u8]) {
    let seq = sequence(seed, bits.len());
    for (b, s) in bits.iter_mut().zip(seq) {
        *b ^= s;
    }
}

/// Recover the seed from the first 7 scrambled bits of the SERVICE field
/// (valid because SERVICE B0–B6 are zero before scrambling). A zero result
/// means the transmitter used the (illegal but observed in the wild)
/// all-zero state, i.e. no scrambling.
pub fn recover_seed(first7: &[u8]) -> u8 {
    debug_assert!(first7.len() >= 7);
    first7[..7]
        .iter()
        .enumerate()
        .fold(0u8, |acc, (i, &b)| acc | ((b & 1) << i))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// NOTE 1 of 17.3.5.5 [p3355]: seed 112 (first 7 bits 0000111) produces
    /// this 127-bit repeating sequence.
    #[test]
    fn golden_sequence_seed_112() {
        let expect_str = "00001110 11110010 11001001 00000010 00100110 00101110 10110110 00001100 \
                          11010100 11100111 10110100 00101010 11111010 01010001 10111000 1111111";
        let expect: Vec<u8> = expect_str
            .chars()
            .filter(|c| !c.is_whitespace())
            .map(|c| c.to_digit(2).unwrap() as u8)
            .collect();
        assert_eq!(expect.len(), 127);
        assert_eq!(sequence(112, 127), expect);
        // Cyclic with period 127:
        let long = sequence(112, 254);
        assert_eq!(&long[127..], &expect[..]);
    }

    #[test]
    fn roundtrip() {
        let data: Vec<u8> = (0..1000u32).map(|i| ((i * 7 + 3) % 2) as u8).collect();
        for seed in [1u8, 42, 112, 127] {
            let mut x = data.clone();
            scramble_in_place(seed, &mut x);
            assert_ne!(x, data);
            scramble_in_place(seed, &mut x);
            assert_eq!(x, data);
        }
    }

    #[test]
    fn seed_recovery() {
        for seed in 0..=127u8 {
            // SERVICE B0..B6 = 0 -> scrambled bits are the sequence itself.
            let mut service = vec![0u8; 7];
            scramble_in_place(seed, &mut service);
            assert_eq!(recover_seed(&service), seed);
        }
        // Seed 0: identity.
        let mut x = vec![1u8, 0, 1, 1, 0, 0, 1, 0, 1, 1];
        scramble_in_place(0, &mut x);
        assert_eq!(x, vec![1u8, 0, 1, 1, 0, 0, 1, 0, 1, 1]);
    }
}
