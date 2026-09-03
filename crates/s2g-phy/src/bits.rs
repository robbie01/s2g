//! Bit-vector helpers.
//!
//! Convention used throughout the crate: bits travel as `u8` values 0/1 in
//! `Vec<u8>`/`&[u8]`. Bytes are serialized **LSB first** (802.11 transmit
//! order): `byte_to_bits(0x01)` yields `[1,0,0,0,0,0,0,0]`.
//!
//! Soft bits (LLRs) are `f32` with the convention **LLR > 0 ⇒ bit = 0**.

/// Expand bytes to bits, LSB of each byte first.
pub fn bytes_to_bits(bytes: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(bytes.len() * 8);
    for &b in bytes {
        for i in 0..8 {
            out.push((b >> i) & 1);
        }
    }
    out
}

/// Pack bits (LSB-first per byte) back into bytes. `bits.len()` must be a
/// multiple of 8.
pub fn bits_to_bytes(bits: &[u8]) -> Vec<u8> {
    debug_assert!(bits.len().is_multiple_of(8));
    bits.chunks(8)
        .map(|c| c.iter().enumerate().fold(0u8, |acc, (i, &b)| acc | ((b & 1) << i)))
        .collect()
}

/// Interpret the first `n` bits (as produced by [`bytes_to_bits`]) as an
/// unsigned integer, first bit = LSB.
pub fn bits_to_uint_lsb_first(bits: &[u8]) -> u64 {
    bits.iter().enumerate().fold(0u64, |acc, (i, &b)| acc | ((b as u64 & 1) << i))
}

/// Write `n` bits of `value` into a bit vector, LSB first.
pub fn push_uint_lsb_first(out: &mut Vec<u8>, value: u64, n: usize) {
    for i in 0..n {
        out.push(((value >> i) & 1) as u8);
    }
}
