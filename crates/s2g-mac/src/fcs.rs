//! Frame check sequence: IEEE CRC-32 (reflected 0xEDB88320, init/xorout
//! all-ones), appended little-endian [9.2.4.9].

pub fn crc32(data: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    for &b in data {
        crc ^= b as u32;
        for _ in 0..8 {
            let mask = (crc & 1).wrapping_neg();
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    !crc
}

/// Append the 4-byte FCS.
pub fn append(frame: &mut Vec<u8>) {
    let c = crc32(frame);
    frame.extend_from_slice(&c.to_le_bytes());
}

/// Verify and strip the FCS; `None` if too short or mismatched.
pub fn check_and_strip(frame: &[u8]) -> Option<&[u8]> {
    if frame.len() < 4 {
        return None;
    }
    let (body, fcs) = frame.split_at(frame.len() - 4);
    (crc32(body).to_le_bytes() == fcs).then_some(body)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn known_vector() {
        // The canonical CRC-32 check value.
        assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
    }

    #[test]
    fn append_check_roundtrip() {
        let mut f = b"hello 802.11".to_vec();
        append(&mut f);
        assert_eq!(check_and_strip(&f), Some(&b"hello 802.11"[..]));
        f[3] ^= 1;
        assert_eq!(check_and_strip(&f), None);
    }
}
