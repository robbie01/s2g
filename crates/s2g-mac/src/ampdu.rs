//! A-MPDU aggregation/deaggregation [9.7, pp1840–1843].
//!
//! Subframe = 4-octet MPDU delimiter ‖ MPDU ‖ pad-to-4. Delimiter bits
//! B0 EOF, B1 reserved, B2–B15 MPDU Length (B2–B3 = high 2 bits, B4–B15 =
//! low 12; L = low + 4096·high [Fig 9-1330/Eq 9-16]), B16–B23 CRC-8
//! (x⁸+x²+x+1, preset all-ones, ones-complemented, c7 transmitted first
//! [9.7.2, Fig 9-1332]), B24–B31 signature 0x4E. In an S1G PPDU the
//! A-MPDU fills the PSDU exactly: EOF delimiters (length 0, EOF 1) plus
//! 0–3 loose octets pad to the symbol capacity [9.7.1, 10.12.6].

pub const DELIM_LEN: usize = 4;
pub const SIGNATURE: u8 = 0x4E;
/// 14-bit length field cap.
pub const MAX_MPDU_LEN: usize = (1 << 14) - 1;

/// CRC-8 over the first 16 delimiter bits (B0 first), G = x⁸+x²+x+1,
/// register preset all-ones, output complemented; returned byte has c7 in
/// bit 0 (= B16, transmitted first).
fn delim_crc8(header: u16) -> u8 {
    let (mut c7, mut c6, mut c5, mut c4, mut c3, mut c2, mut c1, mut c0) = (1u8, 1, 1, 1, 1, 1, 1, 1);
    for i in 0..16 {
        let b = ((header >> i) & 1) as u8;
        let fb = b ^ c7;
        c7 = c6;
        c6 = c5;
        c5 = c4;
        c4 = c3;
        c3 = c2;
        c2 = c1 ^ fb;
        c1 = c0 ^ fb;
        c0 = fb;
    }
    // Complement; pack c7 into bit 0 (B16) ... c0 into bit 7 (B23).
    let regs = [c7, c6, c5, c4, c3, c2, c1, c0];
    regs.iter().enumerate().fold(0u8, |acc, (i, &c)| acc | (((1 ^ c) & 1) << i))
}

/// Build one MPDU delimiter.
pub fn build_delimiter(mpdu_len: usize, eof: bool) -> [u8; 4] {
    debug_assert!(mpdu_len <= MAX_MPDU_LEN);
    let low = (mpdu_len & 0xfff) as u16;
    let high = ((mpdu_len >> 12) & 0x3) as u16;
    let header: u16 = (eof as u16) | (high << 2) | (low << 4);
    let crc = delim_crc8(header);
    [header as u8, (header >> 8) as u8, crc, SIGNATURE]
}

/// Parse a delimiter; `Some((mpdu_len, eof))` if CRC and signature check.
pub fn parse_delimiter(d: &[u8]) -> Option<(usize, bool)> {
    if d.len() < 4 || d[3] != SIGNATURE {
        return None;
    }
    let header = u16::from_le_bytes([d[0], d[1]]);
    if delim_crc8(header) != d[2] {
        return None;
    }
    let low = (header >> 4) as usize & 0xfff;
    let high = (header >> 2) as usize & 0x3;
    Some((low + 4096 * high, header & 1 != 0))
}

fn pad4(n: usize) -> usize {
    n.div_ceil(4) * 4
}

/// Length of the pre-EOF A-MPDU for a single MPDU: delimiter + MPDU. The
/// final-subframe padding is not counted; the EOF fill handles alignment.
pub fn pre_eof_len(mpdu_len: usize) -> usize {
    DELIM_LEN + mpdu_len
}

/// Build an S-MPDU, an A-MPDU carrying a single MPDU whose delimiter has
/// EOF = 1 [9.7.3, 10.12.8], padded to exactly `capacity` octets (the PSDU
/// capacity of the chosen symbol count). An S-MPDU follows the rules of a
/// non-aggregated frame: any MPDU that is valid on its own is valid inside
/// it, and it is acknowledged with an (NDP) Ack, not a BlockAck.
pub fn aggregate(mpdu: &[u8], capacity: usize) -> Vec<u8> {
    assert!(mpdu.len() <= MAX_MPDU_LEN);
    assert!(capacity >= DELIM_LEN + mpdu.len());
    let mut out = Vec::with_capacity(capacity);
    out.extend_from_slice(&build_delimiter(mpdu.len(), true));
    out.extend_from_slice(mpdu);
    // Final-subframe padding to a 4-octet boundary (0–3), then EOF
    // delimiters, then 0–3 loose octets.
    let aligned = pad4(out.len()).min(capacity);
    out.resize(aligned, 0);
    let eof = build_delimiter(0, true);
    while out.len() + DELIM_LEN <= capacity {
        out.extend_from_slice(&eof);
    }
    out.resize(capacity, 0);
    out
}

/// Pre-EOF length of an A-MPDU carrying MPDUs of the given lengths: every
/// subframe but the last is padded to a 4-octet boundary.
pub fn pre_eof_len_many(lens: &[usize]) -> usize {
    let n = lens.len();
    lens.iter().enumerate().map(|(i, &l)| if i + 1 < n { pad4(DELIM_LEN + l) } else { DELIM_LEN + l }).sum()
}

/// Build a multi-MPDU A-MPDU (EOF = 0 on every MPDU delimiter, EOF
/// padding to exactly `capacity` octets) [9.7.3]. For a single MPDU use
/// [`aggregate`] (an S-MPDU).
pub fn aggregate_many(mpdus: &[&[u8]], capacity: usize) -> Vec<u8> {
    assert!(mpdus.len() >= 2, "use aggregate() for an S-MPDU");
    let lens: Vec<usize> = mpdus.iter().map(|m| m.len()).collect();
    assert!(lens.iter().all(|&l| l <= MAX_MPDU_LEN));
    assert!(capacity >= pre_eof_len_many(&lens));
    let mut out = Vec::with_capacity(capacity);
    for m in mpdus {
        out.resize(pad4(out.len()), 0);
        out.extend_from_slice(&build_delimiter(m.len(), false));
        out.extend_from_slice(m);
    }
    let aligned = pad4(out.len()).min(capacity);
    out.resize(aligned, 0);
    let eof = build_delimiter(0, true);
    while out.len() + DELIM_LEN <= capacity {
        out.extend_from_slice(&eof);
    }
    out.resize(capacity, 0);
    out
}

/// Extract MPDUs from an A-MPDU (deaggregation per Annex O.2: scan 4-octet
/// aligned positions, resync on delimiter errors). Each MPDU comes with its
/// delimiter's EOF flag: a lone MPDU with EOF = 1 is an S-MPDU.
pub fn deaggregate_with_eof(psdu: &[u8]) -> Vec<(&[u8], bool)> {
    let mut out = Vec::new();
    let mut pos = 0usize;
    while pos + DELIM_LEN <= psdu.len() {
        match parse_delimiter(&psdu[pos..pos + DELIM_LEN]) {
            Some((0, _eof)) => pos += DELIM_LEN,
            Some((len, eof)) if pos + DELIM_LEN + len <= psdu.len() => {
                out.push((&psdu[pos + DELIM_LEN..pos + DELIM_LEN + len], eof));
                pos = pad4(pos + DELIM_LEN + len);
            }
            _ => pos += DELIM_LEN, // bad delimiter or truncated: resync
        }
    }
    out
}

/// Extract MPDUs from an A-MPDU.
pub fn deaggregate(psdu: &[u8]) -> Vec<&[u8]> {
    deaggregate_with_eof(psdu).into_iter().map(|(m, _)| m).collect()
}

/// True when the PSDU is an S-MPDU: exactly one MPDU, with EOF set.
pub fn is_s_mpdu(psdu: &[u8]) -> bool {
    let m = deaggregate_with_eof(psdu);
    m.len() == 1 && m[0].1
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn delimiter_roundtrip() {
        for (len, eof) in [(0usize, true), (14, false), (511, false), (4095, false), (5000, false), (16383, false)] {
            let d = build_delimiter(len, eof);
            assert_eq!(d[3], SIGNATURE);
            assert_eq!(parse_delimiter(&d), Some((len, eof)), "len {len}");
        }
        // Corruption is caught.
        let mut d = build_delimiter(100, false);
        d[0] ^= 0x10;
        assert_eq!(parse_delimiter(&d), None);
    }

    #[test]
    fn aggregate_deaggregate() {
        let mpdu: Vec<u8> = (0..1337u32).map(|i| (i * 3) as u8).collect();
        let cap = 1500;
        let a = aggregate(&mpdu, cap);
        assert_eq!(a.len(), cap);
        let got = deaggregate(&a);
        assert_eq!(got, vec![&mpdu[..]]);
        assert!(is_s_mpdu(&a));
        // A two-MPDU A-MPDU is not an S-MPDU.
        let mut two = Vec::new();
        two.extend_from_slice(&build_delimiter(mpdu.len(), false));
        two.extend_from_slice(&mpdu);
        two.resize(two.len().div_ceil(4) * 4, 0);
        two.extend_from_slice(&build_delimiter(mpdu.len(), false));
        two.extend_from_slice(&mpdu);
        assert!(!is_s_mpdu(&two));
        assert_eq!(deaggregate(&two).len(), 2);
    }

    #[test]
    fn aggregate_many_roundtrip() {
        let m1: Vec<u8> = (0..101u32).map(|i| i as u8).collect();
        let m2 = vec![7u8; 30];
        let m3 = vec![9u8; 1000];
        let lens = [m1.len(), m2.len(), m3.len()];
        assert_eq!(pre_eof_len_many(&lens), 108 + 36 + 1004);
        let cap = pre_eof_len_many(&lens) + 23;
        let a = aggregate_many(&[&m1, &m2, &m3], cap);
        assert_eq!(a.len(), cap);
        let got = deaggregate_with_eof(&a);
        assert_eq!(got.len(), 3);
        assert_eq!(got[0], (&m1[..], false));
        assert_eq!(got[1], (&m2[..], false));
        assert_eq!(got[2], (&m3[..], false));
        assert!(!is_s_mpdu(&a));
    }

    #[test]
    fn eof_padding_survives_scan() {
        // Small MPDU, big capacity: many EOF delimiters to scan past.
        let mpdu = vec![0xAB; 30];
        let a = aggregate(&mpdu, 400);
        assert_eq!(deaggregate(&a), vec![&mpdu[..]]);
    }

    #[test]
    fn resync_after_corruption() {
        // Two MPDUs hand-built; corrupt the first delimiter → second recovered.
        let m1 = vec![1u8; 20];
        let m2 = vec![2u8; 24];
        let mut a = Vec::new();
        a.extend_from_slice(&build_delimiter(m1.len(), false));
        a.extend_from_slice(&m1);
        a.resize(a.len().div_ceil(4) * 4, 0);
        let second_at = a.len();
        a.extend_from_slice(&build_delimiter(m2.len(), false));
        a.extend_from_slice(&m2);
        a[0] ^= 0xff; // corrupt first delimiter
        let got = deaggregate(&a);
        assert_eq!(got, vec![&m2[..]], "second MPDU at {second_at} recovered");
    }
}
