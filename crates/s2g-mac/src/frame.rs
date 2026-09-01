//! 802.11 frame construction/parsing for OCB operation: Data frames with
//! the wildcard BSSID [9.3.2; OCB per 11.1.4], Ack [9.3.1.4] and RTS
//! [9.3.1.2] control frames. All frames carry an FCS.

use crate::fcs;
use thiserror::Error;

pub type MacAddr = [u8; 6];

/// OCB stations use the wildcard BSSID in Address 3.
pub const WILDCARD_BSSID: MacAddr = [0xff; 6];
pub const BROADCAST: MacAddr = [0xff; 6];

/// Data frame MAC header length (3-address, no QoS).
pub const DATA_HDR_LEN: usize = 24;
/// Ack frame length incl. FCS.
pub const ACK_LEN: usize = 14;
/// RTS frame length incl. FCS.
pub const RTS_LEN: usize = 20;

#[derive(Debug, Error, Clone, PartialEq, Eq)]
pub enum FrameError {
    #[error("frame too short")]
    TooShort,
    #[error("bad FCS")]
    BadFcs,
}

/// Build a (non-QoS) Data frame: ToDS=0 FromDS=0, addr3 = wildcard BSSID.
/// `duration_us` is the Duration/ID field (0 when no response is solicited).
pub fn build_data(dest: MacAddr, src: MacAddr, seq: u16, retry: bool, duration_us: u16, body: &[u8]) -> Vec<u8> {
    let mut f = Vec::with_capacity(DATA_HDR_LEN + body.len() + 4);
    // Frame Control: version 0, type 2 (Data), subtype 0.
    f.push(0x08);
    f.push(if retry { 0x08 } else { 0x00 }); // B11 Retry
    f.extend_from_slice(&(duration_us & 0x7fff).to_le_bytes());
    f.extend_from_slice(&dest);
    f.extend_from_slice(&src);
    f.extend_from_slice(&WILDCARD_BSSID);
    f.extend_from_slice(&((seq & 0x0fff) << 4).to_le_bytes()); // frag 0
    f.extend_from_slice(body);
    fcs::append(&mut f);
    f
}

/// Build an Ack control frame for `ra`.
pub fn build_ack(ra: MacAddr) -> Vec<u8> {
    let mut f = Vec::with_capacity(ACK_LEN);
    f.push(0xD4); // type 1 (Control), subtype 13 (Ack)
    f.push(0x00);
    f.extend_from_slice(&[0, 0]);
    f.extend_from_slice(&ra);
    fcs::append(&mut f);
    f
}

/// Build an RTS control frame [9.3.1.2].
pub fn build_rts(ra: MacAddr, ta: MacAddr, duration_us: u16) -> Vec<u8> {
    let mut f = Vec::with_capacity(RTS_LEN);
    f.push(0xB4); // type 1 (Control), subtype 11 (RTS)
    f.push(0x00);
    f.extend_from_slice(&(duration_us & 0x7fff).to_le_bytes());
    f.extend_from_slice(&ra);
    f.extend_from_slice(&ta);
    fcs::append(&mut f);
    f
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ParsedFrame {
    Data { dest: MacAddr, src: MacAddr, bssid: MacAddr, seq: u16, retry: bool, duration_us: u16, body: Vec<u8> },
    Ack { ra: MacAddr },
    Rts { ra: MacAddr, ta: MacAddr, duration_us: u16 },
    /// Valid FCS but a type/subtype this MAC doesn't handle.
    Other { fc: [u8; 2], duration_us: u16 },
}

fn addr(b: &[u8]) -> MacAddr {
    let mut a = [0u8; 6];
    a.copy_from_slice(&b[..6]);
    a
}

/// Parse an MPDU (with FCS).
pub fn parse(mpdu: &[u8]) -> Result<ParsedFrame, FrameError> {
    let inner = fcs::check_and_strip(mpdu).ok_or(FrameError::BadFcs)?;
    if inner.len() < 10 {
        return Err(FrameError::TooShort);
    }
    let fc0 = inner[0];
    let fc1 = inner[1];
    let ftype = (fc0 >> 2) & 0x3;
    let subtype = fc0 >> 4;
    let duration_us = u16::from_le_bytes([inner[2], inner[3]]) & 0x7fff;
    match (ftype, subtype) {
        // Data (plain or QoS-less null etc. — accept subtype 0 only)
        (2, 0) => {
            if inner.len() < DATA_HDR_LEN {
                return Err(FrameError::TooShort);
            }
            let seq_ctrl = u16::from_le_bytes([inner[22], inner[23]]);
            Ok(ParsedFrame::Data {
                dest: addr(&inner[4..10]),
                src: addr(&inner[10..16]),
                bssid: addr(&inner[16..22]),
                seq: seq_ctrl >> 4,
                retry: fc1 & 0x08 != 0,
                duration_us,
                body: inner[DATA_HDR_LEN..].to_vec(),
            })
        }
        (1, 13) => Ok(ParsedFrame::Ack { ra: addr(&inner[4..10]) }),
        (1, 11) => {
            if inner.len() < RTS_LEN - 4 {
                return Err(FrameError::TooShort);
            }
            Ok(ParsedFrame::Rts { ra: addr(&inner[4..10]), ta: addr(&inner[10..16]), duration_us })
        }
        _ => Ok(ParsedFrame::Other { fc: [fc0, fc1], duration_us }),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const A: MacAddr = [2, 0, 0, 0, 0, 1];
    const B: MacAddr = [2, 0, 0, 0, 0, 2];

    #[test]
    fn data_roundtrip() {
        let f = build_data(B, A, 1234, false, 400, b"payload!");
        match parse(&f).unwrap() {
            ParsedFrame::Data { dest, src, bssid, seq, retry, duration_us, body } => {
                assert_eq!(dest, B);
                assert_eq!(src, A);
                assert_eq!(bssid, WILDCARD_BSSID);
                assert_eq!(seq, 1234);
                assert!(!retry);
                assert_eq!(duration_us, 400);
                assert_eq!(body, b"payload!");
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn retry_bit() {
        let f = build_data(B, A, 7, true, 0, b"x");
        assert!(matches!(parse(&f).unwrap(), ParsedFrame::Data { retry: true, .. }));
    }

    #[test]
    fn ack_roundtrip() {
        let f = build_ack(A);
        assert_eq!(f.len(), ACK_LEN);
        assert_eq!(parse(&f).unwrap(), ParsedFrame::Ack { ra: A });
    }

    #[test]
    fn rts_roundtrip() {
        let f = build_rts(B, A, 1234);
        assert_eq!(f.len(), RTS_LEN);
        assert_eq!(parse(&f).unwrap(), ParsedFrame::Rts { ra: B, ta: A, duration_us: 1234 });
    }

    #[test]
    fn corrupted_fails() {
        let mut f = build_data(B, A, 1, false, 0, b"zzz");
        f[10] ^= 0x40;
        assert_eq!(parse(&f), Err(FrameError::BadFcs));
    }
}
