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
/// QoS Data frame MAC header length (3-address + QoS Control).
pub const QOS_DATA_HDR_LEN: usize = 26;
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

/// Build a QoS Data frame (subtype 8) with Normal Ack policy and the given
/// TID: the MPDU type an A-MPDU carries [9.7.3, Table 9-664].
pub fn build_qos_data(dest: MacAddr, src: MacAddr, seq: u16, retry: bool, duration_us: u16, tid: u8, body: &[u8]) -> Vec<u8> {
    let mut f = Vec::with_capacity(QOS_DATA_HDR_LEN + body.len() + 4);
    f.push(0x88); // version 0, type 2 (Data), subtype 8 (QoS Data)
    f.push(if retry { 0x08 } else { 0x00 });
    f.extend_from_slice(&(duration_us & 0x7fff).to_le_bytes());
    f.extend_from_slice(&dest);
    f.extend_from_slice(&src);
    f.extend_from_slice(&WILDCARD_BSSID);
    f.extend_from_slice(&((seq & 0x0fff) << 4).to_le_bytes());
    // QoS Control: TID, EOSP 0, Ack Policy 00 (Normal Ack), A-MSDU 0.
    f.extend_from_slice(&((tid & 0x0f) as u16).to_le_bytes());
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

/// An address field of a PV1 frame: a full MAC address or a Short ID
/// (AID plus header-presence flags) [9.8.3.2].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Pv1Addr {
    Mac(MacAddr),
    Sid { aid: u16, a3_present: bool, a4_present: bool, amsdu: bool },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ParsedFrame {
    /// PV0 Data (subtype 0) or QoS Data (subtype 8, `tid` set).
    Data { dest: MacAddr, src: MacAddr, bssid: MacAddr, seq: u16, retry: bool, duration_us: u16, tid: Option<u8>, body: Vec<u8> },
    Ack { ra: MacAddr },
    Rts { ra: MacAddr, ta: MacAddr, duration_us: u16 },
    /// A PV1 (short MAC header) frame [9.8]: reception is mandatory for an
    /// S1G STA. QoS Data (type 0/3) and Management (type 1) frames are
    /// parsed fully; PV1 Control frames only to their subtype.
    Pv1 {
        ptype: u8,
        subtype: u8,
        from_ds: bool,
        a1: Pv1Addr,
        a2: Pv1Addr,
        a3: Option<MacAddr>,
        a4: Option<MacAddr>,
        seq: Option<u16>,
        /// Ack Policy Indicator: false = Normal Ack, true = No Ack / Block Ack.
        no_ack: bool,
        protected: bool,
        body: Vec<u8>,
    },
    /// Valid FCS but a type/subtype this MAC doesn't handle.
    Other { fc: [u8; 2], duration_us: u16 },
}

/// PV1 frame types [Table 9-669].
pub const PV1_TYPE_QOS_DATA: u8 = 0;
pub const PV1_TYPE_MANAGEMENT: u8 = 1;
pub const PV1_TYPE_CONTROL: u8 = 2;
/// PV1 QoS Data with both A1 and A2 as full MAC addresses.
pub const PV1_TYPE_QOS_DATA_MAC: u8 = 3;

/// Build a PV1 QoS Data frame with full MAC addresses (type 3) [9.8.2]:
/// FC ‖ A1 ‖ A2 ‖ Sequence Control ‖ body ‖ FCS.
pub fn build_pv1_data(dest: MacAddr, src: MacAddr, seq: u16, tid: u8, no_ack: bool, body: &[u8]) -> Vec<u8> {
    let mut f = Vec::with_capacity(16 + body.len() + 4);
    let fc0 = 1 | (PV1_TYPE_QOS_DATA_MAC << 2) | ((tid & 7) << 5);
    let fc1 = if no_ack { 0x80 } else { 0x00 };
    f.push(fc0);
    f.push(fc1);
    f.extend_from_slice(&dest);
    f.extend_from_slice(&src);
    f.extend_from_slice(&((seq & 0x0fff) << 4).to_le_bytes());
    f.extend_from_slice(body);
    fcs::append(&mut f);
    f
}

fn parse_pv1(inner: &[u8]) -> Result<ParsedFrame, FrameError> {
    let fc0 = inner[0];
    let fc1 = inner[1];
    let ptype = (fc0 >> 2) & 7;
    let subtype = fc0 >> 5;
    let from_ds = fc1 & 1 != 0;
    let protected = fc1 & 0x10 != 0;
    let no_ack = fc1 & 0x80 != 0;
    let mut pos = 2;
    let take_mac = |pos: &mut usize| -> Result<MacAddr, FrameError> {
        if *pos + 6 > inner.len() {
            return Err(FrameError::TooShort);
        }
        let a = addr(&inner[*pos..*pos + 6]);
        *pos += 6;
        Ok(a)
    };
    let take_sid = |pos: &mut usize| -> Result<Pv1Addr, FrameError> {
        if *pos + 2 > inner.len() {
            return Err(FrameError::TooShort);
        }
        let v = u16::from_le_bytes([inner[*pos], inner[*pos + 1]]);
        *pos += 2;
        Ok(Pv1Addr::Sid { aid: v & 0x1fff, a3_present: v & 0x2000 != 0, a4_present: v & 0x4000 != 0, amsdu: v & 0x8000 != 0 })
    };
    let (a1, a2) = match ptype {
        PV1_TYPE_QOS_DATA_MAC => (Pv1Addr::Mac(take_mac(&mut pos)?), Pv1Addr::Mac(take_mac(&mut pos)?)),
        PV1_TYPE_QOS_DATA | PV1_TYPE_MANAGEMENT => {
            if from_ds {
                let a1 = take_sid(&mut pos)?;
                (a1, Pv1Addr::Mac(take_mac(&mut pos)?))
            } else {
                let a1 = Pv1Addr::Mac(take_mac(&mut pos)?);
                (a1, take_sid(&mut pos)?)
            }
        }
        // Control frames: A1 is a SID; A2 depends on the subtype [9.8.4].
        // Parse A1 only and hand the rest over as the body.
        _ => {
            let a1 = take_sid(&mut pos)?;
            return Ok(ParsedFrame::Pv1 {
                ptype,
                subtype,
                from_ds,
                a1,
                a2: Pv1Addr::Sid { aid: 0, a3_present: false, a4_present: false, amsdu: false },
                a3: None,
                a4: None,
                seq: None,
                no_ack,
                protected,
                body: inner[pos..].to_vec(),
            });
        }
    };
    // Sequence Control: all Data frames and Management frames except the
    // PV1 Probe Response (management subtype 2).
    let seq = if ptype != PV1_TYPE_MANAGEMENT || subtype != 2 {
        if pos + 2 > inner.len() {
            return Err(FrameError::TooShort);
        }
        let sc = u16::from_le_bytes([inner[pos], inner[pos + 1]]);
        pos += 2;
        Some(sc >> 4)
    } else {
        None
    };
    let (a3p, a4p) = match (a1, a2) {
        (Pv1Addr::Sid { a3_present, a4_present, .. }, _) | (_, Pv1Addr::Sid { a3_present, a4_present, .. }) => {
            (a3_present, a4_present)
        }
        _ => (false, false),
    };
    let a3 = if a3p { Some(take_mac(&mut pos)?) } else { None };
    let a4 = if a4p { Some(take_mac(&mut pos)?) } else { None };
    Ok(ParsedFrame::Pv1 { ptype, subtype, from_ds, a1, a2, a3, a4, seq, no_ack, protected, body: inner[pos..].to_vec() })
}

/// Locate the MPDU inside a non-aggregated PSDU. Some S1G chips (seen on a
/// commercial HaLow baby monitor) round the SIG Length up to a multiple of
/// 4 octets and pad after the FCS, so try the full PSDU first and then up
/// to 3 shorter prefixes; returns the longest prefix whose FCS verifies.
pub fn locate_mpdu(psdu: &[u8]) -> Option<&[u8]> {
    (0..4usize).filter_map(|trim| psdu.len().checked_sub(trim)).find(|&len| len >= 10 && fcs::check_and_strip(&psdu[..len]).is_some()).map(|len| &psdu[..len])
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
    if fc0 & 3 == 1 {
        return parse_pv1(inner);
    }
    let ftype = (fc0 >> 2) & 0x3;
    let subtype = fc0 >> 4;
    let duration_us = u16::from_le_bytes([inner[2], inner[3]]) & 0x7fff;
    match (ftype, subtype) {
        // Data (subtype 0) and QoS Data (subtype 8); ToDS/FromDS = 0 only.
        (2, 0) | (2, 8) => {
            let qos = subtype == 8;
            let hdr = if qos { QOS_DATA_HDR_LEN } else { DATA_HDR_LEN };
            if inner.len() < hdr || fc1 & 0x03 != 0 {
                return Err(FrameError::TooShort);
            }
            let seq_ctrl = u16::from_le_bytes([inner[22], inner[23]]);
            let tid = qos.then(|| inner[24] & 0x0f);
            Ok(ParsedFrame::Data {
                dest: addr(&inner[4..10]),
                src: addr(&inner[10..16]),
                bssid: addr(&inner[16..22]),
                seq: seq_ctrl >> 4,
                retry: fc1 & 0x08 != 0,
                duration_us,
                tid,
                body: inner[hdr..].to_vec(),
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
            ParsedFrame::Data { dest, src, bssid, seq, retry, duration_us, body, .. } => {
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
    fn pv1_data_roundtrip() {
        let f = build_pv1_data(B, A, 77, 3, false, b"pv1!");
        match parse(&f).unwrap() {
            ParsedFrame::Pv1 { ptype, subtype, a1, a2, seq, no_ack, body, a3, a4, .. } => {
                assert_eq!(ptype, PV1_TYPE_QOS_DATA_MAC);
                assert_eq!(subtype, 3); // TID
                assert_eq!(a1, Pv1Addr::Mac(B));
                assert_eq!(a2, Pv1Addr::Mac(A));
                assert_eq!(seq, Some(77));
                assert!(!no_ack);
                assert_eq!(a3, None);
                assert_eq!(a4, None);
                assert_eq!(body, b"pv1!");
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn pv1_sid_addressing_and_optional_addresses() {
        // Type 0, From DS = 1: A1 = SID (AID 0x123, A3 present), A2 = MAC.
        let mut f = vec![1 | (PV1_TYPE_QOS_DATA << 2), 0x01];
        f.extend_from_slice(&(0x0123u16 | 0x2000).to_le_bytes());
        f.extend_from_slice(&A);
        f.extend_from_slice(&(5u16 << 4).to_le_bytes());
        f.extend_from_slice(&B); // A3
        f.extend_from_slice(b"xyz");
        fcs::append(&mut f);
        match parse(&f).unwrap() {
            ParsedFrame::Pv1 { from_ds, a1, a2, a3, seq, body, .. } => {
                assert!(from_ds);
                assert_eq!(a1, Pv1Addr::Sid { aid: 0x123, a3_present: true, a4_present: false, amsdu: false });
                assert_eq!(a2, Pv1Addr::Mac(A));
                assert_eq!(a3, Some(B));
                assert_eq!(seq, Some(5));
                assert_eq!(body, b"xyz");
            }
            other => panic!("{other:?}"),
        }
        // A PV1 control frame (type 2) is recognised and not misread as PV0.
        let mut c = vec![1 | (PV1_TYPE_CONTROL << 2), 0x00];
        c.extend_from_slice(&0x0042u16.to_le_bytes());
        c.extend_from_slice(&[0xaa; 6]);
        fcs::append(&mut c);
        assert!(matches!(parse(&c).unwrap(), ParsedFrame::Pv1 { ptype: PV1_TYPE_CONTROL, .. }));
    }

    #[test]
    fn padded_psdu_is_located() {
        let f = build_ack(A);
        let mut padded = f.clone();
        padded.extend_from_slice(&[1, 0]);
        assert_eq!(locate_mpdu(&padded), Some(&f[..]));
        assert_eq!(locate_mpdu(&f), Some(&f[..]));
        let mut bad = f.clone();
        bad[5] ^= 1;
        assert_eq!(locate_mpdu(&bad), None);
    }

    #[test]
    fn corrupted_fails() {
        let mut f = build_data(B, A, 1, false, 0, b"zzz");
        f[10] ^= 0x40;
        assert_eq!(parse(&f), Err(FrameError::BadFcs));
    }
}
