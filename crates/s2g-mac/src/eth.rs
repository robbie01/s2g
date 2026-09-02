//! Ethernet ↔ 802.11 payload conversion (RFC 1042 LLC/SNAP encapsulation).

use crate::frame::MacAddr;

pub const ETH_HDR_LEN: usize = 14;
const LLC_SNAP: [u8; 6] = [0xAA, 0xAA, 0x03, 0x00, 0x00, 0x00];

/// Split a raw Ethernet frame from a TAP device.
pub fn parse_ethernet(frame: &[u8]) -> Option<(MacAddr, MacAddr, u16, &[u8])> {
    if frame.len() < ETH_HDR_LEN {
        return None;
    }
    let mut dest = [0u8; 6];
    let mut src = [0u8; 6];
    dest.copy_from_slice(&frame[0..6]);
    src.copy_from_slice(&frame[6..12]);
    let ethertype = u16::from_be_bytes([frame[12], frame[13]]);
    Some((dest, src, ethertype, &frame[ETH_HDR_LEN..]))
}

/// 802.11 frame body for an Ethernet payload: LLC/SNAP + EtherType + data.
pub fn to_body(ethertype: u16, payload: &[u8]) -> Vec<u8> {
    let mut b = Vec::with_capacity(8 + payload.len());
    b.extend_from_slice(&LLC_SNAP);
    b.extend_from_slice(&ethertype.to_be_bytes());
    b.extend_from_slice(payload);
    b
}

/// Split a received 802.11 Data body into its EtherType and payload, if it
/// carries the RFC 1042 LLC/SNAP header.
pub fn split_body(body: &[u8]) -> Option<(u16, &[u8])> {
    if body.len() < 8 || body[..6] != LLC_SNAP {
        return None;
    }
    Some((u16::from_be_bytes([body[6], body[7]]), &body[8..]))
}

/// Rebuild an Ethernet frame from a received 802.11 Data body.
pub fn body_to_ethernet(dest: MacAddr, src: MacAddr, body: &[u8]) -> Option<Vec<u8>> {
    if body.len() < 8 || body[..6] != LLC_SNAP {
        return None;
    }
    let mut f = Vec::with_capacity(ETH_HDR_LEN + body.len() - 8);
    f.extend_from_slice(&dest);
    f.extend_from_slice(&src);
    f.extend_from_slice(&body[6..8]); // EtherType, already big-endian
    f.extend_from_slice(&body[8..]);
    Some(f)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip() {
        let dest = [2, 0, 0, 0, 0, 9];
        let src = [2, 0, 0, 0, 0, 7];
        let mut eth = Vec::new();
        eth.extend_from_slice(&dest);
        eth.extend_from_slice(&src);
        eth.extend_from_slice(&0x0800u16.to_be_bytes());
        eth.extend_from_slice(b"ip packet bytes");
        let (d, s, et, payload) = parse_ethernet(&eth).unwrap();
        assert_eq!((d, s, et), (dest, src, 0x0800));
        let body = to_body(et, payload);
        let back = body_to_ethernet(d, s, &body).unwrap();
        assert_eq!(back, eth);
    }

    #[test]
    fn non_snap_body_rejected() {
        assert!(body_to_ethernet([0; 6], [0; 6], b"raw junk").is_none());
    }
}
