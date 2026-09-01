//! NDP CMAC PPDU bodies for 2 MHz (NDP_2M, 37 bits) [23.3.12]: NDP CTS,
//! NDP Ack and NDP BlockAck — the three CMAC frames every S1G STA must
//! support [4.3.14.1]. The other types are recognised by their type code
//! and passed through opaque.
//!
//! Bit numbering follows the PHY: body bit B0 is the LSB of the `u64`
//! (first transmitted). Numeric subfields are LSB-first within their bit
//! range, as everywhere else in 802.11.

use crate::fcs;
use crate::frame::MacAddr;

/// NDP CMAC PPDU Type field values [Table 23-30].
pub const TYPE_CTS_CF_END: u8 = 0;
pub const TYPE_PS_POLL: u8 = 1;
pub const TYPE_ACK: u8 = 2;
pub const TYPE_PS_POLL_ACK: u8 = 3;
pub const TYPE_BLOCK_ACK: u8 = 4;
pub const TYPE_BF_REPORT_POLL: u8 = 5;
pub const TYPE_PAGING: u8 = 6;
pub const TYPE_PROBE_REQUEST: u8 = 7;

const BODY_MASK: u64 = (1 << 37) - 1;

fn get(body: u64, lo: u32, bits: u32) -> u64 {
    (body >> lo) & ((1u64 << bits) - 1)
}

fn put(body: &mut u64, lo: u32, bits: u32, value: u64) {
    let mask = ((1u64 << bits) - 1) << lo;
    *body = (*body & !mask) | ((value << lo) & mask);
}

/// NDP_2M CTS [23.3.12.2.1.2, Figure 23-23].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NdpCts {
    /// 0: RA/PBSSID holds the receiver's partial AID; 1: the transmitting
    /// AP's partial BSSID (broadcast semantics).
    pub address_indicator: bool,
    /// 9-bit RA / partial BSSID.
    pub ra_pbssid: u16,
    /// Duration, µs (15 bits) [9.3.1.3].
    pub duration_us: u16,
    pub early_sector_indicator: bool,
    /// Bandwidth Indication [Table 9-5]: 0 = 1 MHz, 1 = 2 MHz, 2 = 4, 3 = 8, 4 = 16.
    pub bandwidth: u8,
}

/// NDP_2M Ack [23.3.12.2.4.3, Figure 23-29].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NdpAck {
    /// Scrambler Initialization[0:6] ‖ FCS[23:31] of the soliciting PSDU
    /// (see [`ack_id`]).
    pub ack_id: u16,
    pub more_data: bool,
    /// 0: `duration` sets the NAV (µs); 1: `duration` is an idle period in ms.
    pub idle_indication: bool,
    /// 14 bits.
    pub duration: u16,
    pub relayed_frame: bool,
}

/// NDP_2M BlockAck [23.3.12.2.6.2, Figure 23-33].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NdpBlockAck {
    /// 6 LSBs of the soliciting PSDU's scrambler initialization.
    pub block_ack_id: u8,
    /// Starting sequence number (12 bits).
    pub starting_sequence: u16,
    /// Bit i acknowledges sequence number starting_sequence + i.
    pub bitmap: u16,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NdpFrame {
    Cts(NdpCts),
    Ack(NdpAck),
    BlockAck(NdpBlockAck),
    /// A CMAC type this MAC does not interpret (PS-Poll, Paging, …) or an
    /// NDP CF-End.
    Other { ndp_type: u8, body: u64 },
}

impl NdpFrame {
    /// Serialize to the 37-bit NDP CMAC PPDU body.
    pub fn to_body(&self) -> u64 {
        let mut b = 0u64;
        match *self {
            NdpFrame::Cts(c) => {
                put(&mut b, 0, 3, TYPE_CTS_CF_END as u64);
                put(&mut b, 3, 1, 0); // CTS/CF-End Indicator: CTS
                put(&mut b, 4, 1, c.address_indicator as u64);
                put(&mut b, 5, 9, c.ra_pbssid as u64);
                put(&mut b, 14, 15, c.duration_us as u64);
                put(&mut b, 29, 1, c.early_sector_indicator as u64);
                put(&mut b, 30, 3, c.bandwidth as u64);
            }
            NdpFrame::Ack(a) => {
                put(&mut b, 0, 3, TYPE_ACK as u64);
                put(&mut b, 3, 16, a.ack_id as u64);
                put(&mut b, 19, 1, a.more_data as u64);
                put(&mut b, 20, 1, a.idle_indication as u64);
                put(&mut b, 21, 14, a.duration as u64);
                put(&mut b, 35, 1, a.relayed_frame as u64);
            }
            NdpFrame::BlockAck(ba) => {
                put(&mut b, 0, 3, TYPE_BLOCK_ACK as u64);
                put(&mut b, 3, 6, ba.block_ack_id as u64);
                put(&mut b, 9, 12, ba.starting_sequence as u64);
                put(&mut b, 21, 16, ba.bitmap as u64);
                // Bitmap protection [10.56]: [B3:B18] ^= [B21:B36] (the ID
                // and the low 10 bits of the starting sequence).
                let protected = get(b, 3, 16) ^ ba.bitmap as u64;
                put(&mut b, 3, 16, protected);
            }
            NdpFrame::Other { body, .. } => b = body,
        }
        b & BODY_MASK
    }

    /// Parse a received 37-bit body.
    pub fn parse(body: u64) -> NdpFrame {
        let body = body & BODY_MASK;
        let ndp_type = get(body, 0, 3) as u8;
        match ndp_type {
            TYPE_CTS_CF_END if get(body, 3, 1) == 0 => NdpFrame::Cts(NdpCts {
                address_indicator: get(body, 4, 1) == 1,
                ra_pbssid: get(body, 5, 9) as u16,
                duration_us: get(body, 14, 15) as u16,
                early_sector_indicator: get(body, 29, 1) == 1,
                bandwidth: get(body, 30, 3) as u8,
            }),
            TYPE_ACK => NdpFrame::Ack(NdpAck {
                ack_id: get(body, 3, 16) as u16,
                more_data: get(body, 19, 1) == 1,
                idle_indication: get(body, 20, 1) == 1,
                duration: get(body, 21, 14) as u16,
                relayed_frame: get(body, 35, 1) == 1,
            }),
            TYPE_BLOCK_ACK => {
                let bitmap = get(body, 21, 16) as u16;
                let mut plain = body;
                let unprotected = get(body, 3, 16) ^ bitmap as u64;
                put(&mut plain, 3, 16, unprotected);
                NdpFrame::BlockAck(NdpBlockAck {
                    block_ack_id: get(plain, 3, 6) as u8,
                    starting_sequence: get(plain, 9, 12) as u16,
                    bitmap,
                })
            }
            _ => NdpFrame::Other { ndp_type, body },
        }
    }
}

/// Ack ID of the NDP Ack that acknowledges a PSDU scrambled with `seed`
/// whose soliciting MPDU ends in `fcs` (the 4 FCS octets as they appear in
/// the frame): Scrambler Initialization[0:6] ‖ FCS[23:31] in transmission
/// bit order [23.3.12.2.4.3].
pub fn ack_id(seed: u8, fcs: [u8; 4]) -> u16 {
    let mut id = (seed & 0x7f) as u16;
    for j in 0..9u32 {
        let t = 23 + j; // FCS bit index in transmission order
        let bit = (fcs[(t / 8) as usize] >> (t % 8)) & 1;
        id |= (bit as u16) << (7 + j);
    }
    id
}

/// Ack ID for a complete MPDU (with FCS) sent under `seed`.
pub fn ack_id_for_mpdu(seed: u8, mpdu: &[u8]) -> u16 {
    let n = mpdu.len();
    let fcs = [mpdu[n - 4], mpdu[n - 3], mpdu[n - 2], mpdu[n - 1]];
    ack_id(seed, fcs)
}

/// BlockAck ID: the 6 LSBs of the scrambler initialization [23.3.12.2.6.2].
pub fn block_ack_id(seed: u8) -> u8 {
    seed & 0x3f
}

/// OCB stand-in for the 9-bit partial AID of a station [10.21]: there is
/// no AID without an association, so the low 9 bits of the CRC-32 of the
/// MAC address are used. Deliberately nonstandard (documented in the crate
/// docs); both ends of an OCB link derive it the same way.
pub fn ocb_partial_aid(addr: &MacAddr) -> u16 {
    (fcs::crc32(addr) & 0x1ff) as u16
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cts_roundtrip() {
        let c = NdpCts { address_indicator: false, ra_pbssid: 0x1a5, duration_us: 12345, early_sector_indicator: false, bandwidth: 1 };
        let b = NdpFrame::Cts(c).to_body();
        assert_eq!(b >> 37, 0);
        assert_eq!(b & 7, 0);
        assert_eq!(NdpFrame::parse(b), NdpFrame::Cts(c));
    }

    #[test]
    fn ack_roundtrip_and_id() {
        let a = NdpAck { ack_id: 0xBEEF, more_data: true, idle_indication: false, duration: 0x2ABC, relayed_frame: true };
        let b = NdpFrame::Ack(a).to_body();
        assert_eq!(b & 7, 2);
        assert_eq!(NdpFrame::parse(b), NdpFrame::Ack(a));
        // Ack ID: seed bits first, then FCS bits 23..31.
        let fcs = [0x00, 0x00, 0x80, 0b1010_1100];
        let id = ack_id(0x55, fcs);
        assert_eq!(id & 0x7f, 0x55);
        assert_eq!((id >> 7) & 1, 1); // FCS bit 23 = f2 bit 7
        assert_eq!(id >> 8, 0b1010_1100); // FCS bits 24..31 = f3
    }

    #[test]
    fn block_ack_roundtrip_with_bitmap_protection() {
        let ba = NdpBlockAck { block_ack_id: 0x2a, starting_sequence: 0xabc, bitmap: 0xf00f };
        let b = NdpFrame::BlockAck(ba).to_body();
        assert_eq!(b & 7, 4);
        // Protected field differs from the plain one; the top two SSC bits
        // (B19, B20) are outside the protected range.
        assert_ne!(get(b, 3, 16), 0x2a | ((0xabc & 0x3ff) << 6));
        assert_eq!(get(b, 19, 2), (0xabc >> 10) & 3);
        assert_eq!(NdpFrame::parse(b), NdpFrame::BlockAck(ba));
        assert_eq!(block_ack_id(0x7f), 0x3f);
    }

    #[test]
    fn other_types_pass_through() {
        let body = (TYPE_PAGING as u64) | (0x1234_5678u64 << 3);
        match NdpFrame::parse(body) {
            NdpFrame::Other { ndp_type, body: b } => {
                assert_eq!(ndp_type, TYPE_PAGING);
                assert_eq!(b, body & BODY_MASK);
            }
            other => panic!("{other:?}"),
        }
        // NDP CF-End (type 0, indicator 1) is not a CTS.
        assert!(matches!(NdpFrame::parse(8), NdpFrame::Other { ndp_type: 0, .. }));
    }

    #[test]
    fn partial_aid_is_9_bits_and_stable() {
        let a = ocb_partial_aid(&[2, 0, 0, 0, 0, 0xA]);
        assert!(a < 512);
        assert_eq!(a, ocb_partial_aid(&[2, 0, 0, 0, 0, 0xA]));
        assert_ne!(a, ocb_partial_aid(&[2, 0, 0, 0, 0, 0xB]));
    }
}
