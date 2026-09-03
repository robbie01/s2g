//! OCB (outside-the-context-of-a-BSS) MAC for the S1G PHY.
//!
//! Deliberately nonstandard where the spec assumes a BSS, per this project's
//! goals; standard where the wire format matters:
//! - 802.11 Data frames with the wildcard BSSID (OCB convention), FCS
//!   (CRC-32), sequence numbers, duplicate detection, RTS.
//! - PSDUs ≤ 511 octets go non-aggregated; larger frames ride a
//!   spec-format A-MPDU (delimiters with CRC-8 + 0x4E signature, EOF
//!   padding) since the SIG Length field caps non-aggregated PSDUs.
//! - NDP CMAC PPDUs [23.3.12]: NDP Ack, NDP BlockAck and NDP CTS are
//!   generated and consumed as the standard lays them out (Ack ID from
//!   scrambler seed + FCS, BlockAck bitmap protection, CTS duration
//!   arithmetic). The only OCB liberty is the 9-bit partial AID, which has
//!   no AID to derive from and so comes from the MAC address
//!   (`ndp::ocb_partial_aid`).
//! - RID (response indication deferral) and NAV are honored; CCA comes
//!   from the PHY.
//! - Ethernet payloads are carried with RFC 1042 LLC/SNAP encapsulation, so
//!   a TAP interface plugs straight in.
//! - Channel access: CSMA with DIFS + binary-exponential backoff, and
//!   optional acknowledgment + retry for unicast. Timing constants come
//!   from the PHY (`slot 52 µs / SIFS 160 µs`), but the response wait is
//!   on the 10-100 ms scale because buffered SDR streaming adds tens of ms
//!   of latency; real SIFS turnaround needs hardware timestamping.
//!
//! The engine ([`Mac`]) is IO-free and clock-injected: callers push PHY
//! events and Ethernet frames in, poll actions out. That keeps it fully
//! testable and reusable for an RX-only or non-Pluto system.

pub mod ampdu;
pub mod engine;
pub mod eth;
pub mod fcs;
pub mod frame;
pub mod filter;
pub mod ident;
pub mod ndp;
pub mod rate;

pub use engine::{Mac, MacAction, MacConfig, MacError, MacEvent};
pub use filter::{FilterConfig, Verdict};
pub use ident::IdentConfig;
pub use rate::{PeerRateInfo, RateConfig, RateControl};
pub use frame::MacAddr;
pub use ndp::NdpFrame;
