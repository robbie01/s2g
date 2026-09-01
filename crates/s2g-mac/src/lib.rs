//! OCB (outside-the-context-of-a-BSS) MAC for the S1G PHY.
//!
//! Deliberately nonstandard where the spec assumes a BSS, per this project's
//! goals; standard where the wire format matters:
//! - 802.11 Data frames with the **wildcard BSSID** (OCB convention), FCS
//!   (CRC-32), sequence numbers, duplicate detection.
//! - PSDUs ≤ 511 octets go non-aggregated; larger frames ride a
//!   spec-format **A-MPDU** (delimiters with CRC-8 + 0x4E signature, EOF
//!   padding) since the SIG Length field caps non-aggregated PSDUs.
//! - Ethernet payloads are carried with RFC 1042 LLC/SNAP encapsulation, so
//!   a TAP interface plugs straight in.
//! - Channel access: CSMA with DIFS + binary-exponential backoff, and
//!   optional ACK + retry for unicast. Timing constants come from the PHY
//!   (`slot 52 µs / SIFS 160 µs`), but the ACK timeout defaults are relaxed
//!   far beyond SIFS because buffered SDR streaming adds tens of ms of
//!   latency — real SIFS turnaround needs hardware timestamping.
//!
//! The engine ([`Mac`]) is IO-free and clock-injected: callers push PHY
//! events and Ethernet frames in, poll actions out. That keeps it fully
//! testable and reusable for an RX-only or non-Pluto system.

pub mod ampdu;
pub mod engine;
pub mod eth;
pub mod fcs;
pub mod frame;

pub use engine::{Mac, MacAction, MacConfig, MacEvent};
pub use frame::MacAddr;
