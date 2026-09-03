//! Station identification for amateur-radio operation.
//!
//! An amateur station must transmit its call sign at the start of a
//! communication, at least every ten minutes during it, and at its end
//! (47 CFR 97.119). The MAC does this with a broadcast Data frame whose
//! body is an RFC 1042 LLC/SNAP payload with EtherType 0x88B5 (IEEE 802a
//! "local experimental EtherType 1") carrying plain ASCII of the form
//!
//! ```text
//! DE <CALLSIGN> [free text]
//! ```
//!
//! Plain 7-bit ASCII in a publicly documented frame is a "digital code"
//! readable in any monitor-mode capture, by design. The frame goes out at
//! MCS 0 so it is readable at the edge of range, and it is never
//! acknowledged, aggregated or encrypted. Nothing else in this
//! MAC obscures the meaning of a transmission either: s2g does not
//! encrypt, and a station operating under Part 97 must not run an
//! encrypting upper layer over it (97.113(a)(4)).

use crate::eth;

/// EtherType carried in the LLC/SNAP header of an identification frame.
pub const ETHERTYPE_IDENT: u16 = 0x88B5;

/// Identification settings.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct IdentConfig {
    /// Call sign to announce; `None` disables identification entirely
    /// (unlicensed-band testing, or a receiver-only station).
    pub callsign: Option<String>,
    /// Optional free text appended after the call sign (grid square, node
    /// name, a URL to this documentation).
    pub info: String,
    /// Longest interval between identifications while transmitting, µs.
    pub interval_us: u64,
    /// Idle time after the last data transmission that counts as the end
    /// of a communication and triggers a final identification, µs.
    pub end_idle_us: u64,
}

impl Default for IdentConfig {
    fn default() -> Self {
        Self { callsign: None, info: String::new(), interval_us: 600_000_000, end_idle_us: 30_000_000 }
    }
}

/// The 802.11 frame body of an identification frame.
pub fn body(callsign: &str, info: &str) -> Vec<u8> {
    let mut text = format!("DE {}", callsign.trim().to_ascii_uppercase());
    let info = info.trim();
    if !info.is_empty() {
        text.push(' ');
        text.push_str(info);
    }
    let ascii: String = text.chars().filter(|c| c.is_ascii() && !c.is_ascii_control()).collect();
    eth::to_body(ETHERTYPE_IDENT, ascii.as_bytes())
}

/// The text of an identification frame body, if it is one.
pub fn parse_body(body: &[u8]) -> Option<String> {
    let (ethertype, payload) = eth::split_body(body)?;
    if ethertype != ETHERTYPE_IDENT {
        return None;
    }
    let text = String::from_utf8_lossy(payload).into_owned();
    text.starts_with("DE ").then_some(text)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn body_roundtrip() {
        let b = body("w1aw", "FN31 s2g node");
        let text = parse_body(&b).unwrap();
        assert_eq!(text, "DE W1AW FN31 s2g node");
        // Not an identification frame: ordinary IP payload.
        assert!(parse_body(&eth::to_body(0x0800, b"ip")).is_none());
        // Control characters and non-ASCII are dropped.
        let b = body("W1AW", "caf\u{e9}\n");
        assert_eq!(parse_body(&b).unwrap(), "DE W1AW caf");
    }
}
