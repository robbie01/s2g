//! Local network attachment for `s2g-node`: where decoded Ethernet frames
//! go and where outgoing ones come from.
//!
//! Backends:
//! - [`TapNic`] (Unix: Linux/macOS/*BSD, feature `tap`): a real L2 TAP
//!   interface via `tappers`; the OS routes traffic through the radio.
//! - `wintap::WinTapNic` (Windows): the OpenVPN tap-windows6 adapter.
//! - [`UdpNic`] (all platforms): raw Ethernet frames as UDP datagrams
//!   to/from a local endpoint. Anything that speaks this framing (another
//!   s2g-node's UDP side, a test script, a user-space bridge) can attach.

use anyhow::{Context, Result};
use std::net::{SocketAddr, UdpSocket};
use std::time::Duration;

/// Largest Ethernet frame carried (header + MTU, no FCS at this layer).
pub const MAX_FRAME: usize = 1600;

pub trait Nic: Send {
    /// Blocking-with-timeout read of one Ethernet frame; `Ok(None)` on
    /// timeout.
    fn recv_frame(&mut self, timeout: Duration) -> Result<Option<Vec<u8>>>;
    /// Deliver one Ethernet frame to the local side.
    fn send_frame(&mut self, frame: &[u8]) -> Result<()>;
    fn describe(&self) -> String;
}

/// Ethernet-over-UDP endpoint.
pub struct UdpNic {
    sock: UdpSocket,
    peer: Option<SocketAddr>,
    buf: Vec<u8>,
    timeout: Option<Duration>,
}

impl UdpNic {
    /// `bind` locally; if `peer` is None, the NIC locks onto the first
    /// sender it hears (convenient for point-to-point testing).
    pub fn new(bind: &str, peer: Option<&str>) -> Result<Self> {
        let sock = UdpSocket::bind(bind).with_context(|| format!("bind {bind}"))?;
        let peer = match peer {
            Some(p) => Some(p.parse().with_context(|| format!("peer addr {p}"))?),
            None => None,
        };
        Ok(Self { sock, peer, buf: vec![0u8; MAX_FRAME], timeout: None })
    }
}

impl Nic for UdpNic {
    fn recv_frame(&mut self, timeout: Duration) -> Result<Option<Vec<u8>>> {
        if self.timeout != Some(timeout) {
            self.sock.set_read_timeout(Some(timeout))?;
            self.timeout = Some(timeout);
        }
        match self.sock.recv_from(&mut self.buf) {
            Ok((n, from)) => {
                if self.peer.is_none() {
                    self.peer = Some(from);
                }
                Ok(Some(self.buf[..n].to_vec()))
            }
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock || e.kind() == std::io::ErrorKind::TimedOut => Ok(None),
            Err(e) => Err(e.into()),
        }
    }

    fn send_frame(&mut self, frame: &[u8]) -> Result<()> {
        if let Some(peer) = self.peer {
            self.sock.send_to(frame, peer)?;
        }
        Ok(())
    }

    fn describe(&self) -> String {
        format!("udp {} ↔ {:?}", self.sock.local_addr().map(|a| a.to_string()).unwrap_or_default(), self.peer)
    }
}

/// Real TAP interface (Unix only, feature `tap`).
#[cfg(all(unix, feature = "tap"))]
pub struct TapNic {
    tap: tappers::Tap,
    name: String,
    buf: Vec<u8>,
}

#[cfg(all(unix, feature = "tap"))]
impl TapNic {
    pub fn new(name: Option<&str>) -> Result<Self> {
        let mut tap = match name {
            Some(n) => tappers::Tap::new_named(tappers::Interface::new(n)?)?,
            None => tappers::Tap::new()?,
        };
        tap.set_up()?;
        tap.set_nonblocking(true)?;
        let name = tap.name().map(|i| i.name().to_string_lossy().into_owned()).unwrap_or_else(|_| "?".into());
        Ok(Self { tap, name, buf: vec![0u8; MAX_FRAME] })
    }
}

#[cfg(all(unix, feature = "tap"))]
impl Nic for TapNic {
    fn recv_frame(&mut self, timeout: Duration) -> Result<Option<Vec<u8>>> {
        // Nonblocking + short sleep keeps the trait simple and portable.
        match self.tap.recv(&mut self.buf) {
            Ok(n) => Ok(Some(self.buf[..n].to_vec())),
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                std::thread::sleep(timeout.min(Duration::from_millis(2)));
                Ok(None)
            }
            Err(e) => Err(e.into()),
        }
    }

    fn send_frame(&mut self, frame: &[u8]) -> Result<()> {
        self.tap.send(frame)?;
        Ok(())
    }

    fn describe(&self) -> String {
        format!("tap {}", self.name)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn udp_nic_roundtrip() {
        let mut a = UdpNic::new("127.0.0.1:0", None).unwrap();
        let a_addr = a.sock.local_addr().unwrap();
        let mut b = UdpNic::new("127.0.0.1:0", Some(&a_addr.to_string())).unwrap();
        b.send_frame(b"hello frame").unwrap();
        let got = a.recv_frame(Duration::from_millis(500)).unwrap().unwrap();
        assert_eq!(got, b"hello frame");
        // a learned b as peer; reply works.
        a.send_frame(b"reply").unwrap();
        let got2 = b.recv_frame(Duration::from_millis(500)).unwrap().unwrap();
        assert_eq!(got2, b"reply");
    }
}
