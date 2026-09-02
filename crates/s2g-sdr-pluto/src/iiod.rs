//! Minimal client for the iiod network protocol (legacy text mode).
//! Verified against libiio `iiod-client.c` / `iiod/parser.y`; see
//! docs/iiod-protocol.md at the workspace root.

use s2g_sdr::SdrError;
use std::io::{Read, Write};
use std::net::{TcpStream, ToSocketAddrs};
use std::time::Duration;

/// Channel direction for attribute access.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Dir {
    Input,
    Output,
}

impl Dir {
    fn token(self) -> &'static str {
        match self {
            Dir::Input => "INPUT",
            Dir::Output => "OUTPUT",
        }
    }
}

pub struct Client {
    s: TcpStream,
    rd: Vec<u8>,
    pos: usize,
    /// iiod sends the channel-mask line only with the first READBUF chunk
    /// after OPEN (`thd->new_client` in iiod/ops.c).
    mask_pending: bool,
}

fn ioerr(e: std::io::Error) -> SdrError {
    SdrError::Stream(format!("iiod I/O: {e}"))
}

fn reterr(what: &str, code: i64) -> SdrError {
    SdrError::Backend(format!("iiod {what} failed: retcode {code} (−errno)"))
}

impl Client {
    pub fn connect(host: &str, port: u16, timeout: Duration) -> Result<Self, SdrError> {
        let addr = (host, port)
            .to_socket_addrs()
            .map_err(|e| SdrError::NotFound(format!("resolve {host}: {e}")))?
            .next()
            .ok_or_else(|| SdrError::NotFound(format!("no address for {host}")))?;
        let s = TcpStream::connect_timeout(&addr, timeout).map_err(|e| SdrError::NotFound(format!("connect {addr}: {e}")))?;
        s.set_read_timeout(Some(Duration::from_secs(10))).map_err(ioerr)?;
        s.set_write_timeout(Some(Duration::from_secs(10))).map_err(ioerr)?;
        s.set_nodelay(true).ok();
        Ok(Self { s, rd: Vec::new(), pos: 0, mask_pending: false })
    }

    fn fill(&mut self) -> Result<(), SdrError> {
        let mut tmp = [0u8; 65536];
        let n = self.s.read(&mut tmp).map_err(ioerr)?;
        if n == 0 {
            return Err(SdrError::Stream("iiod connection closed".into()));
        }
        self.rd.drain(..self.pos);
        self.pos = 0;
        self.rd.extend_from_slice(&tmp[..n]);
        Ok(())
    }

    fn read_byte(&mut self) -> Result<u8, SdrError> {
        while self.pos >= self.rd.len() {
            self.fill()?;
        }
        let b = self.rd[self.pos];
        self.pos += 1;
        Ok(b)
    }

    /// Read a `\n`-terminated line (CR stripped), skipping empty lines.
    fn read_line(&mut self) -> Result<String, SdrError> {
        loop {
            let mut line = Vec::new();
            loop {
                let b = self.read_byte()?;
                if b == b'\n' {
                    break;
                }
                line.push(b);
            }
            while line.last() == Some(&b'\r') || line.last() == Some(&0) {
                line.pop();
            }
            if !line.is_empty() {
                return Ok(String::from_utf8_lossy(&line).into_owned());
            }
        }
    }

    fn read_ret(&mut self, what: &str) -> Result<i64, SdrError> {
        let line = self.read_line()?;
        line.trim()
            .parse::<i64>()
            .map_err(|_| SdrError::Backend(format!("iiod {what}: unparseable retcode {line:?}")))
    }

    fn read_exact_n(&mut self, n: usize) -> Result<Vec<u8>, SdrError> {
        let mut out = Vec::with_capacity(n);
        while out.len() < n {
            if self.pos < self.rd.len() {
                let take = (self.rd.len() - self.pos).min(n - out.len());
                out.extend_from_slice(&self.rd[self.pos..self.pos + take]);
                self.pos += take;
            } else {
                self.fill()?;
            }
        }
        Ok(out)
    }

    fn send(&mut self, cmd: &str) -> Result<(), SdrError> {
        self.s.write_all(cmd.as_bytes()).map_err(ioerr)?;
        self.s.write_all(b"\r\n").map_err(ioerr)
    }

    /// Fetch the context XML (PRINT).
    pub fn ctx_xml(&mut self) -> Result<String, SdrError> {
        self.send("PRINT")?;
        let n = self.read_ret("PRINT")?;
        if n < 0 {
            return Err(reterr("PRINT", n));
        }
        let raw = self.read_exact_n(n as usize)?;
        Ok(String::from_utf8_lossy(&raw).into_owned())
    }

    pub fn set_timeout_ms(&mut self, ms: u32) -> Result<(), SdrError> {
        self.send(&format!("TIMEOUT {ms}"))?;
        let r = self.read_ret("TIMEOUT")?;
        if r < 0 {
            return Err(reterr("TIMEOUT", r));
        }
        Ok(())
    }

    pub fn set_buffers_count(&mut self, dev: &str, n: u32) -> Result<(), SdrError> {
        self.send(&format!("SET {dev} BUFFERS_COUNT {n}"))?;
        let r = self.read_ret("SET BUFFERS_COUNT")?;
        if r < 0 {
            return Err(reterr("SET BUFFERS_COUNT", r));
        }
        Ok(())
    }

    pub fn attr_read(&mut self, dev: &str, chan: Option<(Dir, &str)>, attr: &str) -> Result<String, SdrError> {
        let cmd = match chan {
            Some((d, c)) => format!("READ {dev} {} {c} {attr}", d.token()),
            None => format!("READ {dev} {attr}"),
        };
        self.send(&cmd)?;
        let n = self.read_ret("READ")?;
        if n < 0 {
            return Err(reterr(&format!("READ {attr}"), n));
        }
        let raw = self.read_exact_n(n as usize)?;
        Ok(String::from_utf8_lossy(&raw).trim_matches(['\0', '\n', '\r', ' ']).to_string())
    }

    pub fn attr_write(&mut self, dev: &str, chan: Option<(Dir, &str)>, attr: &str, value: &str) -> Result<(), SdrError> {
        let payload = value.as_bytes();
        let cmd = match chan {
            Some((d, c)) => format!("WRITE {dev} {} {c} {attr} {}", d.token(), payload.len()),
            None => format!("WRITE {dev} {attr} {}", payload.len()),
        };
        self.send(&cmd)?;
        self.s.write_all(payload).map_err(ioerr)?;
        let r = self.read_ret("WRITE")?;
        if r < 0 {
            return Err(reterr(&format!("WRITE {attr}={value}"), r));
        }
        Ok(())
    }

    pub fn open(&mut self, dev: &str, samples: usize, mask: u32, cyclic: bool) -> Result<(), SdrError> {
        let cyc = if cyclic { " CYCLIC" } else { "" };
        self.send(&format!("OPEN {dev} {samples} {mask:08x}{cyc}"))?;
        let r = self.read_ret("OPEN")?;
        if r < 0 {
            return Err(reterr("OPEN", r));
        }
        self.mask_pending = true;
        Ok(())
    }

    pub fn close(&mut self, dev: &str) -> Result<(), SdrError> {
        self.send(&format!("CLOSE {dev}"))?;
        let r = self.read_ret("CLOSE")?;
        if r < 0 {
            return Err(reterr("CLOSE", r));
        }
        Ok(())
    }

    /// Read up to `nbytes` of raw buffer data. READBUF framing as iiod
    /// implements it (iiod/ops.c `rw_dev` / `send_data`, libiio
    /// `iiod_client_read_unlocked`): a retcode line with the chunk size,
    /// a hex channel-mask line only with the first chunk after OPEN, then
    /// the raw bytes; chunks repeat until `nbytes` have arrived, and a
    /// retcode of 0 (sent only after a partial delivery) ends the read.
    pub fn readbuf(&mut self, dev: &str, nbytes: usize) -> Result<Vec<u8>, SdrError> {
        self.send(&format!("READBUF {dev} {nbytes}"))?;
        let mut out = Vec::with_capacity(nbytes);
        loop {
            let n = self.read_ret("READBUF")?;
            if n < 0 {
                return Err(reterr("READBUF", n));
            }
            if n == 0 {
                break;
            }
            if self.mask_pending {
                let _mask = self.read_line()?;
                self.mask_pending = false;
            }
            out.extend(self.read_exact_n(n as usize)?);
            if out.len() >= nbytes {
                break;
            }
        }
        Ok(out)
    }

    /// Write raw buffer data (WRITEBUF two-ack flow). Returns bytes consumed.
    pub fn writebuf(&mut self, dev: &str, data: &[u8]) -> Result<usize, SdrError> {
        self.send(&format!("WRITEBUF {dev} {}", data.len()))?;
        let ok = self.read_ret("WRITEBUF")?;
        if ok < 0 {
            return Err(reterr("WRITEBUF", ok));
        }
        self.s.write_all(data).map_err(ioerr)?;
        let n = self.read_ret("WRITEBUF ack")?;
        if n < 0 {
            return Err(reterr("WRITEBUF ack", n));
        }
        Ok(n as usize)
    }
}

/// Map iio device names to ids from the context XML (e.g. "ad9361-phy" →
/// "iio:device1"). Tiny string scan — the XML is machine-generated.
pub fn device_id_by_name(xml: &str, name: &str) -> Option<String> {
    let needle = format!("name=\"{name}\"");
    for seg in xml.split("<device ") {
        if seg.contains(&needle) {
            if let Some(idpos) = seg.find("id=\"") {
                let rest = &seg[idpos + 4..];
                if let Some(end) = rest.find('"') {
                    return Some(rest[..end].to_string());
                }
            }
        }
    }
    None
}
