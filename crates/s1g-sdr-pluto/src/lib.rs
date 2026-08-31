//! ADALM-Pluto (AD9363) backend implementing the `s1g-sdr` traits via a
//! pure-Rust client for the iiod network protocol (TCP port 30431) — the
//! same path libiio's `ip:` backend uses. No native dependencies.
//!
//! Protocol and device/attribute reference: `docs/iiod-protocol.md`.
//! Control, RX streaming, and TX streaming each use their own TCP
//! connection (iiod holds one buffer per connection).

pub mod iiod;

use iiod::{Client, Dir};
use s1g_sdr::{RxGain, SdrDevice, SdrError, SdrRx, SdrTx, StreamConfig};
use std::time::Duration;

pub type Complex32 = num_complex::Complex<f32>;

pub const IIOD_PORT: u16 = 30431;
pub const DEFAULT_HOST: &str = "192.168.2.1";

const PHY: &str = "ad9361-phy";
const RX_DEV: &str = "cf-ad9361-lpc";
const TX_DEV: &str = "cf-ad9361-dds-core-lpc";
/// I+Q channels (voltage0 | voltage1).
const IQ_MASK: u32 = 0x3;
/// Stream buffer size in complex samples (× 4 bytes each).
const BUF_SAMPLES: usize = 16384;

/// A connected Pluto device (control connection).
pub struct Pluto {
    host: String,
    port: u16,
    ctl: Client,
    phy_id: String,
    rx_id: String,
    tx_id: String,
}

fn split_host(host: &str) -> (String, u16) {
    match host.rsplit_once(':') {
        Some((h, p)) if p.chars().all(|c| c.is_ascii_digit()) && !h.is_empty() => {
            (h.to_string(), p.parse().unwrap_or(IIOD_PORT))
        }
        _ => (host.to_string(), IIOD_PORT),
    }
}

impl Pluto {
    /// Connect to iiod at `host[:port]` (e.g. "192.168.2.1").
    pub fn open(host: &str) -> Result<Self, SdrError> {
        let (h, port) = split_host(host);
        let mut ctl = Client::connect(&h, port, Duration::from_secs(5))?;
        let xml = ctl.ctx_xml()?;
        let phy_id = iiod::device_id_by_name(&xml, PHY)
            .ok_or_else(|| SdrError::NotFound(format!("device '{PHY}' not in context — not a Pluto?")))?;
        let rx_id = iiod::device_id_by_name(&xml, RX_DEV)
            .ok_or_else(|| SdrError::NotFound(format!("device '{RX_DEV}' not in context")))?;
        let tx_id = iiod::device_id_by_name(&xml, TX_DEV)
            .ok_or_else(|| SdrError::NotFound(format!("device '{TX_DEV}' not in context")))?;
        Ok(Self { host: h, port, ctl, phy_id, rx_id, tx_id })
    }

    fn cfg_common(&mut self, cfg: &StreamConfig, dir: Dir, lo_chan: &str) -> Result<(), SdrError> {
        let rate = format!("{}", cfg.sample_rate_hz.round() as i64);
        let bw = format!("{}", cfg.rf_bandwidth_hz.round() as i64);
        let freq = format!("{}", cfg.center_freq_hz.round() as i64);
        // LO channels are outputs on the phy device.
        self.ctl.attr_write(&self.phy_id, Some((Dir::Output, lo_chan)), "frequency", &freq)?;
        self.ctl.attr_write(&self.phy_id, Some((dir, "voltage0")), "sampling_frequency", &rate)?;
        self.ctl.attr_write(&self.phy_id, Some((dir, "voltage0")), "rf_bandwidth", &bw)?;
        Ok(())
    }
}

pub struct PlutoRx {
    stream: Client,
    dev: String,
    rate: f64,
    /// Converted samples not yet handed to the caller.
    pending: Vec<Complex32>,
    pending_pos: usize,
}

pub struct PlutoTx {
    stream: Client,
    dev: String,
    rate: f64,
}

impl SdrDevice for Pluto {
    type Rx = PlutoRx;
    type Tx = PlutoTx;

    fn open_rx(&mut self, cfg: &StreamConfig, gain: RxGain) -> Result<PlutoRx, SdrError> {
        if cfg.sample_rate_hz < 2.083e6 {
            return Err(SdrError::Config(format!(
                "AD9363 cannot stream below ~2.083 MS/s (asked {}); run at 4 MS/s and decimate in s1g-dsp",
                cfg.sample_rate_hz
            )));
        }
        // RX LO = altvoltage0.
        self.cfg_common(cfg, Dir::Input, "altvoltage0")?;
        match gain {
            RxGain::Auto => {
                self.ctl.attr_write(&self.phy_id, Some((Dir::Input, "voltage0")), "gain_control_mode", "slow_attack")?;
            }
            RxGain::Manual(db) => {
                self.ctl.attr_write(&self.phy_id, Some((Dir::Input, "voltage0")), "gain_control_mode", "manual")?;
                self.ctl.attr_write(&self.phy_id, Some((Dir::Input, "voltage0")), "hardwaregain", &format!("{db:.3}"))?;
            }
        }
        let mut stream = Client::connect(&self.host, self.port, Duration::from_secs(5))?;
        stream.set_timeout_ms(3000)?;
        stream.set_buffers_count(&self.rx_id, 4)?;
        stream.open(&self.rx_id, BUF_SAMPLES, IQ_MASK, false)?;
        Ok(PlutoRx {
            stream,
            dev: self.rx_id.clone(),
            rate: cfg.sample_rate_hz,
            pending: Vec::new(),
            pending_pos: 0,
        })
    }

    fn open_tx(&mut self, cfg: &StreamConfig, tx_gain_db: f64) -> Result<PlutoTx, SdrError> {
        if cfg.sample_rate_hz < 2.083e6 {
            return Err(SdrError::Config("AD9363 cannot stream below ~2.083 MS/s; interpolate in s1g-dsp".into()));
        }
        // TX LO = altvoltage1.
        self.cfg_common(cfg, Dir::Output, "altvoltage1")?;
        // TX gain is attenuation: hardwaregain ≤ 0 dB.
        let g = if tx_gain_db > 0.0 { 0.0 } else { tx_gain_db };
        self.ctl.attr_write(&self.phy_id, Some((Dir::Output, "voltage0")), "hardwaregain", &format!("{g:.3}"))?;
        // Best effort: silence the DDS tone generators (the DMA buffer path
        // takes over on OPEN, but a stale DDS setup would mix in tones).
        for ch in ["altvoltage0", "altvoltage1", "altvoltage2", "altvoltage3"] {
            let _ = self.ctl.attr_write(&self.tx_id, Some((Dir::Output, ch)), "scale", "0");
        }
        let mut stream = Client::connect(&self.host, self.port, Duration::from_secs(5))?;
        stream.set_timeout_ms(3000)?;
        stream.set_buffers_count(&self.tx_id, 4)?;
        stream.open(&self.tx_id, BUF_SAMPLES, IQ_MASK, false)?;
        Ok(PlutoTx { stream, dev: self.tx_id.clone(), rate: cfg.sample_rate_hz })
    }
}

impl SdrRx for PlutoRx {
    fn recv(&mut self, buf: &mut [Complex32]) -> Result<usize, SdrError> {
        if self.pending_pos >= self.pending.len() {
            let raw = self.stream.readbuf(&self.dev, BUF_SAMPLES * 4)?;
            self.pending.clear();
            self.pending_pos = 0;
            // 12-bit samples sign-extended into LE i16 pairs; scale 1/2048.
            for four in raw.chunks_exact(4) {
                let i = i16::from_le_bytes([four[0], four[1]]) as f32 / 2048.0;
                let q = i16::from_le_bytes([four[2], four[3]]) as f32 / 2048.0;
                self.pending.push(Complex32::new(i, q));
            }
            if self.pending.is_empty() {
                return Err(SdrError::Stream("iiod returned an empty RX buffer".into()));
            }
        }
        let n = buf.len().min(self.pending.len() - self.pending_pos);
        buf[..n].copy_from_slice(&self.pending[self.pending_pos..self.pending_pos + n]);
        self.pending_pos += n;
        Ok(n)
    }

    fn sample_rate_hz(&self) -> f64 {
        self.rate
    }
}

impl SdrTx for PlutoTx {
    fn send(&mut self, samples: &[Complex32]) -> Result<(), SdrError> {
        // Pluto DAC uses the upper 12 bits: clamp ±1.0, scale ×2047, <<4.
        for chunk in samples.chunks(BUF_SAMPLES) {
            let mut raw = Vec::with_capacity(chunk.len() * 4);
            for s in chunk {
                let i = (s.re.clamp(-1.0, 1.0) * 2047.0) as i16;
                let q = (s.im.clamp(-1.0, 1.0) * 2047.0) as i16;
                raw.extend_from_slice(&(i << 4).to_le_bytes());
                raw.extend_from_slice(&(q << 4).to_le_bytes());
            }
            let mut written = 0usize;
            while written < raw.len() {
                let n = self.stream.writebuf(&self.dev, &raw[written..])?;
                if n == 0 {
                    return Err(SdrError::Stream("iiod consumed 0 bytes on WRITEBUF".into()));
                }
                written += n.min(raw.len() - written);
            }
        }
        Ok(())
    }

    fn sample_rate_hz(&self) -> f64 {
        self.rate
    }

    fn flush(&mut self) -> Result<(), SdrError> {
        // iiod pushes buffers as they complete; nothing further to do.
        Ok(())
    }
}

impl Drop for PlutoRx {
    fn drop(&mut self) {
        let _ = self.stream.close(&self.dev);
    }
}

impl Drop for PlutoTx {
    fn drop(&mut self) {
        let _ = self.stream.close(&self.dev);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::{BufRead, BufReader, Read, Write};
    use std::net::TcpListener;
    use std::sync::{Arc, Mutex};

    const XML: &str = r#"<?xml version="1.0"?><context name="network">
<device id="iio:device0" name="ad9361-phy" ><channel id="voltage0" type="input"/></device>
<device id="iio:device2" name="cf-ad9361-lpc" ></device>
<device id="iio:device3" name="cf-ad9361-dds-core-lpc" ></device>
</context>"#;

    /// Canned iiod server good enough for the client's command set.
    fn mock_server(written: Arc<Mutex<Vec<u8>>>, attrs: Arc<Mutex<Vec<String>>>) -> u16 {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let port = listener.local_addr().unwrap().port();
        std::thread::spawn(move || {
            for conn in listener.incoming() {
                let Ok(conn) = conn else { break };
                let written = written.clone();
                let attrs = attrs.clone();
                std::thread::spawn(move || {
                    let mut r = BufReader::new(conn.try_clone().unwrap());
                    let mut w = conn;
                    loop {
                        let mut line = String::new();
                        if r.read_line(&mut line).unwrap_or(0) == 0 {
                            break;
                        }
                        let line = line.trim_end();
                        if line.is_empty() {
                            continue;
                        }
                        let tok: Vec<&str> = line.split_whitespace().collect();
                        match tok[0] {
                            "PRINT" => {
                                write!(w, "{}\n{}", XML.len(), XML).unwrap();
                            }
                            "TIMEOUT" | "OPEN" | "CLOSE" | "SET" => {
                                write!(w, "0\n").unwrap();
                            }
                            "READ" => {
                                // READ dev [DIR chan] attr → value "42"
                                let v = "42";
                                write!(w, "{}\n{}\n", v.len(), v).unwrap();
                            }
                            "WRITE" => {
                                let len: usize = tok.last().unwrap().parse().unwrap();
                                let mut payload = vec![0u8; len];
                                r.read_exact(&mut payload).unwrap();
                                attrs.lock().unwrap().push(format!(
                                    "{} = {}",
                                    tok[1..tok.len() - 1].join(" "),
                                    String::from_utf8_lossy(&payload)
                                ));
                                write!(w, "{len}\n").unwrap();
                            }
                            "READBUF" => {
                                let want: usize = tok[2].parse().unwrap();
                                // Two chunks with a ramp of i16 pairs.
                                let mut data = Vec::with_capacity(want);
                                let mut v: i16 = -64;
                                while data.len() < want {
                                    data.extend_from_slice(&v.to_le_bytes());
                                    data.extend_from_slice(&(-v).to_le_bytes());
                                    v = v.wrapping_add(1);
                                }
                                data.truncate(want);
                                let half = want / 2;
                                write!(w, "{half}\n00000003\n").unwrap();
                                w.write_all(&data[..half]).unwrap();
                                write!(w, "{}\n00000003\n", want - half).unwrap();
                                w.write_all(&data[half..]).unwrap();
                                write!(w, "0\n").unwrap();
                            }
                            "WRITEBUF" => {
                                let len: usize = tok[2].parse().unwrap();
                                write!(w, "0\n").unwrap();
                                let mut payload = vec![0u8; len];
                                r.read_exact(&mut payload).unwrap();
                                written.lock().unwrap().extend_from_slice(&payload);
                                write!(w, "{len}\n").unwrap();
                            }
                            _ => {
                                write!(w, "-22\n").unwrap();
                            }
                        }
                        w.flush().unwrap();
                    }
                });
            }
        });
        port
    }

    #[test]
    fn full_client_flow_against_mock() {
        let written = Arc::new(Mutex::new(Vec::new()));
        let attrs = Arc::new(Mutex::new(Vec::new()));
        let port = mock_server(written.clone(), attrs.clone());

        let mut pluto = Pluto::open(&format!("127.0.0.1:{port}")).expect("open");
        assert_eq!(pluto.phy_id, "iio:device0");
        assert_eq!(pluto.rx_id, "iio:device2");
        assert_eq!(pluto.tx_id, "iio:device3");

        let cfg = StreamConfig { center_freq_hz: 1.25e9, sample_rate_hz: 4e6, rf_bandwidth_hz: 2.2e6 };
        let mut rx = pluto.open_rx(&cfg, RxGain::Manual(40.0)).expect("open_rx");
        let mut buf = vec![Complex32::new(0.0, 0.0); 100];
        let n = rx.recv(&mut buf).expect("recv");
        assert_eq!(n, 100);
        // Ramp check: first sample = (−64, 64)/2048.
        assert!((buf[0].re - (-64.0 / 2048.0)).abs() < 1e-6);
        assert!((buf[0].im - (64.0 / 2048.0)).abs() < 1e-6);
        assert!((buf[63].re - (-1.0 / 2048.0)).abs() < 1e-6);

        let mut tx = pluto.open_tx(&cfg, -10.0).expect("open_tx");
        let samples = [Complex32::new(0.5, -0.25), Complex32::new(1.5, -2.0)];
        tx.send(&samples).expect("send");
        {
            let w = written.lock().unwrap();
            // 0.5 → 1023 << 4; −0.25 → −511 << 4 ("as i16" truncates toward 0);
            // 1.5/−2.0 clamp to ±1 → ±2047 << 4.
            let i0 = i16::from_le_bytes([w[0], w[1]]);
            let q0 = i16::from_le_bytes([w[2], w[3]]);
            let i1 = i16::from_le_bytes([w[4], w[5]]);
            let q1 = i16::from_le_bytes([w[6], w[7]]);
            assert_eq!(i0, 1023 << 4);
            assert_eq!(q0, -511 << 4);
            assert_eq!(i1, 2047 << 4);
            assert_eq!(q1, -2047 << 4);
        }
        // Config attributes reached the phy device with the right values.
        let a = attrs.lock().unwrap().join("\n");
        assert!(a.contains("iio:device0 OUTPUT altvoltage0 frequency = 1250000000"), "{a}");
        assert!(a.contains("iio:device0 OUTPUT altvoltage1 frequency = 1250000000"), "{a}");
        assert!(a.contains("iio:device0 INPUT voltage0 sampling_frequency = 4000000"), "{a}");
        assert!(a.contains("iio:device0 INPUT voltage0 gain_control_mode = manual"), "{a}");
        assert!(a.contains("iio:device0 INPUT voltage0 hardwaregain = 40.000"), "{a}");
        assert!(a.contains("iio:device0 OUTPUT voltage0 hardwaregain = -10.000"), "{a}");
    }

    #[test]
    fn rate_below_minimum_rejected() {
        let written = Arc::new(Mutex::new(Vec::new()));
        let attrs = Arc::new(Mutex::new(Vec::new()));
        let port = mock_server(written, attrs);
        let mut pluto = Pluto::open(&format!("127.0.0.1:{port}")).unwrap();
        let cfg = StreamConfig { center_freq_hz: 1.25e9, sample_rate_hz: 2e6, rf_bandwidth_hz: 2.2e6 };
        assert!(matches!(pluto.open_rx(&cfg, RxGain::Auto), Err(SdrError::Config(_))));
    }

    #[test]
    fn device_id_parsing() {
        assert_eq!(iiod::device_id_by_name(XML, "ad9361-phy").as_deref(), Some("iio:device0"));
        assert_eq!(iiod::device_id_by_name(XML, "nope"), None);
    }

    #[test]
    fn host_port_split() {
        assert_eq!(split_host("192.168.2.1"), ("192.168.2.1".into(), IIOD_PORT));
        assert_eq!(split_host("pluto.local:1234"), ("pluto.local".into(), 1234));
    }
}
