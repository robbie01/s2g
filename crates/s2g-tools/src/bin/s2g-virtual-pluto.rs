//! Virtual Pluto: an iiod server that hosts several fake AD9363 radios and
//! couples them over a simulated air interface, so `s2g-node` (or any
//! libiio client) runs its real streaming code path without hardware.
//!
//! Each radio listens on its own TCP port and speaks the same legacy iiod
//! text protocol the Pluto backend uses (PRINT, READ/WRITE attributes,
//! OPEN/CLOSE, READBUF/WRITEBUF, TIMEOUT, SET). What one radio writes to
//! its TX buffer is what the others read from their RX buffers, after path
//! loss, additive noise, a carrier offset from each radio's oscillator
//! error, and a fixed pipeline latency — all paced in real time at the
//! configured sample rate, with iiod-style back-pressure on the TX side.
//! Underruns happen exactly as on hardware: samples written after the air
//! clock has passed are lost.
//!
//!     s2g-virtual-pluto --radios 2 --base-port 31431 --ppm 0,20
//!     s2g-node --uri 127.0.0.1:31431 --udp 127.0.0.1:5001 --udp-peer 127.0.0.1:5002 --callsign N0CALL
//!     s2g-node --uri 127.0.0.1:31432 --udp 127.0.0.1:6001 --udp-peer 127.0.0.1:6002 --callsign N0CALL

use anyhow::Result;
use clap::Parser;
use num_complex::Complex;
use std::collections::HashMap;
use std::io::{BufRead, BufReader, Read, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

type C32 = Complex<f32>;

#[derive(Parser, Debug)]
#[command(name = "s2g-virtual-pluto", about = "iiod server hosting virtual Plutos coupled over a simulated air interface")]
struct Args {
    /// Number of virtual radios (one TCP port each)
    #[arg(long, default_value_t = 2)]
    radios: usize,
    /// First TCP port; radio k listens on base-port + k
    #[arg(long, default_value_t = 31431)]
    base_port: u16,
    /// Air sample rate the clients are expected to configure, S/s
    #[arg(long, default_value_t = 4_000_000.0)]
    rate: f64,
    /// Carrier frequency assumed for the oscillator-error model, Hz
    #[arg(long, default_value_t = 1.25e9)]
    freq: f64,
    /// Oscillator error of each radio in ppm, comma separated (missing entries = 0)
    #[arg(long, default_value = "0")]
    ppm: String,
    /// Path loss between any two radios, dB
    #[arg(long, default_value_t = 30.0)]
    path_loss_db: f64,
    /// Receiver noise floor, dBFS per complex sample (a Pluto at moderate gain
    /// sits near -60; below about -66 the 12-bit quantiser swallows it)
    #[arg(long, default_value_t = -60.0)]
    noise_dbfs: f64,
    /// Propagation delay between radios, samples
    #[arg(long, default_value_t = 0)]
    delay_samples: u64,
    /// RX pipeline latency in buffers of 16384 samples (iiod keeps this many queued)
    #[arg(long, default_value_t = 2)]
    rx_latency_buffers: u64,
    /// TX buffers a client may queue ahead of the air clock before WRITEBUF blocks
    #[arg(long, default_value_t = 4)]
    tx_ahead_buffers: u64,
    /// Print a line per buffer exchanged
    #[arg(long)]
    verbose: bool,
}

const BUF_SAMPLES: u64 = 16384;
const XML: &str = r#"<?xml version="1.0" encoding="utf-8"?><context name="network" description="s2g virtual pluto">
<device id="iio:device0" name="ad9361-phy"><channel id="voltage0" type="input"/><channel id="voltage0" type="output"/><channel id="altvoltage0" type="output"/><channel id="altvoltage1" type="output"/></device>
<device id="iio:device2" name="cf-ad9361-lpc"><channel id="voltage0" type="input"/><channel id="voltage1" type="input"/></device>
<device id="iio:device3" name="cf-ad9361-dds-core-lpc"><channel id="voltage0" type="output"/><channel id="voltage1" type="output"/></device>
</context>"#;

/// One radio's transmit stream on the shared air timeline.
struct TxStream {
    /// Air sample index of `buf[0]`.
    origin: u64,
    buf: Vec<C32>,
}

impl TxStream {
    fn end(&self) -> u64 {
        self.origin + self.buf.len() as u64
    }
    fn sample(&self, t: u64) -> C32 {
        if t >= self.origin && t < self.end() {
            self.buf[(t - self.origin) as usize]
        } else {
            C32::new(0.0, 0.0)
        }
    }
    /// Drop everything older than `keep_from`.
    fn trim(&mut self, keep_from: u64) {
        if keep_from > self.origin {
            let n = ((keep_from - self.origin) as usize).min(self.buf.len());
            self.buf.drain(..n);
            self.origin += n as u64;
        }
    }
}

struct Radio {
    tx: TxStream,
    /// Next air sample the RX stream delivers.
    rx_pos: Option<u64>,
    ppm: f64,
    attrs: HashMap<String, String>,
}

struct Air {
    t0: Instant,
    rate: f64,
    freq: f64,
    gain: f32,
    noise_amp: f32,
    delay: u64,
    radios: Vec<Radio>,
    rng: u64,
}

impl Air {
    fn now(&self) -> u64 {
        (self.t0.elapsed().as_secs_f64() * self.rate) as u64
    }

    fn gauss(&mut self) -> f32 {
        let mut s = 0.0f32;
        for _ in 0..6 {
            self.rng = self.rng.wrapping_add(0x9E37_79B9_7F4A_7C15);
            let mut z = self.rng;
            z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
            z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
            z ^= z >> 31;
            s += (z >> 32) as f32 / (1u64 << 31) as f32 - 1.0;
        }
        s / (2.0f32).sqrt()
    }

    /// What radio `k` hears during air samples [from, from + n).
    fn receive(&mut self, k: usize, from: u64, n: usize) -> Vec<C32> {
        let mut out = vec![C32::new(0.0, 0.0); n];
        let others: Vec<usize> = (0..self.radios.len()).filter(|&j| j != k).collect();
        for j in others {
            let dppm = self.radios[j].ppm - self.radios[k].ppm;
            let cfo_hz = dppm * 1e-6 * self.freq;
            let w = 2.0 * std::f64::consts::PI * cfo_hz / self.rate;
            for (i, o) in out.iter_mut().enumerate() {
                let t = from + i as u64;
                let s = self.radios[j].tx.sample(t.wrapping_sub(self.delay));
                if s.re != 0.0 || s.im != 0.0 {
                    let rot = C32::from_polar(1.0, (w * t as f64 % (2.0 * std::f64::consts::PI)) as f32);
                    *o += s * self.gain * rot;
                }
            }
        }
        let na = self.noise_amp * std::f32::consts::FRAC_1_SQRT_2;
        for o in out.iter_mut() {
            *o += C32::new(self.gauss() * na, self.gauss() * na);
        }
        out
    }
}

fn serve(conn: TcpStream, k: usize, air: Arc<Mutex<Air>>, args: Arc<Args>) {
    let mut r = BufReader::new(conn.try_clone().expect("clone"));
    let mut w = conn;
    // iiod sends the channel mask only with the first READBUF after OPEN.
    let mut new_client = false;
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
        let reply: Result<(), std::io::Error> = match tok[0] {
            "PRINT" => write!(w, "{}\n{}", XML.len(), XML),
            "VERSION" => write!(w, "0\n"),
            "TIMEOUT" | "SET" | "CLOSE" => {
                write!(w, "0\n")
            }
            "EXIT" => break,
            "OPEN" => {
                new_client = true;
                if tok.get(1) == Some(&"iio:device2") {
                    let mut a = air.lock().unwrap();
                    let now = a.now();
                    a.radios[k].rx_pos = Some(now);
                }
                write!(w, "0\n")
            }
            "READ" => {
                let attr = tok.last().copied().unwrap_or("");
                let key = tok[1..].join(" ");
                let a = air.lock().unwrap();
                let v = a.radios[k]
                    .attrs
                    .get(&key)
                    .cloned()
                    .unwrap_or_else(|| default_attr(attr).to_string());
                write!(w, "{}\n{}\n", v.len(), v)
            }
            "WRITE" => {
                let len: usize = tok.last().and_then(|s| s.parse().ok()).unwrap_or(0);
                let mut payload = vec![0u8; len];
                if r.read_exact(&mut payload).is_err() {
                    break;
                }
                let key = tok[1..tok.len() - 1].join(" ");
                let val = String::from_utf8_lossy(&payload).trim().to_string();
                if args.verbose {
                    eprintln!("radio {k}: {key} = {val}");
                }
                air.lock().unwrap().radios[k].attrs.insert(key, val);
                write!(w, "{len}\n")
            }
            "READBUF" => {
                let want: usize = tok.get(2).and_then(|s| s.parse().ok()).unwrap_or(0);
                let n = want / 4;
                // Pace: samples are available once the air clock has passed
                // them plus the pipeline latency.
                let (from, ready_at) = {
                    let mut a = air.lock().unwrap();
                    let from = a.radios[k].rx_pos.unwrap_or_else(|| a.now());
                    a.radios[k].rx_pos = Some(from);
                    (from, from + n as u64 + args.rx_latency_buffers * BUF_SAMPLES)
                };
                loop {
                    let now = air.lock().unwrap().now();
                    if now >= ready_at {
                        break;
                    }
                    let wait = (ready_at - now) as f64 / args.rate;
                    std::thread::sleep(Duration::from_secs_f64(wait.clamp(0.0005, 0.05)));
                }
                let samples = {
                    let mut a = air.lock().unwrap();
                    let s = a.receive(k, from, n);
                    a.radios[k].rx_pos = Some(from + n as u64);
                    s
                };
                let mut raw = Vec::with_capacity(n * 4);
                for s in &samples {
                    let i = (s.re.clamp(-1.0, 1.0) * 2047.0) as i16;
                    let q = (s.im.clamp(-1.0, 1.0) * 2047.0) as i16;
                    raw.extend_from_slice(&i.to_le_bytes());
                    raw.extend_from_slice(&q.to_le_bytes());
                }
                if args.verbose {
                    eprintln!("radio {k}: RX {} samples from {from}", n);
                }
                let mask = if new_client { "00000003\n" } else { "" };
                new_client = false;
                write!(w, "{}\n{mask}", raw.len()).and_then(|_| w.write_all(&raw))
            }
            "WRITEBUF" => {
                let len: usize = tok.get(2).and_then(|s| s.parse().ok()).unwrap_or(0);
                if write!(w, "0\n").and_then(|_| w.flush()).is_err() {
                    break;
                }
                let mut payload = vec![0u8; len];
                if r.read_exact(&mut payload).is_err() {
                    break;
                }
                let samples: Vec<C32> = payload
                    .chunks_exact(4)
                    .map(|c| {
                        let i = i16::from_le_bytes([c[0], c[1]]) as f32 / 32768.0;
                        let q = i16::from_le_bytes([c[2], c[3]]) as f32 / 32768.0;
                        C32::new(i, q)
                    })
                    .collect();
                // Back-pressure like iiod: block while too many buffers are
                // queued ahead of the air clock.
                loop {
                    let (now, end) = {
                        let a = air.lock().unwrap();
                        (a.now(), a.radios[k].tx.end())
                    };
                    if end <= now + args.tx_ahead_buffers * BUF_SAMPLES {
                        break;
                    }
                    std::thread::sleep(Duration::from_millis(1));
                }
                {
                    let mut a = air.lock().unwrap();
                    let now = a.now();
                    let keep_from = now.saturating_sub((args.rate as u64).max(1));
                    let tx = &mut a.radios[k].tx;
                    if tx.end() < now {
                        // Underrun (or first buffer): the stream restarts now.
                        tx.origin = now + BUF_SAMPLES / 4;
                        tx.buf.clear();
                    }
                    tx.buf.extend_from_slice(&samples);
                    tx.trim(keep_from);
                    if args.verbose {
                        eprintln!("radio {k}: TX {} samples, stream ends at {} (air now {now})", samples.len(), tx.end());
                    }
                }
                write!(w, "{len}\n")
            }
            _ => write!(w, "-22\n"),
        };
        if reply.and_then(|_| w.flush()).is_err() {
            break;
        }
    }
}

fn default_attr(attr: &str) -> &'static str {
    match attr {
        "xo_correction" => "40000000",
        "sampling_frequency" => "4000000",
        "frequency" => "1250000000",
        "rf_bandwidth" => "2200000",
        "hardwaregain" => "0",
        "gain_control_mode" => "manual",
        _ => "0",
    }
}

fn main() -> Result<()> {
    let args = Arc::new(Args::parse());
    let ppm: Vec<f64> = args.ppm.split(',').map(|s| s.trim().parse().unwrap_or(0.0)).collect();
    let radios = (0..args.radios)
        .map(|k| Radio { tx: TxStream { origin: 0, buf: Vec::new() }, rx_pos: None, ppm: ppm.get(k).copied().unwrap_or(0.0), attrs: HashMap::new() })
        .collect();
    let air = Arc::new(Mutex::new(Air {
        t0: Instant::now(),
        rate: args.rate,
        freq: args.freq,
        gain: 10f32.powf(-(args.path_loss_db as f32) / 20.0),
        noise_amp: 10f32.powf(args.noise_dbfs as f32 / 20.0),
        delay: args.delay_samples,
        radios,
        rng: 0x1234_5678_9ABC_DEF1,
    }));
    let mut handles = Vec::new();
    for k in 0..args.radios {
        let port = args.base_port + k as u16;
        let listener = TcpListener::bind(("127.0.0.1", port))?;
        eprintln!("virtual pluto {k}: 127.0.0.1:{port} (oscillator {:+} ppm)", ppm.get(k).copied().unwrap_or(0.0));
        let air = air.clone();
        let args = args.clone();
        handles.push(std::thread::spawn(move || {
            for conn in listener.incoming() {
                let Ok(conn) = conn else { continue };
                conn.set_nodelay(true).ok();
                let air = air.clone();
                let args = args.clone();
                std::thread::spawn(move || serve(conn, k, air, args));
            }
        }));
    }
    eprintln!(
        "air: {} S/s, path loss {} dB, noise {} dBFS, delay {} samples, RX latency {} buffers",
        args.rate, args.path_loss_db, args.noise_dbfs, args.delay_samples, args.rx_latency_buffers
    );
    for h in handles {
        let _ = h.join();
    }
    Ok(())
}
