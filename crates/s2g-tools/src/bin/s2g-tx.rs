//! Transmit S1G PPDUs to a .cf32 file or a PlutoSDR at 1250 MHz.

use anyhow::{bail, Context, Result};
use clap::Parser;
use s2g_phy::params::SAMPLE_RATE_HZ;
use s2g_phy::sim::Rng;
use s2g_phy::vector::{Coding, GuardInterval, PreambleType, TxVector};
use s2g_phy::Transmitter;
use s2g_tools::{parse_hex_psdu, write_cf32, Complex32, DEFAULT_CENTER_FREQ_HZ, DEFAULT_DEVICE_RATE_HZ};
use std::path::PathBuf;

#[derive(Parser, Debug)]
#[command(name = "s2g-tx", about = "S1G (802.11ah) 2 MHz PPDU transmitter")]
struct Args {
    /// MCS index (0-8 or 11)
    #[arg(long, default_value_t = 0)]
    mcs: u8,
    /// PSDU as hex bytes (e.g. "dead beef")
    #[arg(long, conflicts_with_all = ["file", "random"])]
    hex: Option<String>,
    /// PSDU from a binary file
    #[arg(long, conflicts_with = "random")]
    file: Option<PathBuf>,
    /// Random PSDU of N octets
    #[arg(long)]
    random: Option<usize>,
    /// Send as A-MPDU (aggregation bit; PSDU must fill the symbol payload)
    #[arg(long)]
    aggregation: bool,
    /// LDPC coding instead of BCC
    #[arg(long)]
    ldpc: bool,
    /// Traveling pilots
    #[arg(long)]
    traveling_pilots: bool,
    /// Short guard interval (4 µs from the second Data symbol on)
    #[arg(long)]
    sgi: bool,
    /// S1G_LONG (SU) preamble instead of S1G_SHORT
    #[arg(long)]
    long_preamble: bool,
    /// Transmit an NDP CMAC PPDU with this 37-bit body (hex) instead of a data PPDU
    #[arg(long, conflicts_with_all = ["hex", "file", "random"])]
    ndp: Option<String>,
    /// Scrambler seed 1..=127 (default: random per PPDU)
    #[arg(long)]
    seed: Option<u8>,
    /// Number of PPDUs to send
    #[arg(long, default_value_t = 1)]
    count: usize,
    /// Gap between PPDUs in ms
    #[arg(long, default_value_t = 20.0)]
    interval_ms: f64,
    /// Output waveform amplitude (RMS at full scale 1.0)
    #[arg(long, default_value_t = 0.25)]
    amplitude: f32,

    /// Write the waveform to a .cf32 file instead of transmitting
    #[arg(long, conflicts_with = "uri")]
    out: Option<PathBuf>,
    /// Sample rate of the output, Hz: 2e6 (native) or 4e6 (×2 interpolated)
    #[arg(long, default_value_t = SAMPLE_RATE_HZ)]
    out_rate_hz: f64,

    /// Pluto iiod address, e.g. 192.168.2.1[:30431]
    #[arg(long)]
    uri: Option<String>,
    /// RF center frequency, Hz
    #[arg(long, default_value_t = DEFAULT_CENTER_FREQ_HZ)]
    freq_hz: f64,
    /// TX gain (attenuation), dB ≤ 0
    #[arg(long, default_value_t = -10.0)]
    tx_gain_db: f64,
    /// Analog RF bandwidth hint, Hz
    #[arg(long, default_value_t = 2_200_000.0)]
    rf_bandwidth_hz: f64,
}

fn interpolate_2x(wave: &[Complex32]) -> Vec<Complex32> {
    let mut up = Vec::with_capacity(wave.len() * 2 + 128);
    let mut it = s2g_dsp::HalfbandInterp2::new();
    it.process(wave, &mut up);
    // Flush the filter tail.
    it.process(&[Complex32::new(0.0, 0.0); 32], &mut up);
    up
}

fn main() -> Result<()> {
    let args = Args::parse();
    let psdu: Vec<u8> = if let Some(h) = &args.hex {
        parse_hex_psdu(h)?
    } else if let Some(f) = &args.file {
        std::fs::read(f).with_context(|| format!("read {}", f.display()))?
    } else {
        let n = args.random.unwrap_or(100);
        Rng(0xC0FFEE).bytes(n)
    };

    let coding = if args.ldpc { Coding::Ldpc } else { Coding::Bcc };
    let txv = TxVector {
        mcs: args.mcs,
        aggregation: args.aggregation,
        fec_coding: coding,
        traveling_pilots: args.traveling_pilots,
        gi: if args.sgi { GuardInterval::Short } else { GuardInterval::Long },
        preamble_type: if args.long_preamble { PreambleType::S1gLong } else { PreambleType::S1gShort },
        scrambler_seed: args.seed,
        ..Default::default()
    };
    let tx = Transmitter { amplitude: args.amplitude };
    let wave = if let Some(body_hex) = &args.ndp {
        let body = u64::from_str_radix(body_hex.trim_start_matches("0x"), 16).context("--ndp must be hex")?;
        let w = tx.generate_ndp(body).map_err(|e| anyhow::anyhow!("PHY: {e}"))?;
        eprintln!("NDP CMAC PPDU: body 0x{body:010x} | {} samples @ 2 MS/s | 240 µs", w.len());
        w
    } else {
        let (w, info) = tx.generate_with_info(&txv, &psdu).map_err(|e| anyhow::anyhow!("PHY: {e}"))?;
        eprintln!(
            "PPDU: MCS {} {:?}{}{}{} | {} octets | {} samples @ 2 MS/s | TXTIME {} µs | seed {}",
            args.mcs,
            coding,
            if args.traveling_pilots { " + traveling pilots" } else { "" },
            if args.sgi { " + short GI" } else { "" },
            if args.long_preamble { " + S1G_LONG" } else { "" },
            psdu.len(),
            w.len(),
            info.txtime_us,
            info.scrambler_seed
        );
        w
    };

    if let Some(out) = &args.out {
        let gap = vec![Complex32::new(0.0, 0.0); (args.interval_ms / 1000.0 * SAMPLE_RATE_HZ) as usize];
        let mut stream = Vec::new();
        for _ in 0..args.count {
            stream.extend_from_slice(&wave);
            stream.extend_from_slice(&gap);
        }
        let stream = if (args.out_rate_hz - 4e6).abs() < 1.0 {
            interpolate_2x(&stream)
        } else if (args.out_rate_hz - SAMPLE_RATE_HZ).abs() < 1.0 {
            stream
        } else {
            bail!("--out-rate-hz must be 2e6 or 4e6");
        };
        write_cf32(out, &stream)?;
        eprintln!("wrote {} samples @ {} S/s to {}", stream.len(), args.out_rate_hz, out.display());
        return Ok(());
    }

    let uri = args.uri.as_deref().unwrap_or("192.168.2.1");
    transmit_pluto(&args, uri, &wave)
}

#[cfg(feature = "pluto")]
fn transmit_pluto(args: &Args, uri: &str, wave: &[Complex32]) -> Result<()> {
    use s2g_sdr::{SdrDevice, SdrTx, StreamConfig};
    let mut pluto = s2g_sdr_pluto::Pluto::open(uri).map_err(|e| anyhow::anyhow!("pluto: {e}"))?;
    let cfg = StreamConfig { center_freq_hz: args.freq_hz, sample_rate_hz: DEFAULT_DEVICE_RATE_HZ, rf_bandwidth_hz: args.rf_bandwidth_hz };
    let mut tx = pluto.open_tx(&cfg, args.tx_gain_db).map_err(|e| anyhow::anyhow!("pluto tx: {e}"))?;
    eprintln!("Pluto @ {} Hz, {} S/s, gain {} dB", args.freq_hz, DEFAULT_DEVICE_RATE_HZ, args.tx_gain_db);
    let up = interpolate_2x(wave);
    for i in 0..args.count {
        tx.send(&up).map_err(|e| anyhow::anyhow!("send: {e}"))?;
        eprintln!("sent PPDU {}/{}", i + 1, args.count);
        if args.interval_ms > 0.0 && i + 1 < args.count {
            std::thread::sleep(std::time::Duration::from_secs_f64(args.interval_ms / 1000.0));
        }
    }
    tx.flush().ok();
    Ok(())
}

#[cfg(not(feature = "pluto"))]
fn transmit_pluto(_args: &Args, _uri: &str, _wave: &[Complex32]) -> Result<()> {
    bail!("built without the 'pluto' feature; use --out FILE or rebuild with --features pluto")
}
