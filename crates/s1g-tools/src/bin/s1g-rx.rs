//! Receive S1G PPDUs — from a .cf32 file or a PlutoSDR at 1250 MHz.

use anyhow::{bail, Result};
use clap::Parser;
use s1g_phy::rx::{Receiver, RxConfig, RxEvent};
use s1g_tools::{read_cf32, Complex32, DEFAULT_CENTER_FREQ_HZ, DEFAULT_DEVICE_RATE_HZ};
use std::path::PathBuf;

#[derive(Parser, Debug)]
#[command(name = "s1g-rx", about = "S1G (802.11ah) 2 MHz PPDU receiver")]
struct Args {
    /// Replay a .cf32 file instead of a radio
    #[arg(long, conflicts_with = "uri")]
    r#in: Option<PathBuf>,
    /// Sample rate of the input file: 2e6 (native) or 4e6 (decimated ×2)
    #[arg(long, default_value_t = 2_000_000.0)]
    rate: f64,

    /// Pluto iiod address, e.g. 192.168.2.1[:30431]
    #[arg(long)]
    uri: Option<String>,
    /// RF center frequency, Hz
    #[arg(long, default_value_t = DEFAULT_CENTER_FREQ_HZ)]
    freq: f64,
    /// RX gain: "auto" or dB value
    #[arg(long, default_value = "auto")]
    gain: String,
    /// Analog RF bandwidth hint, Hz
    #[arg(long, default_value_t = 2_200_000.0)]
    rf_bandwidth: f64,

    /// Exit after N PSDUs (0 = run forever / until file end)
    #[arg(long, default_value_t = 0)]
    count: usize,
    /// One line per PSDU only
    #[arg(long)]
    quiet: bool,
    /// Detection threshold (0..1)
    #[arg(long, default_value_t = 0.55)]
    threshold: f32,
}

struct Printer {
    quiet: bool,
    psdus: usize,
}

impl Printer {
    fn handle(&mut self, ev: &RxEvent) {
        match ev {
            RxEvent::PpduStart { sample_index, coarse_cfo_hz } => {
                if !self.quiet {
                    eprintln!("[{sample_index}] detect (coarse CFO {coarse_cfo_hz:+.0} Hz)");
                }
            }
            RxEvent::SigDecoded { sample_index, rxvector } => {
                if !self.quiet {
                    eprintln!(
                        "[{sample_index}] SIG: MCS {} | {} octets | {} sym | agg {} | ri {:?}",
                        rxvector.mcs, rxvector.psdu_length, rxvector.n_sym, rxvector.aggregation, rxvector.response_indication
                    );
                }
            }
            RxEvent::NdpReceived { sample_index, body, metrics } => {
                self.psdus += 1;
                println!(
                    "[{sample_index}] NDP body=0x{body:010x} snr={:.1}dB cfo={:+.0}Hz rssi={:.1}dBFS",
                    metrics.snr_db, metrics.cfo_hz, metrics.rssi_dbfs
                );
            }
            RxEvent::PsduReceived { sample_index, rxvector, psdu, metrics } => {
                self.psdus += 1;
                let hex: String = psdu.iter().map(|b| format!("{b:02x}")).collect();
                println!(
                    "[{sample_index}] PSDU mcs={} len={} seed={} snr={:.1}dB cfo={:+.0}Hz evm={:.1}dB rssi={:.1}dBFS\n{hex}",
                    rxvector.mcs, psdu.len(), rxvector.scrambler_seed, metrics.snr_db, metrics.cfo_hz, metrics.evm_db, metrics.rssi_dbfs
                );
            }
            RxEvent::Error { sample_index, kind } => {
                if !self.quiet {
                    eprintln!("[{sample_index}] error: {kind:?}");
                }
            }
        }
    }
}

fn main() -> Result<()> {
    let args = Args::parse();
    let mut rx = Receiver::new(RxConfig { detect_threshold: args.threshold, emit_ppdu_start: !args.quiet });
    let mut printer = Printer { quiet: args.quiet, psdus: 0 };
    let mut events: Vec<RxEvent> = Vec::new();

    if let Some(path) = &args.r#in {
        let samples = read_cf32(path)?;
        eprintln!("read {} samples from {}", samples.len(), path.display());
        let native = if (args.rate - 4e6).abs() < 1.0 {
            let mut dec = s1g_dsp::HalfbandDecim2::new();
            let mut out = Vec::with_capacity(samples.len() / 2);
            dec.process(&samples, &mut out);
            out
        } else if (args.rate - 2e6).abs() < 1.0 {
            samples
        } else {
            bail!("--rate must be 2e6 or 4e6");
        };
        for chunk in native.chunks(65536) {
            rx.process(chunk, &mut events);
            for e in events.drain(..) {
                printer.handle(&e);
            }
            if args.count > 0 && printer.psdus >= args.count {
                return Ok(());
            }
        }
        rx.finish(&mut events);
        for e in events.drain(..) {
            printer.handle(&e);
        }
        eprintln!("done: {} PSDU(s)/NDP(s)", printer.psdus);
        return Ok(());
    }

    receive_pluto(&args, &mut rx, &mut printer)
}

#[cfg(feature = "pluto")]
fn receive_pluto(args: &Args, rx: &mut Receiver, printer: &mut Printer) -> Result<()> {
    use s1g_sdr::{RxGain, SdrDevice, SdrRx, StreamConfig};
    let uri = args.uri.as_deref().unwrap_or("192.168.2.1");
    let gain = if args.gain == "auto" {
        RxGain::Auto
    } else {
        RxGain::Manual(args.gain.parse().map_err(|_| anyhow::anyhow!("--gain must be 'auto' or a dB value"))?)
    };
    let mut pluto = s1g_sdr_pluto::Pluto::open(uri).map_err(|e| anyhow::anyhow!("pluto: {e}"))?;
    let cfg = StreamConfig {
        center_freq_hz: args.freq,
        sample_rate_hz: DEFAULT_DEVICE_RATE_HZ,
        rf_bandwidth_hz: args.rf_bandwidth,
    };
    let mut stream = pluto.open_rx(&cfg, gain).map_err(|e| anyhow::anyhow!("pluto rx: {e}"))?;
    eprintln!("Pluto RX @ {} Hz, {} S/s (decimating to 2 MS/s)", args.freq, DEFAULT_DEVICE_RATE_HZ);
    let mut dec = s1g_dsp::HalfbandDecim2::new();
    let mut dev_buf = vec![Complex32::new(0.0, 0.0); 16384];
    let mut native = Vec::with_capacity(8192);
    let mut events = Vec::new();
    loop {
        let n = stream.recv(&mut dev_buf).map_err(|e| anyhow::anyhow!("recv: {e}"))?;
        native.clear();
        dec.process(&dev_buf[..n], &mut native);
        rx.process(&native, &mut events);
        for e in events.drain(..) {
            printer.handle(&e);
        }
        if args.count > 0 && printer.psdus >= args.count {
            return Ok(());
        }
    }
}

#[cfg(not(feature = "pluto"))]
fn receive_pluto(_args: &Args, _rx: &mut Receiver, _printer: &mut Printer) -> Result<()> {
    bail!("built without the 'pluto' feature; use --in FILE or rebuild with --features pluto")
}
