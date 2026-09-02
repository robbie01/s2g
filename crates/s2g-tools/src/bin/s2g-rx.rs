//! Receive S1G PPDUs — from a recording (.cf32 / SigMF / ci16 at any sample
//! rate) or a PlutoSDR at 1250 MHz. Optionally parses the MAC frames and
//! writes them to a PCAP.

use anyhow::{bail, Result};
use clap::Parser;
use s2g_mac::frame::ParsedFrame;
use s2g_phy::rx::{Receiver, RxConfig, RxEndStatus, RxEvent};
use s2g_phy::vector::PreambleType;
use s2g_tools::{read_recording, to_native_rate, Complex32, PcapWriter, DEFAULT_CENTER_FREQ_HZ, DEFAULT_DEVICE_RATE_HZ};
use std::collections::BTreeMap;
use std::path::PathBuf;

#[derive(Parser, Debug)]
#[command(name = "s2g-rx", about = "S1G (802.11ah) 2 MHz PPDU receiver")]
struct Args {
    /// Replay a recording instead of a radio: .cf32, .sigmf-data (+ .sigmf-meta),
    /// .sigmf archive, or raw complex int16 (.ci16/.cs16)
    #[arg(long, conflicts_with = "uri")]
    r#in: Option<PathBuf>,
    /// Sample rate of the input file, Hz (default: from SigMF metadata, else 2e6).
    /// Anything other than 2e6 / 4e6 is resampled with an anti-alias filter.
    #[arg(long)]
    rate: Option<f64>,
    /// Signal centre relative to the capture centre, Hz (translated to baseband first)
    #[arg(long, default_value_t = 0.0)]
    shift_hz: f64,
    /// Skip this many seconds of the recording
    #[arg(long, default_value_t = 0.0)]
    skip_sec: f64,
    /// Process at most this many seconds (default: everything)
    #[arg(long)]
    duration_sec: Option<f64>,
    /// Parse MAC frames (A-MPDU deaggregation, FCS, frame type / addresses)
    #[arg(long)]
    mac: bool,
    /// Write FCS-valid MAC frames to a PCAP (link type 802.11)
    #[arg(long)]
    pcap: Option<PathBuf>,

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
    /// Calibration: dBm = dBFS + this offset (drives RCPI and the CCA thresholds).
    /// Default: derived from the measured noise floor, assumed to be -104 dBm
    #[arg(long)]
    cal_offset_db: Option<f32>,
    /// CCA channel classification: 1 or 2
    #[arg(long, default_value_t = 1)]
    cca_type: u8,
    /// Disable sampling-clock drift tracking (diagnostics)
    #[arg(long)]
    no_timing_tracking: bool,
}

#[derive(Default)]
struct Stats {
    detections: usize,
    starts_short: usize,
    starts_long: usize,
    ends: BTreeMap<String, usize>,
    psdu_by_mcs: BTreeMap<String, usize>,
    ndps: usize,
    ndp_types: BTreeMap<String, usize>,
    mpdus: usize,
    fcs_ok: usize,
    /// Non-aggregated PSDUs whose MPDU was shorter than the SIG length.
    padded: usize,
    frame_types: BTreeMap<String, usize>,
}

struct Printer {
    quiet: bool,
    mac: bool,
    psdus: usize,
    stats: Stats,
    pcap: Option<PcapWriter>,
}

fn frame_type_name(fc0: u8) -> String {
    let (t, s) = ((fc0 >> 2) & 3, fc0 >> 4);
    let name = match (t, s) {
        (0, 0) => "AssocReq",
        (0, 1) => "AssocResp",
        (0, 4) => "ProbeReq",
        (0, 5) => "ProbeResp",
        (0, 8) => "Beacon",
        (0, 11) => "Auth",
        (0, 13) => "Action",
        (1, 8) => "BAR",
        (1, 9) => "BlockAck",
        (1, 11) => "RTS",
        (1, 12) => "CTS",
        (1, 13) => "Ack",
        (1, 7) => "CtrlWrapper",
        (3, 1) => "S1GBeacon",
        (2, 0) => "Data",
        (2, 4) => "Null",
        (2, 8) => "QoSData",
        (2, 12) => "QoSNull",
        _ => "",
    };
    if fc0 & 3 == 1 {
        format!("PV1 type{}/sub{}", (fc0 >> 2) & 7, fc0 >> 5)
    } else if name.is_empty() {
        format!("type{t}/sub{s}")
    } else {
        name.to_string()
    }
}

fn mac_str(a: &[u8]) -> String {
    a.iter().map(|b| format!("{b:02x}")).collect::<Vec<_>>().join(":")
}

impl Printer {
    fn handle(&mut self, ev: &RxEvent) {
        match ev {
            RxEvent::Cca { sample_index, busy, reason, hold_us } => {
                if !self.quiet {
                    eprintln!("[{sample_index}] CCA {} {reason:?} hold={hold_us} µs", if *busy { "BUSY" } else { "IDLE" });
                }
            }
            RxEvent::PpduStart { sample_index, coarse_cfo_hz } => {
                self.stats.detections += 1;
                if !self.quiet {
                    eprintln!("[{sample_index}] detect (coarse CFO {coarse_cfo_hz:+.0} Hz)");
                }
            }
            RxEvent::RxStart { sample_index, rxvector } => {
                if rxvector.preamble_type == PreambleType::S1gLong {
                    self.stats.starts_long += 1;
                } else {
                    self.stats.starts_short += 1;
                }
                if !self.quiet {
                    eprintln!(
                        "[{sample_index}] RXSTART {:?}{}: MCS {} {:?} {:?} | {} octets | {} sym | agg {} | ri {:?} | tp {} | rssi {} rcpi {} ({:.1} dBm) snr {:.1} dB | {} µs",
                        rxvector.preamble_type,
                        if rxvector.mu { " MU" } else { "" },
                        rxvector.mcs,
                        rxvector.fec_coding,
                        rxvector.gi,
                        rxvector.psdu_length,
                        rxvector.n_sym,
                        rxvector.aggregation,
                        rxvector.response_indication,
                        rxvector.traveling_pilots,
                        rxvector.rssi,
                        rxvector.rcpi,
                        rxvector.rcpi_dbm,
                        rxvector.snr_db,
                        rxvector.ppdu_duration_us()
                    );
                }
            }
            RxEvent::NdpReceived { sample_index, body, metrics } => {
                self.psdus += 1;
                self.stats.ndps += 1;
                let f = s2g_mac::ndp::NdpFrame::parse(*body);
                let kind = match f {
                    s2g_mac::ndp::NdpFrame::Cts(_) => "NDP CTS".to_string(),
                    s2g_mac::ndp::NdpFrame::Ack(_) => "NDP Ack".to_string(),
                    s2g_mac::ndp::NdpFrame::BlockAck(_) => "NDP BlockAck".to_string(),
                    s2g_mac::ndp::NdpFrame::Other { ndp_type, .. } => {
                        format!("NDP type {ndp_type}")
                    }
                };
                *self.stats.ndp_types.entry(kind).or_default() += 1;
                println!(
                    "[{sample_index}] NDP body=0x{body:010x} {f:?} snr={:.1}dB cfo={:+.0}Hz rssi={:.1}dBFS",
                    metrics.snr_db, metrics.cfo_hz, metrics.rssi_dbfs
                );
            }
            RxEvent::PsduReceived { sample_index, rxvector, psdu, metrics } => {
                self.psdus += 1;
                let sgi = if rxvector.gi == s2g_phy::vector::GuardInterval::Short { " SGI" } else { "" };
                *self
                    .stats
                    .psdu_by_mcs
                    .entry(format!("MCS{} {:?} {:?}{sgi}", rxvector.mcs, rxvector.fec_coding, rxvector.preamble_type))
                    .or_default() += 1;
                println!(
                    "[{sample_index}] PSDU {:?}{sgi} mcs={} {:?} len={} agg={} tp={} seed={} snr={:.1}dB cfo={:+.0}Hz evm={:.1}dB rssi={:.1}dBFS drift={:+.2} ldpc_fail={}",
                    rxvector.preamble_type,
                    rxvector.mcs,
                    rxvector.fec_coding,
                    psdu.len(),
                    rxvector.aggregation,
                    rxvector.traveling_pilots,
                    rxvector.scrambler_seed,
                    metrics.snr_db,
                    metrics.cfo_hz,
                    metrics.evm_db,
                    metrics.rssi_dbfs,
                    metrics.timing_drift_samples,
                    metrics.ldpc_failures
                );
                if self.mac || self.pcap.is_some() {
                    let mpdus: Vec<Vec<u8>> = if rxvector.aggregation {
                        s2g_mac::ampdu::deaggregate(psdu)
                    } else {
                        let located = s2g_mac::frame::locate_mpdu(psdu);
                        if located.is_some_and(|m| m.len() < psdu.len()) {
                            self.stats.padded += 1;
                        }
                        vec![located.unwrap_or(psdu).to_vec()]
                    };
                    for m in &mpdus {
                        self.stats.mpdus += 1;
                        let ok = s2g_mac::fcs::check_and_strip(m).is_some();
                        if ok {
                            self.stats.fcs_ok += 1;
                            *self.stats.frame_types.entry(frame_type_name(m[0])).or_default() += 1;
                            if let Some(p) = self.pcap.as_mut() {
                                let _ = p.write(sample_index / 2, m);
                            }
                        }
                        if self.mac {
                            let desc = match s2g_mac::frame::parse(m) {
                                Ok(ParsedFrame::Data { dest, src, seq, .. }) => {
                                    format!("Data {} -> {} seq {seq}", mac_str(&src), mac_str(&dest))
                                }
                                Ok(ParsedFrame::Ack { ra }) => format!("Ack ra {}", mac_str(&ra)),
                                Ok(ParsedFrame::Rts { ra, ta, duration_us }) => {
                                    format!("RTS {} -> {} dur {duration_us}", mac_str(&ta), mac_str(&ra))
                                }
                                Ok(ParsedFrame::Pv1 { ptype, subtype, a1, a2, seq, .. }) => {
                                    format!("PV1 type {ptype} sub {subtype} a1 {a1:?} a2 {a2:?} seq {seq:?}")
                                }
                                Ok(ParsedFrame::Other { fc, duration_us }) => {
                                    let addr1 = if m.len() >= 10 { mac_str(&m[4..10]) } else { "-".into() };
                                    let addr2 = if m.len() >= 16 { mac_str(&m[10..16]) } else { "-".into() };
                                    format!(
                                        "{} fc={:02x}{:02x} dur={duration_us} a1 {addr1} a2 {addr2}",
                                        frame_type_name(fc[0]),
                                        fc[0],
                                        fc[1]
                                    )
                                }
                                Err(e) => format!("{e} ({} octets)", m.len()),
                            };
                            println!("    MPDU {} octets FCS {}: {desc}", m.len(), if ok { "ok" } else { "BAD" });
                        }
                    }
                } else if !self.quiet {
                    let hex: String = psdu.iter().map(|b| format!("{b:02x}")).collect();
                    println!("{hex}");
                }
            }
            RxEvent::RxEnd { sample_index, status } => {
                *self.stats.ends.entry(format!("{status:?}")).or_default() += 1;
                if !self.quiet && *status != RxEndStatus::NoError {
                    eprintln!("[{sample_index}] RXEND {status:?}");
                }
            }
        }
    }

    fn summary(&self) {
        let s = &self.stats;
        eprintln!("--- summary ---");
        eprintln!(
            "preamble detections: {} | RXSTART short: {} long: {} | NDP: {} {:?}",
            s.detections, s.starts_short, s.starts_long, s.ndps, s.ndp_types
        );
        eprintln!("RXEND: {:?}", s.ends);
        eprintln!("PSDUs by MCS: {:?}", s.psdu_by_mcs);
        if self.mac || self.pcap.is_some() {
            eprintln!("MPDUs: {} | FCS ok: {} | padded PSDUs: {} | types: {:?}", s.mpdus, s.fcs_ok, s.padded, s.frame_types);
        }
    }
}

fn print_calibration(rx: &Receiver) {
    match rx.noise_floor_dbfs() {
        Some(f) => eprintln!("noise floor {f:.1} dBFS | cal offset {:+.1} dB | energy detect at {:.1} dBFS", rx.cal_offset_db(), -72.0 - rx.cal_offset_db()),
        None => eprintln!("noise floor: not measured (needs 20 ms of samples)"),
    }
}

fn main() -> Result<()> {
    let args = Args::parse();
    let mut rx = Receiver::new(RxConfig {
        detect_threshold: args.threshold,
        emit_ppdu_start: !args.quiet,
        emit_cca: !args.quiet,
        cal_offset_db: args.cal_offset_db.unwrap_or(0.0),
        auto_cal: args.cal_offset_db.is_none(),
        cca_type: if args.cca_type == 2 { s2g_phy::params::rf::CcaType::Type2 } else { s2g_phy::params::rf::CcaType::Type1 },
        timing_tracking: !args.no_timing_tracking,
        ..Default::default()
    });
    let pcap = match &args.pcap {
        Some(p) => Some(PcapWriter::create(p)?),
        None => None,
    };
    let mut printer = Printer { quiet: args.quiet, mac: args.mac, psdus: 0, stats: Stats::default(), pcap };
    let mut events: Vec<RxEvent> = Vec::new();

    if let Some(path) = &args.r#in {
        // Rate: CLI > SigMF metadata > 2 MS/s.
        let probe = read_recording(path, 0, Some(0))?;
        let rate = args.rate.or(probe.sample_rate_hz).unwrap_or(2.0e6);
        let skip = (args.skip_sec * rate) as usize;
        let max = args.duration_sec.map(|d| (d * rate) as usize);
        let rec = read_recording(path, skip, max)?;
        eprintln!(
            "read {} samples from {} @ {} S/s{}{}",
            rec.samples.len(),
            path.display(),
            rate,
            rec.center_freq_hz.map(|f| format!(", centre {f} Hz")).unwrap_or_default(),
            if args.shift_hz != 0.0 { format!(", shifting {:+} Hz", args.shift_hz) } else { String::new() }
        );
        if rate < 1.9e6 {
            bail!("input rate {rate} S/s is below the 2 MHz channel bandwidth");
        }
        let native = to_native_rate(&rec.samples, rate, args.shift_hz);
        eprintln!("{} samples @ 2 MS/s ({:.1} s)", native.len(), native.len() as f64 / 2e6);
        for chunk in native.chunks(65536) {
            rx.process(chunk, &mut events);
            for e in events.drain(..) {
                printer.handle(&e);
            }
            if args.count > 0 && printer.psdus >= args.count {
                printer.summary();
                print_calibration(&rx);
                return Ok(());
            }
        }
        rx.finish(&mut events);
        for e in events.drain(..) {
            printer.handle(&e);
        }
        printer.summary();
        print_calibration(&rx);
        return Ok(());
    }

    receive_pluto(&args, &mut rx, &mut printer)
}

#[cfg(feature = "pluto")]
fn receive_pluto(args: &Args, rx: &mut Receiver, printer: &mut Printer) -> Result<()> {
    use s2g_sdr::{RxGain, SdrDevice, SdrRx, StreamConfig};
    let uri = args.uri.as_deref().unwrap_or("192.168.2.1");
    let gain = if args.gain == "auto" {
        RxGain::Auto
    } else {
        RxGain::Manual(args.gain.parse().map_err(|_| anyhow::anyhow!("--gain must be 'auto' or a dB value"))?)
    };
    let mut pluto = s2g_sdr_pluto::Pluto::open(uri).map_err(|e| anyhow::anyhow!("pluto: {e}"))?;
    let cfg = StreamConfig { center_freq_hz: args.freq, sample_rate_hz: DEFAULT_DEVICE_RATE_HZ, rf_bandwidth_hz: args.rf_bandwidth };
    let mut stream = pluto.open_rx(&cfg, gain).map_err(|e| anyhow::anyhow!("pluto rx: {e}"))?;
    eprintln!("Pluto RX @ {} Hz, {} S/s (decimating to 2 MS/s)", args.freq, DEFAULT_DEVICE_RATE_HZ);
    let mut dec = s2g_dsp::HalfbandDecim2::new();
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
            printer.summary();
            return Ok(());
        }
    }
}

#[cfg(not(feature = "pluto"))]
fn receive_pluto(_args: &Args, _rx: &mut Receiver, _printer: &mut Printer) -> Result<()> {
    bail!("built without the 'pluto' feature; use --in FILE or rebuild with --features pluto")
}
