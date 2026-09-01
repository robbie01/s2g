//! Full network node: NIC (TAP or UDP) ↔ OCB MAC ↔ S1G PHY ↔ PlutoSDR.
//!
//! On Unix with `--features tap`: `s2g-node --tap s2g0` creates a real L2
//! interface the OS can route through. On Windows (no L2 TAP support yet):
//! `s2g-node --udp 127.0.0.1:5001` shuttles raw Ethernet frames as UDP
//! datagrams instead.

use anyhow::{bail, Result};
use clap::Parser;
use s2g_mac::{Mac, MacAction, MacConfig, MacEvent};
use s2g_phy::vector::Coding;
use s2g_tools::nic::Nic;
use s2g_tools::DEFAULT_CENTER_FREQ_HZ;

#[derive(Parser, Debug)]
#[command(name = "s2g-node", about = "S1G OCB network node (NIC ↔ MAC ↔ PHY ↔ Pluto)")]
struct Args {
    /// Create/attach a TAP interface (optionally named). Unix + feature "tap".
    #[arg(long, num_args = 0..=1, default_missing_value = "", conflicts_with = "udp")]
    tap: Option<String>,
    /// Ethernet-over-UDP NIC: local bind address (e.g. 127.0.0.1:5001)
    #[arg(long)]
    udp: Option<String>,
    /// UDP NIC: fixed peer (default: lock onto first sender)
    #[arg(long)]
    udp_peer: Option<String>,

    /// Our MAC address (default: locally-administered random)
    #[arg(long)]
    mac: Option<String>,
    /// Data MCS (0-8 or 11)
    #[arg(long, default_value_t = 2)]
    mcs: u8,
    /// LDPC coding for data frames (optional feature; peer must support it)
    #[arg(long)]
    ldpc: bool,
    /// Traveling pilots on data frames (optional feature; peer must support it)
    #[arg(long)]
    traveling_pilots: bool,
    /// Disable ACK/retry for unicast
    #[arg(long)]
    no_ack: bool,
    /// Solicit legacy Ack frames instead of NDP Ack / NDP BlockAck
    #[arg(long)]
    no_ndp_ack: bool,
    /// Protect unicast MPDUs longer than this with RTS / NDP CTS
    #[arg(long)]
    rts_threshold: Option<usize>,
    /// Response timeout in ms (SDR buffering makes SIFS-scale impossible)
    #[arg(long, default_value_t = 150)]
    ack_timeout_ms: u64,
    #[arg(long, default_value_t = 3)]
    retries: u32,

    /// Pluto iiod address
    #[arg(long, default_value = "192.168.2.1")]
    uri: String,
    /// RF center frequency, Hz
    #[arg(long, default_value_t = DEFAULT_CENTER_FREQ_HZ)]
    freq: f64,
    /// RX gain: "auto" or dB
    #[arg(long, default_value = "auto")]
    gain: String,
    /// TX gain (attenuation), dB ≤ 0
    #[arg(long, default_value_t = -10.0)]
    tx_gain: f64,
    #[arg(long, default_value_t = 2_200_000.0)]
    rf_bandwidth: f64,
    /// Receiver calibration: dBm = dBFS + this offset (RCPI, CCA thresholds)
    #[arg(long, default_value_t = 0.0)]
    cal_offset_db: f32,
    /// CCA channel classification: 1 or 2
    #[arg(long, default_value_t = 1)]
    cca_type: u8,
    /// Verbose per-frame logging
    #[arg(long)]
    verbose: bool,
}

fn parse_mac(s: &str) -> Result<[u8; 6]> {
    let parts: Vec<&str> = s.split([':', '-']).collect();
    if parts.len() != 6 {
        bail!("MAC address must be six octets");
    }
    let mut m = [0u8; 6];
    for (i, p) in parts.iter().enumerate() {
        m[i] = u8::from_str_radix(p, 16)?;
    }
    Ok(m)
}

fn random_mac() -> [u8; 6] {
    let t = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0x5151);
    let mut m = [0x02, 0x53, 0x32, 0x47, 0, 0]; // 02:"S2G"
    m[4] = (t >> 8) as u8;
    m[5] = t as u8;
    m
}

fn fmt_mac(m: &[u8; 6]) -> String {
    m.iter().map(|b| format!("{b:02x}")).collect::<Vec<_>>().join(":")
}

fn make_nic(args: &Args) -> Result<Box<dyn Nic>> {
    if let Some(udp) = &args.udp {
        return Ok(Box::new(s2g_tools::nic::UdpNic::new(udp, args.udp_peer.as_deref())?));
    }
    #[cfg(all(unix, feature = "tap"))]
    if let Some(name) = &args.tap {
        let n = if name.is_empty() { None } else { Some(name.as_str()) };
        return Ok(Box::new(s2g_tools::nic::TapNic::new(n)?));
    }
    #[cfg(not(all(unix, feature = "tap")))]
    if args.tap.is_some() {
        bail!("TAP support needs a Unix OS and --features tap; on Windows use --udp BIND");
    }
    bail!("choose a NIC: --tap [NAME] (Unix) or --udp BIND")
}

fn main() -> Result<()> {
    let args = Args::parse();
    let addr = match &args.mac {
        Some(s) => parse_mac(s)?,
        None => random_mac(),
    };
    let mut cfg = MacConfig::new(addr);
    cfg.mcs = args.mcs;
    cfg.fec_coding = if args.ldpc { Coding::Ldpc } else { Coding::Bcc };
    cfg.traveling_pilots = args.traveling_pilots;
    cfg.ack_enabled = !args.no_ack;
    cfg.ndp_ack = !args.no_ndp_ack;
    cfg.rts_threshold = args.rts_threshold;
    cfg.ack_timeout_us = args.ack_timeout_ms * 1000;
    cfg.max_retries = args.retries;
    let nic = make_nic(&args)?;
    eprintln!(
        "s2g-node: mac {} | mcs {} {:?}{} | ack {} ({}) | rts {:?} | nic {}",
        fmt_mac(&addr),
        args.mcs,
        cfg.fec_coding,
        if args.traveling_pilots { " + traveling pilots" } else { "" },
        cfg.ack_enabled,
        if cfg.ndp_ack { "NDP" } else { "legacy" },
        cfg.rts_threshold,
        nic.describe()
    );
    run_radio(&args, Mac::new(cfg), nic)
}

#[cfg(feature = "pluto")]
fn run_radio(args: &Args, mut mac: Mac, nic: Box<dyn Nic>) -> Result<()> {
    use num_complex::Complex;
    use s2g_phy::rx::{Receiver, RxConfig, RxEvent};
    use s2g_phy::Transmitter;
    use s2g_sdr::{RxGain, SdrDevice, SdrRx, SdrTx, StreamConfig};
    use s2g_tools::DEFAULT_DEVICE_RATE_HZ;
    use std::sync::mpsc;
    use std::time::{Duration, Instant};

    type C32 = Complex<f32>;

    enum Msg {
        Phy(RxEvent),
        Eth(Vec<u8>),
    }

    let gain = if args.gain == "auto" {
        RxGain::Auto
    } else {
        RxGain::Manual(args.gain.parse().map_err(|_| anyhow::anyhow!("--gain must be 'auto' or dB"))?)
    };
    let mut pluto = s2g_sdr_pluto::Pluto::open(&args.uri).map_err(|e| anyhow::anyhow!("pluto: {e}"))?;
    let scfg = StreamConfig {
        center_freq_hz: args.freq,
        sample_rate_hz: DEFAULT_DEVICE_RATE_HZ,
        rf_bandwidth_hz: args.rf_bandwidth,
    };
    let mut sdr_rx = pluto.open_rx(&scfg, gain).map_err(|e| anyhow::anyhow!("pluto rx: {e}"))?;
    let mut sdr_tx = pluto.open_tx(&scfg, args.tx_gain).map_err(|e| anyhow::anyhow!("pluto tx: {e}"))?;
    eprintln!("radio: {} @ {} Hz, {} S/s device rate", args.uri, args.freq, DEFAULT_DEVICE_RATE_HZ);
    let rx_cfg = RxConfig {
        cal_offset_db: args.cal_offset_db,
        cca_type: if args.cca_type == 2 { s2g_phy::params::rf::CcaType::Type2 } else { s2g_phy::params::rf::CcaType::Type1 },
        ..Default::default()
    };

    let (msg_tx, msg_rx) = mpsc::channel::<Msg>();
    let (nic_out_tx, nic_out_rx) = mpsc::channel::<Vec<u8>>();

    // RX thread: radio → decimate → PHY receiver → events.
    {
        let msg_tx = msg_tx.clone();
        std::thread::spawn(move || {
            let mut dec = s2g_dsp::HalfbandDecim2::new();
            let mut rx = Receiver::new(rx_cfg);
            let mut dev = vec![C32::new(0.0, 0.0); 16384];
            let mut native = Vec::with_capacity(8192);
            let mut events = Vec::new();
            loop {
                match sdr_rx.recv(&mut dev) {
                    Ok(n) => {
                        native.clear();
                        dec.process(&dev[..n], &mut native);
                        rx.process(&native, &mut events);
                        for e in events.drain(..) {
                            if msg_tx.send(Msg::Phy(e)).is_err() {
                                return;
                            }
                        }
                    }
                    Err(e) => {
                        eprintln!("radio rx error: {e}");
                        std::thread::sleep(Duration::from_millis(200));
                    }
                }
            }
        });
    }

    // NIC thread: owns the NIC; shuttles frames both ways.
    {
        let msg_tx = msg_tx.clone();
        let mut nic = nic;
        std::thread::spawn(move || loop {
            while let Ok(f) = nic_out_rx.try_recv() {
                if let Err(e) = nic.send_frame(&f) {
                    eprintln!("nic send error: {e}");
                }
            }
            match nic.recv_frame(Duration::from_millis(2)) {
                Ok(Some(f)) => {
                    if msg_tx.send(Msg::Eth(f)).is_err() {
                        return;
                    }
                }
                Ok(None) => {}
                Err(e) => {
                    eprintln!("nic recv error: {e}");
                    std::thread::sleep(Duration::from_millis(200));
                }
            }
        });
    }

    // Main loop: MAC engine + transmit path.
    let phy_tx = Transmitter::new();
    let t0 = Instant::now();
    let mut interp = s2g_dsp::HalfbandInterp2::new();
    let mut mac_events: Vec<MacEvent> = Vec::new();
    loop {
        let now_us = t0.elapsed().as_micros() as u64;
        match msg_rx.recv_timeout(Duration::from_millis(2)) {
            Ok(Msg::Phy(ev)) => {
                if args.verbose {
                    match &ev {
                        RxEvent::PsduReceived { metrics, rxvector, .. } => eprintln!(
                            "rx: mcs {} {:?} {}B snr {:.1} dB cfo {:+.0} Hz rcpi {:.1} dBm evm {:.1} dB",
                            rxvector.mcs, rxvector.fec_coding, rxvector.psdu_length, metrics.snr_db, metrics.cfo_hz, rxvector.rcpi_dbm, metrics.evm_db
                        ),
                        RxEvent::RxEnd { status, .. } if *status != s2g_phy::RxEndStatus::NoError => eprintln!("rx end: {status:?}"),
                        RxEvent::RxStart { rxvector, .. } if rxvector.preamble_type == s2g_phy::PreambleType::S1gLong => {
                            eprintln!("rx: S1G_LONG PPDU ({} µs), not decoded", rxvector.ppdu_duration_us())
                        }
                        _ => {}
                    }
                }
                mac.on_phy_event(&ev, now_us, &mut mac_events);
            }
            Ok(Msg::Eth(f)) => {
                if let Err(e) = mac.enqueue_eth(&f) {
                    eprintln!("drop outgoing frame: {e}");
                }
            }
            Err(mpsc::RecvTimeoutError::Timeout) => {}
            Err(mpsc::RecvTimeoutError::Disconnected) => bail!("worker thread died"),
        }
        for e in mac_events.drain(..) {
            match e {
                MacEvent::EthReceived(f) => {
                    let _ = nic_out_tx.send(f);
                }
                MacEvent::TxComplete { dest, acked, retries } => {
                    if args.verbose {
                        eprintln!("tx done → {} acked={acked} retries={retries}", fmt_mac(&dest));
                    }
                }
                MacEvent::TxDropped { dest, reason } => {
                    eprintln!("tx DROPPED → {}: {reason}", fmt_mac(&dest));
                }
                MacEvent::NdpReceived { frame } => {
                    if args.verbose {
                        eprintln!("ndp: {frame:?}");
                    }
                }
            }
        }
        let now_us = t0.elapsed().as_micros() as u64;
        let wave = match mac.poll(now_us, &mut mac_events) {
            Some(MacAction::Transmit { txv, psdu }) => match phy_tx.generate(&txv, &psdu) {
                Ok(w) => Some(w),
                Err(e) => {
                    eprintln!("phy tx error: {e}");
                    None
                }
            },
            Some(MacAction::TransmitNdp { body }) => match phy_tx.generate_ndp(body) {
                Ok(w) => Some(w),
                Err(e) => {
                    eprintln!("phy ndp tx error: {e}");
                    None
                }
            },
            None => None,
        };
        if let Some(wave) = wave {
            let mut up = Vec::with_capacity(wave.len() * 2 + 64);
            interp.process(&wave, &mut up);
            interp.process(&vec![C32::new(0.0, 0.0); 32], &mut up);
            if let Err(e) = sdr_tx.send(&up) {
                eprintln!("radio tx error: {e}");
            }
        }
    }
}

#[cfg(not(feature = "pluto"))]
fn run_radio(_args: &Args, _mac: Mac, _nic: Box<dyn Nic>) -> Result<()> {
    bail!("s2g-node requires the 'pluto' feature (default) — rebuild without --no-default-features")
}
