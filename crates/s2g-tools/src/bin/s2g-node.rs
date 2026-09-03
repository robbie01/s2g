//! Full network node: NIC (TAP or UDP) ↔ OCB MAC ↔ S1G PHY ↔ PlutoSDR.
//!
//! On Unix with `--features tap`: `s2g-node --tap s2g0` creates a real L2
//! interface the OS can route through. On Windows (no L2 TAP support yet):
//! `s2g-node --udp 127.0.0.1:5001` shuttles raw Ethernet frames as UDP
//! datagrams instead.

use anyhow::{bail, Result};
use clap::Parser;
use s2g_mac::{FilterConfig, IdentConfig, Mac, MacAction, MacConfig, MacError, MacEvent, RateConfig};
use s2g_phy::vector::Coding;
use s2g_tools::nic::Nic;
use s2g_tools::DEFAULT_CENTER_FREQ_HZ;

#[derive(Parser, Debug)]
#[command(name = "s2g-node", about = "S1G OCB network node (NIC ↔ MAC ↔ PHY ↔ Pluto)")]
struct Args {
    /// Attach a TAP interface (optionally named): tappers on Unix (feature "tap"),
    /// the OpenVPN tap-windows6 adapter on Windows ("TAP-Windows Adapter V9")
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
    /// Longest response wait in ms (SDR buffering makes SIFS-scale impossible);
    /// the MAC shrinks it to what responses actually take
    #[arg(long, default_value_t = 150)]
    ack_timeout_ms: u64,
    /// Keep --ack-timeout-ms fixed instead of learning the response delay
    #[arg(long)]
    fixed_ack_timeout: bool,
    #[arg(long, default_value_t = 3)]
    retries: u32,
    /// Send every unicast frame at --mcs instead of adapting the MCS per peer
    #[arg(long)]
    fixed_mcs: bool,
    /// Lowest MCS rate control falls back to
    #[arg(long, default_value_t = 0)]
    min_mcs: u8,
    /// Highest MCS rate control probes
    #[arg(long, default_value_t = 8)]
    max_mcs: u8,
    /// Most Ethernet frames packed into one A-MPDU (1 = no aggregation, 16 = the NDP BlockAck bitmap)
    #[arg(long, default_value_t = 16)]
    ampdu: usize,
    /// Amateur call sign: transmitted in the clear before the first frame, every
    /// --id-interval-min while sending, and at the end of a communication
    #[arg(long)]
    callsign: Option<String>,
    /// Free text appended to the identification (grid square, node name)
    #[arg(long, default_value = "")]
    id_info: String,
    /// Minutes between identifications while transmitting
    #[arg(long, default_value_t = 10)]
    id_interval_min: u64,
    /// Disable the good-neighbor filter entirely
    #[arg(long)]
    no_filter: bool,
    /// Filter only frames leaving for the air, deliver everything received
    #[arg(long)]
    filter_egress_only: bool,
    /// Additionally block this TCP/UDP port (repeatable); see README for the recommended list
    #[arg(long)]
    block_port: Vec<u16>,
    /// Remove this port from the blocked list (repeatable)
    #[arg(long)]
    allow_port: Vec<u16>,
    /// Let IPv4 and ARP through
    #[arg(long)]
    allow_ipv4: bool,
    /// Let ICMPv6 Router Solicitation/Advertisement/Redirect through
    #[arg(long)]
    allow_router_discovery: bool,
    /// Let DHCPv6 through
    #[arg(long)]
    allow_dhcpv6: bool,
    /// Let MLD through
    #[arg(long)]
    allow_mld: bool,
    /// Let mDNS/LLMNR/SSDP/WS-Discovery/NetBIOS/SMB through
    #[arg(long)]
    allow_discovery: bool,
    /// Let every ESP packet through, not only those recognized as ESP-NULL
    #[arg(long)]
    allow_esp: bool,
    /// Let IPv6 destinations outside link-local/multicast/ULA through (tunnels always may)
    #[arg(long)]
    allow_global: bool,

    /// Pluto iiod address
    #[arg(long, default_value = "192.168.2.1")]
    uri: String,
    /// RF center frequency, Hz
    #[arg(long, default_value_t = DEFAULT_CENTER_FREQ_HZ)]
    freq_hz: f64,
    /// Pluto oscillator trim (ad9361-phy xo_correction, Hz, nominal 40000000):
    /// scale it by (1 + ppm/1e6) to pull the peer offset the node reports to zero
    #[arg(long)]
    xo_correction: Option<u64>,
    /// RX gain: "auto" or dB
    #[arg(long, default_value = "auto")]
    rx_gain: String,
    /// TX gain (attenuation), dB ≤ 0
    #[arg(long, default_value_t = -10.0)]
    tx_gain_db: f64,
    /// Analog RF bandwidth hint, Hz
    #[arg(long, default_value_t = 2_200_000.0)]
    rf_bandwidth_hz: f64,
    /// Receiver calibration: dBm = dBFS + this offset (RCPI, CCA thresholds)
    #[arg(long)]
    cal_offset_db: Option<f32>,
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
    #[cfg(windows)]
    if let Some(name) = &args.tap {
        let n = if name.is_empty() { None } else { Some(name.as_str()) };
        let mut tap = s2g_tools::wintap::WinTapNic::new(n)?;
        if let Some((maj, min)) = tap.driver_version() {
            eprintln!("tap-windows6 driver {maj}.{min}");
        }
        return Ok(Box::new(tap));
    }
    #[cfg(not(any(windows, all(unix, feature = "tap"))))]
    if args.tap.is_some() {
        bail!("TAP support needs --features tap on this platform; or use --udp BIND");
    }
    bail!("choose a NIC: --tap [NAME] (tappers on Unix, tap-windows6 on Windows) or --udp BIND")
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
    cfg.ack_timeout_adaptive = !args.fixed_ack_timeout;
    cfg.max_retries = args.retries;
    cfg.rate = RateConfig {
        enabled: !args.fixed_mcs,
        start_mcs: args.mcs,
        min_mcs: args.min_mcs,
        max_mcs: args.max_mcs,
        ..Default::default()
    };
    cfg.ampdu_max_mpdus = args.ampdu;
    cfg.filter = if args.no_filter {
        FilterConfig::off()
    } else {
        let mut f = FilterConfig::good_neighbor();
        f.ingress = !args.filter_egress_only;
        f.allow_ipv4 = args.allow_ipv4;
        f.allow_router_discovery = args.allow_router_discovery;
        f.allow_dhcpv6 = args.allow_dhcpv6;
        f.allow_mld = args.allow_mld;
        f.allow_discovery = args.allow_discovery;
        f.allow_esp = args.allow_esp;
        f.allow_global_dst = args.allow_global;
        for p in &args.block_port {
            if !f.blocked_ports.contains(p) {
                f.blocked_ports.push(*p);
            }
        }
        f.blocked_ports.retain(|p| !args.allow_port.contains(p));
        f
    };
    eprintln!("good-neighbor filter: {}", cfg.filter.describe());
    cfg.ident = IdentConfig {
        callsign: args.callsign.clone(),
        info: args.id_info.clone(),
        interval_us: args.id_interval_min.max(1) * 60_000_000,
        ..Default::default()
    };
    match &args.callsign {
        Some(c) => eprintln!("station identification: DE {} every {} min and at the end of each communication", c.to_ascii_uppercase(), args.id_interval_min.max(1)),
        None => eprintln!("WARNING: no --callsign: no station identification will be transmitted (required under Part 97)"),
    }
    let nic = make_nic(&args)?;
    eprintln!(
        "s2g-node: mac {} | mcs {} {:?}{} | rate control {} | ack {} ({}) | rts {:?} | nic {}",
        fmt_mac(&addr),
        args.mcs,
        cfg.fec_coding,
        if args.traveling_pilots { " + traveling pilots" } else { "" },
        if cfg.rate.enabled { format!("on ({}..={}, start {})", cfg.rate.min_mcs, cfg.rate.max_mcs, cfg.rate.start_mcs) } else { "off".to_string() },
        cfg.ack_enabled,
        if cfg.ndp_ack { "NDP" } else { "legacy" },
        cfg.rts_threshold,
        nic.describe()
    );
    run_radio(&args, Mac::new(cfg), nic)
}

/// Rate-limited log of what the good-neighbor filter dropped: the first
/// frame of each (direction, reason) at once, then a count every 30 s.
#[derive(Default)]
struct FilterLog {
    seen: std::collections::HashMap<(&'static str, &'static str), (u64, Option<std::time::Instant>)>,
}

impl FilterLog {
    fn note(&mut self, direction: &'static str, reason: &'static str) {
        let e = self.seen.entry((direction, reason)).or_insert((0, None));
        e.0 += 1;
        let due = e.1.is_none_or(|t| t.elapsed() >= std::time::Duration::from_secs(30));
        if due {
            eprintln!("filter: dropped {} {direction}: {reason}", e.0);
            e.1 = Some(std::time::Instant::now());
        }
    }
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

    let gain = if args.rx_gain == "auto" {
        RxGain::Auto
    } else {
        RxGain::Manual(args.rx_gain.parse().map_err(|_| anyhow::anyhow!("--rx-gain must be 'auto' or dB"))?)
    };
    let mut pluto = s2g_sdr_pluto::Pluto::open(&args.uri).map_err(|e| anyhow::anyhow!("pluto: {e}"))?;
    let scfg = StreamConfig {
        center_freq_hz: args.freq_hz,
        sample_rate_hz: DEFAULT_DEVICE_RATE_HZ,
        rf_bandwidth_hz: args.rf_bandwidth_hz,
    };
    let mut sdr_rx = pluto.open_rx(&scfg, gain).map_err(|e| anyhow::anyhow!("pluto rx: {e}"))?;
    let mut sdr_tx = pluto.open_tx(&scfg, args.tx_gain_db).map_err(|e| anyhow::anyhow!("pluto tx: {e}"))?;
    eprintln!("radio: {} @ {} Hz, {} S/s device rate", args.uri, args.freq_hz, DEFAULT_DEVICE_RATE_HZ);
    if let Some(xo) = args.xo_correction {
        pluto.set_xo_correction(xo).map_err(|e| anyhow::anyhow!("pluto xo_correction: {e}"))?;
    }
    match pluto.xo_correction() {
        Ok(xo) => eprintln!("oscillator trim (xo_correction): {xo} Hz"),
        Err(e) => eprintln!("oscillator trim not readable: {e}"),
    }
    let rx_cfg = RxConfig {
        cal_offset_db: args.cal_offset_db.unwrap_or(0.0),
        auto_cal: args.cal_offset_db.is_none(),
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
            let mut cal_announced = false;
            let mut dev = vec![C32::new(0.0, 0.0); 16384];
            let mut native = Vec::with_capacity(8192);
            let mut events = Vec::new();
            loop {
                match sdr_rx.recv(&mut dev) {
                    Ok(n) => {
                        native.clear();
                        dec.process(&dev[..n], &mut native);
                        rx.process(&native, &mut events);
                        if !cal_announced {
                            if let Some(f) = rx.noise_floor_dbfs() {
                                cal_announced = true;
                                eprintln!("noise floor {f:.1} dBFS | cal offset {:+.1} dB | energy detect at {:.1} dBFS", rx.cal_offset_db(), -72.0 - rx.cal_offset_db());
                            }
                        }
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
    let mut rate_seen: std::collections::HashMap<[u8; 6], u8> = std::collections::HashMap::new();
    let mut filter_log = FilterLog::default();
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
                            eprintln!("rx: S1G_LONG PPDU ({} µs)", rxvector.ppdu_duration_us())
                        }
                        _ => {}
                    }
                }
                mac.on_phy_event(&ev, now_us, &mut mac_events);
            }
            Ok(Msg::Eth(f)) => match mac.enqueue_eth(&f) {
                Ok(()) => {}
                Err(MacError::Filtered(reason)) => filter_log.note("to air", reason),
                Err(e) => eprintln!("drop outgoing frame: {e}"),
            },
            Err(mpsc::RecvTimeoutError::Timeout) => {}
            Err(mpsc::RecvTimeoutError::Disconnected) => bail!("worker thread died"),
        }
        for e in mac_events.drain(..) {
            match e {
                MacEvent::EthReceived(f) => {
                    let _ = nic_out_tx.send(f);
                }
                MacEvent::TxComplete { dest, acked, retries, mcs } => {
                    if args.verbose {
                        eprintln!("tx done → {} acked={acked} retries={retries} mcs={mcs}", fmt_mac(&dest));
                    }
                    if let Some(now_mcs) = mac.peer_mcs(&dest) {
                        if rate_seen.insert(dest, now_mcs) != Some(now_mcs) {
                            let cfo = mac
                                .rate_control()
                                .peer_cfo_hz(&dest)
                                .map(|c| format!(" | peer carrier offset {c:+.0} Hz ({:+.1} ppm)", c as f64 / args.freq_hz * 1e6))
                                .unwrap_or_default();
                            let resp = mac
                                .response_delay_us()
                                .map(|(srtt, wait)| format!(" | responses take {} ms, waiting up to {} ms", srtt / 1000, wait / 1000))
                                .unwrap_or_default();
                            eprintln!("rate → {}: MCS {now_mcs}{cfo}{resp}", fmt_mac(&dest));
                        }
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
                MacEvent::Filtered { reason } => filter_log.note("from air", reason),
                MacEvent::IdentSent { text } => eprintln!("id sent: {text}"),
                MacEvent::IdentReceived { src, text } => eprintln!("id heard from {}: {text}", fmt_mac(&src)),
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
            interp.process(&[C32::new(0.0, 0.0); 32], &mut up);
            if let Err(e) = sdr_tx.send(&up) {
                eprintln!("radio tx error: {e}");
            }
        }
    }
}

#[cfg(not(feature = "pluto"))]
fn run_radio(_args: &Args, _mac: Mac, _nic: Box<dyn Nic>) -> Result<()> {
    bail!("s2g-node requires the 'pluto' feature (default); rebuild without --no-default-features")
}
