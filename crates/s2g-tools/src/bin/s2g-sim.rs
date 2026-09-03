//! Loopback simulation: TX chain → channel impairments → RX chain.
//! Reports per-MCS PER over an SNR sweep. No radio required.

use anyhow::Result;
use clap::Parser;
use s2g_phy::params::valid_mcs;
use s2g_phy::rx::{Receiver, RxConfig, RxEvent};
use s2g_phy::vector::{Coding, GuardInterval, PreambleType, TxVector};
use s2g_phy::Transmitter;
use s2g_phy::sim::{Impairments, Rng};
use s2g_tools::Complex32;

#[derive(Parser, Debug)]
#[command(name = "s2g-sim", about = "S1G PHY loopback simulator (PER vs SNR)")]
struct Args {
    /// MCS to test: "all" or an index
    #[arg(long, default_value = "all")]
    mcs: String,
    /// Use LDPC instead of BCC
    #[arg(long)]
    ldpc: bool,
    /// Use traveling pilots
    #[arg(long)]
    traveling_pilots: bool,
    /// Short guard interval (4 µs from the second Data symbol on)
    #[arg(long)]
    sgi: bool,
    /// S1G_LONG (SU) preamble instead of S1G_SHORT
    #[arg(long)]
    long_preamble: bool,
    /// Send as A-MPDU (aggregation bit)
    #[arg(long)]
    aggregation: bool,
    /// SNR points in dB, comma separated
    #[arg(long, default_value = "5,10,15,20,25,30,40")]
    snr_db: String,
    /// PPDUs per (MCS, SNR) point
    #[arg(long, default_value_t = 50)]
    count: usize,
    /// PSDU length in octets
    #[arg(long, default_value_t = 100)]
    len: usize,
    /// Carrier frequency offset to apply, Hz
    #[arg(long, default_value_t = 10_000.0)]
    cfo_hz: f32,
    /// Fractional-sample timing offset (0..1)
    #[arg(long, default_value_t = 0.35)]
    frac_delay: f32,
    /// Sampling-clock offset, ppm (receiver fast when positive)
    #[arg(long, default_value_t = 0.0)]
    sfo_ppm: f64,
    /// Static echo delay in samples (0 = none)
    #[arg(long, default_value_t = 0)]
    echo_delay: usize,
    /// Static echo gain (linear)
    #[arg(long, default_value_t = 0.5)]
    echo_gain: f32,
    /// RNG seed
    #[arg(long, default_value_t = 1)]
    seed: u64,
    /// Also print, per MCS, the mean SNR the receiver reports at each point
    /// (what rate control sees), from the PPDUs it decoded
    #[arg(long)]
    report_snr: bool,
}

fn main() -> Result<()> {
    let args = Args::parse();
    let mcs_list: Vec<u8> = if args.mcs == "all" { valid_mcs().collect() } else { vec![args.mcs.parse()?] };
    let snrs: Vec<f32> = args.snr_db.split(',').map(|s| s.trim().parse().unwrap()).collect();
    let coding = if args.ldpc { Coding::Ldpc } else { Coding::Bcc };
    let tx = Transmitter::new();
    let mut rng = Rng(args.seed);

    println!("{:>4} | {}", "MCS", snrs.iter().map(|s| format!("{s:>7.1}")).collect::<Vec<_>>().join(" "));
    println!(
        "     | PER at each SNR (dB); {coding:?}{}{}{} cfo={:+.0} Hz, mu={}, sfo={} ppm, echo={}",
        if args.traveling_pilots { " + traveling pilots" } else { "" },
        if args.sgi { " + short GI" } else { "" },
        if args.long_preamble { " + S1G_LONG" } else { "" },
        args.cfo_hz,
        args.frac_delay,
        args.sfo_ppm,
        if args.echo_delay > 0 { format!("{}@{} samples", args.echo_gain, args.echo_delay) } else { "none".into() }
    );
    println!("-----+{}", "-".repeat(8 * snrs.len()));

    for mcs in mcs_list {
        let mut row = Vec::new();
        let mut reported = Vec::new();
        for &snr in &snrs {
            let mut errors = 0usize;
            let (mut snr_sum, mut snr_n) = (0.0f32, 0usize);
            for _ in 0..args.count {
                let psdu = rng.bytes(args.len);
                let txv = TxVector {
                    mcs,
                    fec_coding: coding,
                    traveling_pilots: args.traveling_pilots,
                    aggregation: args.aggregation,
                    gi: if args.sgi { GuardInterval::Short } else { GuardInterval::Long },
                    preamble_type: if args.long_preamble { PreambleType::S1gLong } else { PreambleType::S1gShort },
                    ..Default::default()
                };
                let wave = tx.generate(&txv, &psdu).expect("tx");
                let mut stream: Vec<Complex32> = (0..400).map(|_| Complex32::new(rng.gauss(), rng.gauss()) * 1e-4).collect();
                stream.extend(&wave);
                stream.extend((0..200).map(|_| Complex32::new(rng.gauss(), rng.gauss()) * 1e-4));
                let imp = Impairments {
                    snr_db: Some(snr),
                    cfo_hz: args.cfo_hz,
                    frac_delay: args.frac_delay,
                    amplitude: 1.0,
                    sfo_ppm: args.sfo_ppm,
                    echo: (args.echo_delay > 0).then_some((args.echo_delay, Complex32::new(args.echo_gain, -0.3 * args.echo_gain))),
                };
                let noisy = imp.apply(&stream, &mut rng);
                let mut rx = Receiver::new(RxConfig { emit_cca: false, ..Default::default() });
                let mut ev = Vec::new();
                rx.process(&noisy, &mut ev);
                rx.finish(&mut ev);
                // An aggregated PSDU comes back padded to the symbol capacity.
                let ok = ev
                    .iter()
                    .any(|e| matches!(e, RxEvent::PsduReceived { psdu: p, .. } if p.len() >= psdu.len() && p[..psdu.len()] == psdu[..]));
                if !ok {
                    errors += 1;
                }
                for e in &ev {
                    if let RxEvent::PsduReceived { metrics, .. } = e {
                        snr_sum += metrics.snr_db;
                        snr_n += 1;
                    }
                }
            }
            row.push(format!("{:>7.3}", errors as f32 / args.count as f32));
            reported.push(if snr_n > 0 { format!("{:>7.1}", snr_sum / snr_n as f32) } else { format!("{:>7}", "-") });
        }
        println!("{mcs:>4} | {}", row.join(" "));
        if args.report_snr {
            println!(" snr | {}", reported.join(" "));
        }
    }
    Ok(())
}
