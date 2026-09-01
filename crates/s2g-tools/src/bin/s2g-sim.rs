//! Loopback simulation: TX chain → channel impairments → RX chain.
//! Reports per-MCS PER over an SNR sweep. No radio required.

use anyhow::Result;
use clap::Parser;
use s2g_phy::params::valid_mcs;
use s2g_phy::rx::{Receiver, RxConfig, RxEvent};
use s2g_phy::vector::TxVector;
use s2g_phy::Transmitter;
use s2g_tools::{apply_channel, Complex32, Impairments, Rng};

#[derive(Parser, Debug)]
#[command(name = "s2g-sim", about = "S1G PHY loopback simulator (PER vs SNR)")]
struct Args {
    /// MCS to test: "all" or an index
    #[arg(long, default_value = "all")]
    mcs: String,
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
    /// RNG seed
    #[arg(long, default_value_t = 1)]
    seed: u64,
}

fn main() -> Result<()> {
    let args = Args::parse();
    let mcs_list: Vec<u8> = if args.mcs == "all" {
        valid_mcs().collect()
    } else {
        vec![args.mcs.parse()?]
    };
    let snrs: Vec<f32> = args.snr_db.split(',').map(|s| s.trim().parse().unwrap()).collect();
    let tx = Transmitter::new();
    let mut rng = Rng(args.seed);

    println!(
        "{:>4} | {}",
        "MCS",
        snrs.iter().map(|s| format!("{s:>7.1}")).collect::<Vec<_>>().join(" ")
    );
    println!("     | PER at each SNR (dB); cfo={:+.0} Hz, mu={}", args.cfo_hz, args.frac_delay);
    println!("-----+{}", "-".repeat(8 * snrs.len()));

    for mcs in mcs_list {
        let mut row = Vec::new();
        for &snr in &snrs {
            let mut errors = 0usize;
            for _ in 0..args.count {
                let psdu = rng.bytes(args.len);
                let wave = tx
                    .generate(&TxVector { mcs, ..Default::default() }, &psdu)
                    .expect("tx");
                let mut stream: Vec<Complex32> = (0..400)
                    .map(|_| Complex32::new(rng.gauss(), rng.gauss()) * 1e-4)
                    .collect();
                stream.extend(&wave);
                stream.extend((0..200).map(|_| Complex32::new(rng.gauss(), rng.gauss()) * 1e-4));
                let imp = Impairments {
                    snr_db: Some(snr),
                    cfo_hz: args.cfo_hz,
                    frac_delay: args.frac_delay,
                    amplitude: 1.0,
                };
                let noisy = apply_channel(&stream, &imp, &mut rng);
                let mut rx = Receiver::new(RxConfig::default());
                let mut ev = Vec::new();
                rx.process(&noisy, &mut ev);
                rx.finish(&mut ev);
                let ok = ev.iter().any(|e| matches!(e, RxEvent::PsduReceived { psdu: p, .. } if p == &psdu));
                if !ok {
                    errors += 1;
                }
            }
            row.push(format!("{:>7.3}", errors as f32 / args.count as f32));
        }
        println!("{mcs:>4} | {}", row.join(" "));
    }
    Ok(())
}
