//! Link-level simulation for tuning the rate controller without a radio.
//!
//! What it models, per attempt, the way `engine.rs` does it:
//! * batch → PPDU packing (`plan_attempt`: as many MPDUs as fit), airtime
//!   from the PHY's TXTIME for the MCS, guard interval and coding in use,
//!   DIFS + binary-exponential backoff;
//! * a lost PPDU (preamble/SIG not decoded, or every MPDU failing its FCS)
//!   costs the response timeout; anything acknowledged costs the response
//!   turnaround (about 45 ms through two SDR pipelines, measured on the
//!   virtual link); unacknowledged MPDUs are retried, then dropped;
//! * per-MPDU FCS failure from a PER-vs-SNR model fitted to the PHY's own
//!   loopback (`s2g-sim --report-snr`), in the receiver's reported-SNR
//!   units; LDPC as a fixed SNR gain; a short-GI or long-GI PPDU lost
//!   outright once the channel's delay spread passes what the PHY tolerates
//!   (`s2g-sim --echo-delay`); an LDPC PPDU lost outright at a peer that
//!   cannot decode it;
//! * the hints the controller gets from acknowledgments: the forward SNR
//!   plus a link asymmetry and estimation noise, and the delay spread plus
//!   estimation noise.
//!
//! Channels: static, log-normal shadowing (AR(1) in dB), Rician / Rayleigh
//! fading (sum of sinusoids with a Jakes-like Doppler spread), a step; each
//! with a delay spread and a peer that does or does not decode LDPC.
//! Traffic: a saturated queue of frames of one size, in batches.
//!
//! A configuration's score in a scenario is its goodput relative to the
//! best fixed MCS, guard interval and coding for that scenario, a
//! reference that knows the channel and the peer in advance, so an
//! adaptive controller only beats it on a changing channel. The defaults
//! in `RateConfig` came out of `sweep`:
//!
//!     cargo test -p s2g-mac --release --test rate_sim -- --nocapture
//!     cargo test -p s2g-mac --release --test rate_sim sweep -- --ignored --nocapture

use s2g_mac::rate::{Adapt, RateConfig, RateControl, TxChoice, LADDER};
use s2g_mac::{ampdu, frame};
use s2g_phy::sim::Rng;
use s2g_phy::tx::{aggregated_capacity, txtime_us};
use s2g_phy::vector::{Coding, GuardInterval, TxVector};
use std::f32::consts::PI;

const PEER: [u8; 6] = [2, 0, 0, 0, 0, 0xB];

/// Per LADDER entry: reported SNR at which half of the 100-octet PSDUs are
/// lost, the shift of that point per decade of PSDU length, and the
/// logistic slope per dB. Fitted to `s2g-sim --report-snr` at 100 and 1000
/// octets (BCC, 10 kHz CFO).
const PER_MODEL: [(f32, f32, f32); 10] = [
    (4.8, 1.3, 1.0),
    (6.1, 2.1, 1.4),
    (7.8, 1.2, 1.8),
    (9.4, 0.9, 1.8),
    (12.9, 1.4, 1.8),
    (16.4, 1.4, 1.8),
    (16.7, 2.5, 1.8),
    (19.5, 1.2, 1.8),
    (21.9, 3.0, 1.8),
    (27.8, 1.6, 1.5),
];

/// LDPC's gain over BCC in reported SNR at every MCS above 0
/// (`s2g-sim --ldpc --report-snr`, 1000 octets), dB.
const LDPC_GAIN_DB: f32 = 1.8;

/// Reported RMS delay spread (µs) past which this PHY loses short-GI and
/// long-GI PPDUs at high MCS (`s2g-sim --sgi --echo-delay` against the
/// `--report-snr` reading of the same channel).
const SGI_SPREAD_LIMIT_US: f32 = 0.95;
const LGI_SPREAD_LIMIT_US: f32 = 1.3;

/// Probability that a BCC long-GI PSDU of `len` octets at `LADDER[idx]` is
/// lost at a reported SNR of `snr_db`.
fn per(idx: usize, snr_db: f32, len: usize) -> f32 {
    let (s50, per_decade, k) = PER_MODEL[idx];
    let s = s50 + per_decade * (len as f32 / 100.0).log10();
    1.0 / (1.0 + (k * (snr_db - s)).exp())
}

fn ladder_index(mcs: u8) -> usize {
    LADDER.iter().position(|&m| m == mcs).expect("ladder MCS")
}

/// Probability that a PSDU of `len` octets sent with `choice` is lost at
/// `snr_db` over a channel of `spread_us` RMS delay spread, to a peer that
/// decodes LDPC or not.
fn loss(choice: TxChoice, snr_db: f32, spread_us: f32, len: usize, peer_ldpc: bool) -> f32 {
    let idx = ladder_index(choice.mcs);
    let ldpc = choice.fec_coding == Coding::Ldpc;
    if ldpc && !peer_ldpc {
        return 1.0;
    }
    let limit = if choice.gi == GuardInterval::Short { SGI_SPREAD_LIMIT_US } else { LGI_SPREAD_LIMIT_US };
    if spread_us > limit {
        return 1.0;
    }
    let gain = if ldpc && idx > 0 { LDPC_GAIN_DB } else { 0.0 };
    per(idx, snr_db + gain, len)
}

#[derive(Clone, Copy, Debug)]
enum Channel {
    /// Constant reported SNR, dB.
    Static(f32),
    /// Log-normal shadowing: AR(1) in dB with `sigma` and correlation time `tau_s`.
    Shadow { mean: f32, sigma: f32, tau_s: f32 },
    /// Rician fading with K factor `k` (0 = Rayleigh) and maximum Doppler shift.
    Fading { mean: f32, doppler_hz: f32, k: f32 },
    /// `hi` dB, dropping to `lo` between `from_s` and `to_s`.
    Step { hi: f32, lo: f32, from_s: f32, to_s: f32 },
}

const PATHS: usize = 12;

struct ChannelState {
    shadow: f32,
    last_us: u64,
    /// (angular Doppler, phase) per scattered path.
    paths: [(f32, f32); PATHS],
}

impl ChannelState {
    fn new(ch: &Channel, rng: &mut Rng) -> Self {
        let mut paths = [(0.0f32, 0.0f32); PATHS];
        if let Channel::Fading { doppler_hz, .. } = ch {
            for p in paths.iter_mut() {
                let angle = 2.0 * PI * rng.uniform();
                *p = (2.0 * PI * doppler_hz * angle.cos(), 2.0 * PI * rng.uniform());
            }
        }
        Self { shadow: 0.0, last_us: 0, paths }
    }

    fn snr_at(&mut self, ch: &Channel, t_us: u64, rng: &mut Rng) -> f32 {
        match *ch {
            Channel::Static(s) => s,
            Channel::Shadow { mean, sigma, tau_s } => {
                let dt = t_us.saturating_sub(self.last_us) as f32 * 1e-6;
                self.last_us = t_us;
                let rho = (-dt / tau_s).exp();
                self.shadow = rho * self.shadow + (1.0 - rho * rho).sqrt() * sigma * rng.gauss();
                mean + self.shadow
            }
            Channel::Fading { mean, k, .. } => {
                let t = t_us as f32 * 1e-6;
                let (mut re, mut im) = (0.0f32, 0.0f32);
                for (w, phi) in self.paths {
                    re += (w * t + phi).cos();
                    im += (w * t + phi).sin();
                }
                // Scattered power 1/(K+1), line of sight K/(K+1): mean power 1.
                let s = (1.0 / (k + 1.0) / PATHS as f32).sqrt();
                let los = (k / (k + 1.0)).sqrt();
                let power = (re * s + los).powi(2) + (im * s).powi(2);
                mean + 10.0 * power.max(1e-6).log10()
            }
            Channel::Step { hi, lo, from_s, to_s } => {
                let t = t_us as f32 * 1e-6;
                if t >= from_s && t < to_s {
                    lo
                } else {
                    hi
                }
            }
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct Link {
    /// Response turnaround on success, µs.
    rtt_us: u64,
    /// What a lost attempt costs, µs (the response timeout).
    timeout_us: u64,
    max_retries: u32,
    /// PPDU loss unrelated to SNR (interference, collisions).
    loss_floor: f32,
    /// Reverse-link minus forward-link SNR, dB: what the hint overstates.
    asym_db: f32,
    /// Standard deviation of one frame's SNR estimate, dB.
    est_noise_db: f32,
    /// RMS delay spread of the channel as the PHY reports it, µs (both
    /// directions).
    spread_us: f32,
    /// Standard deviation of one frame's delay-spread reading, µs.
    spread_noise_us: f32,
    /// The peer decodes LDPC.
    peer_ldpc: bool,
}

fn link(timeout_ms: u64) -> Link {
    Link {
        rtt_us: 45_000,
        timeout_us: timeout_ms * 1000,
        max_retries: 3,
        loss_floor: 0.0,
        asym_db: 0.0,
        est_noise_db: 0.7,
        spread_us: 0.4,
        spread_noise_us: 0.1,
        peer_ldpc: true,
    }
}

#[derive(Clone, Copy, Debug)]
struct Traffic {
    body_len: usize,
    per_batch: usize,
}

#[derive(Default, Debug, Clone, Copy)]
struct Outcome {
    bytes: u64,
    time_us: u64,
    attempts: u32,
    timeouts: u32,
    timeout_us: u64,
    drops: u32,
}

impl Outcome {
    fn goodput_kbps(&self) -> f32 {
        self.bytes as f32 * 8.0 / self.time_us.max(1) as f32 * 1e3
    }
    fn timeout_share(&self) -> f32 {
        self.timeout_us as f32 / self.time_us.max(1) as f32
    }
}

enum Policy {
    Adaptive(RateConfig),
    Fixed(TxChoice),
}

/// `plan_attempt` for `n` outstanding MPDUs of `mpdu_len` octets at `mcs`
/// with `coding`: how many fit one PPDU, the PSDU length, aggregated or not.
fn plan(mcs: u8, coding: Coding, mpdu_len: usize, n: usize) -> (usize, usize, bool) {
    let mut fit = 0;
    for i in 0..n.min(16) {
        let ok = if i == 0 {
            mpdu_len <= 511 || aggregated_capacity(mcs, ampdu::pre_eof_len(mpdu_len), coding).is_ok()
        } else {
            aggregated_capacity(mcs, ampdu::pre_eof_len_many(&vec![mpdu_len; i + 1]), coding).is_ok()
        };
        if !ok {
            break;
        }
        fit = i + 1;
    }
    assert!(fit >= 1, "an MPDU of {mpdu_len} octets fits no PPDU at MCS {mcs}");
    if fit == 1 && mpdu_len <= 511 {
        return (1, mpdu_len, false);
    }
    let pre = if fit == 1 { ampdu::pre_eof_len(mpdu_len) } else { ampdu::pre_eof_len_many(&vec![mpdu_len; fit]) };
    (fit, aggregated_capacity(mcs, pre, coding).expect("fits"), true)
}

const GIS: [GuardInterval; 2] = [GuardInterval::Long, GuardInterval::Short];
const CODINGS: [Coding; 2] = [Coding::Bcc, Coding::Ldpc];

/// (MPDUs that fit, airtime µs) per ladder index, guard interval, coding
/// and outstanding count.
type Plans = Vec<[[Vec<(usize, u64)>; 2]; 2]>;

fn plans(mpdu_len: usize, per_batch: usize) -> Plans {
    LADDER
        .iter()
        .map(|&m| {
            let table = |gi: GuardInterval, coding: Coding| -> Vec<(usize, u64)> {
                (0..=per_batch)
                    .map(|n| {
                        if n == 0 {
                            return (0, 0);
                        }
                        let (fit, psdu_len, aggregation) = plan(m, coding, mpdu_len, n);
                        let txv = TxVector { mcs: m, gi, fec_coding: coding, aggregation, ..Default::default() };
                        (fit, txtime_us(&txv, psdu_len).expect("txtime") as u64)
                    })
                    .collect()
            };
            [[table(GIS[0], CODINGS[0]), table(GIS[0], CODINGS[1])], [table(GIS[1], CODINGS[0]), table(GIS[1], CODINGS[1])]]
        })
        .collect()
}

fn run(policy: &Policy, link: &Link, traffic: &Traffic, channel: &Channel, duration_us: u64, seed: u64, mut trace: Option<&mut Vec<String>>) -> Outcome {
    let mut rng = Rng(seed);
    let mut ch = ChannelState::new(channel, &mut rng);
    let mut ctl = match policy {
        Policy::Adaptive(cfg) => Some(RateControl::new(cfg.clone())),
        Policy::Fixed(_) => None,
    };
    let hdr = if traffic.per_batch > 1 { frame::QOS_DATA_HDR_LEN } else { frame::DATA_HDR_LEN };
    let mpdu_len = hdr + traffic.body_len + 4;
    let plans = plans(mpdu_len, traffic.per_batch);
    let mut out = Outcome::default();
    let mut t = 0u64;
    while t < duration_us {
        // Retries so far of each outstanding MPDU of the batch.
        let mut pending: Vec<u32> = vec![0; traffic.per_batch];
        let mut failures = 0u32;
        let mut cw_exp = 4u32;
        while !pending.is_empty() && t < duration_us {
            let offered = pending.len() * mpdu_len;
            let choice = match policy {
                Policy::Adaptive(_) => ctl.as_mut().expect("controller").select(&PEER, failures, offered),
                Policy::Fixed(c) => *c,
            };
            let idx = ladder_index(choice.mcs);
            let (g, c) = ((choice.gi == GuardInterval::Short) as usize, (choice.fec_coding == Coding::Ldpc) as usize);
            let (n_fit, airtime) = plans[idx][g][c][pending.len()];
            t += 264 + (rng.next_u64() % (1u64 << cw_exp)) * 52;
            let snr = ch.snr_at(channel, t + airtime / 2, &mut rng);
            let ppdu_lost = rng.uniform() < link.loss_floor || rng.uniform() < per(0, snr, 10);
            let p_fail = loss(choice, snr, link.spread_us, mpdu_len, link.peer_ldpc);
            let mut n_ok = 0usize;
            let mut kept = Vec::with_capacity(pending.len());
            for (i, &r) in pending.iter().enumerate() {
                if i >= n_fit {
                    kept.push(r);
                } else if !ppdu_lost && rng.uniform() >= p_fail {
                    n_ok += 1;
                    out.bytes += traffic.body_len as u64;
                } else if r + 1 > link.max_retries {
                    out.drops += 1;
                } else {
                    kept.push(r + 1);
                }
            }
            out.attempts += 1;
            if n_ok == 0 {
                t += airtime + link.timeout_us;
                out.timeouts += 1;
                out.timeout_us += link.timeout_us;
            } else {
                t += airtime + link.rtt_us;
                if let Some(ctl) = ctl.as_mut() {
                    ctl.observe_snr(&PEER, snr + link.asym_db + link.est_noise_db * rng.gauss());
                    ctl.observe_delay_spread(&PEER, link.spread_us + link.spread_noise_us * rng.gauss());
                }
            }
            // As the engine judges it: the attempt succeeded for its rate
            // when it delivered at least what one PPDU at the next-lower
            // rate (the long GI at the same MCS for a short-GI attempt)
            // would have carried of this batch, scaled by the ratio of
            // airtime plus turnaround; the bottom of the ladder has to
            // deliver everything. Only consecutive such failures step the
            // rate down; any unacknowledged MPDU doubles the contention
            // window.
            let bar = |(fit, lower_airtime): (usize, u64)| (fit as f64 * (airtime + link.rtt_us) as f64 / (lower_airtime + link.rtt_us) as f64).ceil() as usize;
            let must = if g == 1 {
                bar(plans[idx][0][c][pending.len()])
            } else if idx == 0 {
                n_fit
            } else {
                bar(plans[idx - 1][0][c][pending.len()])
            };
            let success = n_ok > 0 && n_ok >= must.min(n_fit);
            if let Some(tr) = &mut trace {
                tr.push(format!("t {:>6.2} s {} fit {:>2} ok {:>2} snr {:>5.1}{}", t as f64 / 1e6, short(&choice), n_fit, n_ok, snr, if success { "" } else { " FAIL" }));
            }
            if let Some(ctl) = ctl.as_mut() {
                ctl.report(&PEER, choice, success);
            }
            failures = if success { 0 } else { failures + 1 };
            if n_ok < n_fit {
                cw_exp = (cw_exp + 1).min(10);
            } else {
                cw_exp = 4;
            }
            pending = kept;
        }
    }
    if let (Some(tr), Some(ctl)) = (&mut trace, &ctl) {
        tr.push(format!("{:?}", ctl.info(&PEER)));
    }
    out.time_us = t;
    out
}

struct Scenario {
    name: String,
    channel: Channel,
    traffic: Traffic,
    link: Link,
}

fn scenarios() -> Vec<Scenario> {
    let small = Traffic { body_len: 118, per_batch: 1 };
    let burst = Traffic { body_len: 118, per_batch: 16 };
    let big = Traffic { body_len: 1500, per_batch: 16 };
    let mut v = Vec::new();
    for (tn, tr) in [("1x118B", small), ("16x118B", burst), ("16x1500B", big)] {
        let mut add = |what: &str, channel: Channel, link: Link| v.push(Scenario { name: format!("{what}, {tn}"), channel, traffic: tr, link });
        for snr in [8.0f32, 12.0, 16.0, 20.0, 25.0, 30.0] {
            add(&format!("static {snr} dB"), Channel::Static(snr), link(150));
        }
        add("shadow 18±4 dB τ3s", Channel::Shadow { mean: 18.0, sigma: 4.0, tau_s: 3.0 }, link(150));
        add("rician K3 20 dB 5 Hz", Channel::Fading { mean: 20.0, doppler_hz: 5.0, k: 3.0 }, link(150));
        add("rayleigh 24 dB 1 Hz", Channel::Fading { mean: 24.0, doppler_hz: 1.0, k: 0.0 }, link(150));
        add("step 24→10→24 dB", Channel::Step { hi: 24.0, lo: 10.0, from_s: 20.0, to_s: 40.0 }, link(150));
        add("static 24 dB, 3 % loss", Channel::Static(24.0), Link { loss_floor: 0.03, ..link(150) });
        add("static 20 dB, hint +3 dB", Channel::Static(20.0), Link { asym_db: 3.0, ..link(150) });
        add("shadow 18±4 dB, 65 ms timeout", Channel::Shadow { mean: 18.0, sigma: 4.0, tau_s: 3.0 }, link(65));
        add("rician K3 20 dB, 65 ms timeout", Channel::Fading { mean: 20.0, doppler_hz: 5.0, k: 3.0 }, link(65));
        add("static 20 dB, spread 0.8 µs", Channel::Static(20.0), Link { spread_us: 0.8, ..link(150) });
        add("static 20 dB, spread 1.1 µs", Channel::Static(20.0), Link { spread_us: 1.1, ..link(150) });
        add("static 25 dB, BCC-only peer", Channel::Static(25.0), Link { peer_ldpc: false, ..link(150) });
        add("shadow 18±4 dB, BCC-only peer", Channel::Shadow { mean: 18.0, sigma: 4.0, tau_s: 3.0 }, Link { peer_ldpc: false, ..link(150) });
    }
    v
}

const DURATION_US: u64 = 90_000_000;

/// Goodput of the best fixed MCS, guard interval and coding in a scenario
/// (and which).
fn best_fixed(s: &Scenario, seed: u64) -> (f32, TxChoice) {
    let mut best: Option<(f32, TxChoice)> = None;
    for &mcs in LADDER.iter().filter(|&&m| m <= 8) {
        for gi in GIS {
            for fec_coding in CODINGS {
                let choice = TxChoice { mcs, gi, fec_coding };
                let goodput = run(&Policy::Fixed(choice), &s.link, &s.traffic, &s.channel, DURATION_US, seed, None).goodput_kbps();
                if best.is_none_or(|(g, _)| goodput > g) {
                    best = Some((goodput, choice));
                }
            }
        }
    }
    best.expect("ladder")
}

struct Score {
    /// Per scenario: (goodput / best fixed goodput, timeout share, drops).
    per_scenario: Vec<(f32, f32, u32)>,
}

impl Score {
    fn mean(&self) -> f32 {
        self.per_scenario.iter().map(|s| s.0).sum::<f32>() / self.per_scenario.len() as f32
    }
    fn min(&self) -> f32 {
        self.per_scenario.iter().map(|s| s.0).fold(f32::INFINITY, f32::min)
    }
    fn timeout_share(&self) -> f32 {
        self.per_scenario.iter().map(|s| s.1).sum::<f32>() / self.per_scenario.len() as f32
    }
}

fn evaluate(cfg: &RateConfig, scenarios: &[Scenario], references: &[(f32, TxChoice)], seed: u64) -> Score {
    let per_scenario = scenarios
        .iter()
        .zip(references)
        .map(|(s, r)| {
            // As the engine does: a lost attempt costs the response timeout.
            let policy = Policy::Adaptive(RateConfig { enabled: true, fail_cost_us: s.link.timeout_us, ..cfg.clone() });
            let o = run(&policy, &s.link, &s.traffic, &s.channel, DURATION_US, seed, None);
            (o.goodput_kbps() / r.0.max(1e-3), o.timeout_share(), o.drops)
        })
        .collect();
    Score { per_scenario }
}

fn describe(cfg: &RateConfig) -> String {
    format!(
        "p_min {:.2} alpha {:.2} interval {:>2} cap {:>4} margin {:.0} rearm {:.0} sgi<{:.2}µs",
        cfg.p_min, cfg.alpha, cfg.probe_interval, cfg.probe_backoff_max, cfg.snr_margin_db, cfg.probe_rearm_snr_db, cfg.sgi_max_delay_spread_us
    )
}

fn short(c: &TxChoice) -> String {
    format!("{:>2}{}{}", c.mcs, if c.gi == GuardInterval::Short { "S" } else { "L" }, if c.fec_coding == Coding::Ldpc { "ldpc" } else { "bcc " })
}

/// The PER model reproduces the loopback: 10 % loss of 1000-octet PSDUs
/// within a dB of the controller's table, and longer PSDUs need more.
#[test]
fn per_model_matches_the_controllers_table() {
    for (i, &m) in LADDER.iter().enumerate() {
        let need = s2g_mac::rate::snr_required_db(m);
        let at_table = per(i, need, 1000);
        assert!((0.03..=0.25).contains(&at_table), "MCS {m}: PER {at_table:.3} at the table's {need} dB");
        assert!(per(i, need, 100) < at_table);
        assert!(per(i, need, 12_000) > at_table);
        assert!(per(i, need + 3.0, 1000) < 0.02, "MCS {m}: PER {:.3} 3 dB above the table", per(i, need + 3.0, 1000));
    }
}

/// The defaults stay within reach of the best fixed rate everywhere, and
/// spend little time in timeouts.
#[test]
fn defaults_hold_up_across_the_scenarios() {
    let scenarios = scenarios();
    let refs: Vec<(f32, TxChoice)> = scenarios.iter().map(|s| best_fixed(s, 1)).collect();
    let cfg = RateConfig::default();
    let score = evaluate(&cfg, &scenarios, &refs, 1);
    println!("{}", describe(&cfg));
    println!("{:<44} {:>8} {:>8} {:>7} {:>5}", "scenario", "best", "adaptive", "timeout", "drops");
    for ((s, r), (eff, to, drops)) in scenarios.iter().zip(&refs).zip(&score.per_scenario) {
        println!("{:<44} {} {:>7.0} % {:>6.1} % {:>5}", s.name, short(&r.1), eff * 100.0, to * 100.0, drops);
    }
    println!("mean {:.3}, min {:.3}, timeout share {:.3}", score.mean(), score.min(), score.timeout_share());
    assert!(score.mean() >= 0.93, "mean efficiency {:.3}", score.mean());
    assert!(score.min() >= 0.75, "worst scenario {:.3}", score.min());
}

/// Attempt-by-attempt trace of the defaults in the scenarios whose names
/// start with the `S2G_TRACE` environment variable (default: the 8 dB
/// batches); run with --ignored --nocapture.
#[test]
#[ignore]
fn trace() {
    let prefix = std::env::var("S2G_TRACE").unwrap_or_else(|_| "static 8 dB, 16x".into());
    for s in scenarios().iter().filter(|s| s.name.starts_with(&prefix)) {
        let policy = Policy::Adaptive(RateConfig { enabled: true, fail_cost_us: s.link.timeout_us, ..Default::default() });
        let mut lines = Vec::new();
        let o = run(&policy, &s.link, &s.traffic, &s.channel, 20_000_000, 1, Some(&mut lines));
        println!("\n{}: {:.0} kbit/s, {} attempts, {} timeouts", s.name, o.goodput_kbps(), o.attempts, o.timeouts);
        let n = lines.len();
        for (i, l) in lines.iter().enumerate() {
            if i < 50 || i + 40 >= n {
                println!("{l}");
            } else if i == 50 {
                println!("...");
            }
        }
    }
}

/// Parameter sweep behind the defaults. Prints the best configurations by
/// worst-case and by mean efficiency; run in release with --nocapture.
#[test]
#[ignore]
fn sweep() {
    let scenarios = scenarios();
    let refs: Vec<(f32, TxChoice)> = scenarios.iter().map(|s| best_fixed(s, 1)).collect();
    let mut rows: Vec<(f32, f32, f32, RateConfig)> = Vec::new();
    for &p_min in &[0.7f32, 0.8, 0.85, 0.9, 0.95] {
        for &alpha in &[0.1f32, 0.2, 0.3] {
            for &probe_interval in &[8u32, 16, 32] {
                for &probe_backoff_max in &[512u32, 2048, 8192] {
                    for &snr_margin_db in &[0.0f32, 1.0, 2.0, 3.0, 4.0] {
                        let cfg = RateConfig { p_min, alpha, probe_interval, probe_backoff_max, snr_margin_db, ..Default::default() };
                        let s = evaluate(&cfg, &scenarios, &refs, 1);
                        rows.push((s.mean(), s.min(), s.timeout_share(), cfg));
                    }
                }
            }
        }
    }
    let show = |title: &str, rows: &[(f32, f32, f32, RateConfig)]| {
        println!("\n{title}");
        for (mean, min, to, cfg) in rows.iter().take(12) {
            println!("mean {:.3} min {:.3} timeouts {:.3}  {}", mean, min, to, describe(cfg));
        }
    };
    rows.sort_by(|a, b| b.0.total_cmp(&a.0));
    show("by mean efficiency", &rows);
    rows.sort_by(|a, b| b.1.total_cmp(&a.1));
    show("by worst scenario", &rows);
    rows.sort_by(|a, b| (b.0 + b.1).total_cmp(&(a.0 + a.1)));
    show("by mean + worst", &rows);
    let d = RateConfig::default();
    let s = evaluate(&d, &scenarios, &refs, 1);
    println!("\ndefaults: mean {:.3} min {:.3} timeouts {:.3}  {}", s.mean(), s.min(), s.timeout_share(), describe(&d));
    let mcs_only = RateConfig { gi: Adapt::Fixed(GuardInterval::Long), fec_coding: Adapt::Fixed(Coding::Bcc), ..Default::default() };
    let s = evaluate(&mcs_only, &scenarios, &refs, 1);
    println!("MCS only (long GI, BCC): mean {:.3} min {:.3} timeouts {:.3}", s.mean(), s.min(), s.timeout_share());
}
