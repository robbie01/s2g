//! Per-peer adaptation of unicast data frames: MCS, guard interval and
//! FEC coding ("rate control").
//!
//! 802.11 leaves rate adaptation to the implementation. This is a small
//! Minstrel-style controller for a link on which a failed attempt costs a
//! long response timeout rather than a SIFS, so reliability is weighted
//! over raw throughput:
//!
//! * per peer, per MCS and per guard interval, an exponentially weighted
//!   success probability of transmission attempts (an attempt succeeds
//!   when it delivered, per unit of airtime plus response turnaround, at
//!   least what one PPDU at the next-lower rate would have: a lone frame
//!   must get through, a big A-MPDU may lose an MPDU or two and still beat
//!   the rate below it);
//! * the MCS in use is the highest whose probability is still above a
//!   floor: `p_min`, or higher when one failure (a response timeout)
//!   costs more airtime than the rate saves over the rate below for the
//!   batches at hand, so single small frames only ride rates that are
//!   flawless while a big A-MPDU may lose an MPDU or two; when the
//!   probability drops below the floor the controller falls back to the
//!   best rate below it;
//! * every few frames one attempt *probes* the next-higher MCS; a
//!   successful probe promotes it at once (ARF style), a failed probe
//!   doubles the wait before the next probe, up to a cap; a clear rise in
//!   the SNR heard from the peer re-arms probing at once;
//! * the short guard interval (10 % less Data airtime, half the
//!   delay-spread tolerance) is probed the same way at the MCS in use and
//!   stays in use while its probability holds the floor;
//! * LDPC (1.5–2 dB at every MCS above 0, `s2g-sim --ldpc`) needs a peer
//!   that decodes it, and OCB gives no way to ask; a probe finds out: an
//!   acknowledged LDPC frame switches the peer to LDPC, a lost one backs
//!   off like a failed MCS probe;
//! * retries walk down the ladder from the attempt that failed: a
//!   short-GI attempt first goes long-GI at the same MCS, then one MCS
//!   per retry (a failed probe retries from the rate it probed from, a
//!   failed LDPC probe retries with BCC);
//! * the SNR the PHY reports for frames (or NDP Acks) *received from* the
//!   peer bounds where MCS probes go, through a table of what this PHY
//!   needs per MCS measured in the receiver's own units (less LDPC's gain
//!   when LDPC is in use). The reverse link is not the forward link, but
//!   on a symmetric OCB link it is the best hint available before any
//!   acknowledgement statistics exist, and it stops hopeless probes. A
//!   rate that has been flawless for a while may still probe above the
//!   bound, rarely, and only when the airtime the next rate would save
//!   over the coming frames outweighs the cost of one failure, so big
//!   A-MPDUs find a pessimistic hint's ceiling, and single small frames
//!   (which gain nothing worth a timeout) do not; and a rate that works
//!   below what the table demands shows the table is pessimistic for the
//!   link, so probes proceed at their normal cadence;
//! * the RMS delay spread the PHY reports for receptions from the peer
//!   bounds short-GI probes the same way: this PHY loses short-GI PPDUs
//!   once its reading passes about 0.9 µs (`s2g-sim --sgi --echo-delay`
//!   against `--report-snr`), long-GI ones past about 1.3 µs; a flat
//!   channel reads 0.4–0.6 µs.
//!
//! The constants were tuned in `tests/rate_sim.rs`, a link-level
//! simulation with the PHY's measured PER-vs-SNR curves, the engine's
//! batch/retry rules, a response turnaround of tens of ms and a lost
//! attempt costing the response timeout, over static, shadowed, fading and
//! stepped channels with and without delay spread and LDPC-capable peers.
//!
//! Everything is per destination address, so a mesh node talks to a close
//! neighbor at 64-QAM while keeping BPSK for a distant one.

use crate::frame::MacAddr;
use s2g_phy::params;
use s2g_phy::vector::{Coding, GuardInterval};
use std::collections::HashMap;

/// MCS indices valid at 2 MHz / 1 SS, in increasing data-rate order.
pub const LADDER: [u8; 10] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 11];

/// SNR each MCS needs, in the units the receiver reports (its LTF-based
/// estimate, which reads 1–3 dB above the true SNR): the point where this
/// PHY delivers 90 % of 1000-octet PSDUs in an AWGN loopback with 10 kHz
/// CFO (`s2g-sim --report-snr`, BCC). Indexed like [`LADDER`].
const SNR_REQUIRED_DB: [f32; 10] = [8.5, 10.0, 10.5, 11.5, 15.5, 19.0, 20.5, 22.0, 26.0, 31.0];

/// LDPC's gain over BCC at every MCS above 0, in the same units
/// (`s2g-sim --ldpc --report-snr`).
const LDPC_GAIN_DB: f32 = 1.8;

/// A probe above what the SNR hint allows waits this many probe intervals
/// (plus the back-off), needs the rate in use to be at [`P_FLAWLESS`], and
/// must promise to save more airtime over that many intervals than one
/// failure costs.
const OVERRIDE_INTERVALS: u32 = 4;
const P_FLAWLESS: f32 = 0.98;

/// How a per-frame parameter is chosen.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Adapt<T> {
    /// Probed and adapted per peer.
    Auto,
    Fixed(T),
}

/// Controller settings.
#[derive(Debug, Clone, PartialEq)]
pub struct RateConfig {
    /// Adapt unicast frames per peer (false: every unicast frame uses
    /// `MacConfig::mcs`, `gi` and `fec_coding`).
    pub enabled: bool,
    /// Rate for a new peer nothing has been heard from yet.
    pub start_mcs: u8,
    /// Lowest MCS the controller falls back to.
    pub min_mcs: u8,
    /// Highest MCS the controller probes.
    pub max_mcs: u8,
    /// Guard interval: probed per peer, or fixed.
    pub gi: Adapt<GuardInterval>,
    /// FEC coding: LDPC probed per peer, or fixed.
    pub fec_coding: Adapt<Coding>,
    /// Frames between probes.
    pub probe_interval: u32,
    /// Longest wait between probes after repeated failures, frames.
    pub probe_backoff_max: u32,
    /// A rise of this much (dB) in the smoothed SNR heard from the peer
    /// since a probe failed re-arms MCS probing at once.
    pub probe_rearm_snr_db: f32,
    /// Weight of a new success/failure sample in the per-rate average.
    pub alpha: f32,
    /// A rate stays in use while its success probability is at least this,
    /// or more when a failure costs more than the rate saves (see
    /// [`RateConfig::fail_cost_us`]).
    pub p_min: f32,
    /// Extra SNR (dB) demanded over an MCS's requirement before probing it.
    pub snr_margin_db: f32,
    /// Short-GI probes need the RMS delay spread heard from the peer (µs)
    /// at or below this; nothing heard counts as allowed.
    pub sgi_max_delay_spread_us: f32,
    /// What a lost attempt costs, µs (the MAC sets its response timeout
    /// here): decides how reliable a rate has to be for the airtime it
    /// saves, and when probing above the SNR hint is worth the risk.
    pub fail_cost_us: u64,
}

impl Default for RateConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            start_mcs: 0,
            min_mcs: 0,
            max_mcs: 8,
            gi: Adapt::Auto,
            fec_coding: Adapt::Auto,
            probe_interval: 8,
            probe_backoff_max: 2048,
            probe_rearm_snr_db: 2.0,
            alpha: 0.1,
            p_min: 0.7,
            snr_margin_db: 3.0,
            sgi_max_delay_spread_us: 0.9,
            fail_cost_us: 150_000,
        }
    }
}

/// What one attempt goes out with.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TxChoice {
    pub mcs: u8,
    pub gi: GuardInterval,
    pub fec_coding: Coding,
}

#[derive(Debug, Clone, Copy, Default)]
struct Stat {
    /// Smoothed success probability; `None` until the first attempt.
    p: Option<f32>,
    attempts: u32,
    successes: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Probe {
    /// The next-higher MCS; `above_hint` when the SNR hint did not allow it.
    Mcs { above_hint: bool },
    ShortGi,
    Ldpc,
}

/// The batch's first failed attempt: retries count down from it.
#[derive(Debug, Clone, Copy)]
struct Failed {
    idx: usize,
    /// It used the short GI while the GI is adapted: the first retry only
    /// drops to the long GI.
    gi_step: bool,
    ldpc_probe: bool,
}

#[derive(Debug, Clone)]
struct Peer {
    /// Per ladder index and guard interval (0 long, 1 short).
    stats: [[Stat; 2]; LADDER.len()],
    /// Index into `LADDER` of the MCS in use.
    cur: usize,
    short_gi: bool,
    ldpc: bool,
    since_probe: u32,
    /// Extra frames to wait before the next probe of each kind (doubles
    /// per failed probe).
    mcs_backoff: u32,
    gi_backoff: u32,
    fec_backoff: u32,
    /// Ladder index and kind of an in-flight probe attempt.
    probing: Option<(usize, Probe)>,
    /// Kind of the most recent probe.
    last_probe: Option<Probe>,
    failed: Option<Failed>,
    /// Octets the latest batch had to carry (sets the reliability floors).
    octets: usize,
    /// Smoothed SNR at which the last MCS probe failed (re-arm reference).
    probe_fail_snr_db: Option<f32>,
    /// Smoothed SNR of receptions from this peer, dB.
    snr_db: Option<f32>,
    /// Smoothed carrier frequency offset of receptions from this peer, Hz
    /// (the peer's oscillator relative to this station's, as the PHY sees
    /// it).
    cfo_hz: Option<f32>,
    /// Smoothed RMS delay spread of receptions from this peer, µs.
    delay_spread_us: Option<f32>,
}

/// Per-peer statistics snapshot.
#[derive(Debug, Clone, PartialEq)]
pub struct PeerRateInfo {
    pub rate: TxChoice,
    pub snr_db: Option<f32>,
    pub cfo_hz: Option<f32>,
    pub delay_spread_us: Option<f32>,
    /// (MCS, guard interval, attempts, successes, smoothed success
    /// probability) of every combination tried.
    pub per_rate: Vec<(u8, GuardInterval, u32, u32, Option<f32>)>,
}

/// Rate controller for all peers of one station.
#[derive(Debug, Clone)]
pub struct RateControl {
    cfg: RateConfig,
    lo: usize,
    hi: usize,
    peers: HashMap<MacAddr, Peer>,
}

fn ladder_index(mcs: u8) -> Option<usize> {
    LADDER.iter().position(|&m| m == mcs)
}

/// Reported SNR at which this PHY delivers 90 % of 1000-octet PSDUs at
/// `mcs` (infinite for an MCS the ladder does not carry).
pub fn snr_required_db(mcs: u8) -> f32 {
    ladder_index(mcs).map_or(f32::INFINITY, |i| SNR_REQUIRED_DB[i])
}

/// Reported SNR the table demands for `LADDER[idx]` with `ldpc` in use.
fn snr_required(idx: usize, ldpc: bool) -> f32 {
    SNR_REQUIRED_DB[idx] - if ldpc && idx > 0 { LDPC_GAIN_DB } else { 0.0 }
}

fn snr_allows(margin_db: f32, snr_db: Option<f32>, idx: usize, ldpc: bool) -> bool {
    match snr_db {
        Some(s) => snr_required(idx, ldpc) + margin_db <= s,
        None => true,
    }
}

/// Long-GI Data airtime per octet at `LADDER[idx]`, µs.
fn airtime_per_octet_us(idx: usize) -> f64 {
    320.0 / params::n_dbps_2mhz(LADDER[idx], 1).unwrap_or(26) as f64
}

/// Airtime saved by carrying `octets` at `LADDER[to]` instead of
/// `LADDER[from]`, µs (long-GI Data symbols only; the preamble is the
/// same).
fn airtime_saving_us(from: usize, to: usize, octets: usize) -> u64 {
    ((airtime_per_octet_us(from) - airtime_per_octet_us(to)).max(0.0) * octets as f64) as u64
}

/// Success probability a rate must hold to stay in use when the rate
/// below it succeeds with `below` and it saves `saving_us` per batch:
/// `p_min`, or more when the failures it may add cost more than that
/// (a rate-independent loss counts against neither).
fn floor(p_min: f32, fail_cost_us: u64, below: f32, saving_us: f64) -> f32 {
    (below - saving_us as f32 / fail_cost_us.max(1) as f32).clamp(p_min, P_FLAWLESS)
}

impl Peer {
    /// Floor for `LADDER[idx]` with guard interval `g` against the MCS
    /// below it, for the latest batch.
    fn floor_mcs(&self, p_min: f32, fail_cost_us: u64, idx: usize, g: usize) -> f32 {
        if idx == 0 {
            return p_min;
        }
        let below = self.stats[idx - 1][g].p.unwrap_or(1.0);
        floor(p_min, fail_cost_us, below, airtime_saving_us(idx - 1, idx, self.octets) as f64)
    }

    /// Floor for the short GI at `LADDER[idx]`, which saves a tenth of the
    /// Data airtime over the long GI there.
    fn floor_gi(&self, p_min: f32, fail_cost_us: u64, idx: usize) -> f32 {
        let long = self.stats[idx][0].p.unwrap_or(1.0);
        floor(p_min, fail_cost_us, long, 0.1 * airtime_per_octet_us(idx) * self.octets as f64)
    }
}

/// The ladder entry below `mcs` (`mcs` itself at the bottom or off the
/// ladder): the rate a long-GI attempt at `mcs` has to beat. A short-GI
/// attempt has to beat the long GI at its own MCS, which carries the same
/// MPDUs.
pub fn next_lower(mcs: u8) -> u8 {
    ladder_index(mcs).map_or(mcs, |i| LADDER[i.saturating_sub(1)])
}

fn gi_index(gi: GuardInterval) -> usize {
    (gi == GuardInterval::Short) as usize
}

impl RateControl {
    pub fn new(cfg: RateConfig) -> Self {
        let lo = ladder_index(cfg.min_mcs).unwrap_or(0);
        let hi = ladder_index(cfg.max_mcs).unwrap_or(LADDER.len() - 1).max(lo);
        Self { cfg, lo, hi, peers: HashMap::new() }
    }

    pub fn config(&self) -> &RateConfig {
        &self.cfg
    }

    /// Update what a lost attempt costs (the MAC's current response
    /// timeout), µs.
    pub fn set_fail_cost_us(&mut self, us: u64) {
        self.cfg.fail_cost_us = us;
    }

    /// Starting rate for a peer: the configured start, or with an SNR hint
    /// the highest allowed MCS.
    fn start_index(&self, snr_db: Option<f32>) -> usize {
        match snr_db {
            Some(_) => (self.lo..=self.hi).filter(|&i| snr_allows(self.cfg.snr_margin_db, snr_db, i, false)).max().unwrap_or(self.lo),
            None => ladder_index(self.cfg.start_mcs).unwrap_or(self.lo).clamp(self.lo, self.hi),
        }
    }

    fn peer_mut(&mut self, addr: &MacAddr) -> &mut Peer {
        if !self.peers.contains_key(addr) {
            let cur = self.start_index(None);
            self.peers.insert(
                *addr,
                Peer {
                    stats: [[Stat::default(); 2]; LADDER.len()],
                    cur,
                    short_gi: false,
                    ldpc: false,
                    since_probe: 0,
                    mcs_backoff: 0,
                    gi_backoff: 0,
                    fec_backoff: 0,
                    probing: None,
                    last_probe: None,
                    failed: None,
                    octets: 0,
                    probe_fail_snr_db: None,
                    snr_db: None,
                    cfo_hz: None,
                    delay_spread_us: None,
                },
            );
        }
        self.peers.get_mut(addr).expect("inserted")
    }

    /// Guard interval and coding a peer's fresh attempts go out with.
    fn in_use(&self, p: &Peer) -> (GuardInterval, Coding) {
        let gi = match self.cfg.gi {
            Adapt::Fixed(g) => g,
            Adapt::Auto if p.short_gi => GuardInterval::Short,
            Adapt::Auto => GuardInterval::Long,
        };
        let fec = match self.cfg.fec_coding {
            Adapt::Fixed(c) => c,
            Adapt::Auto if p.ldpc => Coding::Ldpc,
            Adapt::Auto => Coding::Bcc,
        };
        (gi, fec)
    }

    /// The PHY reported `snr_db` for something received from `addr`.
    pub fn observe_snr(&mut self, addr: &MacAddr, snr_db: f32) {
        if !snr_db.is_finite() {
            return;
        }
        let rearm = self.cfg.probe_rearm_snr_db;
        let p = self.peer_mut(addr);
        p.snr_db = Some(match p.snr_db {
            Some(s) => s + 0.3 * (snr_db - s),
            None => snr_db,
        });
        if p.probe_fail_snr_db.is_some_and(|f| p.snr_db.is_some_and(|s| s >= f + rearm)) {
            // The channel has clearly improved since the last failed probe.
            p.mcs_backoff = 0;
            p.probe_fail_snr_db = None;
        }
        if p.stats.iter().flatten().all(|s| s.attempts == 0) {
            // Nothing sent yet: let the hint pick the opening rate.
            let snr = p.snr_db;
            let cur = self.start_index(snr);
            self.peer_mut(addr).cur = cur;
        }
    }

    /// The PHY measured `cfo_hz` on something received from `addr`.
    pub fn observe_cfo(&mut self, addr: &MacAddr, cfo_hz: f32) {
        if !cfo_hz.is_finite() {
            return;
        }
        let p = self.peer_mut(addr);
        p.cfo_hz = Some(match p.cfo_hz {
            Some(c) => c + 0.3 * (cfo_hz - c),
            None => cfo_hz,
        });
    }

    /// The PHY measured an RMS delay spread of `us` on something received
    /// from `addr`.
    pub fn observe_delay_spread(&mut self, addr: &MacAddr, us: f32) {
        if !us.is_finite() {
            return;
        }
        let p = self.peer_mut(addr);
        p.delay_spread_us = Some(match p.delay_spread_us {
            Some(d) => d + 0.3 * (us - d),
            None => us,
        });
    }

    /// Smoothed carrier offset of `addr`, Hz, if anything was heard.
    pub fn peer_cfo_hz(&self, addr: &MacAddr) -> Option<f32> {
        self.peers.get(addr).and_then(|p| p.cfo_hz)
    }

    /// Choice for the next transmission attempt to `addr`: `retry` is 0
    /// for a fresh attempt, else the number of consecutive attempts of the
    /// batch that fell short of their rate; `octets` is what the batch
    /// still has to carry (0 if unknown). Call [`RateControl::report`]
    /// with the outcome.
    pub fn select(&mut self, addr: &MacAddr, retry: u32, octets: usize) -> TxChoice {
        let (lo, hi, interval, margin) = (self.lo, self.hi, self.cfg.probe_interval, self.cfg.snr_margin_db);
        let (gi_cfg, fec_cfg, sgi_max_us, fail_cost) = (self.cfg.gi, self.cfg.fec_coding, self.cfg.sgi_max_delay_spread_us, self.cfg.fail_cost_us);
        let p = self.peer_mut(addr);
        let (gi_in_use, fec_in_use) = {
            let gi = match gi_cfg {
                Adapt::Fixed(g) => g,
                Adapt::Auto if p.short_gi => GuardInterval::Short,
                Adapt::Auto => GuardInterval::Long,
            };
            let fec = match fec_cfg {
                Adapt::Fixed(c) => c,
                Adapt::Auto if p.ldpc => Coding::Ldpc,
                Adapt::Auto => Coding::Bcc,
            };
            (gi, fec)
        };
        p.probing = None;
        p.octets = octets;
        if retry > 0 {
            // Down the ladder from the attempt that failed: the long GI
            // first, then one MCS per retry.
            let f = p.failed.unwrap_or(Failed { idx: p.cur, gi_step: gi_in_use == GuardInterval::Short && gi_cfg == Adapt::Auto, ldpc_probe: false });
            let idx = f.idx.saturating_sub((retry as usize).saturating_sub(f.gi_step as usize)).max(lo);
            let gi = match gi_cfg {
                Adapt::Fixed(g) => g,
                Adapt::Auto => GuardInterval::Long,
            };
            let fec = if f.ldpc_probe { Coding::Bcc } else { fec_in_use };
            return TxChoice { mcs: LADDER[idx], gi, fec_coding: fec };
        }
        p.failed = None;
        p.since_probe += 1;
        let wait = p.since_probe;
        let cand = p.cur + 1;
        if cand <= hi {
            // A rate in use that works below what the table demands (with
            // no margin at all) shows the table is pessimistic for this
            // link: the bound then does not apply.
            let ldpc = fec_in_use == Coding::Ldpc;
            let pessimistic = p.snr_db.is_some_and(|s| s < snr_required(p.cur, ldpc));
            let allowed = pessimistic || snr_allows(margin, p.snr_db, cand, ldpc);
            let due = if allowed {
                wait >= interval + p.mcs_backoff
            } else {
                // The hint is only the reverse link through one PHY's
                // average table. A rate that has been flawless for a while
                // gets a rare probe above the bound when the airtime the
                // next rate would save over the coming frames outweighs
                // one failure.
                let saving = airtime_saving_us(p.cur, cand, octets) * (OVERRIDE_INTERVALS * interval) as u64;
                let flawless = p.stats[p.cur][gi_index(gi_in_use)].p.is_some_and(|v| v >= P_FLAWLESS);
                wait >= OVERRIDE_INTERVALS * interval + p.mcs_backoff && flawless && saving >= fail_cost
            };
            if due {
                p.since_probe = 0;
                p.probing = Some((cand, Probe::Mcs { above_hint: !allowed }));
                p.last_probe = Some(Probe::Mcs { above_hint: !allowed });
                return TxChoice { mcs: LADDER[cand], gi: gi_in_use, fec_coding: fec_in_use };
            }
        }
        if fec_cfg == Adapt::Auto && !p.ldpc && wait >= interval + p.fec_backoff {
            p.since_probe = 0;
            p.probing = Some((p.cur, Probe::Ldpc));
            p.last_probe = Some(Probe::Ldpc);
            return TxChoice { mcs: LADDER[p.cur], gi: gi_in_use, fec_coding: Coding::Ldpc };
        }
        let clean = p.delay_spread_us.is_none_or(|d| d <= sgi_max_us);
        if gi_cfg == Adapt::Auto && !p.short_gi && clean && wait >= interval + p.gi_backoff {
            p.since_probe = 0;
            p.probing = Some((p.cur, Probe::ShortGi));
            p.last_probe = Some(Probe::ShortGi);
            return TxChoice { mcs: LADDER[p.cur], gi: GuardInterval::Short, fec_coding: fec_in_use };
        }
        TxChoice { mcs: LADDER[p.cur], gi: gi_in_use, fec_coding: fec_in_use }
    }

    /// Outcome of an attempt to `addr` with `choice`. The caller judges
    /// `success` by what the attempt delivered against what one PPDU at
    /// the next-lower rate would have carried of the same batch, scaled by
    /// the ratio of airtime plus response turnaround: a lone frame must
    /// get through, while a big A-MPDU may lose an MPDU or two and still
    /// beat the rate below it (see `engine::Mac::resolve_attempt`).
    pub fn report(&mut self, addr: &MacAddr, choice: TxChoice, success: bool) {
        let Some(idx) = ladder_index(choice.mcs) else { return };
        let g = gi_index(choice.gi);
        let (lo, hi, alpha, p_min) = (self.lo, self.hi, self.cfg.alpha, self.cfg.p_min);
        let (interval, backoff_max, fail_cost) = (self.cfg.probe_interval, self.cfg.probe_backoff_max, self.cfg.fail_cost_us);
        let gi_cfg = self.cfg.gi;
        let p = self.peer_mut(addr);
        let st = &mut p.stats[idx][g];
        st.attempts += 1;
        st.successes += success as u32;
        let sample = if success { 1.0 } else { 0.0 };
        st.p = Some(match st.p {
            Some(v) => v + alpha * (sample - v),
            None => sample,
        });
        let probe = p.probing.filter(|&(i, kind)| {
            i == idx
                && match kind {
                    Probe::Mcs { .. } => true,
                    Probe::ShortGi => g == 1,
                    Probe::Ldpc => choice.fec_coding == Coding::Ldpc,
                }
        });
        if success {
            p.failed = None;
        } else if p.failed.is_none() {
            p.failed = Some(Failed { idx, gi_step: g == 1 && gi_cfg == Adapt::Auto, ldpc_probe: matches!(probe, Some((_, Probe::Ldpc))) });
        }
        let backed_off = |b: u32| (b.max(interval) * 2).min(backoff_max);
        if let Some((probed, kind)) = probe {
            p.probing = None;
            match (kind, success) {
                (Probe::Mcs { .. }, true) => {
                    // A successful probe promotes at once, starting the
                    // rate at its floor: one failure then demotes again
                    // (ARF behavior).
                    let floor = p.floor_mcs(p_min, fail_cost, probed, g);
                    p.stats[probed][g].p = Some(p.stats[probed][g].p.unwrap_or(0.0).max(floor));
                    p.mcs_backoff = 0;
                    p.probe_fail_snr_db = None;
                }
                (Probe::Mcs { above_hint }, false) => {
                    // A failed probe above the hint confirms the hint: wait
                    // much longer before doubting it again.
                    let base = if above_hint { OVERRIDE_INTERVALS * interval } else { interval };
                    p.mcs_backoff = (p.mcs_backoff.max(base) * 2).min(backoff_max);
                    p.probe_fail_snr_db = p.snr_db;
                }
                (Probe::ShortGi, true) => {
                    p.short_gi = true;
                    p.gi_backoff = 0;
                    let floor = p.floor_gi(p_min, fail_cost, probed);
                    p.stats[probed][1].p = Some(p.stats[probed][1].p.unwrap_or(0.0).max(floor));
                }
                (Probe::ShortGi, false) => p.gi_backoff = backed_off(p.gi_backoff),
                (Probe::Ldpc, true) => {
                    p.ldpc = true;
                    p.fec_backoff = 0;
                }
                (Probe::Ldpc, false) => p.fec_backoff = backed_off(p.fec_backoff),
            }
        }
        if gi_cfg == Adapt::Auto && p.short_gi && p.stats[p.cur][1].p.is_some_and(|v| v + 1e-4 < p.floor_gi(p_min, fail_cost, p.cur)) {
            // The short GI in use is failing: back to the long GI at the
            // same MCS, and wait before trying it again.
            p.short_gi = false;
            p.gi_backoff = backed_off(p.gi_backoff);
        }
        let gi_used = match gi_cfg {
            Adapt::Fixed(gi) => gi_index(gi),
            Adapt::Auto => p.short_gi as usize,
        };
        let good = |p: &Peer, i: usize| p.stats[i][gi_used].p.is_some_and(|v| v + 1e-4 >= p.floor_mcs(p_min, fail_cost, i, gi_used));
        let known_good = (lo..=hi).filter(|&i| good(p, i)).max();
        if p.stats[p.cur][gi_used].p.is_some() && !good(p, p.cur) {
            // The rate in use is failing: fall back to the best known-good
            // rate below it, or one step down. A rate an MCS probe promoted
            // within the last probe interval counts as a failed probe: the
            // probe happened to get through a rate that does not hold.
            let cur = p.cur;
            p.cur = known_good.filter(|&k| k < cur).unwrap_or(cur.saturating_sub(1).max(lo));
            if p.cur < cur && p.since_probe < interval && matches!(p.last_probe, Some(Probe::Mcs { .. })) {
                p.mcs_backoff = backed_off(p.mcs_backoff);
                p.probe_fail_snr_db = p.snr_db;
            }
        } else if let Some(k) = known_good {
            if k > p.cur {
                p.cur = k;
            }
        }
    }

    /// Rate currently in use for `addr`, if the peer is known.
    pub fn current(&self, addr: &MacAddr) -> Option<TxChoice> {
        self.peers.get(addr).map(|p| {
            let (gi, fec_coding) = self.in_use(p);
            TxChoice { mcs: LADDER[p.cur], gi, fec_coding }
        })
    }

    /// Statistics for `addr`, if known.
    pub fn info(&self, addr: &MacAddr) -> Option<PeerRateInfo> {
        let p = self.peers.get(addr)?;
        let (gi, fec_coding) = self.in_use(p);
        let per_rate = LADDER
            .iter()
            .zip(&p.stats)
            .flat_map(|(&m, s)| [(m, GuardInterval::Long, s[0]), (m, GuardInterval::Short, s[1])])
            .filter(|(_, _, s)| s.attempts > 0)
            .map(|(m, gi, s)| (m, gi, s.attempts, s.successes, s.p))
            .collect();
        Some(PeerRateInfo {
            rate: TxChoice { mcs: LADDER[p.cur], gi, fec_coding },
            snr_db: p.snr_db,
            cfo_hz: p.cfo_hz,
            delay_spread_us: p.delay_spread_us,
            per_rate,
        })
    }

    /// All known peers.
    pub fn peers(&self) -> impl Iterator<Item = &MacAddr> {
        self.peers.keys()
    }
}

/// Nominal data rate of an MCS relative to MCS 0 (N_DBPS ratio), for
/// reporting.
pub fn relative_rate(mcs: u8) -> f32 {
    let base = params::n_dbps_2mhz(0, 1).unwrap_or(26) as f32;
    params::n_dbps_2mhz(mcs, 1).map_or(0.0, |n| n as f32 / base)
}

#[cfg(test)]
mod tests {
    use super::*;
    use GuardInterval::{Long, Short};

    const P: MacAddr = [2, 0, 0, 0, 0, 1];
    const Q: MacAddr = [2, 0, 0, 0, 0, 2];
    /// Batch size used by the tests: a single small frame.
    const OCTETS: usize = 150;

    fn ctl() -> RateControl {
        RateControl::new(RateConfig { enabled: true, ..Default::default() })
    }

    /// MCS-only controller: the guard interval and coding stay fixed.
    fn mcs_ctl(cfg: RateConfig) -> RateControl {
        RateControl::new(RateConfig { enabled: true, gi: Adapt::Fixed(Long), fec_coding: Adapt::Fixed(Coding::Bcc), ..cfg })
    }

    /// Drive `n` fresh frames of `octets`; `works(choice)` decides success;
    /// retries step down until one succeeds (max 4).
    fn drive_octets(ctl: &mut RateControl, peer: &MacAddr, n: usize, octets: usize, works: impl Fn(TxChoice) -> bool) -> Vec<TxChoice> {
        let mut used = Vec::new();
        for _ in 0..n {
            let mut retry = 0;
            loop {
                let c = ctl.select(peer, retry, octets);
                used.push(c);
                let ok = works(c);
                ctl.report(peer, c, ok);
                if ok || retry >= 4 {
                    break;
                }
                retry += 1;
            }
        }
        used
    }

    fn drive(ctl: &mut RateControl, peer: &MacAddr, n: usize, works: impl Fn(TxChoice) -> bool) -> Vec<TxChoice> {
        drive_octets(ctl, peer, n, OCTETS, works)
    }

    fn mcs_used(used: &[TxChoice]) -> Vec<u8> {
        used.iter().map(|c| c.mcs).collect()
    }

    #[test]
    fn climbs_to_the_cap_on_a_perfect_link() {
        let mut c = mcs_ctl(RateConfig::default());
        let interval = c.config().probe_interval as usize;
        let used = mcs_used(&drive(&mut c, &P, 200, |_| true));
        assert_eq!(c.current(&P).map(|r| r.mcs), Some(8));
        // Probes go one step at a time: 0..=8 all appear, nothing above.
        for m in [0u8, 1, 2, 3, 4, 5, 6, 7, 8] {
            assert!(used.contains(&m), "MCS {m} never tried: {used:?}");
        }
        assert!(!used.contains(&11));
        // Reached the cap within a few probe intervals.
        let first_8 = used.iter().position(|&m| m == 8).unwrap();
        assert!(first_8 < interval * 9 + 8, "took {first_8} attempts to reach MCS 8");
    }

    #[test]
    fn settles_below_a_ceiling_and_backs_off_probing() {
        let mut c = mcs_ctl(RateConfig::default());
        let used = mcs_used(&drive(&mut c, &P, 600, |r| r.mcs <= 4));
        assert_eq!(c.current(&P).map(|r| r.mcs), Some(4), "{:?}", c.info(&P));
        // Almost every attempt in the second half goes at the ceiling;
        // failed probes above it become rare (exponential back-off).
        let tail = &used[used.len() / 2..];
        let above = tail.iter().filter(|&&m| m > 4).count();
        let at = tail.iter().filter(|&&m| m == 4).count();
        assert!(above * 20 < tail.len(), "{above} probes above the ceiling in {} attempts", tail.len());
        assert!(at * 10 > tail.len() * 8, "only {at} of {} attempts at MCS 4", tail.len());
    }

    #[test]
    fn a_failed_probe_retries_at_the_rate_it_probed_from() {
        let mut c = mcs_ctl(RateConfig::default());
        drive(&mut c, &P, 100, |r| r.mcs <= 4);
        assert_eq!(c.current(&P).map(|r| r.mcs), Some(4));
        // Force the next fresh frame to be a probe.
        let n = (c.config().probe_interval + c.config().probe_backoff_max) as usize;
        let mut choice = c.select(&P, 0, OCTETS);
        for _ in 0..n {
            if choice.mcs == 5 {
                break;
            }
            c.report(&P, choice, true);
            choice = c.select(&P, 0, OCTETS);
        }
        assert_eq!(choice.mcs, 5, "no probe within {n} frames: {:?}", c.info(&P));
        c.report(&P, choice, false);
        assert_eq!(c.select(&P, 1, OCTETS).mcs, 4, "first retry after a failed probe goes at the rate in use");
        c.report(&P, TxChoice { mcs: 4, gi: Long, fec_coding: Coding::Bcc }, false);
        assert_eq!(c.select(&P, 2, OCTETS).mcs, 3);
    }

    #[test]
    fn probing_backs_off_to_the_cap_and_rearms_on_better_snr() {
        let mut c = mcs_ctl(RateConfig { probe_backoff_max: 64, ..Default::default() });
        let interval = c.config().probe_interval as usize;
        // 20 dB admits probes of MCS 4, which the channel then refuses.
        c.observe_snr(&P, 20.0);
        drive(&mut c, &P, 1500, |r| r.mcs <= 3);
        assert_eq!(c.current(&P).map(|r| r.mcs), Some(3));
        // Probes are now at most one per (interval + cap) frames.
        let used = mcs_used(&drive(&mut c, &P, 400, |r| r.mcs <= 3));
        let probes = used.iter().filter(|&&m| m == 4).count();
        assert!(probes <= 400 / (interval + 64) + 1, "{probes} probes in 400 frames");
        // The peer suddenly sounds much better: probing resumes at once.
        for _ in 0..10 {
            c.observe_snr(&P, 30.0);
        }
        let used = mcs_used(&drive(&mut c, &P, interval + 1, |r| r.mcs <= 3));
        assert!(used.contains(&4), "no probe after the SNR rose: {used:?}");
    }

    #[test]
    fn falls_back_when_the_channel_degrades_and_recovers() {
        let mut c = mcs_ctl(RateConfig::default());
        drive(&mut c, &P, 150, |r| r.mcs <= 6);
        assert_eq!(c.current(&P).map(|r| r.mcs), Some(6));
        drive(&mut c, &P, 40, |r| r.mcs <= 2);
        assert_eq!(c.current(&P).map(|r| r.mcs), Some(2), "{:?}", c.info(&P));
        drive(&mut c, &P, 300, |r| r.mcs <= 6);
        assert_eq!(c.current(&P).map(|r| r.mcs), Some(6), "{:?}", c.info(&P));
    }

    #[test]
    fn retries_step_down_and_do_not_probe() {
        let mut c = mcs_ctl(RateConfig::default());
        drive(&mut c, &P, 100, |r| r.mcs <= 5);
        assert_eq!(c.current(&P).map(|r| r.mcs), Some(5));
        assert_eq!(c.select(&P, 1, OCTETS).mcs, 4);
        assert_eq!(c.select(&P, 2, OCTETS).mcs, 3);
        assert_eq!(c.select(&P, 9, OCTETS).mcs, 0);
    }

    #[test]
    fn peers_are_independent() {
        let mut c = mcs_ctl(RateConfig::default());
        drive(&mut c, &P, 150, |r| r.mcs <= 7);
        drive(&mut c, &Q, 150, |r| r.mcs <= 1);
        assert_eq!(c.current(&P).map(|r| r.mcs), Some(7));
        assert_eq!(c.current(&Q).map(|r| r.mcs), Some(1));
        assert_eq!(c.peers().count(), 2);
    }

    #[test]
    fn snr_hint_bounds_probing_and_picks_the_opening_rate() {
        // The table is what the PHY was measured to need, in its own units.
        assert_eq!(snr_required_db(0), 8.5);
        assert_eq!(snr_required_db(2), 10.5);
        assert_eq!(snr_required_db(8), 26.0);
        assert_eq!(snr_required_db(9), f32::INFINITY);
        let mut c = mcs_ctl(RateConfig::default());
        let margin = c.config().snr_margin_db;
        let allowed = |snr: f32| LADDER.iter().copied().filter(|&m| m <= 8 && snr_required_db(m) + margin <= snr).max().unwrap();
        c.observe_snr(&P, 14.0);
        // Opening rate: the highest MCS allowed at 14 dB.
        assert_eq!(c.current(&P).map(|r| r.mcs), Some(allowed(14.0)));
        // Small single frames never probe above the bound: the airtime
        // they would save is not worth a timeout.
        let used = mcs_used(&drive(&mut c, &P, 400, |_| true));
        assert!(used.iter().all(|&m| m <= allowed(14.0)), "probed above the SNR bound: {used:?}");
        // A better SNR later lifts the bound.
        for _ in 0..20 {
            c.observe_snr(&P, 30.0);
        }
        drive(&mut c, &P, 300, |_| true);
        assert_eq!(c.current(&P).map(|r| r.mcs), Some(8));
        // Without a hint a new peer starts at the configured start rate.
        let mut d = mcs_ctl(RateConfig { start_mcs: 3, ..Default::default() });
        assert_eq!(d.select(&Q, 0, OCTETS).mcs, 3);
    }

    #[test]
    fn big_batches_probe_above_a_pessimistic_hint_and_back_off_hard_when_wrong() {
        let mut c = mcs_ctl(RateConfig::default());
        let interval = c.config().probe_interval;
        let margin = c.config().snr_margin_db;
        let bound = LADDER.iter().copied().filter(|&m| m <= 8 && snr_required_db(m) + margin <= 14.0).max().unwrap();
        c.observe_snr(&P, 14.0);
        assert_eq!(c.current(&P).map(|r| r.mcs), Some(bound));
        // 16 × 1500 octets: the next rate saves tens of ms per batch.
        let used = mcs_used(&drive_octets(&mut c, &P, 600, 16 * 1530, |_| true));
        assert!(used.iter().any(|&m| m > bound), "never probed above the bound: {:?}", c.info(&P));
        assert!(c.current(&P).unwrap().mcs > bound, "{:?}", c.info(&P));
        // With the hint being right, failed probes above it are rare.
        let mut d = mcs_ctl(RateConfig::default());
        d.observe_snr(&P, 14.0);
        let used = mcs_used(&drive_octets(&mut d, &P, 2000, 16 * 1530, |r| r.mcs <= bound));
        let above = used.iter().filter(|&&m| m > bound).count();
        let first = used.iter().position(|&m| m > bound).unwrap() + 1;
        assert!(first >= (OVERRIDE_INTERVALS * interval) as usize, "probed above the hint after only {first} frames");
        assert!(above <= 5, "{above} probes above the hint in 2000 frames");
        assert_eq!(d.current(&P).map(|r| r.mcs), Some(bound));
    }

    #[test]
    fn bounds_are_respected() {
        let mut c = mcs_ctl(RateConfig { min_mcs: 2, max_mcs: 5, ..Default::default() });
        let used = mcs_used(&drive(&mut c, &P, 200, |_| true));
        assert!(used.iter().all(|&m| (2..=5).contains(&m)), "{used:?}");
        assert_eq!(c.current(&P).map(|r| r.mcs), Some(5));
        let used = mcs_used(&drive(&mut c, &P, 100, |_| false));
        assert!(used.iter().all(|&m| (2..=5).contains(&m)), "{used:?}");
        assert_eq!(c.current(&P).map(|r| r.mcs), Some(2));
    }

    #[test]
    fn short_gi_is_probed_kept_while_it_works_and_dropped_when_it_fails() {
        let mut c = ctl();
        let interval = c.config().probe_interval as usize;
        // A strong hint opens at the top MCS, so MCS probes do not come first.
        c.observe_snr(&P, 40.0);
        let used = drive(&mut c, &P, 200, |_| true);
        let first_short = used.iter().position(|r| r.gi == Short).expect("short GI never probed");
        assert!(first_short < 3 * interval, "first short-GI probe only at attempt {first_short}");
        assert_eq!(c.current(&P), Some(TxChoice { mcs: 8, gi: Short, fec_coding: Coding::Ldpc }), "{:?}", c.info(&P));
        // Delay spread appears: short-GI frames fail, long-GI ones do not.
        let used = drive(&mut c, &P, 60, |r| r.gi == Long);
        let cur = c.current(&P).unwrap();
        assert_eq!((cur.gi, cur.mcs), (Long, 8), "{:?}", c.info(&P));
        // A short-GI failure retries long-GI at the same MCS before any
        // MCS step: no retry of the failed frames went below MCS 8.
        assert!(used.iter().all(|r| r.mcs == 8), "{used:?}");
        // Short-GI probes back off: few in the next stretch.
        let used = drive(&mut c, &P, 600, |r| r.gi == Long);
        let short = used.iter().filter(|r| r.gi == Short).count();
        assert!(short <= 6, "{short} short-GI probes in 600 frames");
    }

    #[test]
    fn short_gi_probes_wait_for_a_clean_reverse_channel() {
        let mut c = ctl();
        let limit = c.config().sgi_max_delay_spread_us;
        for _ in 0..5 {
            c.observe_delay_spread(&P, limit + 1.0);
        }
        let used = drive(&mut c, &P, 300, |_| true);
        assert!(used.iter().all(|r| r.gi == Long), "probed short GI on a dispersive reverse channel");
        for _ in 0..20 {
            c.observe_delay_spread(&P, 0.3);
        }
        let used = drive(&mut c, &P, 100, |_| true);
        assert!(used.iter().any(|r| r.gi == Short), "no short-GI probe after the channel cleared");
        assert_eq!(c.current(&P).map(|r| r.gi), Some(Short));
        assert!(c.info(&P).unwrap().delay_spread_us.unwrap() < limit);
    }

    #[test]
    fn a_short_gi_failure_retries_long_at_the_same_mcs_then_steps_down() {
        let mut c = ctl();
        drive(&mut c, &P, 150, |r| r.mcs <= 5);
        let cur = c.current(&P).unwrap();
        assert_eq!((cur.mcs, cur.gi), (5, Short), "{:?}", c.info(&P));
        let fresh = c.select(&P, 0, OCTETS);
        c.report(&P, fresh, false);
        let r1 = c.select(&P, 1, OCTETS);
        assert_eq!((r1.mcs, r1.gi), (5, Long));
        c.report(&P, r1, false);
        let r2 = c.select(&P, 2, OCTETS);
        assert_eq!((r2.mcs, r2.gi), (4, Long));
    }

    #[test]
    fn ldpc_is_probed_and_kept_once_acknowledged() {
        let mut c = ctl();
        let interval = c.config().probe_interval as usize;
        // A strong hint opens at the top MCS, so MCS probes do not come first.
        c.observe_snr(&P, 40.0);
        let used = drive(&mut c, &P, 100, |_| true);
        let first = used.iter().position(|r| r.fec_coding == Coding::Ldpc).expect("LDPC never probed");
        assert!(first < 2 * interval, "first LDPC probe only at attempt {first}");
        assert_eq!(c.current(&P).map(|r| r.fec_coding), Some(Coding::Ldpc));
        // Once acknowledged, LDPC stays: no BCC frame afterwards.
        let used = drive(&mut c, &P, 200, |_| true);
        assert!(used.iter().all(|r| r.fec_coding == Coding::Ldpc), "{used:?}");
    }

    #[test]
    fn a_peer_without_ldpc_gets_rare_probes_and_bcc_retries() {
        let mut c = RateControl::new(RateConfig { enabled: true, probe_backoff_max: 64, ..Default::default() });
        let interval = c.config().probe_interval as usize;
        let bcc_only = |r: TxChoice| r.fec_coding == Coding::Bcc;
        let used = drive(&mut c, &P, 1500, bcc_only);
        assert_eq!(c.current(&P).map(|r| r.fec_coding), Some(Coding::Bcc));
        // The first retry after a failed LDPC probe went BCC at the same
        // MCS: no batch needed a third attempt.
        let mut retries = 0;
        for w in used.windows(3) {
            if w[0].fec_coding == Coding::Ldpc && w[1].fec_coding == Coding::Ldpc && w[2].fec_coding == Coding::Ldpc {
                retries += 1;
            }
        }
        assert_eq!(retries, 0, "{:?}", c.info(&P));
        let used = drive(&mut c, &P, 400, bcc_only);
        let probes = used.iter().filter(|r| r.fec_coding == Coding::Ldpc).count();
        assert!(probes <= 400 / (interval + 64) + 1, "{probes} LDPC probes in 400 frames");
    }

    #[test]
    fn fixed_gi_and_coding_are_never_probed() {
        let mut c = RateControl::new(RateConfig { enabled: true, gi: Adapt::Fixed(Short), fec_coding: Adapt::Fixed(Coding::Bcc), ..Default::default() });
        let used = drive(&mut c, &P, 300, |_| true);
        assert!(used.iter().all(|r| r.gi == Short && r.fec_coding == Coding::Bcc), "{used:?}");
        assert_eq!(c.current(&P).map(|r| r.mcs), Some(8));
        // Retries keep the fixed guard interval.
        let fresh = c.select(&P, 0, OCTETS);
        c.report(&P, fresh, false);
        assert_eq!(c.select(&P, 1, OCTETS), TxChoice { mcs: 7, gi: Short, fec_coding: Coding::Bcc });
        let mut d = RateControl::new(RateConfig { enabled: true, gi: Adapt::Fixed(Long), fec_coding: Adapt::Fixed(Coding::Ldpc), ..Default::default() });
        let used = drive(&mut d, &P, 300, |_| true);
        assert!(used.iter().all(|r| r.gi == Long && r.fec_coding == Coding::Ldpc), "{used:?}");
    }
}
