//! Per-peer adaptive MCS selection ("rate control") for unicast data.
//!
//! 802.11 leaves rate adaptation entirely to the implementation. This is a
//! small Minstrel-style controller for a link on which a failed attempt
//! costs a long response timeout rather than a SIFS, so reliability is
//! weighted over raw throughput:
//!
//! * per peer and per MCS, an exponentially weighted success probability of
//!   transmission attempts (an attempt succeeds when it delivered at least
//!   what one PPDU at the next-lower rate would have carried: a lone frame
//!   must get through, a big A-MPDU may lose an MPDU or two and still beat
//!   the rate below it);
//! * the rate in use is the highest MCS whose probability is still above a
//!   floor; when it drops below the floor the controller falls back to the
//!   best rate below it;
//! * every few frames one attempt *probes* the next-higher MCS; a
//!   successful probe promotes it at once (ARF style), a failed probe
//!   doubles the wait before the next probe, up to a cap; a clear rise in
//!   the SNR heard from the peer re-arms probing at once;
//! * retries step down one MCS per retry from the rate that failed (a
//!   failed probe retries at the rate it probed from);
//! * the SNR the PHY reports for frames (or NDP Acks) *received from* the
//!   peer bounds where probes go, through a table of what this PHY needs
//!   per MCS measured in the receiver's own units. The reverse link is not
//!   the forward link, but on a symmetric OCB link it is the best hint
//!   available before any acknowledgement statistics exist, and it stops
//!   hopeless probes. A rate that has been flawless for a while may still
//!   probe above the bound, rarely, and only when the airtime the next
//!   rate would save over the coming frames outweighs the cost of one
//!   failure, so big A-MPDUs find a pessimistic hint's ceiling, and
//!   single small frames (which gain nothing worth a timeout) do not.
//!
//! The constants were tuned in `tests/rate_sim.rs`, a link-level
//! simulation with the PHY's measured PER-vs-SNR curves, the engine's
//! batch/retry rules, a response turnaround of tens of ms and a lost
//! attempt costing the response timeout, over static, shadowed, fading and
//! stepped channels. See that file for the numbers.
//!
//! Everything is per destination address, so a mesh node talks to a close
//! neighbour at 64-QAM while keeping BPSK for a distant one.

use crate::frame::MacAddr;
use s2g_phy::params;
use std::collections::HashMap;

/// MCS indices valid at 2 MHz / 1 SS, in increasing data-rate order.
pub const LADDER: [u8; 10] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 11];

/// SNR each MCS needs, in the units the receiver reports (its LTF-based
/// estimate, which reads 1–3 dB above the true SNR): the point where this
/// PHY delivers 90 % of 1000-octet PSDUs in an AWGN loopback with 10 kHz
/// CFO (`s2g-sim --report-snr`, BCC). Indexed like [`LADDER`].
const SNR_REQUIRED_DB: [f32; 10] = [8.5, 10.0, 10.5, 11.5, 15.5, 19.0, 20.5, 22.0, 26.0, 31.0];

/// A probe above what the SNR hint allows waits this many probe intervals
/// (plus the back-off), needs the rate in use to be at [`P_FLAWLESS`], and
/// must promise to save more airtime over that many intervals than one
/// failure costs.
const OVERRIDE_INTERVALS: u32 = 4;
const P_FLAWLESS: f32 = 0.98;

/// Controller settings.
#[derive(Debug, Clone, PartialEq)]
pub struct RateConfig {
    /// Adapt the MCS per peer (false: every unicast frame uses
    /// `MacConfig::mcs`).
    pub enabled: bool,
    /// Rate for a new peer nothing has been heard from yet.
    pub start_mcs: u8,
    /// Lowest MCS the controller falls back to.
    pub min_mcs: u8,
    /// Highest MCS the controller probes.
    pub max_mcs: u8,
    /// Frames between probes of the next-higher MCS.
    pub probe_interval: u32,
    /// Longest wait between probes after repeated failures, frames.
    pub probe_backoff_max: u32,
    /// A rise of this much (dB) in the smoothed SNR heard from the peer
    /// since a probe failed re-arms probing at once.
    pub probe_rearm_snr_db: f32,
    /// Weight of a new success/failure sample in the per-MCS average.
    pub alpha: f32,
    /// A rate stays in use while its success probability is at least this.
    pub p_min: f32,
    /// Extra SNR (dB) demanded over an MCS's requirement before probing it.
    pub snr_margin_db: f32,
    /// What a lost attempt costs, µs (the MAC sets its response timeout
    /// here): decides when probing above the SNR hint is worth the risk.
    pub fail_cost_us: u64,
}

impl Default for RateConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            start_mcs: 0,
            min_mcs: 0,
            max_mcs: 8,
            probe_interval: 8,
            probe_backoff_max: 2048,
            probe_rearm_snr_db: 2.0,
            alpha: 0.1,
            p_min: 0.85,
            snr_margin_db: 3.0,
            fail_cost_us: 150_000,
        }
    }
}

#[derive(Debug, Clone, Copy, Default)]
struct McsStat {
    /// Smoothed success probability; `None` until the first attempt.
    p: Option<f32>,
    attempts: u32,
    successes: u32,
}

#[derive(Debug, Clone)]
struct Peer {
    stats: [McsStat; LADDER.len()],
    /// Index into `LADDER` of the rate in use.
    cur: usize,
    since_probe: u32,
    /// Extra frames to wait before the next probe (doubles per failed probe).
    probe_backoff: u32,
    /// Index of an in-flight probe attempt, and whether it went above the
    /// SNR hint.
    probing: Option<(usize, bool)>,
    /// The batch's first failure was a probe: retries count from the
    /// probed rate, not from the rate in use.
    failed_probe: bool,
    /// Smoothed SNR at which the last probe failed (re-arm reference).
    probe_fail_snr_db: Option<f32>,
    /// Smoothed SNR of receptions from this peer, dB.
    snr_db: Option<f32>,
    /// Smoothed carrier frequency offset of receptions from this peer, Hz
    /// (the peer's oscillator relative to this station's, as the PHY sees
    /// it).
    cfo_hz: Option<f32>,
}

/// Per-peer statistics snapshot.
#[derive(Debug, Clone, PartialEq)]
pub struct PeerRateInfo {
    pub mcs: u8,
    pub snr_db: Option<f32>,
    pub cfo_hz: Option<f32>,
    /// (MCS, attempts, successes, smoothed success probability).
    pub per_mcs: Vec<(u8, u32, u32, Option<f32>)>,
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

fn snr_allows(margin_db: f32, snr_db: Option<f32>, idx: usize) -> bool {
    match snr_db {
        Some(s) => SNR_REQUIRED_DB[idx] + margin_db <= s,
        None => true,
    }
}

/// Airtime saved by carrying `octets` at `LADDER[to]` instead of
/// `LADDER[from]`, µs (Data symbols only; the preamble is the same).
fn airtime_saving_us(from: usize, to: usize, octets: usize) -> u64 {
    let per_octet = |i: usize| 320.0 / params::n_dbps_2mhz(LADDER[i], 1).unwrap_or(26) as f64;
    ((per_octet(from) - per_octet(to)).max(0.0) * octets as f64) as u64
}

/// The ladder entry below `mcs` (`mcs` itself at the bottom or off the
/// ladder): the rate an attempt at `mcs` has to beat.
pub fn next_lower(mcs: u8) -> u8 {
    ladder_index(mcs).map_or(mcs, |i| LADDER[i.saturating_sub(1)])
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
            Some(_) => (self.lo..=self.hi).filter(|&i| snr_allows(self.cfg.snr_margin_db, snr_db, i)).max().unwrap_or(self.lo),
            None => ladder_index(self.cfg.start_mcs).unwrap_or(self.lo).clamp(self.lo, self.hi),
        }
    }

    fn peer_mut(&mut self, addr: &MacAddr) -> &mut Peer {
        if !self.peers.contains_key(addr) {
            let cur = self.start_index(None);
            self.peers.insert(
                *addr,
                Peer {
                    stats: [McsStat::default(); LADDER.len()],
                    cur,
                    since_probe: 0,
                    probe_backoff: 0,
                    probing: None,
                    failed_probe: false,
                    probe_fail_snr_db: None,
                    snr_db: None,
                    cfo_hz: None,
                },
            );
        }
        self.peers.get_mut(addr).expect("inserted")
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
            p.probe_backoff = 0;
            p.probe_fail_snr_db = None;
        }
        if p.stats.iter().all(|s| s.attempts == 0) {
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

    /// Smoothed carrier offset of `addr`, Hz, if anything was heard.
    pub fn peer_cfo_hz(&self, addr: &MacAddr) -> Option<f32> {
        self.peers.get(addr).and_then(|p| p.cfo_hz)
    }

    /// MCS for the next transmission attempt to `addr`: `retry` is 0 for a
    /// fresh batch, else the number of failed attempts so far; `octets` is
    /// what the batch still has to carry (0 if unknown). Call
    /// [`RateControl::report`] with the outcome.
    pub fn select(&mut self, addr: &MacAddr, retry: u32, octets: usize) -> u8 {
        let (lo, hi, interval, margin) = (self.lo, self.hi, self.cfg.probe_interval, self.cfg.snr_margin_db);
        let fail_cost = self.cfg.fail_cost_us;
        let p = self.peer_mut(addr);
        p.probing = None;
        if retry > 0 {
            // One step down per failed attempt, from the probed rate when
            // the batch's first failure was a probe.
            let from = p.cur + p.failed_probe as usize;
            return LADDER[from.saturating_sub(retry as usize).max(lo)];
        }
        p.failed_probe = false;
        p.since_probe += 1;
        let cand = p.cur + 1;
        if cand <= hi {
            let wait = p.since_probe;
            let allowed = snr_allows(margin, p.snr_db, cand);
            let due = if allowed {
                wait >= interval + p.probe_backoff
            } else {
                // The hint is only the reverse link through one PHY's
                // average table. A rate that has been flawless for a while
                // gets a rare probe above the bound when the airtime the
                // next rate would save over the coming frames outweighs
                // one failure.
                let saving = airtime_saving_us(p.cur, cand, octets) * (OVERRIDE_INTERVALS * interval) as u64;
                wait >= OVERRIDE_INTERVALS * interval + p.probe_backoff && p.stats[p.cur].p.is_some_and(|v| v >= P_FLAWLESS) && saving >= fail_cost
            };
            if due {
                p.since_probe = 0;
                p.probing = Some((cand, !allowed));
                return LADDER[cand];
            }
        }
        LADDER[p.cur]
    }

    /// Outcome of an attempt to `addr` at `mcs`. The caller judges
    /// `success` by what the attempt delivered against what one PPDU at
    /// [`next_lower`]`(mcs)` would have carried of the same batch: a lone
    /// frame must get through, while a big A-MPDU may lose an MPDU or two
    /// and still beat the rate below it (see `engine::Mac::resolve_attempt`).
    pub fn report(&mut self, addr: &MacAddr, mcs: u8, success: bool) {
        let Some(idx) = ladder_index(mcs) else { return };
        let (lo, hi, alpha, p_min) = (self.lo, self.hi, self.cfg.alpha, self.cfg.p_min);
        let (interval, backoff_max) = (self.cfg.probe_interval, self.cfg.probe_backoff_max);
        let p = self.peer_mut(addr);
        let st = &mut p.stats[idx];
        st.attempts += 1;
        st.successes += success as u32;
        let sample = if success { 1.0 } else { 0.0 };
        st.p = Some(match st.p {
            Some(v) => v + alpha * (sample - v),
            None => sample,
        });
        if success {
            p.failed_probe = false;
        }
        if let Some((probed, above_hint)) = p.probing.filter(|&(i, _)| i == idx) {
            p.probing = None;
            if success {
                // A successful probe promotes at once; one failure then
                // demotes again (ARF behavior).
                p.stats[probed].p = Some(p.stats[probed].p.unwrap_or(p_min).max(p_min));
                p.probe_backoff = 0;
                p.probe_fail_snr_db = None;
            } else {
                // A failed probe above the hint confirms the hint: wait
                // much longer before doubting it again.
                let base = if above_hint { OVERRIDE_INTERVALS * interval } else { interval };
                p.probe_backoff = (p.probe_backoff.max(base) * 2).min(backoff_max);
                p.probe_fail_snr_db = p.snr_db;
                p.failed_probe = true;
            }
        }
        let good = |v: f32| v + 1e-4 >= p_min;
        let known_good = (lo..=hi).filter(|&i| p.stats[i].p.is_some_and(good)).max();
        if p.stats[p.cur].p.is_some_and(|v| !good(v)) {
            // The rate in use is failing: fall back to the best known-good
            // rate below it, or one step down.
            let cur = p.cur;
            p.cur = known_good.filter(|&g| g < cur).unwrap_or(cur.saturating_sub(1).max(lo));
        } else if let Some(g) = known_good {
            if g > p.cur {
                p.cur = g;
            }
        }
    }

    /// Rate currently in use for `addr`, if the peer is known.
    pub fn current(&self, addr: &MacAddr) -> Option<u8> {
        self.peers.get(addr).map(|p| LADDER[p.cur])
    }

    /// Statistics for `addr`, if known.
    pub fn info(&self, addr: &MacAddr) -> Option<PeerRateInfo> {
        self.peers.get(addr).map(|p| PeerRateInfo {
            mcs: LADDER[p.cur],
            snr_db: p.snr_db,
            cfo_hz: p.cfo_hz,
            per_mcs: LADDER.iter().zip(&p.stats).map(|(&m, s)| (m, s.attempts, s.successes, s.p)).collect(),
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

    const P: MacAddr = [2, 0, 0, 0, 0, 1];
    const Q: MacAddr = [2, 0, 0, 0, 0, 2];
    /// Batch size used by the tests: a single small frame.
    const OCTETS: usize = 150;

    fn ctl() -> RateControl {
        RateControl::new(RateConfig { enabled: true, ..Default::default() })
    }

    /// Drive `n` fresh frames of `octets`; `works(mcs)` decides success;
    /// retries step down until one succeeds (max 4).
    fn drive_octets(ctl: &mut RateControl, peer: &MacAddr, n: usize, octets: usize, works: impl Fn(u8) -> bool) -> Vec<u8> {
        let mut used = Vec::new();
        for _ in 0..n {
            let mut retry = 0;
            loop {
                let mcs = ctl.select(peer, retry, octets);
                used.push(mcs);
                let ok = works(mcs);
                ctl.report(peer, mcs, ok);
                if ok || retry >= 4 {
                    break;
                }
                retry += 1;
            }
        }
        used
    }

    fn drive(ctl: &mut RateControl, peer: &MacAddr, n: usize, works: impl Fn(u8) -> bool) -> Vec<u8> {
        drive_octets(ctl, peer, n, OCTETS, works)
    }

    #[test]
    fn climbs_to_the_cap_on_a_perfect_link() {
        let mut c = ctl();
        let interval = c.config().probe_interval as usize;
        let used = drive(&mut c, &P, 200, |_| true);
        assert_eq!(c.current(&P), Some(8));
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
        let mut c = ctl();
        let used = drive(&mut c, &P, 600, |m| m <= 4);
        assert_eq!(c.current(&P), Some(4), "{:?}", c.info(&P));
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
        let mut c = ctl();
        drive(&mut c, &P, 100, |m| m <= 4);
        assert_eq!(c.current(&P), Some(4));
        // Force the next fresh frame to be a probe.
        let n = (c.config().probe_interval + c.config().probe_backoff_max) as usize;
        let mut mcs = 4;
        for _ in 0..n {
            mcs = c.select(&P, 0, OCTETS);
            if mcs == 5 {
                break;
            }
            c.report(&P, mcs, true);
        }
        assert_eq!(mcs, 5, "no probe within {n} frames: {:?}", c.info(&P));
        c.report(&P, 5, false);
        assert_eq!(c.select(&P, 1, OCTETS), 4, "first retry after a failed probe goes at the rate in use");
        c.report(&P, 4, false);
        assert_eq!(c.select(&P, 2, OCTETS), 3);
    }

    #[test]
    fn probing_backs_off_to_the_cap_and_rearms_on_better_snr() {
        let mut c = RateControl::new(RateConfig { enabled: true, probe_backoff_max: 64, ..Default::default() });
        let interval = c.config().probe_interval as usize;
        // 20 dB admits probes of MCS 4, which the channel then refuses.
        c.observe_snr(&P, 20.0);
        drive(&mut c, &P, 1500, |m| m <= 3);
        assert_eq!(c.current(&P), Some(3));
        // Probes are now at most one per (interval + cap) frames.
        let used = drive(&mut c, &P, 400, |m| m <= 3);
        let probes = used.iter().filter(|&&m| m == 4).count();
        assert!(probes <= 400 / (interval + 64) + 1, "{probes} probes in 400 frames");
        // The peer suddenly sounds much better: probing resumes at once.
        for _ in 0..10 {
            c.observe_snr(&P, 30.0);
        }
        let used = drive(&mut c, &P, interval + 1, |m| m <= 3);
        assert!(used.contains(&4), "no probe after the SNR rose: {used:?}");
    }

    #[test]
    fn falls_back_when_the_channel_degrades_and_recovers() {
        let mut c = ctl();
        drive(&mut c, &P, 150, |m| m <= 6);
        assert_eq!(c.current(&P), Some(6));
        drive(&mut c, &P, 40, |m| m <= 2);
        assert_eq!(c.current(&P), Some(2), "{:?}", c.info(&P));
        drive(&mut c, &P, 300, |m| m <= 6);
        assert_eq!(c.current(&P), Some(6), "{:?}", c.info(&P));
    }

    #[test]
    fn retries_step_down_and_do_not_probe() {
        let mut c = ctl();
        drive(&mut c, &P, 100, |m| m <= 5);
        assert_eq!(c.current(&P), Some(5));
        assert_eq!(c.select(&P, 1, OCTETS), 4);
        assert_eq!(c.select(&P, 2, OCTETS), 3);
        assert_eq!(c.select(&P, 9, OCTETS), 0);
    }

    #[test]
    fn peers_are_independent() {
        let mut c = ctl();
        drive(&mut c, &P, 150, |m| m <= 7);
        drive(&mut c, &Q, 150, |m| m <= 1);
        assert_eq!(c.current(&P), Some(7));
        assert_eq!(c.current(&Q), Some(1));
        assert_eq!(c.peers().count(), 2);
    }

    #[test]
    fn snr_hint_bounds_probing_and_picks_the_opening_rate() {
        // The table is what the PHY was measured to need, in its own units.
        assert_eq!(snr_required_db(0), 8.5);
        assert_eq!(snr_required_db(2), 10.5);
        assert_eq!(snr_required_db(8), 26.0);
        assert_eq!(snr_required_db(9), f32::INFINITY);
        let mut c = ctl();
        let margin = c.config().snr_margin_db;
        let allowed = |snr: f32| LADDER.iter().copied().filter(|&m| m <= 8 && snr_required_db(m) + margin <= snr).max().unwrap();
        c.observe_snr(&P, 14.0);
        // Opening rate: the highest MCS allowed at 14 dB (MCS 2 at a 2 dB margin).
        assert_eq!(c.current(&P), Some(allowed(14.0)));
        // Small single frames never probe above the bound: the airtime
        // they would save is not worth a timeout.
        let used = drive(&mut c, &P, 400, |_| true);
        assert!(used.iter().all(|&m| m <= allowed(14.0)), "probed above the SNR bound: {used:?}");
        // A better SNR later lifts the bound.
        for _ in 0..20 {
            c.observe_snr(&P, 30.0);
        }
        drive(&mut c, &P, 300, |_| true);
        assert_eq!(c.current(&P), Some(8));
        // Without a hint a new peer starts at the configured start rate.
        let mut d = RateControl::new(RateConfig { enabled: true, start_mcs: 3, ..Default::default() });
        assert_eq!(d.select(&Q, 0, OCTETS), 3);
    }

    #[test]
    fn big_batches_probe_above_a_pessimistic_hint_and_back_off_hard_when_wrong() {
        let mut c = ctl();
        let interval = c.config().probe_interval;
        let margin = c.config().snr_margin_db;
        let bound = LADDER.iter().copied().filter(|&m| m <= 8 && snr_required_db(m) + margin <= 14.0).max().unwrap();
        c.observe_snr(&P, 14.0);
        assert_eq!(c.current(&P), Some(bound));
        // 16 × 1500 octets: the next rate saves tens of ms per batch.
        let used = drive_octets(&mut c, &P, 600, 16 * 1530, |_| true);
        assert!(used.iter().any(|&m| m > bound), "never probed above the bound: {:?}", c.info(&P));
        assert!(c.current(&P).unwrap() > bound, "{:?}", c.info(&P));
        // With the hint being right, failed probes above it are rare.
        let mut d = ctl();
        d.observe_snr(&P, 14.0);
        let used = drive_octets(&mut d, &P, 2000, 16 * 1530, |m| m <= bound);
        let above = used.iter().filter(|&&m| m > bound).count();
        let first = used.iter().position(|&m| m > bound).unwrap() + 1;
        assert!(first >= (OVERRIDE_INTERVALS * interval) as usize, "probed above the hint after only {first} frames");
        assert!(above <= 5, "{above} probes above the hint in 2000 frames");
        assert_eq!(d.current(&P), Some(bound));
    }

    #[test]
    fn bounds_are_respected() {
        let mut c = RateControl::new(RateConfig { enabled: true, min_mcs: 2, max_mcs: 5, ..Default::default() });
        let used = drive(&mut c, &P, 200, |_| true);
        assert!(used.iter().all(|&m| (2..=5).contains(&m)), "{used:?}");
        assert_eq!(c.current(&P), Some(5));
        let used = drive(&mut c, &P, 100, |_| false);
        assert!(used.iter().all(|&m| (2..=5).contains(&m)), "{used:?}");
        assert_eq!(c.current(&P), Some(2));
    }
}
