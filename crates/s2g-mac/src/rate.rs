//! Per-peer adaptive MCS selection ("rate control") for unicast data.
//!
//! 802.11 leaves rate adaptation entirely to the implementation. This is a
//! small Minstrel-flavoured controller tuned for a link on which a failed
//! attempt costs a long response timeout rather than a SIFS, so
//! reliability is weighted over raw throughput:
//!
//! * per peer and per MCS, an exponentially weighted success probability of
//!   transmission attempts;
//! * the rate in use is the highest MCS whose probability is still above a
//!   floor; when it drops below the floor the controller falls back to the
//!   best rate below it;
//! * every few frames one attempt *probes* the next-higher MCS; a
//!   successful probe promotes it at once (ARF style), a failed probe
//!   doubles the wait before the next probe;
//! * retries step down one MCS per retry;
//! * the SNR the PHY reports for frames (or NDP Acks) *received from* the
//!   peer bounds where probes go. The reverse link is not the forward link,
//!   but on a symmetric OCB link it is the best hint available before any
//!   acknowledgement statistics exist, and it stops hopeless probes.
//!
//! Everything is per destination address, so a mesh node talks to a close
//! neighbour at 64-QAM while keeping BPSK for a distant one.

use crate::frame::MacAddr;
use s2g_phy::params::{self, rf};
use std::collections::HashMap;

/// MCS indices valid at 2 MHz / 1 SS, in increasing data-rate order.
const LADDER: [u8; 10] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 11];

/// SNR (dB) at which MCS 0 is assumed to deliver most frames; the other
/// MCSes are offset from it by the Table 23-35 sensitivity differences.
const MCS0_SNR_DB: f32 = 5.0;

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
    /// Weight of a new success/failure sample in the per-MCS average.
    pub alpha: f32,
    /// A rate stays in use while its success probability is at least this.
    pub p_min: f32,
    /// Extra SNR (dB) demanded over an MCS's requirement before probing it.
    pub snr_margin_db: f32,
}

impl Default for RateConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            start_mcs: 0,
            min_mcs: 0,
            max_mcs: 8,
            probe_interval: 8,
            alpha: 0.2,
            p_min: 0.7,
            snr_margin_db: 0.0,
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
    /// Index of an in-flight probe attempt.
    probing: Option<usize>,
    /// Smoothed SNR of what we hear from this peer, dB.
    snr_db: Option<f32>,
}

/// Per-peer statistics snapshot.
#[derive(Debug, Clone, PartialEq)]
pub struct PeerRateInfo {
    pub mcs: u8,
    pub snr_db: Option<f32>,
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

/// SNR needed for `mcs`: the Table 23-35 sensitivity relative to MCS 0 on
/// top of [`MCS0_SNR_DB`].
pub fn snr_required_db(mcs: u8) -> f32 {
    let base = rf::min_sensitivity_2mhz_dbm(0).unwrap_or(-92.0);
    rf::min_sensitivity_2mhz_dbm(mcs).map_or(f32::INFINITY, |s| s - base + MCS0_SNR_DB)
}

fn snr_allows(margin_db: f32, snr_db: Option<f32>, idx: usize) -> bool {
    match snr_db {
        Some(s) => snr_required_db(LADDER[idx]) + margin_db <= s,
        None => true,
    }
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
                Peer { stats: [McsStat::default(); LADDER.len()], cur, since_probe: 0, probe_backoff: 0, probing: None, snr_db: None },
            );
        }
        self.peers.get_mut(addr).expect("inserted")
    }

    /// The PHY reported `snr_db` for something received from `addr`.
    pub fn observe_snr(&mut self, addr: &MacAddr, snr_db: f32) {
        if !snr_db.is_finite() {
            return;
        }
        let p = self.peer_mut(addr);
        p.snr_db = Some(match p.snr_db {
            Some(s) => s + 0.3 * (snr_db - s),
            None => snr_db,
        });
        if p.stats.iter().all(|s| s.attempts == 0) {
            // Nothing sent yet: let the hint pick the opening rate.
            let snr = p.snr_db;
            let cur = self.start_index(snr);
            self.peer_mut(addr).cur = cur;
        }
    }

    /// MCS for the next transmission attempt to `addr` (`retry` = 0 for a
    /// fresh frame). Call [`RateControl::report`] with the outcome.
    pub fn select(&mut self, addr: &MacAddr, retry: u32) -> u8 {
        let (lo, hi, interval, margin) = (self.lo, self.hi, self.cfg.probe_interval, self.cfg.snr_margin_db);
        let p = self.peer_mut(addr);
        p.probing = None;
        if retry > 0 {
            return LADDER[p.cur.saturating_sub(retry as usize).max(lo)];
        }
        p.since_probe += 1;
        if p.cur < hi && p.since_probe >= interval + p.probe_backoff {
            p.since_probe = 0;
            let cand = p.cur + 1;
            if snr_allows(margin, p.snr_db, cand) {
                p.probing = Some(cand);
                return LADDER[cand];
            }
        }
        LADDER[p.cur]
    }

    /// Outcome of an attempt to `addr` at `mcs`: acknowledged or not.
    pub fn report(&mut self, addr: &MacAddr, mcs: u8, success: bool) {
        let Some(idx) = ladder_index(mcs) else { return };
        let (lo, hi, alpha, p_min, interval) = (self.lo, self.hi, self.cfg.alpha, self.cfg.p_min, self.cfg.probe_interval);
        let p = self.peer_mut(addr);
        let st = &mut p.stats[idx];
        st.attempts += 1;
        st.successes += success as u32;
        let sample = if success { 1.0 } else { 0.0 };
        st.p = Some(match st.p {
            Some(v) => v + alpha * (sample - v),
            None => sample,
        });
        if p.probing == Some(idx) {
            p.probing = None;
            if success {
                // A successful probe promotes at once; one failure then
                // demotes again (classic ARF behaviour).
                p.stats[idx].p = Some(p.stats[idx].p.unwrap_or(p_min).max(p_min));
                p.probe_backoff = 0;
            } else {
                p.probe_backoff = (p.probe_backoff.max(interval) * 2).min(512);
            }
        }
        let known_good = (lo..=hi).filter(|&i| p.stats[i].p.is_some_and(|v| v >= p_min)).max();
        if p.stats[p.cur].p.is_some_and(|v| v < p_min) {
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

    fn ctl() -> RateControl {
        RateControl::new(RateConfig { enabled: true, ..Default::default() })
    }

    /// Drive `n` fresh frames; `works(mcs)` decides success; retries step
    /// down until one succeeds (max 4).
    fn drive(ctl: &mut RateControl, peer: &MacAddr, n: usize, works: impl Fn(u8) -> bool) -> Vec<u8> {
        let mut used = Vec::new();
        for _ in 0..n {
            let mut retry = 0;
            loop {
                let mcs = ctl.select(peer, retry);
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

    #[test]
    fn climbs_to_the_cap_on_a_perfect_link() {
        let mut c = ctl();
        let used = drive(&mut c, &P, 200, |_| true);
        assert_eq!(c.current(&P), Some(8));
        // Probes go one step at a time: 0..=8 all appear, nothing above.
        for m in [0u8, 1, 2, 3, 4, 5, 6, 7, 8] {
            assert!(used.contains(&m), "MCS {m} never tried: {used:?}");
        }
        assert!(!used.contains(&11));
        // Reached the cap within a few probe intervals.
        let first_8 = used.iter().position(|&m| m == 8).unwrap();
        assert!(first_8 < 8 * 9 + 8, "took {first_8} attempts to reach MCS 8");
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
        assert_eq!(c.select(&P, 1), 4);
        assert_eq!(c.select(&P, 2), 3);
        assert_eq!(c.select(&P, 9), 0);
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
        // Requirements from Table 23-35 relative to MCS 0 at 5 dB:
        // MCS 0..=8 → 5, 8, 10, 13, 17, 21, 22, 23, 28 dB.
        assert_eq!(snr_required_db(0), 5.0);
        assert_eq!(snr_required_db(2), 10.0);
        assert_eq!(snr_required_db(8), 28.0);
        let mut c = ctl();
        c.observe_snr(&P, 12.0);
        // Opening rate: the highest MCS allowed at 12 dB.
        assert_eq!(c.current(&P), Some(2));
        let used = drive(&mut c, &P, 200, |_| true);
        assert!(used.iter().all(|&m| m <= 2), "probed above the SNR bound: {used:?}");
        // A better SNR later lifts the bound.
        for _ in 0..20 {
            c.observe_snr(&P, 30.0);
        }
        drive(&mut c, &P, 200, |_| true);
        assert_eq!(c.current(&P), Some(8));
        // Without a hint a new peer starts at the configured start rate.
        let mut d = RateControl::new(RateConfig { enabled: true, start_mcs: 3, ..Default::default() });
        assert_eq!(d.select(&Q, 0), 3);
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
