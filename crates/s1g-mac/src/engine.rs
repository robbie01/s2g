//! The OCB MAC engine: IO-free, clock-injected state machine.
//!
//! Callers feed PHY `RxEvent`s and Ethernet frames in, and poll
//! [`MacAction`]s out (PPDUs to transmit). Time is caller-supplied
//! microseconds, so tests can drive it deterministically.
//!
//! Channel access (deliberately simplified vs EDCA, documented nonstandard):
//! DIFS + uniform backoff in [0, CW] slots after the medium goes idle;
//! CW doubles per retry between cw_min and cw_max; the backoff is redrawn
//! (not frozen) if the medium turns busy — acceptable at SDR latencies.
//! ACK timing is likewise relaxed: buffered SDR streaming adds tens of ms,
//! so the ACK timeout defaults to 150 ms instead of SIFS-scale.

use crate::frame::{self, MacAddr, ParsedFrame};
use crate::{ampdu, eth};
use s1g_phy::params;
use s1g_phy::rx::RxEvent;
use s1g_phy::vector::{ResponseIndication, TxVector};
use std::collections::{HashMap, VecDeque};
use thiserror::Error;

#[derive(Debug, Error, Clone, PartialEq, Eq)]
pub enum MacError {
    #[error("transmit queue full")]
    QueueFull,
    #[error("not an Ethernet frame")]
    BadEthernet,
    #[error("frame too large for any PPDU at this MCS")]
    FrameTooBig,
    #[error("PHY: {0}")]
    Phy(#[from] s1g_phy::PhyError),
}

#[derive(Debug, Clone)]
pub struct MacConfig {
    pub addr: MacAddr,
    /// MCS for data frames (ACKs always go at MCS 0).
    pub mcs: u8,
    /// ACK + retry for unicast data.
    pub ack_enabled: bool,
    /// ACK wait beyond the data PPDU airtime, µs.
    pub ack_timeout_us: u64,
    pub max_retries: u32,
    pub cw_min_exp: u32,
    pub cw_max_exp: u32,
    /// aSlotTime, µs [23.3.15].
    pub slot_us: u64,
    /// DIFS = SIFS + 2·slot, µs.
    pub difs_us: u64,
    pub queue_limit: usize,
}

impl MacConfig {
    pub fn new(addr: MacAddr) -> Self {
        use params::characteristics::*;
        Self {
            addr,
            mcs: 0,
            ack_enabled: true,
            ack_timeout_us: 150_000,
            max_retries: 3,
            cw_min_exp: 4,
            cw_max_exp: 10,
            slot_us: A_SLOT_TIME_US as u64,
            difs_us: (A_SIFS_TIME_US + 2 * A_SLOT_TIME_US) as u64,
            queue_limit: 64,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum MacEvent {
    /// A frame for the local network stack (raw Ethernet bytes).
    EthReceived(Vec<u8>),
    /// A queued frame finished: delivered (or fire-and-forget broadcast).
    TxComplete { dest: MacAddr, acked: bool, retries: u32 },
    /// A queued frame was dropped.
    TxDropped { dest: MacAddr, reason: &'static str },
    /// An NDP CMAC PPDU arrived (PHY-level 37-bit body, for future control).
    NdpReceived { body: u64 },
}

#[derive(Debug, Clone, PartialEq)]
pub enum MacAction {
    /// Hand this PSDU to `Transmitter::generate` and send it.
    Transmit { txv: TxVector, psdu: Vec<u8> },
}

#[derive(Debug)]
enum TxState {
    Idle,
    Backoff { until_us: u64 },
    AwaitAck { deadline_us: u64 },
}

struct CurrentTx {
    dest: MacAddr,
    src: MacAddr,
    body: Vec<u8>,
    seq: u16,
    retries: u32,
}

pub struct Mac {
    cfg: MacConfig,
    seq: u16,
    queue: VecDeque<(MacAddr, MacAddr, Vec<u8>)>,
    cur: Option<CurrentTx>,
    ack_queue: VecDeque<MacAddr>,
    state: TxState,
    busy_until_us: u64,
    cw_exp: u32,
    dedup: HashMap<MacAddr, VecDeque<u16>>,
    rng: u64,
}

fn is_group(addr: &MacAddr) -> bool {
    addr[0] & 1 != 0
}

impl Mac {
    pub fn new(cfg: MacConfig) -> Self {
        let rng = cfg.addr.iter().fold(0x9E37_79B9u64, |a, &b| (a << 8) ^ b as u64 ^ (a >> 3)) | 1;
        let cw_exp = cfg.cw_min_exp;
        Self {
            cfg,
            seq: 0,
            queue: VecDeque::new(),
            cur: None,
            ack_queue: VecDeque::new(),
            state: TxState::Idle,
            busy_until_us: 0,
            cw_exp,
            dedup: HashMap::new(),
            rng,
        }
    }

    pub fn config(&self) -> &MacConfig {
        &self.cfg
    }

    fn rand_u32(&mut self) -> u32 {
        let mut x = self.rng;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.rng = x;
        (x >> 32) as u32
    }

    /// Queue an outgoing Ethernet frame (from the TAP).
    pub fn enqueue_eth(&mut self, eth_frame: &[u8]) -> Result<(), MacError> {
        let (dest, src, ethertype, payload) = eth::parse_ethernet(eth_frame).ok_or(MacError::BadEthernet)?;
        if self.queue.len() >= self.cfg.queue_limit {
            return Err(MacError::QueueFull);
        }
        // Pre-flight size check at the configured MCS.
        let body = eth::to_body(ethertype, payload);
        let mpdu_len = frame::DATA_HDR_LEN + body.len() + 4;
        if mpdu_len > 511 {
            let pre = ampdu::pre_eof_len(mpdu_len);
            if mpdu_len > ampdu::MAX_MPDU_LEN || s1g_phy::tx::n_sym(self.cfg.mcs, pre, true).is_err() {
                return Err(MacError::FrameTooBig);
            }
        }
        self.queue.push_back((dest, src, body));
        Ok(())
    }

    fn note_duplicate(&mut self, src: MacAddr, seq: u16) -> bool {
        let seqs = self.dedup.entry(src).or_default();
        if seqs.contains(&seq) {
            return true;
        }
        seqs.push_back(seq);
        if seqs.len() > 16 {
            seqs.pop_front();
        }
        false
    }

    /// Feed a PHY receive event. `now_us` is the caller's clock.
    pub fn on_phy_event(&mut self, ev: &RxEvent, now_us: u64, out: &mut Vec<MacEvent>) {
        match ev {
            RxEvent::PpduStart { .. } => {
                // At least a preamble is in the air.
                self.busy_until_us = self.busy_until_us.max(now_us + 240);
            }
            RxEvent::SigDecoded { rxvector, .. } => {
                self.busy_until_us = self.busy_until_us.max(now_us + 40 * rxvector.n_sym as u64);
            }
            RxEvent::NdpReceived { body, .. } => {
                out.push(MacEvent::NdpReceived { body: *body });
            }
            RxEvent::Error { .. } => {}
            RxEvent::PsduReceived { rxvector, psdu, .. } => {
                let mpdus: Vec<Vec<u8>> = if rxvector.aggregation {
                    ampdu::deaggregate(psdu)
                } else {
                    vec![psdu.clone()]
                };
                for mpdu in mpdus {
                    self.on_mpdu(&mpdu, out);
                }
            }
        }
    }

    fn on_mpdu(&mut self, mpdu: &[u8], out: &mut Vec<MacEvent>) {
        match frame::parse(mpdu) {
            Ok(ParsedFrame::Data { dest, src, seq, body, .. }) => {
                if src == self.cfg.addr {
                    return; // our own transmission looping back
                }
                let for_us = dest == self.cfg.addr;
                if !for_us && !is_group(&dest) {
                    return; // someone else's unicast
                }
                if for_us && self.cfg.ack_enabled {
                    // ACK even duplicates — the peer may have missed our ACK.
                    self.ack_queue.push_back(src);
                }
                if self.note_duplicate(src, seq) {
                    return;
                }
                if let Some(ethf) = eth::body_to_ethernet(dest, src, &body) {
                    out.push(MacEvent::EthReceived(ethf));
                }
            }
            Ok(ParsedFrame::Ack { ra }) => {
                if ra == self.cfg.addr {
                    if let (TxState::AwaitAck { .. }, Some(cur)) = (&self.state, self.cur.take()) {
                        out.push(MacEvent::TxComplete { dest: cur.dest, acked: true, retries: cur.retries });
                        self.state = TxState::Idle;
                        self.cw_exp = self.cfg.cw_min_exp;
                    }
                }
            }
            _ => {}
        }
    }

    fn start_backoff(&mut self, now_us: u64) {
        let cw = (1u64 << self.cw_exp) - 1;
        let slots = (self.rand_u32() as u64) % (cw + 1);
        let base = now_us.max(self.busy_until_us) + self.cfg.difs_us;
        self.state = TxState::Backoff { until_us: base + slots * self.cfg.slot_us };
    }

    fn make_psdu(&self, mpdu: Vec<u8>) -> Result<(Vec<u8>, bool), MacError> {
        if mpdu.len() <= 511 {
            return Ok((mpdu, false));
        }
        let pre = ampdu::pre_eof_len(mpdu.len());
        let n_sym = s1g_phy::tx::n_sym(self.cfg.mcs, pre, true)?;
        let p = params::mcs_params(self.cfg.mcs)?;
        let cap = (n_sym * p.n_dbps - 14) / 8;
        Ok((ampdu::aggregate(&mpdu, cap), true))
    }

    /// Advance the state machine; may return one PPDU to transmit.
    pub fn poll(&mut self, now_us: u64, out: &mut Vec<MacEvent>) -> Option<MacAction> {
        // 1. Pending ACKs preempt everything (our stand-in for SIFS priority).
        if let Some(ra) = self.ack_queue.pop_front() {
            let psdu = frame::build_ack(ra);
            let txv = TxVector { mcs: 0, ..Default::default() };
            return Some(MacAction::Transmit { txv, psdu });
        }

        // 2. ACK timeout → retry or drop.
        if let TxState::AwaitAck { deadline_us } = self.state {
            if now_us >= deadline_us {
                let cur = self.cur.as_mut().expect("AwaitAck implies cur");
                cur.retries += 1;
                if cur.retries > self.cfg.max_retries {
                    let cur = self.cur.take().unwrap();
                    out.push(MacEvent::TxDropped { dest: cur.dest, reason: "retry limit" });
                    self.state = TxState::Idle;
                    self.cw_exp = self.cfg.cw_min_exp;
                } else {
                    self.cw_exp = (self.cw_exp + 1).min(self.cfg.cw_max_exp);
                    self.start_backoff(now_us);
                }
            }
        }

        // 3. Pull new work.
        if matches!(self.state, TxState::Idle) && self.cur.is_none() {
            if let Some((dest, src, body)) = self.queue.pop_front() {
                let seq = self.seq;
                self.seq = (self.seq + 1) & 0x0fff;
                self.cur = Some(CurrentTx { dest, src, body, seq, retries: 0 });
                self.start_backoff(now_us);
            }
        }

        // 4. Backoff expiry → transmit.
        if let TxState::Backoff { until_us } = self.state {
            if now_us < until_us {
                return None;
            }
            if now_us < self.busy_until_us {
                // Medium became busy: redraw after it clears (nonstandard
                // simplification of backoff freezing).
                self.start_backoff(now_us);
                return None;
            }
            let cur = self.cur.as_ref().expect("Backoff implies cur");
            let mpdu = frame::build_data(cur.dest, cur.src, cur.seq, cur.retries > 0, &cur.body);
            let (psdu, aggregated) = match self.make_psdu(mpdu) {
                Ok(x) => x,
                Err(_) => {
                    let cur = self.cur.take().unwrap();
                    out.push(MacEvent::TxDropped { dest: cur.dest, reason: "PSDU build failed" });
                    self.state = TxState::Idle;
                    return None;
                }
            };
            let want_ack = self.cfg.ack_enabled && !is_group(&cur.dest);
            let txv = TxVector {
                mcs: self.cfg.mcs,
                aggregation: aggregated,
                response_indication: if want_ack { ResponseIndication::Normal } else { ResponseIndication::None },
                ..Default::default()
            };
            let airtime = s1g_phy::tx::txtime_us(self.cfg.mcs, psdu.len(), aggregated).unwrap_or(10_000) as u64;
            if want_ack {
                self.state = TxState::AwaitAck { deadline_us: now_us + airtime + self.cfg.ack_timeout_us };
            } else {
                let cur = self.cur.take().unwrap();
                out.push(MacEvent::TxComplete { dest: cur.dest, acked: false, retries: cur.retries });
                self.state = TxState::Idle;
            }
            return Some(MacAction::Transmit { txv, psdu });
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const A: MacAddr = [2, 0, 0, 0, 0, 0xA];
    const B: MacAddr = [2, 0, 0, 0, 0, 0xB];
    const BCAST: MacAddr = [0xff; 6];

    fn eth_frame(dest: MacAddr, src: MacAddr, n: usize) -> Vec<u8> {
        let mut f = Vec::new();
        f.extend_from_slice(&dest);
        f.extend_from_slice(&src);
        f.extend_from_slice(&0x0800u16.to_be_bytes());
        f.extend((0..n).map(|i| i as u8));
        f
    }

    fn drain_tx(mac: &mut Mac, now: &mut u64, out: &mut Vec<MacEvent>) -> Option<MacAction> {
        // Advance time until backoff expires (bounded).
        for _ in 0..10_000 {
            if let Some(a) = mac.poll(*now, out) {
                return Some(a);
            }
            *now += 100;
        }
        None
    }

    #[test]
    fn broadcast_fire_and_forget() {
        let mut mac = Mac::new(MacConfig::new(A));
        let mut out = Vec::new();
        mac.enqueue_eth(&eth_frame(BCAST, A, 50)).unwrap();
        let mut now = 0u64;
        let act = drain_tx(&mut mac, &mut now, &mut out).expect("tx");
        let MacAction::Transmit { txv, psdu } = act;
        assert!(!txv.aggregation);
        assert!(psdu.len() <= 511);
        assert!(out.iter().any(|e| matches!(e, MacEvent::TxComplete { acked: false, .. })));
        // Nothing further pending.
        assert!(mac.poll(now + 1_000_000, &mut out).is_none());
    }

    #[test]
    fn large_frame_aggregates() {
        let mut cfg = MacConfig::new(A);
        cfg.mcs = 5;
        cfg.ack_enabled = false;
        let mut mac = Mac::new(cfg);
        let mut out = Vec::new();
        mac.enqueue_eth(&eth_frame(B, A, 1400)).unwrap();
        let mut now = 0;
        let MacAction::Transmit { txv, psdu } = drain_tx(&mut mac, &mut now, &mut out).unwrap();
        assert!(txv.aggregation);
        // PSDU fills the symbol capacity exactly.
        let p = params::mcs_params(5).unwrap();
        let n_sym = s1g_phy::tx::n_sym(5, psdu.len(), true).unwrap();
        assert_eq!(psdu.len(), (n_sym * p.n_dbps - 14) / 8);
        // And deaggregates back to one valid data frame.
        let mpdus = ampdu::deaggregate(&psdu);
        assert_eq!(mpdus.len(), 1);
        assert!(matches!(frame::parse(&mpdus[0]).unwrap(), ParsedFrame::Data { .. }));
    }

    #[test]
    fn retry_then_drop_without_ack() {
        let mut cfg = MacConfig::new(A);
        cfg.max_retries = 2;
        cfg.ack_timeout_us = 1000;
        let mut mac = Mac::new(cfg);
        let mut out = Vec::new();
        mac.enqueue_eth(&eth_frame(B, A, 40)).unwrap();
        let mut now = 0;
        let mut transmissions = 0;
        for _ in 0..100_000 {
            if let Some(MacAction::Transmit { psdu, .. }) = mac.poll(now, &mut out) {
                transmissions += 1;
                if transmissions > 1 {
                    // Retry bit must be set on retransmissions.
                    match frame::parse(&psdu).unwrap() {
                        ParsedFrame::Data { retry, seq, .. } => {
                            assert!(retry);
                            assert_eq!(seq, 0);
                        }
                        other => panic!("{other:?}"),
                    }
                }
            }
            if out.iter().any(|e| matches!(e, MacEvent::TxDropped { .. })) {
                break;
            }
            now += 500;
        }
        assert_eq!(transmissions, 3, "initial + 2 retries");
        assert!(out.iter().any(|e| matches!(e, MacEvent::TxDropped { reason: "retry limit", .. })));
    }

    #[test]
    fn ack_completes_and_dedup_on_rx() {
        let mut mac_a = Mac::new(MacConfig::new(A));
        let mut mac_b = Mac::new(MacConfig::new(B));
        let mut out_a = Vec::new();
        let mut out_b = Vec::new();
        mac_a.enqueue_eth(&eth_frame(B, A, 60)).unwrap();
        let mut now = 0;
        let MacAction::Transmit { psdu, .. } = drain_tx(&mut mac_a, &mut now, &mut out_a).unwrap();

        // Deliver to B twice (simulating a duplicate).
        let rxv = s1g_phy::vector::RxVector {
            mcs: 0,
            gi: Default::default(),
            aggregation: false,
            response_indication: Default::default(),
            smoothing: true,
            traveling_pilots: false,
            uplink_indication: false,
            color: 0,
            partial_aid: 0,
            psdu_length: psdu.len(),
            n_sym: 10,
            scrambler_seed: 7,
            rssi_dbfs: -30.0,
        };
        for _ in 0..2 {
            let ev = RxEvent::PsduReceived {
                sample_index: 0,
                rxvector: rxv.clone(),
                psdu: psdu.clone(),
                metrics: s1g_phy::rx::RxMetrics { snr_db: 30.0, cfo_hz: 0.0, evm_db: -30.0, rssi_dbfs: -30.0 },
            };
            mac_b.on_phy_event(&ev, now, &mut out_b);
        }
        // Exactly one delivery despite two receptions.
        let delivered: Vec<_> = out_b.iter().filter(|e| matches!(e, MacEvent::EthReceived(_))).collect();
        assert_eq!(delivered.len(), 1);
        // B queues an ACK for each reception; take one and feed it to A.
        let MacAction::Transmit { psdu: ack, txv: ack_txv } = mac_b.poll(now, &mut out_b).unwrap();
        assert_eq!(ack_txv.mcs, 0);
        let ev = RxEvent::PsduReceived {
            sample_index: 0,
            rxvector: s1g_phy::vector::RxVector { psdu_length: ack.len(), ..rxv.clone() },
            psdu: ack,
            metrics: s1g_phy::rx::RxMetrics { snr_db: 30.0, cfo_hz: 0.0, evm_db: -30.0, rssi_dbfs: -30.0 },
        };
        mac_a.on_phy_event(&ev, now, &mut out_a);
        assert!(out_a.iter().any(|e| matches!(e, MacEvent::TxComplete { acked: true, retries: 0, .. })));
    }

    #[test]
    fn medium_busy_defers() {
        let mut cfg = MacConfig::new(A);
        cfg.ack_enabled = false;
        let mut mac = Mac::new(cfg);
        let mut out = Vec::new();
        // Mark medium busy for 100 ms.
        let rxv_dummy = RxEvent::PpduStart { sample_index: 0, coarse_cfo_hz: 0.0 };
        mac.on_phy_event(&rxv_dummy, 0, &mut out);
        let sig = RxEvent::SigDecoded {
            sample_index: 0,
            rxvector: s1g_phy::vector::RxVector {
                mcs: 0,
                gi: Default::default(),
                aggregation: false,
                response_indication: Default::default(),
                smoothing: true,
                traveling_pilots: false,
                uplink_indication: false,
                color: 0,
                partial_aid: 0,
                psdu_length: 100,
                n_sym: 2500,
                scrambler_seed: 0,
                rssi_dbfs: -40.0,
            },
        };
        mac.on_phy_event(&sig, 0, &mut out);
        mac.enqueue_eth(&eth_frame(BCAST, A, 30)).unwrap();
        // Cannot transmit while the medium is busy (n_sym·40 µs = 100 ms).
        for t in (0..99_000).step_by(1000) {
            assert!(mac.poll(t, &mut out).is_none(), "transmitted at {t} while busy");
        }
        // After busy + DIFS + backoff it goes out.
        let mut now = 100_500;
        assert!(drain_tx(&mut mac, &mut now, &mut out).is_some());
    }
}
