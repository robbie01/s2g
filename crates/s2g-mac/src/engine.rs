//! The OCB MAC engine: IO-free, clock-injected state machine.
//!
//! Callers feed PHY `RxEvent`s and Ethernet frames in, and poll
//! [`MacAction`]s out (PPDUs / NDP CMAC PPDUs to transmit). Time is
//! caller-supplied microseconds, so tests can drive it deterministically.
//!
//! Medium access:
//! * **CCA** comes from the PHY (`RxEvent::Cca`, energy detect + preamble
//!   detect + predicted PPDU hold) — the MAC never transmits while the PHY
//!   says BUSY.
//! * **NAV** from Duration fields of frames not addressed to us (RTS, NDP
//!   CTS, Data) [10.3.2.4].
//! * **RID** (response indication deferral) from every received PPDU's
//!   RESPONSE_INDICATION [10.3.2.5] — mandatory for S1G STAs. There is no
//!   BSS here, so every PPDU is a "nonmember PPDU": the RID is extended,
//!   never reset, except when the PPDU is addressed to us.
//! * DIFS + uniform backoff in [0, CW] slots after the medium goes idle;
//!   CW doubles per retry between cw_min and cw_max; the backoff is redrawn
//!   (not frozen) if the medium turns busy — a documented simplification.
//!
//! Acknowledgement: unicast data solicits an **NDP Ack** (or an **NDP
//! BlockAck** when the PSDU is an A-MPDU), signalled with
//! RESPONSE_INDICATION = NDP Response [10.3.2.17, Table 10-7]; the Ack ID /
//! BlockAck ID are derived from the scrambler seed the MAC chose for the
//! PPDU [23.3.12.2.4/6]. Legacy Ack frames (Normal Response) remain
//! available for interop testing. Frames above `rts_threshold` are
//! protected by RTS → **NDP CTS** [10.3.2.9]. Response frames go out at the
//! next poll (our stand-in for SIFS timing; buffered SDR streaming makes
//! real SIFS turnaround impossible without hardware timestamping, which is
//! also why the ACK timeout defaults to 150 ms).

use crate::frame::{self, MacAddr, ParsedFrame};
use crate::ndp::{self, NdpAck, NdpBlockAck, NdpCts, NdpFrame};
use crate::{ampdu, eth};
use s2g_phy::params::{characteristics::*, T_PREAMBLE_US};
use s2g_phy::rx::RxEvent;
use s2g_phy::vector::{Coding, ResponseIndication, RxVector, TxVector};
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
    Phy(#[from] s2g_phy::PhyError),
}

#[derive(Debug, Clone)]
pub struct MacConfig {
    pub addr: MacAddr,
    /// MCS for data frames (control responses always go at MCS 0).
    pub mcs: u8,
    /// FEC for data frames (BCC or LDPC).
    pub fec_coding: Coding,
    /// Traveling pilots on data frames (only if the peer supports them).
    pub traveling_pilots: bool,
    /// ACK + retry for unicast data.
    pub ack_enabled: bool,
    /// Solicit NDP Ack / NDP BlockAck (true) or legacy Ack frames (false).
    pub ndp_ack: bool,
    /// Protect unicast MPDUs longer than this with RTS / NDP CTS.
    pub rts_threshold: Option<usize>,
    /// Response wait beyond the eliciting PPDU airtime, µs.
    pub ack_timeout_us: u64,
    pub max_retries: u32,
    pub cw_min_exp: u32,
    pub cw_max_exp: u32,
    /// aSlotTime, µs [23.3.15].
    pub slot_us: u64,
    /// DIFS = SIFS + 2·slot, µs.
    pub difs_us: u64,
    /// LongTxTime for RID on a Long Response indication (largest TXOP
    /// limit) [10.3.2.5.2], µs.
    pub long_tx_time_us: u64,
    pub queue_limit: usize,
}

impl MacConfig {
    pub fn new(addr: MacAddr) -> Self {
        Self {
            addr,
            mcs: 0,
            fec_coding: Coding::Bcc,
            traveling_pilots: false,
            ack_enabled: true,
            ndp_ack: true,
            rts_threshold: None,
            ack_timeout_us: 150_000,
            max_retries: 3,
            cw_min_exp: 4,
            cw_max_exp: 10,
            slot_us: A_SLOT_TIME_US as u64,
            difs_us: (A_SIFS_TIME_US + 2 * A_SLOT_TIME_US) as u64,
            long_tx_time_us: 3008,
            queue_limit: 64,
        }
    }

    fn partial_aid(&self) -> u16 {
        ndp::ocb_partial_aid(&self.addr)
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
    /// An NDP CMAC PPDU arrived.
    NdpReceived { frame: NdpFrame },
}

#[derive(Debug, Clone, PartialEq)]
pub enum MacAction {
    /// Hand this PSDU to `Transmitter::generate` and send it.
    Transmit { txv: TxVector, psdu: Vec<u8> },
    /// Hand this 37-bit body to `Transmitter::generate_ndp` and send it.
    TransmitNdp { body: u64 },
}

/// What we are waiting for after a transmission.
#[derive(Debug, Clone, PartialEq, Eq)]
enum Expect {
    Ack,
    NdpAck { ack_id: u16 },
    NdpBlockAck { block_ack_id: u8, seq: u16 },
    NdpCts { partial_aid: u16 },
}

#[derive(Debug)]
enum TxState {
    Idle,
    Backoff { until_us: u64 },
    AwaitResponse { deadline_us: u64, expect: Expect },
}

struct CurrentTx {
    dest: MacAddr,
    src: MacAddr,
    body: Vec<u8>,
    seq: u16,
    retries: u32,
    /// RTS/CTS handshake completed for this attempt.
    cts_ok: bool,
}

enum Response {
    Ndp(u64),
    Frame(Vec<u8>),
}

pub struct Mac {
    cfg: MacConfig,
    seq: u16,
    queue: VecDeque<(MacAddr, MacAddr, Vec<u8>)>,
    cur: Option<CurrentTx>,
    responses: VecDeque<Response>,
    state: TxState,
    // ---- medium state ----
    cca_busy: bool,
    busy_until_us: u64,
    nav_until_us: u64,
    rid_until_us: u64,
    /// Start of the most recent PPDU (RxStart), for RID/NAV anchoring.
    last_ppdu_end_us: u64,
    cw_exp: u32,
    dedup: HashMap<MacAddr, VecDeque<u16>>,
    rng: u64,
}

fn is_group(addr: &MacAddr) -> bool {
    addr[0] & 1 != 0
}

/// TXTIME of an MCS 0 control frame of `octets` [Table 10-3 NormalTxTime].
fn normal_tx_time_us(octets: usize) -> u64 {
    s2g_phy::tx::txtime_us(0, octets, false).unwrap_or(1000) as u64
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
            responses: VecDeque::new(),
            state: TxState::Idle,
            cca_busy: false,
            busy_until_us: 0,
            nav_until_us: 0,
            rid_until_us: 0,
            last_ppdu_end_us: 0,
            cw_exp,
            dedup: HashMap::new(),
            rng,
        }
    }

    pub fn config(&self) -> &MacConfig {
        &self.cfg
    }

    /// True while CCA, NAV or RID forbids transmission.
    pub fn medium_busy(&self, now_us: u64) -> bool {
        self.cca_busy || now_us < self.busy_until_us || now_us < self.nav_until_us || now_us < self.rid_until_us
    }

    fn rand_u32(&mut self) -> u32 {
        let mut x = self.rng;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.rng = x;
        (x >> 32) as u32
    }

    fn pick_seed(&mut self) -> u8 {
        (self.rand_u32() % 127) as u8 + 1
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
            if mpdu_len > ampdu::MAX_MPDU_LEN || s2g_phy::tx::aggregated_capacity(self.cfg.mcs, pre, self.cfg.fec_coding).is_err() {
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

    /// RID length implied by a PPDU's RESPONSE_INDICATION [10.3.2.5.2].
    fn rid_length_us(&self, rxv: &RxVector) -> u64 {
        let sifs = A_SIFS_TIME_US as u64;
        match rxv.response_indication {
            ResponseIndication::None => 0,
            ResponseIndication::Ndp => NDP_TX_TIME_US as u64 + sifs,
            ResponseIndication::Normal => normal_tx_time_us(if rxv.aggregation { 32 } else { 14 }) + sifs,
            ResponseIndication::Long => self.cfg.long_tx_time_us + sifs,
        }
    }

    fn extend_rid(&mut self, until: u64) {
        // Nonmember PPDU: never reset, only extend.
        self.rid_until_us = self.rid_until_us.max(until);
    }

    /// Feed a PHY receive event. `now_us` is the caller's clock.
    pub fn on_phy_event(&mut self, ev: &RxEvent, now_us: u64, out: &mut Vec<MacEvent>) {
        match ev {
            RxEvent::Cca { busy, hold_us, .. } => {
                self.cca_busy = *busy;
                if *busy && *hold_us > 0 {
                    self.busy_until_us = self.busy_until_us.max(now_us + *hold_us as u64);
                }
            }
            RxEvent::PpduStart { .. } => {
                // At least a preamble is in the air.
                self.busy_until_us = self.busy_until_us.max(now_us + T_PREAMBLE_US as u64);
            }
            RxEvent::RxStart { rxvector, .. } => {
                // The preamble (240 µs) has already elapsed when SIG is known.
                let remaining = rxvector.ppdu_duration_us().saturating_sub(T_PREAMBLE_US) as u64;
                let end = now_us + remaining;
                self.busy_until_us = self.busy_until_us.max(end);
                self.last_ppdu_end_us = end;
                let rid = self.rid_length_us(rxvector);
                if rid > 0 {
                    self.extend_rid(end + rid);
                }
            }
            RxEvent::NdpReceived { body, .. } => {
                let f = NdpFrame::parse(*body);
                self.on_ndp(&f, now_us, out);
                out.push(MacEvent::NdpReceived { frame: f });
            }
            RxEvent::RxEnd { .. } => {}
            RxEvent::PsduReceived { rxvector, psdu, .. } => {
                let mpdus: Vec<Vec<u8>> = if rxvector.aggregation {
                    ampdu::deaggregate(psdu)
                } else {
                    // Tolerate chips that pad the PSDU after the FCS.
                    vec![frame::locate_mpdu(psdu).unwrap_or(psdu).to_vec()]
                };
                for mpdu in mpdus {
                    self.on_mpdu(&mpdu, rxvector, now_us, out);
                }
            }
        }
    }

    fn on_ndp(&mut self, f: &NdpFrame, now_us: u64, out: &mut Vec<MacEvent>) {
        match f {
            NdpFrame::Ack(a) => {
                if let TxState::AwaitResponse { expect: Expect::NdpAck { ack_id }, .. } = &self.state {
                    if *ack_id == a.ack_id {
                        self.complete_current(true, out);
                    }
                }
                if !a.idle_indication && a.duration > 0 {
                    self.nav_until_us = self.nav_until_us.max(now_us + a.duration as u64);
                }
            }
            NdpFrame::BlockAck(ba) => {
                if let TxState::AwaitResponse { expect: Expect::NdpBlockAck { block_ack_id, seq }, .. } = &self.state {
                    if *block_ack_id == ba.block_ack_id && *seq == ba.starting_sequence && ba.bitmap & 1 == 1 {
                        self.complete_current(true, out);
                    }
                }
            }
            NdpFrame::Cts(c) => {
                let for_us = !c.address_indicator && c.ra_pbssid == self.cfg.partial_aid();
                if for_us {
                    if let TxState::AwaitResponse { expect: Expect::NdpCts { .. }, .. } = &self.state {
                        if let Some(cur) = self.cur.as_mut() {
                            cur.cts_ok = true;
                        }
                        self.state = TxState::Backoff { until_us: now_us }; // transmit at once
                    }
                } else if c.duration_us > 0 {
                    self.nav_until_us = self.nav_until_us.max(now_us + c.duration_us as u64);
                }
            }
            NdpFrame::Other { .. } => {}
        }
    }

    fn complete_current(&mut self, acked: bool, out: &mut Vec<MacEvent>) {
        if let Some(cur) = self.cur.take() {
            out.push(MacEvent::TxComplete { dest: cur.dest, acked, retries: cur.retries });
        }
        self.state = TxState::Idle;
        self.cw_exp = self.cfg.cw_min_exp;
    }

    fn on_mpdu(&mut self, mpdu: &[u8], rxv: &RxVector, now_us: u64, out: &mut Vec<MacEvent>) {
        match frame::parse(mpdu) {
            Ok(ParsedFrame::Data { dest, src, seq, duration_us, body, .. }) => {
                if src == self.cfg.addr {
                    return; // our own transmission looping back
                }
                let for_us = dest == self.cfg.addr;
                if !for_us && !is_group(&dest) {
                    if duration_us > 0 {
                        self.nav_until_us = self.nav_until_us.max(now_us + duration_us as u64);
                    }
                    return; // someone else's unicast
                }
                if for_us {
                    // Addressed to us: we are the responder, so the RID does
                    // not apply to us [10.3.2.5.1].
                    self.rid_until_us = 0;
                    if self.cfg.ack_enabled {
                        // ACK even duplicates — the peer may have missed our ACK.
                        self.queue_ack(rxv, seq, mpdu, src);
                    }
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
                    if let TxState::AwaitResponse { expect: Expect::Ack, .. } = &self.state {
                        self.complete_current(true, out);
                    }
                }
            }
            Ok(ParsedFrame::Rts { ra, ta, duration_us }) => {
                if ra == self.cfg.addr {
                    self.rid_until_us = 0;
                    // NDP CTS [10.3.2.9]: Duration = RTS duration − SIFS − NDPTxTime.
                    let dur = (duration_us as u64).saturating_sub(A_SIFS_TIME_US as u64 + NDP_TX_TIME_US as u64);
                    let cts = NdpCts {
                        address_indicator: false,
                        ra_pbssid: ndp::ocb_partial_aid(&ta),
                        duration_us: dur.min(0x7fff) as u16,
                        early_sector_indicator: false,
                        bandwidth: 1, // 2 MHz [Table 9-5]
                    };
                    self.responses.push_back(Response::Ndp(NdpFrame::Cts(cts).to_body()));
                } else if duration_us > 0 {
                    self.nav_until_us = self.nav_until_us.max(now_us + duration_us as u64);
                }
            }
            Ok(ParsedFrame::Other { duration_us, .. }) => {
                if duration_us > 0 {
                    self.nav_until_us = self.nav_until_us.max(now_us + duration_us as u64);
                }
            }
            Err(_) => {}
        }
    }

    /// Queue the acknowledgement the eliciting PPDU asked for [Table 10-7].
    fn queue_ack(&mut self, rxv: &RxVector, seq: u16, mpdu: &[u8], src: MacAddr) {
        match rxv.response_indication {
            ResponseIndication::Ndp => {
                let f = if rxv.aggregation {
                    NdpFrame::BlockAck(NdpBlockAck {
                        block_ack_id: ndp::block_ack_id(rxv.scrambler_seed),
                        starting_sequence: seq,
                        bitmap: 1,
                    })
                } else {
                    NdpFrame::Ack(NdpAck {
                        ack_id: ndp::ack_id_for_mpdu(rxv.scrambler_seed, mpdu),
                        more_data: false,
                        idle_indication: false,
                        duration: 0,
                        relayed_frame: false,
                    })
                };
                self.responses.push_back(Response::Ndp(f.to_body()));
            }
            ResponseIndication::Normal => self.responses.push_back(Response::Frame(frame::build_ack(src))),
            ResponseIndication::None | ResponseIndication::Long => {}
        }
    }

    fn start_backoff(&mut self, now_us: u64) {
        let cw = (1u64 << self.cw_exp) - 1;
        let slots = (self.rand_u32() as u64) % (cw + 1);
        let idle_at = now_us.max(self.busy_until_us).max(self.nav_until_us).max(self.rid_until_us);
        let base = idle_at + self.cfg.difs_us;
        self.state = TxState::Backoff { until_us: base + slots * self.cfg.slot_us };
    }

    fn make_psdu(&self, mpdu: Vec<u8>) -> Result<(Vec<u8>, bool), MacError> {
        if mpdu.len() <= 511 {
            return Ok((mpdu, false));
        }
        let pre = ampdu::pre_eof_len(mpdu.len());
        let cap = s2g_phy::tx::aggregated_capacity(self.cfg.mcs, pre, self.cfg.fec_coding)?;
        Ok((ampdu::aggregate(&mpdu, cap), true))
    }

    fn fail_attempt(&mut self, now_us: u64, out: &mut Vec<MacEvent>) {
        let cur = self.cur.as_mut().expect("attempt implies cur");
        cur.retries += 1;
        cur.cts_ok = false;
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

    /// Advance the state machine; may return one PPDU to transmit.
    pub fn poll(&mut self, now_us: u64, out: &mut Vec<MacEvent>) -> Option<MacAction> {
        // 1. Pending control responses preempt everything (SIFS priority).
        if let Some(r) = self.responses.pop_front() {
            return Some(match r {
                Response::Ndp(body) => MacAction::TransmitNdp { body },
                Response::Frame(psdu) => MacAction::Transmit { txv: TxVector { mcs: 0, ..Default::default() }, psdu },
            });
        }

        // 2. Response timeout → retry or drop.
        if let TxState::AwaitResponse { deadline_us, .. } = &self.state {
            if now_us >= *deadline_us {
                self.fail_attempt(now_us, out);
            }
        }

        // 3. Pull new work.
        if matches!(self.state, TxState::Idle) && self.cur.is_none() {
            if let Some((dest, src, body)) = self.queue.pop_front() {
                let seq = self.seq;
                self.seq = (self.seq + 1) & 0x0fff;
                self.cur = Some(CurrentTx { dest, src, body, seq, retries: 0, cts_ok: false });
                self.start_backoff(now_us);
            }
        }

        // 4. Backoff expiry → transmit (RTS first if required).
        if let TxState::Backoff { until_us } = self.state {
            if now_us < until_us {
                return None;
            }
            if self.medium_busy(now_us) {
                // Medium became busy: redraw after it clears (nonstandard
                // simplification of backoff freezing).
                self.start_backoff(now_us);
                return None;
            }
            let cur = self.cur.as_ref().expect("Backoff implies cur");
            let want_ack = self.cfg.ack_enabled && !is_group(&cur.dest);
            let sifs = A_SIFS_TIME_US as u64;
            let resp_time = if self.cfg.ndp_ack { NDP_TX_TIME_US as u64 } else { normal_tx_time_us(14) };
            let mpdu_len = frame::DATA_HDR_LEN + cur.body.len() + 4;
            let need_rts = want_ack && !cur.cts_ok && self.cfg.rts_threshold.is_some_and(|t| mpdu_len > t);
            if need_rts {
                // Duration covers CTS + data + response [9.2.5.2].
                let data_time = s2g_phy::tx::txtime_us_coded(self.cfg.mcs, mpdu_len.min(511), mpdu_len > 511, self.cfg.fec_coding)
                    .unwrap_or(10_000) as u64;
                let duration = sifs + NDP_TX_TIME_US as u64 + sifs + data_time + sifs + resp_time;
                let psdu = frame::build_rts(cur.dest, cur.src, duration.min(0x7fff) as u16);
                let seed = self.pick_seed();
                let txv = TxVector {
                    mcs: 0,
                    response_indication: ResponseIndication::Ndp,
                    scrambler_seed: Some(seed),
                    ..Default::default()
                };
                let airtime = s2g_phy::tx::txtime_us(0, psdu.len(), false).unwrap_or(1000) as u64;
                let partial_aid = self.cfg.partial_aid();
                self.state = TxState::AwaitResponse {
                    deadline_us: now_us + airtime + self.cfg.ack_timeout_us,
                    expect: Expect::NdpCts { partial_aid },
                };
                return Some(MacAction::Transmit { txv, psdu });
            }
            let duration = if want_ack { (sifs + resp_time).min(0x7fff) as u16 } else { 0 };
            let mpdu = frame::build_data(cur.dest, cur.src, cur.seq, cur.retries > 0, duration, &cur.body);
            let (psdu, aggregated) = match self.make_psdu(mpdu.clone()) {
                Ok(x) => x,
                Err(_) => {
                    let cur = self.cur.take().unwrap();
                    out.push(MacEvent::TxDropped { dest: cur.dest, reason: "PSDU build failed" });
                    self.state = TxState::Idle;
                    return None;
                }
            };
            let seed = self.pick_seed();
            let response_indication = match (want_ack, self.cfg.ndp_ack) {
                (false, _) => ResponseIndication::None,
                (true, true) => ResponseIndication::Ndp,
                (true, false) => ResponseIndication::Normal,
            };
            let txv = TxVector {
                mcs: self.cfg.mcs,
                fec_coding: self.cfg.fec_coding,
                traveling_pilots: self.cfg.traveling_pilots,
                aggregation: aggregated,
                response_indication,
                scrambler_seed: Some(seed),
                ..Default::default()
            };
            let airtime =
                s2g_phy::tx::txtime_us_coded(self.cfg.mcs, psdu.len(), aggregated, self.cfg.fec_coding).unwrap_or(10_000) as u64;
            if want_ack {
                let cur = self.cur.as_ref().unwrap();
                let expect = match (self.cfg.ndp_ack, aggregated) {
                    (false, _) => Expect::Ack,
                    (true, false) => Expect::NdpAck { ack_id: ndp::ack_id_for_mpdu(seed, &mpdu) },
                    (true, true) => Expect::NdpBlockAck { block_ack_id: ndp::block_ack_id(seed), seq: cur.seq },
                };
                self.state = TxState::AwaitResponse { deadline_us: now_us + airtime + self.cfg.ack_timeout_us, expect };
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
    use s2g_phy::rx::RxMetrics;

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

    fn metrics() -> RxMetrics {
        RxMetrics { snr_db: 30.0, cfo_hz: 0.0, evm_db: -30.0, rssi_dbfs: -30.0, timing_drift_samples: 0.0, ldpc_failures: 0 }
    }

    fn psdu_event(txv: &TxVector, psdu: &[u8]) -> RxEvent {
        let g = s2g_phy::tx::data_geometry(txv.mcs, psdu.len(), txv.aggregation, txv.fec_coding).unwrap();
        RxEvent::PsduReceived {
            sample_index: 0,
            rxvector: RxVector {
                mcs: txv.mcs,
                aggregation: txv.aggregation,
                response_indication: txv.response_indication,
                fec_coding: txv.fec_coding,
                psdu_length: psdu.len(),
                n_sym: g.n_sym,
                scrambler_seed: txv.scrambler_seed.unwrap_or(1),
                ..Default::default()
            },
            psdu: psdu.to_vec(),
            metrics: metrics(),
        }
    }

    #[test]
    fn broadcast_fire_and_forget() {
        let mut mac = Mac::new(MacConfig::new(A));
        let mut out = Vec::new();
        mac.enqueue_eth(&eth_frame(BCAST, A, 50)).unwrap();
        let mut now = 0u64;
        let act = drain_tx(&mut mac, &mut now, &mut out).expect("tx");
        let MacAction::Transmit { txv, psdu } = act else { panic!() };
        assert!(!txv.aggregation);
        assert_eq!(txv.response_indication, ResponseIndication::None);
        assert!(psdu.len() <= 511);
        assert!(out.iter().any(|e| matches!(e, MacEvent::TxComplete { acked: false, .. })));
        // Nothing further pending.
        assert!(mac.poll(now + 1_000_000, &mut out).is_none());
    }

    #[test]
    fn large_frame_aggregates_and_expects_ndp_block_ack() {
        let mut cfg = MacConfig::new(A);
        cfg.mcs = 5;
        let mut mac = Mac::new(cfg);
        let mut out = Vec::new();
        mac.enqueue_eth(&eth_frame(B, A, 1400)).unwrap();
        let mut now = 0;
        let Some(MacAction::Transmit { txv, psdu }) = drain_tx(&mut mac, &mut now, &mut out) else { panic!() };
        assert!(txv.aggregation);
        assert_eq!(txv.response_indication, ResponseIndication::Ndp);
        let seed = txv.scrambler_seed.unwrap();
        // PSDU fills the symbol capacity exactly.
        let cap = s2g_phy::tx::aggregated_capacity(5, psdu.len(), Coding::Bcc).unwrap();
        assert_eq!(psdu.len(), cap);
        // And deaggregates back to one valid data frame.
        let mpdus = ampdu::deaggregate(&psdu);
        assert_eq!(mpdus.len(), 1);
        let ParsedFrame::Data { seq, .. } = frame::parse(&mpdus[0]).unwrap() else { panic!() };
        // The matching NDP BlockAck completes the exchange.
        let ba = NdpFrame::BlockAck(NdpBlockAck { block_ack_id: ndp::block_ack_id(seed), starting_sequence: seq, bitmap: 1 });
        mac.on_phy_event(&RxEvent::NdpReceived { sample_index: 0, body: ba.to_body(), metrics: metrics() }, now, &mut out);
        mac.poll(now, &mut out);
        assert!(out.iter().any(|e| matches!(e, MacEvent::TxComplete { acked: true, .. })), "{out:?}");
    }

    #[test]
    fn ldpc_and_traveling_pilots_are_signalled() {
        let mut cfg = MacConfig::new(A);
        cfg.mcs = 3;
        cfg.fec_coding = Coding::Ldpc;
        cfg.traveling_pilots = true;
        cfg.ack_enabled = false;
        let mut mac = Mac::new(cfg);
        let mut out = Vec::new();
        mac.enqueue_eth(&eth_frame(B, A, 900)).unwrap();
        let mut now = 0;
        let Some(MacAction::Transmit { txv, psdu }) = drain_tx(&mut mac, &mut now, &mut out) else { panic!() };
        assert_eq!(txv.fec_coding, Coding::Ldpc);
        assert!(txv.traveling_pilots);
        assert_eq!(psdu.len(), s2g_phy::tx::aggregated_capacity(3, psdu.len(), Coding::Ldpc).unwrap());
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
    fn ndp_ack_completes_and_dedup_on_rx() {
        let mut mac_a = Mac::new(MacConfig::new(A));
        let mut mac_b = Mac::new(MacConfig::new(B));
        let mut out_a = Vec::new();
        let mut out_b = Vec::new();
        mac_a.enqueue_eth(&eth_frame(B, A, 60)).unwrap();
        let mut now = 0;
        let Some(MacAction::Transmit { txv, psdu }) = drain_tx(&mut mac_a, &mut now, &mut out_a) else { panic!() };
        assert_eq!(txv.response_indication, ResponseIndication::Ndp);
        assert!(txv.scrambler_seed.is_some());

        // Deliver to B twice (simulating a duplicate).
        for _ in 0..2 {
            mac_b.on_phy_event(&psdu_event(&txv, &psdu), now, &mut out_b);
        }
        // Exactly one delivery despite two receptions.
        let delivered: Vec<_> = out_b.iter().filter(|e| matches!(e, MacEvent::EthReceived(_))).collect();
        assert_eq!(delivered.len(), 1);
        // B queues an NDP Ack for each reception; take one and feed it to A.
        let Some(MacAction::TransmitNdp { body }) = mac_b.poll(now, &mut out_b) else { panic!() };
        match NdpFrame::parse(body) {
            NdpFrame::Ack(a) => assert_eq!(a.ack_id, ndp::ack_id_for_mpdu(txv.scrambler_seed.unwrap(), &psdu)),
            other => panic!("{other:?}"),
        }
        mac_a.on_phy_event(&RxEvent::NdpReceived { sample_index: 0, body, metrics: metrics() }, now, &mut out_a);
        mac_a.poll(now, &mut out_a);
        assert!(out_a.iter().any(|e| matches!(e, MacEvent::TxComplete { acked: true, retries: 0, .. })), "{out_a:?}");
        // A wrong Ack ID is ignored.
        let mut mac_c = Mac::new(MacConfig::new(A));
        mac_c.enqueue_eth(&eth_frame(B, A, 60)).unwrap();
        let mut out_c = Vec::new();
        let mut now_c = 0;
        drain_tx(&mut mac_c, &mut now_c, &mut out_c).unwrap();
        let bad = NdpFrame::Ack(NdpAck { ack_id: 0x1234, more_data: false, idle_indication: false, duration: 0, relayed_frame: false });
        mac_c.on_phy_event(&RxEvent::NdpReceived { sample_index: 0, body: bad.to_body(), metrics: metrics() }, now_c, &mut out_c);
        mac_c.poll(now_c, &mut out_c);
        assert!(!out_c.iter().any(|e| matches!(e, MacEvent::TxComplete { .. })));
    }

    #[test]
    fn legacy_ack_when_ndp_disabled() {
        let mut cfg = MacConfig::new(A);
        cfg.ndp_ack = false;
        let mut mac_a = Mac::new(cfg.clone());
        let mut mac_b = Mac::new(MacConfig::new(B));
        let (mut out_a, mut out_b) = (Vec::new(), Vec::new());
        mac_a.enqueue_eth(&eth_frame(B, A, 60)).unwrap();
        let mut now = 0;
        let Some(MacAction::Transmit { txv, psdu }) = drain_tx(&mut mac_a, &mut now, &mut out_a) else { panic!() };
        assert_eq!(txv.response_indication, ResponseIndication::Normal);
        mac_b.on_phy_event(&psdu_event(&txv, &psdu), now, &mut out_b);
        let Some(MacAction::Transmit { psdu: ack, txv: ack_txv }) = mac_b.poll(now, &mut out_b) else { panic!() };
        assert_eq!(ack_txv.mcs, 0);
        assert_eq!(frame::parse(&ack).unwrap(), ParsedFrame::Ack { ra: A });
        mac_a.on_phy_event(&psdu_event(&ack_txv, &ack), now, &mut out_a);
        mac_a.poll(now, &mut out_a);
        assert!(out_a.iter().any(|e| matches!(e, MacEvent::TxComplete { acked: true, .. })));
    }

    #[test]
    fn rts_ndp_cts_handshake() {
        let mut cfg = MacConfig::new(A);
        cfg.rts_threshold = Some(100);
        let mut mac_a = Mac::new(cfg);
        let mut mac_b = Mac::new(MacConfig::new(B));
        let (mut out_a, mut out_b) = (Vec::new(), Vec::new());
        mac_a.enqueue_eth(&eth_frame(B, A, 300)).unwrap();
        let mut now = 0;
        // 1. A sends an RTS soliciting an NDP response.
        let Some(MacAction::Transmit { txv, psdu }) = drain_tx(&mut mac_a, &mut now, &mut out_a) else { panic!() };
        let ParsedFrame::Rts { ra, ta, duration_us } = frame::parse(&psdu).unwrap() else { panic!("not RTS") };
        assert_eq!((ra, ta), (B, A));
        assert!(duration_us > 1000);
        assert_eq!(txv.response_indication, ResponseIndication::Ndp);
        // 2. B answers with an NDP CTS addressed to A's partial AID.
        mac_b.on_phy_event(&psdu_event(&txv, &psdu), now, &mut out_b);
        let Some(MacAction::TransmitNdp { body }) = mac_b.poll(now, &mut out_b) else { panic!() };
        let NdpFrame::Cts(c) = NdpFrame::parse(body) else { panic!() };
        assert_eq!(c.ra_pbssid, ndp::ocb_partial_aid(&A));
        assert_eq!(c.bandwidth, 1);
        assert_eq!(c.duration_us as u64, duration_us as u64 - 160 - 240);
        // A third station sets its NAV from the CTS.
        let mut mac_c = Mac::new(MacConfig::new([2, 0, 0, 0, 0, 0xC]));
        mac_c.on_phy_event(&RxEvent::NdpReceived { sample_index: 0, body, metrics: metrics() }, now, &mut Vec::new());
        assert!(mac_c.medium_busy(now + 10));
        // 3. A receives the CTS and sends the data at once.
        mac_a.on_phy_event(&RxEvent::NdpReceived { sample_index: 0, body, metrics: metrics() }, now, &mut out_a);
        let Some(MacAction::Transmit { txv: dtxv, psdu: data }) = mac_a.poll(now, &mut out_a) else { panic!("no data after CTS") };
        assert!(matches!(frame::parse(&data).unwrap(), ParsedFrame::Data { .. }));
        // 4. B acks it with an NDP Ack, completing the exchange.
        mac_b.on_phy_event(&psdu_event(&dtxv, &data), now, &mut out_b);
        let Some(MacAction::TransmitNdp { body }) = mac_b.poll(now, &mut out_b) else { panic!() };
        mac_a.on_phy_event(&RxEvent::NdpReceived { sample_index: 0, body, metrics: metrics() }, now, &mut out_a);
        mac_a.poll(now, &mut out_a);
        assert!(out_a.iter().any(|e| matches!(e, MacEvent::TxComplete { acked: true, .. })), "{out_a:?}");
        assert!(out_b.iter().any(|e| matches!(e, MacEvent::EthReceived(_))));
    }

    #[test]
    fn cca_hold_and_rid_defer() {
        let mut cfg = MacConfig::new(A);
        cfg.ack_enabled = false;
        let mut mac = Mac::new(cfg);
        let mut out = Vec::new();
        // PHY says BUSY with a 100 ms hold.
        mac.on_phy_event(&RxEvent::Cca { sample_index: 0, busy: true, reason: None, hold_us: 100_000 }, 0, &mut out);
        mac.on_phy_event(&RxEvent::Cca { sample_index: 0, busy: false, reason: None, hold_us: 0 }, 5, &mut out);
        mac.enqueue_eth(&eth_frame(BCAST, A, 30)).unwrap();
        for t in (0..99_000).step_by(1000) {
            assert!(mac.poll(t, &mut out).is_none(), "transmitted at {t} while busy");
        }
        let mut now = 100_500;
        assert!(drain_tx(&mut mac, &mut now, &mut out).is_some());

        // RID: a PPDU for someone else expecting an NDP response holds us
        // for its duration + SIFS + NDPTxTime.
        let mut mac2 = Mac::new({
            let mut c = MacConfig::new(A);
            c.ack_enabled = false;
            c
        });
        let rxv = RxVector { n_sym: 10, response_indication: ResponseIndication::Ndp, ..Default::default() };
        mac2.on_phy_event(&RxEvent::RxStart { sample_index: 0, rxvector: rxv.clone() }, 1000, &mut out);
        // Remaining PPDU = 400 µs, then RID = 240 + 160.
        assert!(mac2.medium_busy(1000 + 400 + 300));
        assert!(!mac2.medium_busy(1000 + 400 + 400 + 1));
        // A PPDU addressed to us clears the RID (we are the responder).
        let txv = TxVector { response_indication: ResponseIndication::Ndp, scrambler_seed: Some(5), ..Default::default() };
        let data = frame::build_data(A, B, 1, false, 400, b"hi");
        mac2.on_phy_event(&psdu_event(&txv, &data), 1400, &mut out);
        assert!(!mac2.medium_busy(1401));
    }

    #[test]
    fn indefinite_cca_busy_blocks_until_idle() {
        let mut cfg = MacConfig::new(A);
        cfg.ack_enabled = false;
        let mut mac = Mac::new(cfg);
        let mut out = Vec::new();
        mac.on_phy_event(&RxEvent::Cca { sample_index: 0, busy: true, reason: None, hold_us: 0 }, 0, &mut out);
        mac.enqueue_eth(&eth_frame(BCAST, A, 30)).unwrap();
        for t in (0..500_000).step_by(5000) {
            assert!(mac.poll(t, &mut out).is_none());
        }
        mac.on_phy_event(&RxEvent::Cca { sample_index: 0, busy: false, reason: None, hold_us: 0 }, 500_000, &mut out);
        let mut now = 500_000;
        assert!(drain_tx(&mut mac, &mut now, &mut out).is_some());
    }
}
