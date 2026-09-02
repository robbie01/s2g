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
//! Transmission: queued Ethernet frames for the same destination are
//! packed into one PPDU — an **A-MPDU** of QoS Data MPDUs (up to
//! `ampdu_max_mpdus`, at most the 16 bits of an NDP BlockAck bitmap),
//! acknowledged with an **NDP BlockAck** whose bitmap drives selective
//! retransmission; a lone frame goes as a plain MPDU or an **S-MPDU** and
//! solicits an **NDP Ack** (or a legacy Ack) [10.3.2.17, Table 10-7]. The
//! Ack ID / BlockAck ID derive from the scrambler seed the MAC chose for
//! the PPDU [23.3.12.2.4/6]. Frames above `rts_threshold` are protected by
//! RTS → **NDP CTS** [10.3.2.9]. Response frames go out at the next poll
//! (our stand-in for SIFS timing; buffered SDR streaming makes real SIFS
//! turnaround impossible without hardware timestamping, which is also why
//! the ACK timeout defaults to 150 ms). A-MPDUs are sent without a block
//! ack agreement — there is no ADDBA in OCB — which s2g peers accept and a
//! standard STA would not.
//!
//! Station identification for amateur use is built in: with a call sign
//! configured, a broadcast identification frame [`crate::ident`] precedes
//! the first data frame, repeats every `interval_us` while transmitting,
//! and closes a communication after `end_idle_us` of silence.

use crate::filter::{self, FilterConfig, Verdict};
use crate::frame::{self, MacAddr, ParsedFrame, Pv1Addr};
use crate::ident::{self, IdentConfig};
use crate::ndp::{self, NdpAck, NdpBlockAck, NdpCts, NdpFrame};
use crate::rate::{RateConfig, RateControl};
use crate::{ampdu, eth};
use s2g_phy::params::{characteristics::*, T_PREAMBLE_US};
use s2g_phy::rx::{RxEndStatus, RxEvent};
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
    /// Dropped by the good-neighbour filter (not an error for the caller
    /// to retry; the reason is for logging).
    #[error("filtered: {0}")]
    Filtered(&'static str),
    #[error("PHY: {0}")]
    Phy(#[from] s2g_phy::PhyError),
}

/// Most MPDUs one NDP BlockAck bitmap can acknowledge [Figure 23-33].
pub const AMPDU_MAX_MPDUS: usize = 16;

#[derive(Debug, Clone)]
pub struct MacConfig {
    pub addr: MacAddr,
    /// MCS for broadcast data frames, and for unicast when rate control is
    /// off (control responses and identification frames always go at MCS 0).
    pub mcs: u8,
    /// FEC for data frames (BCC or LDPC).
    pub fec_coding: Coding,
    /// Traveling pilots on data frames (only if the peer supports them).
    pub traveling_pilots: bool,
    /// ACK + retry for unicast data.
    pub ack_enabled: bool,
    /// Solicit NDP Ack / NDP BlockAck (true) or legacy Ack frames (false).
    pub ndp_ack: bool,
    /// Protect unicast PSDUs longer than this with RTS / NDP CTS.
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
    /// Per-peer adaptive MCS selection for unicast data; when enabled
    /// `mcs` is only the broadcast rate.
    pub rate: RateConfig,
    /// Most MPDUs packed into one A-MPDU (1 = never aggregate frames; the
    /// cap is [`AMPDU_MAX_MPDUS`]). Needs NDP acknowledgements.
    pub ampdu_max_mpdus: usize,
    /// Amateur-radio station identification.
    pub ident: IdentConfig,
    /// Stateless good-neighbour filter applied to frames leaving for the
    /// air and arriving from it. Off in the library default; `s2g-node`
    /// enables [`FilterConfig::good_neighbor`].
    pub filter: FilterConfig,
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
            rate: RateConfig::default(),
            ampdu_max_mpdus: 8,
            ident: IdentConfig::default(),
            filter: FilterConfig::off(),
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
    TxComplete { dest: MacAddr, acked: bool, retries: u32, mcs: u8 },
    /// A queued frame was dropped.
    TxDropped { dest: MacAddr, reason: &'static str },
    /// An NDP CMAC PPDU arrived.
    NdpReceived { frame: NdpFrame },
    /// A station identification frame went out.
    IdentSent { text: String },
    /// A station identification frame was heard.
    IdentReceived { src: MacAddr, text: String },
    /// A frame from the air was dropped by the good-neighbour filter
    /// (egress drops are reported as `MacError::Filtered` by `enqueue_eth`).
    Filtered { reason: &'static str },
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
    NdpBlockAck { block_ack_id: u8 },
    NdpCts { partial_aid: u16 },
}

#[derive(Debug)]
enum TxState {
    Idle,
    Backoff { until_us: u64 },
    AwaitResponse { deadline_us: u64, expect: Expect },
}

/// One MSDU awaiting delivery.
struct PendingMpdu {
    seq: u16,
    body: Vec<u8>,
    retries: u32,
}

/// The batch of MPDUs for one destination being worked on.
struct CurrentTx {
    dest: MacAddr,
    src: MacAddr,
    /// Outstanding (not yet acknowledged or dropped) MPDUs, in sequence order.
    mpdus: Vec<PendingMpdu>,
    /// Indices into `mpdus` of the attempt on the air.
    in_flight: Vec<usize>,
    /// Failed attempts so far (drives CW doubling and rate step-down).
    attempts: u32,
    /// RTS/CTS handshake completed for this attempt.
    cts_ok: bool,
    /// MCS of the attempt in flight (rate-control bookkeeping).
    mcs: u8,
    /// The batch started with several MPDUs: they travel as QoS Data.
    qos: bool,
    /// A station identification frame (broadcast, MCS 0, never acked).
    ident: bool,
}

enum Response {
    Ndp(u64),
    Frame(Vec<u8>),
}

/// An attempt planned for the current batch.
struct Attempt {
    in_flight: Vec<usize>,
    psdu: Vec<u8>,
    aggregated: bool,
    /// The single MPDU when the attempt carries one (for the Ack ID).
    single: Option<Vec<u8>>,
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
    /// The last PPDU ended in PHY-RXEND(FormatViolation): the next access
    /// waits EIFS instead of DIFS [10.3.7], until an error-free frame
    /// resynchronises us.
    eifs_pending: bool,
    cw_exp: u32,
    dedup: HashMap<MacAddr, VecDeque<u16>>,
    rng: u64,
    rate: RateControl,
    // ---- station identification ----
    last_ident_us: Option<u64>,
    /// Data went out since the last identification.
    sent_since_ident: bool,
    last_data_tx_us: u64,
    /// CFO the PHY measured on the PSDU being processed, Hz.
    rx_cfo_hz: f32,
    /// Frames the filter dropped on the way to the air / from the air.
    filtered_egress: u64,
    filtered_ingress: u64,
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
        let rate = RateControl::new(cfg.rate.clone());
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
            eifs_pending: false,
            cw_exp,
            dedup: HashMap::new(),
            rng,
            rate,
            last_ident_us: None,
            sent_since_ident: false,
            last_data_tx_us: 0,
            rx_cfo_hz: 0.0,
            filtered_egress: 0,
            filtered_ingress: 0,
        }
    }

    pub fn config(&self) -> &MacConfig {
        &self.cfg
    }

    /// True while CCA, NAV or RID forbids transmission.
    pub fn medium_busy(&self, now_us: u64) -> bool {
        self.cca_busy || now_us < self.busy_until_us || now_us < self.nav_until_us || now_us < self.rid_until_us
    }

    /// EIFS for an S1G STA [10.3.7]: aSIFSTime + NDPTxTime + DIFS.
    pub fn eifs_us(&self) -> u64 {
        A_SIFS_TIME_US as u64 + NDP_TX_TIME_US as u64 + self.cfg.difs_us
    }

    /// MCS the rate controller currently uses for `peer` (`None`: unknown
    /// peer, or rate control disabled).
    pub fn peer_mcs(&self, peer: &MacAddr) -> Option<u8> {
        self.cfg.rate.enabled.then(|| self.rate.current(peer)).flatten()
    }

    /// The per-peer rate controller (statistics, SNR hints).
    pub fn rate_control(&self) -> &RateControl {
        &self.rate
    }

    /// Frames waiting in the transmit queue (not counting the batch in
    /// progress).
    pub fn queued(&self) -> usize {
        self.queue.len()
    }

    /// Frames the good-neighbour filter dropped: (towards the air, from the air).
    pub fn filtered(&self) -> (u64, u64) {
        (self.filtered_egress, self.filtered_ingress)
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

    fn next_seq(&mut self) -> u16 {
        let s = self.seq;
        self.seq = (self.seq + 1) & 0x0fff;
        s
    }

    /// Queue an outgoing Ethernet frame (from the TAP).
    pub fn enqueue_eth(&mut self, eth_frame: &[u8]) -> Result<(), MacError> {
        if self.cfg.filter.egress {
            if let Verdict::Drop(reason) = filter::check(&self.cfg.filter, eth_frame) {
                self.filtered_egress += 1;
                return Err(MacError::Filtered(reason));
            }
        }
        let (dest, src, ethertype, payload) = eth::parse_ethernet(eth_frame).ok_or(MacError::BadEthernet)?;
        if self.queue.len() >= self.cfg.queue_limit {
            return Err(MacError::QueueFull);
        }
        // Pre-flight size check at the lowest rate the frame may go out at.
        let body = eth::to_body(ethertype, payload);
        let mpdu_len = frame::QOS_DATA_HDR_LEN + body.len() + 4;
        let floor_mcs = if self.cfg.rate.enabled { self.cfg.rate.min_mcs.min(self.cfg.mcs) } else { self.cfg.mcs };
        if mpdu_len > 511 {
            let pre = ampdu::pre_eof_len(mpdu_len);
            if mpdu_len > ampdu::MAX_MPDU_LEN || s2g_phy::tx::aggregated_capacity(floor_mcs, pre, self.cfg.fec_coding).is_err() {
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
        if seqs.len() > 64 {
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
            RxEvent::NdpReceived { body, metrics, .. } => {
                let f = NdpFrame::parse(*body);
                self.rx_cfo_hz = metrics.cfo_hz;
                self.on_ndp(&f, metrics.snr_db, now_us, out);
                out.push(MacEvent::NdpReceived { frame: f });
            }
            RxEvent::RxEnd { status: RxEndStatus::FormatViolation, .. } => {
                // An S1G STA uses EIFS only after a FormatViolation [10.3.7].
                self.eifs_pending = true;
            }
            RxEvent::RxEnd { .. } => {}
            RxEvent::PsduReceived { rxvector, psdu, metrics, .. } => {
                self.rx_cfo_hz = metrics.cfo_hz;
                let (mpdus, s_mpdu): (Vec<Vec<u8>>, bool) = if rxvector.aggregation {
                    let with_eof = ampdu::deaggregate_with_eof(psdu);
                    let s = with_eof.len() == 1 && with_eof[0].1;
                    (with_eof.into_iter().map(|(m, _)| m).collect(), s)
                } else {
                    // Tolerate chips that pad the PSDU after the FCS.
                    (vec![frame::locate_mpdu(psdu).unwrap_or(psdu).to_vec()], false)
                };
                // An S-MPDU follows non-aggregated rules [10.12.8]: it is
                // acknowledged with an (NDP) Ack, not a BlockAck. A genuine
                // A-MPDU gets one NDP BlockAck covering every MPDU we got.
                let block_ack = rxvector.aggregation && !s_mpdu;
                let mut ba_src = None;
                let mut ba_seqs = Vec::new();
                for mpdu in mpdus {
                    if let Some((src, seq)) = self.on_mpdu(&mpdu, rxvector, block_ack, now_us, out) {
                        ba_src = Some(src);
                        ba_seqs.push(seq);
                    }
                }
                if let Some(src) = ba_src {
                    self.queue_block_ack(rxvector, src, &ba_seqs);
                }
            }
        }
    }

    /// RID contribution of an NDP CMAC PPDU [Table 10-2]: its RESPONSE
    /// INDICATION is implied by the frame type (and, for an NDP Ack, by the
    /// Idle Indication / Duration fields).
    fn ndp_rid_length_us(&self, f: &NdpFrame) -> u64 {
        let sifs = A_SIFS_TIME_US as u64;
        match f {
            NdpFrame::Ack(a) if a.idle_indication && a.duration == 0 => self.cfg.long_tx_time_us + sifs,
            NdpFrame::Ack(_) | NdpFrame::BlockAck(_) | NdpFrame::Cts(_) => 0,
            NdpFrame::Other { ndp_type, .. } => match *ndp_type {
                ndp::TYPE_PS_POLL | ndp::TYPE_PROBE_REQUEST => NDP_TX_TIME_US as u64 + sifs,
                ndp::TYPE_BF_REPORT_POLL => self.cfg.long_tx_time_us + sifs,
                _ => 0,
            },
        }
    }

    fn on_ndp(&mut self, f: &NdpFrame, snr_db: f32, now_us: u64, out: &mut Vec<MacEvent>) {
        // An NDP CMAC PPDU has no Data field, so it has just ended.
        let rid = self.ndp_rid_length_us(f);
        if rid > 0 {
            self.extend_rid(now_us + rid);
        }
        match f {
            NdpFrame::Ack(a) => {
                let expected = matches!(&self.state, TxState::AwaitResponse { expect: Expect::NdpAck { ack_id }, .. } if *ack_id == a.ack_id);
                if expected {
                    if let Some(c) = &self.cur {
                        self.rate.observe_snr(&c.dest, snr_db);
                        self.rate.observe_cfo(&c.dest, self.rx_cfo_hz);
                    }
                    self.resolve_attempt(|_| true, now_us, out);
                }
                if !a.idle_indication && a.duration > 0 {
                    self.nav_until_us = self.nav_until_us.max(now_us + a.duration as u64);
                }
            }
            NdpFrame::BlockAck(ba) => {
                let expected = matches!(&self.state, TxState::AwaitResponse { expect: Expect::NdpBlockAck { block_ack_id }, .. } if *block_ack_id == ba.block_ack_id);
                if expected {
                    if let Some(c) = &self.cur {
                        self.rate.observe_snr(&c.dest, snr_db);
                    }
                    let ssn = ba.starting_sequence;
                    let bitmap = ba.bitmap;
                    self.resolve_attempt(
                        move |seq| {
                            let d = (seq.wrapping_sub(ssn)) & 0x0fff;
                            d < 16 && bitmap & (1 << d) != 0
                        },
                        now_us,
                        out,
                    );
                }
            }
            NdpFrame::Cts(c) => {
                let for_us = !c.address_indicator && c.ra_pbssid == self.cfg.partial_aid();
                if for_us {
                    if let Some(cur) = &self.cur {
                        self.rate.observe_snr(&cur.dest, snr_db);
                    }
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

    /// The response (or timeout) for the attempt on the air has arrived:
    /// `acked(seq)` says which in-flight MPDUs were acknowledged. Completes
    /// those, retries or drops the rest, and reports to rate control.
    fn resolve_attempt(&mut self, acked: impl Fn(u16) -> bool, now_us: u64, out: &mut Vec<MacEvent>) {
        let max_retries = self.cfg.max_retries;
        let Some(cur) = self.cur.as_mut() else { return };
        let (dest, mcs) = (cur.dest, cur.mcs);
        let in_flight = std::mem::take(&mut cur.in_flight);
        let mut n_acked = 0usize;
        let mut kept = Vec::with_capacity(cur.mpdus.len());
        for (i, mut m) in cur.mpdus.drain(..).enumerate() {
            let flown = in_flight.contains(&i);
            if flown && acked(m.seq) {
                n_acked += 1;
                out.push(MacEvent::TxComplete { dest, acked: true, retries: m.retries, mcs });
                continue;
            }
            if flown {
                // Only MPDUs that were on the air and went unacknowledged
                // burn a retry.
                m.retries += 1;
                if m.retries > max_retries {
                    out.push(MacEvent::TxDropped { dest, reason: "retry limit" });
                    continue;
                }
            }
            kept.push(m);
        }
        cur.mpdus = kept;
        let failed = n_acked < in_flight.len();
        // A batch whose MPDUs mostly got through counts as a success for
        // the rate in use; losing most of them counts against it.
        let success = n_acked * 2 >= in_flight.len().max(1);
        self.rate.report(&dest, mcs, success);
        let all_done = self.cur.as_ref().is_some_and(|c| c.mpdus.is_empty());
        if all_done {
            self.cur = None;
            self.state = TxState::Idle;
            self.cw_exp = self.cfg.cw_min_exp;
            return;
        }
        let cur = self.cur.as_mut().expect("batch continues");
        cur.cts_ok = false;
        if failed {
            cur.attempts += 1;
            self.cw_exp = (self.cw_exp + 1).min(self.cfg.cw_max_exp);
        } else {
            // Everything on the air was acknowledged; the rest of the
            // batch did not fit and starts a fresh access.
            self.cw_exp = self.cfg.cw_min_exp;
        }
        self.start_backoff(now_us);
    }

    /// Handle one received MPDU. Returns `Some((src, seq))` for a data
    /// MPDU addressed to us inside an A-MPDU that a BlockAck must cover.
    fn on_mpdu(&mut self, mpdu: &[u8], rxv: &RxVector, block_ack: bool, now_us: u64, out: &mut Vec<MacEvent>) -> Option<(MacAddr, u16)> {
        let parsed = frame::parse(mpdu);
        if parsed.is_ok() {
            // An error-free frame resynchronises us: back to DIFS [10.3.7].
            self.eifs_pending = false;
        }
        match parsed {
            Ok(ParsedFrame::Data { dest, src, seq, duration_us, body, .. }) => {
                return self.on_data(dest, src, seq, duration_us, false, &body, rxv, block_ack, mpdu, now_us, out);
            }
            Ok(ParsedFrame::Pv1 { ptype, a1, a2, seq, no_ack, body, .. }) => {
                // PV1 QoS Data with full MAC addresses is deliverable; frames
                // that identify a station by AID cannot be resolved without an
                // association and are dropped (received, counted as valid).
                if let (frame::PV1_TYPE_QOS_DATA_MAC, Pv1Addr::Mac(dest), Pv1Addr::Mac(src), Some(seq)) = (ptype, a1, a2, seq) {
                    return self.on_data(dest, src, seq, 0, no_ack, &body, rxv, block_ack, mpdu, now_us, out);
                }
            }
            Ok(ParsedFrame::Ack { ra }) => {
                if ra == self.cfg.addr && matches!(&self.state, TxState::AwaitResponse { expect: Expect::Ack, .. }) {
                    self.resolve_attempt(|_| true, now_us, out);
                }
            }
            Ok(ParsedFrame::Rts { ra, ta, duration_us }) => {
                if ra == self.cfg.addr {
                    self.rate.observe_snr(&ta, rxv.snr_db);
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
        None
    }

    /// Common handling of a data MPDU (PV0 Data / QoS Data or PV1 QoS Data).
    #[allow(clippy::too_many_arguments)]
    fn on_data(
        &mut self,
        dest: MacAddr,
        src: MacAddr,
        seq: u16,
        duration_us: u16,
        no_ack: bool,
        body: &[u8],
        rxv: &RxVector,
        block_ack: bool,
        mpdu: &[u8],
        now_us: u64,
        out: &mut Vec<MacEvent>,
    ) -> Option<(MacAddr, u16)> {
        if src == self.cfg.addr {
            return None; // our own transmission looping back
        }
        self.rate.observe_snr(&src, rxv.snr_db);
        self.rate.observe_cfo(&src, self.rx_cfo_hz);
        let for_us = dest == self.cfg.addr;
        if !for_us && !is_group(&dest) {
            if duration_us > 0 {
                self.nav_until_us = self.nav_until_us.max(now_us + duration_us as u64);
            }
            return None; // someone else's unicast
        }
        let mut needs_block_ack = None;
        if for_us {
            // Addressed to us: we are the responder, so the RID does not
            // apply to us [10.3.2.5.1].
            self.rid_until_us = 0;
            if self.cfg.ack_enabled && !no_ack {
                if block_ack {
                    needs_block_ack = Some((src, seq));
                } else {
                    // ACK even duplicates — the peer may have missed our ACK.
                    self.queue_ack(rxv, mpdu, src);
                }
            }
        }
        if self.note_duplicate(src, seq) {
            return needs_block_ack;
        }
        if let Some(text) = ident::parse_body(body) {
            out.push(MacEvent::IdentReceived { src, text });
        } else if let Some(ethf) = eth::body_to_ethernet(dest, src, body) {
            match if self.cfg.filter.ingress { filter::check(&self.cfg.filter, &ethf) } else { Verdict::Pass } {
                Verdict::Pass => out.push(MacEvent::EthReceived(ethf)),
                Verdict::Drop(reason) => {
                    self.filtered_ingress += 1;
                    out.push(MacEvent::Filtered { reason });
                }
            }
        }
        needs_block_ack
    }

    /// Queue the acknowledgement a single MPDU / S-MPDU asked for
    /// [Table 10-7, 10.12.8].
    fn queue_ack(&mut self, rxv: &RxVector, mpdu: &[u8], src: MacAddr) {
        match rxv.response_indication {
            ResponseIndication::Ndp => {
                let f = NdpFrame::Ack(NdpAck {
                    ack_id: ndp::ack_id_for_mpdu(rxv.scrambler_seed, mpdu),
                    more_data: false,
                    idle_indication: false,
                    duration: 0,
                    relayed_frame: false,
                });
                self.responses.push_back(Response::Ndp(f.to_body()));
            }
            ResponseIndication::Normal => self.responses.push_back(Response::Frame(frame::build_ack(src))),
            ResponseIndication::None | ResponseIndication::Long => {}
        }
    }

    /// Queue the NDP BlockAck for a received A-MPDU: bit i of the bitmap
    /// acknowledges sequence number `ssn + i`, with `ssn` the first MPDU we
    /// received [23.3.12.2.6.2].
    fn queue_block_ack(&mut self, rxv: &RxVector, src: MacAddr, seqs: &[u16]) {
        if seqs.is_empty() {
            return;
        }
        match rxv.response_indication {
            ResponseIndication::Ndp => {
                let ssn = seqs[0];
                let mut bitmap = 0u16;
                for &s in seqs {
                    let d = s.wrapping_sub(ssn) & 0x0fff;
                    if d < 16 {
                        bitmap |= 1 << d;
                    }
                }
                let f = NdpFrame::BlockAck(NdpBlockAck { block_ack_id: ndp::block_ack_id(rxv.scrambler_seed), starting_sequence: ssn, bitmap });
                self.responses.push_back(Response::Ndp(f.to_body()));
            }
            // A legacy BlockAck frame is not implemented; a Normal Response
            // A-MPDU gets a plain Ack for its first MPDU, which is at least
            // something a legacy sender can act on.
            ResponseIndication::Normal => self.responses.push_back(Response::Frame(frame::build_ack(src))),
            ResponseIndication::None | ResponseIndication::Long => {}
        }
    }

    fn start_backoff(&mut self, now_us: u64) {
        let cw = (1u64 << self.cw_exp) - 1;
        let slots = (self.rand_u32() as u64) % (cw + 1);
        let idle_at = now_us.max(self.busy_until_us).max(self.nav_until_us).max(self.rid_until_us);
        let ifs = if self.eifs_pending { self.eifs_us() } else { self.cfg.difs_us };
        self.eifs_pending = false;
        let base = idle_at + ifs;
        self.state = TxState::Backoff { until_us: base + slots * self.cfg.slot_us };
    }

    /// Serialize one MPDU of the current batch.
    fn build_mpdu(cur: &CurrentTx, i: usize, duration_us: u16) -> Vec<u8> {
        let m = &cur.mpdus[i];
        if cur.qos {
            frame::build_qos_data(cur.dest, cur.src, m.seq, m.retries > 0, duration_us, 0, &m.body)
        } else {
            frame::build_data(cur.dest, cur.src, m.seq, m.retries > 0, duration_us, &m.body)
        }
    }

    /// Choose which outstanding MPDUs go into the next attempt at `mcs`
    /// and build the PSDU: a plain MPDU, an S-MPDU, or an A-MPDU of as many
    /// MPDUs as the PPDU can carry.
    fn plan_attempt(&self, cur: &CurrentTx, mcs: u8, duration_us: u16) -> Result<Attempt, MacError> {
        let coding = self.cfg.fec_coding;
        let mut chosen: Vec<usize> = Vec::new();
        let mut built: Vec<Vec<u8>> = Vec::new();
        for i in 0..cur.mpdus.len().min(AMPDU_MAX_MPDUS) {
            let m = Self::build_mpdu(cur, i, duration_us);
            let mut lens: Vec<usize> = built.iter().map(|b| b.len()).collect();
            lens.push(m.len());
            let fits = if lens.len() == 1 {
                m.len() <= 511 || s2g_phy::tx::aggregated_capacity(mcs, ampdu::pre_eof_len(m.len()), coding).is_ok()
            } else {
                s2g_phy::tx::aggregated_capacity(mcs, ampdu::pre_eof_len_many(&lens), coding).is_ok()
            };
            if !fits {
                break;
            }
            chosen.push(i);
            built.push(m);
        }
        if built.is_empty() {
            return Err(MacError::FrameTooBig);
        }
        if built.len() == 1 {
            let m = built.pop().unwrap();
            if m.len() <= 511 {
                return Ok(Attempt { in_flight: chosen, psdu: m.clone(), aggregated: false, single: Some(m) });
            }
            let cap = s2g_phy::tx::aggregated_capacity(mcs, ampdu::pre_eof_len(m.len()), coding)?;
            return Ok(Attempt { in_flight: chosen, psdu: ampdu::aggregate(&m, cap), aggregated: true, single: Some(m) });
        }
        let lens: Vec<usize> = built.iter().map(|b| b.len()).collect();
        let cap = s2g_phy::tx::aggregated_capacity(mcs, ampdu::pre_eof_len_many(&lens), coding)?;
        let refs: Vec<&[u8]> = built.iter().map(|b| b.as_slice()).collect();
        Ok(Attempt { in_flight: chosen, psdu: ampdu::aggregate_many(&refs, cap), aggregated: true, single: None })
    }

    fn fail_attempt(&mut self, now_us: u64, out: &mut Vec<MacEvent>) {
        let waiting_for_cts = matches!(self.state, TxState::AwaitResponse { expect: Expect::NdpCts { .. }, .. });
        if waiting_for_cts {
            // An unanswered RTS went out at MCS 0: it says nothing about the
            // data rate, but it does burn a retry so a dead peer is given up.
            let Some(cur) = self.cur.as_mut() else { return };
            let dest = cur.dest;
            let mut still = Vec::new();
            for mut m in cur.mpdus.drain(..) {
                m.retries += 1;
                if m.retries > self.cfg.max_retries {
                    out.push(MacEvent::TxDropped { dest, reason: "retry limit" });
                } else {
                    still.push(m);
                }
            }
            cur.mpdus = still;
            cur.in_flight.clear();
            cur.cts_ok = false;
            if cur.mpdus.is_empty() {
                self.cur = None;
                self.state = TxState::Idle;
                self.cw_exp = self.cfg.cw_min_exp;
            } else {
                cur.attempts += 1;
                self.cw_exp = (self.cw_exp + 1).min(self.cfg.cw_max_exp);
                self.start_backoff(now_us);
            }
            return;
        }
        self.resolve_attempt(|_| false, now_us, out);
    }

    /// Identification is due when a communication starts or resumes after
    /// `interval_us`, or when one ends (`end_idle_us` of silence after data
    /// went out).
    fn ident_due(&self, now_us: u64) -> bool {
        if self.cfg.ident.callsign.is_none() {
            return false;
        }
        let since_ident = self.last_ident_us.map(|t| now_us.saturating_sub(t));
        let start_due = !self.queue.is_empty() && since_ident.is_none_or(|d| d >= self.cfg.ident.interval_us);
        let end_due = self.queue.is_empty()
            && self.sent_since_ident
            && now_us.saturating_sub(self.last_data_tx_us) >= self.cfg.ident.end_idle_us
            && since_ident.is_none_or(|d| d >= 60_000_000);
        start_due || end_due
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

        // 3. Pull new work: an identification frame when due, else a batch
        //    of queued frames for one destination.
        if matches!(self.state, TxState::Idle) && self.cur.is_none() {
            if self.ident_due(now_us) {
                let call = self.cfg.ident.callsign.clone().unwrap_or_default();
                let body = ident::body(&call, &self.cfg.ident.info);
                let seq = self.next_seq();
                self.cur = Some(CurrentTx {
                    dest: frame::BROADCAST,
                    src: self.cfg.addr,
                    mpdus: vec![PendingMpdu { seq, body, retries: 0 }],
                    in_flight: Vec::new(),
                    attempts: 0,
                    cts_ok: false,
                    mcs: 0,
                    qos: false,
                    ident: true,
                });
                self.start_backoff(now_us);
            } else if let Some((dest, src, body)) = self.queue.pop_front() {
                let can_aggregate = self.cfg.ack_enabled && self.cfg.ndp_ack && !is_group(&dest);
                let max = if can_aggregate { self.cfg.ampdu_max_mpdus.clamp(1, AMPDU_MAX_MPDUS) } else { 1 };
                let seq = self.next_seq();
                let mut mpdus = vec![PendingMpdu { seq, body, retries: 0 }];
                while mpdus.len() < max && self.queue.front().is_some_and(|(d, s, _)| *d == dest && *s == src) {
                    let (_, _, body) = self.queue.pop_front().unwrap();
                    let seq = self.next_seq();
                    mpdus.push(PendingMpdu { seq, body, retries: 0 });
                }
                let qos = mpdus.len() > 1;
                self.cur = Some(CurrentTx { dest, src, mpdus, in_flight: Vec::new(), attempts: 0, cts_ok: false, mcs: self.cfg.mcs, qos, ident: false });
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
            let want_ack = self.cfg.ack_enabled && !is_group(&cur.dest) && !cur.ident;
            let sifs = A_SIFS_TIME_US as u64;
            let resp_time = if self.cfg.ndp_ack { NDP_TX_TIME_US as u64 } else { normal_tx_time_us(14) };
            let hdr = if cur.qos { frame::QOS_DATA_HDR_LEN } else { frame::DATA_HDR_LEN };
            let total_len: usize = cur.mpdus.iter().map(|m| hdr + m.body.len() + 4).sum();
            let need_rts = want_ack && !cur.cts_ok && self.cfg.rts_threshold.is_some_and(|t| total_len > t);
            if need_rts {
                // Duration covers CTS + data + response [9.2.5.2], at the
                // rate the data is expected to go out with.
                let est_mcs = if self.cfg.rate.enabled { self.rate.current(&cur.dest).unwrap_or(self.cfg.mcs) } else { self.cfg.mcs };
                let data_time = s2g_phy::tx::txtime_us_coded(est_mcs, total_len.min(511), total_len > 511, self.cfg.fec_coding).unwrap_or(10_000) as u64;
                let duration = sifs + NDP_TX_TIME_US as u64 + sifs + data_time + sifs + resp_time;
                let psdu = frame::build_rts(cur.dest, cur.src, duration.min(0x7fff) as u16);
                let seed = self.pick_seed();
                let txv = TxVector { mcs: 0, response_indication: ResponseIndication::Ndp, scrambler_seed: Some(seed), ..Default::default() };
                let airtime = s2g_phy::tx::txtime_us(0, psdu.len(), false).unwrap_or(1000) as u64;
                let partial_aid = self.cfg.partial_aid();
                self.state = TxState::AwaitResponse { deadline_us: now_us + airtime + self.cfg.ack_timeout_us, expect: Expect::NdpCts { partial_aid } };
                return Some(MacAction::Transmit { txv, psdu });
            }
            // Rate control picks the MCS of this attempt (failed attempts
            // step down); broadcast, identification and unacknowledged
            // frames use the fixed rates.
            let mcs = if cur.ident {
                0
            } else if want_ack && self.cfg.rate.enabled {
                self.rate.select(&cur.dest, cur.attempts)
            } else {
                self.cfg.mcs
            };
            self.cur.as_mut().expect("Backoff implies cur").mcs = mcs;
            let cur = self.cur.as_ref().expect("Backoff implies cur");
            let duration = if want_ack { (sifs + resp_time).min(0x7fff) as u16 } else { 0 };
            let attempt = match self.plan_attempt(cur, mcs, duration) {
                Ok(a) => a,
                Err(_) => {
                    let cur = self.cur.take().unwrap();
                    for _ in &cur.mpdus {
                        out.push(MacEvent::TxDropped { dest: cur.dest, reason: "PSDU build failed" });
                    }
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
                mcs,
                fec_coding: self.cfg.fec_coding,
                traveling_pilots: self.cfg.traveling_pilots,
                aggregation: attempt.aggregated,
                response_indication,
                scrambler_seed: Some(seed),
                ..Default::default()
            };
            let airtime = s2g_phy::tx::txtime_us_coded(mcs, attempt.psdu.len(), attempt.aggregated, self.cfg.fec_coding).unwrap_or(10_000) as u64;
            let cur = self.cur.as_mut().expect("Backoff implies cur");
            cur.in_flight = attempt.in_flight.clone();
            if want_ack {
                // A single MPDU or S-MPDU is acknowledged like a single MPDU
                // [10.12.8]; a multi-MPDU A-MPDU gets an NDP BlockAck.
                let expect = match (&attempt.single, self.cfg.ndp_ack) {
                    (Some(m), true) => Expect::NdpAck { ack_id: ndp::ack_id_for_mpdu(seed, m) },
                    (None, true) => Expect::NdpBlockAck { block_ack_id: ndp::block_ack_id(seed) },
                    (_, false) => Expect::Ack,
                };
                self.state = TxState::AwaitResponse { deadline_us: now_us + airtime + self.cfg.ack_timeout_us, expect };
                self.sent_since_ident = true;
                self.last_data_tx_us = now_us;
            } else {
                let cur = self.cur.take().unwrap();
                if cur.ident {
                    let text = String::from_utf8_lossy(&cur.mpdus[0].body[8..]).into_owned();
                    out.push(MacEvent::IdentSent { text });
                    self.last_ident_us = Some(now_us);
                    self.sent_since_ident = false;
                } else {
                    for m in &cur.mpdus {
                        out.push(MacEvent::TxComplete { dest: cur.dest, acked: false, retries: m.retries, mcs });
                    }
                    self.sent_since_ident = true;
                    self.last_data_tx_us = now_us;
                }
                self.state = TxState::Idle;
            }
            return Some(MacAction::Transmit { txv, psdu: attempt.psdu });
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
    fn large_frame_goes_as_s_mpdu_and_expects_ndp_ack() {
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
        // PSDU fills the symbol capacity exactly and is an S-MPDU.
        let cap = s2g_phy::tx::aggregated_capacity(5, psdu.len(), Coding::Bcc).unwrap();
        assert_eq!(psdu.len(), cap);
        assert!(ampdu::is_s_mpdu(&psdu));
        // And deaggregates back to one valid data frame.
        let mpdus = ampdu::deaggregate(&psdu);
        assert_eq!(mpdus.len(), 1);
        assert!(matches!(frame::parse(&mpdus[0]).unwrap(), ParsedFrame::Data { .. }));
        // The receiving MAC answers an S-MPDU with an NDP Ack [10.12.8]...
        let mut mac_b = Mac::new(MacConfig::new(B));
        let mut out_b = Vec::new();
        mac_b.on_phy_event(&psdu_event(&txv, &psdu), now, &mut out_b);
        let Some(MacAction::TransmitNdp { body }) = mac_b.poll(now, &mut out_b) else { panic!() };
        let NdpFrame::Ack(a) = NdpFrame::parse(body) else { panic!("expected NDP Ack, got {:?}", NdpFrame::parse(body)) };
        assert_eq!(a.ack_id, ndp::ack_id_for_mpdu(seed, &mpdus[0]));
        // ...which completes the exchange at the sender.
        mac.on_phy_event(&RxEvent::NdpReceived { sample_index: 0, body, metrics: metrics() }, now, &mut out);
        mac.poll(now, &mut out);
        assert!(out.iter().any(|e| matches!(e, MacEvent::TxComplete { acked: true, .. })), "{out:?}");
        // A genuine multi-MPDU A-MPDU (EOF = 0) would still get an NDP BlockAck.
        let mpdu = frame::build_data(B, A, 9, false, 0, b"two");
        let mut two = Vec::new();
        two.extend_from_slice(&ampdu::build_delimiter(mpdu.len(), false));
        two.extend_from_slice(&mpdu);
        two.resize(two.len().div_ceil(4) * 4, 0);
        two.extend_from_slice(&ampdu::build_delimiter(mpdu.len(), false));
        two.extend_from_slice(&mpdu);
        let txv2 = TxVector { aggregation: true, response_indication: ResponseIndication::Ndp, scrambler_seed: Some(seed), ..Default::default() };
        let mut mac_c = Mac::new(MacConfig::new(B));
        let mut out_c = Vec::new();
        mac_c.on_phy_event(&psdu_event(&txv2, &two), now, &mut out_c);
        let Some(MacAction::TransmitNdp { body }) = mac_c.poll(now, &mut out_c) else { panic!() };
        assert!(matches!(NdpFrame::parse(body), NdpFrame::BlockAck(_)));
    }

    #[test]
    fn pv1_data_is_received_and_acked() {
        let mut mac_b = Mac::new(MacConfig::new(B));
        let mut out_b = Vec::new();
        let body = eth::to_body(0x0800, &[1, 2, 3, 4]);
        let f = frame::build_pv1_data(B, A, 3, 0, false, &body);
        let txv = TxVector { response_indication: ResponseIndication::Ndp, scrambler_seed: Some(21), ..Default::default() };
        mac_b.on_phy_event(&psdu_event(&txv, &f), 0, &mut out_b);
        assert!(out_b.iter().any(|e| matches!(e, MacEvent::EthReceived(_))), "{out_b:?}");
        let Some(MacAction::TransmitNdp { body: ndp_body }) = mac_b.poll(0, &mut out_b) else { panic!("no ack") };
        let NdpFrame::Ack(a) = NdpFrame::parse(ndp_body) else { panic!() };
        assert_eq!(a.ack_id, ndp::ack_id_for_mpdu(21, &f));
        // No Ack policy → delivered but not acknowledged.
        let f2 = frame::build_pv1_data(B, A, 4, 0, true, &body);
        mac_b.on_phy_event(&psdu_event(&txv, &f2), 0, &mut out_b);
        assert!(mac_b.poll(0, &mut out_b).is_none());
        assert_eq!(out_b.iter().filter(|e| matches!(e, MacEvent::EthReceived(_))).count(), 2);
    }

    #[test]
    fn eifs_after_format_violation() {
        let mut cfg = MacConfig::new(A);
        cfg.ack_enabled = false;
        cfg.cw_min_exp = 0; // no random backoff: measure the IFS alone
        cfg.cw_max_exp = 0;
        let mut mac = Mac::new(cfg.clone());
        let mut out = Vec::new();
        // Normal case: DIFS.
        mac.enqueue_eth(&eth_frame(BCAST, A, 20)).unwrap();
        assert!(mac.poll(0, &mut out).is_none());
        assert!(mac.poll(cfg.difs_us - 1, &mut out).is_none());
        assert!(mac.poll(cfg.difs_us, &mut out).is_some());
        // After a FormatViolation: EIFS = SIFS + NDPTxTime + DIFS.
        let mut mac2 = Mac::new(cfg.clone());
        mac2.on_phy_event(&RxEvent::RxEnd { sample_index: 0, status: RxEndStatus::FormatViolation }, 0, &mut out);
        mac2.enqueue_eth(&eth_frame(BCAST, A, 20)).unwrap();
        assert!(mac2.poll(0, &mut out).is_none());
        assert!(mac2.poll(mac2.eifs_us() - 1, &mut out).is_none());
        assert!(mac2.poll(mac2.eifs_us(), &mut out).is_some());
        assert_eq!(mac2.eifs_us(), 160 + 240 + cfg.difs_us);
        // An error-free frame in between resynchronises to DIFS.
        let mut mac3 = Mac::new(cfg.clone());
        mac3.on_phy_event(&RxEvent::RxEnd { sample_index: 0, status: RxEndStatus::FormatViolation }, 0, &mut out);
        let good = frame::build_data(B, [2, 0, 0, 0, 0, 0xC], 1, false, 0, b"ok");
        mac3.on_phy_event(&psdu_event(&TxVector::default(), &good), 0, &mut out);
        mac3.enqueue_eth(&eth_frame(BCAST, A, 20)).unwrap();
        assert!(mac3.poll(0, &mut out).is_none());
        assert!(mac3.poll(cfg.difs_us, &mut out).is_some());
    }

    #[test]
    fn rid_from_ndp_frames_per_table_10_2() {
        let mut cfg = MacConfig::new(A);
        cfg.ack_enabled = false;
        let mut mac = Mac::new(cfg);
        let mut out = Vec::new();
        // NDP Ack with Idle Indication = 1 and Duration = 0 → Long Response.
        let a = NdpFrame::Ack(NdpAck { ack_id: 1, more_data: false, idle_indication: true, duration: 0, relayed_frame: false });
        mac.on_phy_event(&RxEvent::NdpReceived { sample_index: 0, body: a.to_body(), metrics: metrics() }, 1000, &mut out);
        assert!(mac.medium_busy(1000 + 3008 + 160 - 1));
        assert!(!mac.medium_busy(1000 + 3008 + 160 + 1));
        // NDP CTS → No Response.
        let mut mac2 = Mac::new({
            let mut c = MacConfig::new(A);
            c.ack_enabled = false;
            c
        });
        let cts = NdpFrame::Cts(NdpCts { address_indicator: true, ra_pbssid: 5, duration_us: 0, early_sector_indicator: false, bandwidth: 1 });
        mac2.on_phy_event(&RxEvent::NdpReceived { sample_index: 0, body: cts.to_body(), metrics: metrics() }, 1000, &mut out);
        assert!(!mac2.medium_busy(1001));
        // NDP PS-Poll (type 1) → NDP Response: 240 + 160.
        let ps = NdpFrame::Other { ndp_type: ndp::TYPE_PS_POLL, body: ndp::TYPE_PS_POLL as u64 };
        mac2.on_phy_event(&RxEvent::NdpReceived { sample_index: 0, body: ps.to_body(), metrics: metrics() }, 2000, &mut out);
        assert!(mac2.medium_busy(2000 + 399));
        assert!(!mac2.medium_busy(2000 + 401));
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

    /// Deliver an A-MPDU to B, dropping the MPDUs whose index is in `lose`
    /// (their FCS is corrupted), and return B's response body.
    fn deliver_ampdu(mac_b: &mut Mac, txv: &TxVector, psdu: &[u8], lose: &[usize], now: u64, out_b: &mut Vec<MacEvent>) -> Option<u64> {
        let mut psdu = psdu.to_vec();
        // Corrupt the last byte of each lost MPDU in place.
        let mut pos = 0usize;
        let mut idx = 0usize;
        while pos + 4 <= psdu.len() {
            match ampdu::parse_delimiter(&psdu[pos..pos + 4]) {
                Some((0, _)) => pos += 4,
                Some((len, _)) => {
                    if lose.contains(&idx) {
                        psdu[pos + 4 + len - 1] ^= 0xff;
                    }
                    idx += 1;
                    pos = (pos + 4 + len).div_ceil(4) * 4;
                }
                None => pos += 4,
            }
        }
        mac_b.on_phy_event(&psdu_event(txv, &psdu), now, out_b);
        match mac_b.poll(now, out_b) {
            Some(MacAction::TransmitNdp { body }) => Some(body),
            _ => None,
        }
    }

    #[test]
    fn frames_for_one_peer_are_aggregated_and_block_acked() {
        let mut cfg = MacConfig::new(A);
        cfg.mcs = 4;
        cfg.ampdu_max_mpdus = 8;
        let mut mac = Mac::new(cfg);
        let mut out = Vec::new();
        for i in 0..5 {
            mac.enqueue_eth(&eth_frame(B, A, 100 + i)).unwrap();
        }
        mac.enqueue_eth(&eth_frame([2, 0, 0, 0, 0, 0xC], A, 50)).unwrap(); // another peer: not packed
        let mut now = 0;
        let Some(MacAction::Transmit { txv, psdu }) = drain_tx(&mut mac, &mut now, &mut out) else { panic!() };
        assert!(txv.aggregation);
        assert_eq!(txv.response_indication, ResponseIndication::Ndp);
        let seed = txv.scrambler_seed.unwrap();
        let mpdus = ampdu::deaggregate_with_eof(&psdu);
        assert_eq!(mpdus.len(), 5, "all five frames in one A-MPDU");
        assert!(mpdus.iter().all(|(_, eof)| !eof), "EOF = 0 on real MPDUs");
        for (i, (m, _)) in mpdus.iter().enumerate() {
            match frame::parse(m).unwrap() {
                ParsedFrame::Data { seq, tid, retry, .. } => {
                    assert_eq!(seq, i as u16);
                    assert_eq!(tid, Some(0), "QoS Data inside an A-MPDU");
                    assert!(!retry);
                }
                other => panic!("{other:?}"),
            }
        }
        assert_eq!(psdu.len(), s2g_phy::tx::aggregated_capacity(4, psdu.len(), Coding::Bcc).unwrap());
        assert_eq!(mac.queued(), 1);

        // B receives all five, delivers them once, and answers with ONE NDP
        // BlockAck whose bitmap covers sequence numbers 0..5.
        let mut mac_b = Mac::new(MacConfig::new(B));
        let mut out_b = Vec::new();
        let body = deliver_ampdu(&mut mac_b, &txv, &psdu, &[], now, &mut out_b).expect("block ack");
        let NdpFrame::BlockAck(ba) = NdpFrame::parse(body) else { panic!("{:?}", NdpFrame::parse(body)) };
        assert_eq!(ba.block_ack_id, ndp::block_ack_id(seed));
        assert_eq!(ba.starting_sequence, 0);
        assert_eq!(ba.bitmap, 0b11111);
        assert!(mac_b.poll(now, &mut out_b).is_none(), "exactly one response");
        assert_eq!(out_b.iter().filter(|e| matches!(e, MacEvent::EthReceived(_))).count(), 5);

        // The BlockAck completes all five at A.
        mac.on_phy_event(&RxEvent::NdpReceived { sample_index: 0, body, metrics: metrics() }, now, &mut out);
        mac.poll(now, &mut out);
        assert_eq!(out.iter().filter(|e| matches!(e, MacEvent::TxComplete { acked: true, .. })).count(), 5);
        // …and the frame for the other peer follows on its own.
        let Some(MacAction::Transmit { txv, .. }) = drain_tx(&mut mac, &mut now, &mut out) else { panic!() };
        assert!(!txv.aggregation);
    }

    #[test]
    fn block_ack_bitmap_drives_selective_retransmission() {
        let mut cfg = MacConfig::new(A);
        cfg.mcs = 2;
        cfg.max_retries = 2;
        cfg.ack_timeout_us = 1000;
        let mut mac = Mac::new(cfg);
        let mut out = Vec::new();
        for i in 0..4 {
            mac.enqueue_eth(&eth_frame(B, A, 60 + i)).unwrap();
        }
        let mut now = 0;
        let Some(MacAction::Transmit { txv, psdu }) = drain_tx(&mut mac, &mut now, &mut out) else { panic!() };
        assert_eq!(ampdu::deaggregate(&psdu).len(), 4);
        // B loses MPDUs 1 and 3 (sequence numbers 1 and 3).
        let mut mac_b = Mac::new(MacConfig::new(B));
        let mut out_b = Vec::new();
        let body = deliver_ampdu(&mut mac_b, &txv, &psdu, &[1, 3], now, &mut out_b).expect("block ack");
        let NdpFrame::BlockAck(ba) = NdpFrame::parse(body) else { panic!() };
        assert_eq!((ba.starting_sequence, ba.bitmap), (0, 0b0101));
        mac.on_phy_event(&RxEvent::NdpReceived { sample_index: 0, body, metrics: metrics() }, now, &mut out);
        mac.poll(now, &mut out);
        assert_eq!(out.iter().filter(|e| matches!(e, MacEvent::TxComplete { acked: true, .. })).count(), 2);
        // The retry carries exactly the two lost MPDUs, Retry bit set.
        let Some(MacAction::Transmit { txv: txv2, psdu: psdu2 }) = drain_tx(&mut mac, &mut now, &mut out) else { panic!() };
        let again = ampdu::deaggregate(&psdu2);
        assert_eq!(again.len(), 2);
        let seqs: Vec<u16> = again
            .iter()
            .map(|m| match frame::parse(m).unwrap() {
                ParsedFrame::Data { seq, retry, .. } => {
                    assert!(retry);
                    seq
                }
                other => panic!("{other:?}"),
            })
            .collect();
        assert_eq!(seqs, vec![1, 3]);
        // B gets both this time; its bitmap starts at the first received
        // sequence number (1) and marks 1 and 3.
        let body = deliver_ampdu(&mut mac_b, &txv2, &psdu2, &[], now, &mut out_b).expect("block ack");
        let NdpFrame::BlockAck(ba) = NdpFrame::parse(body) else { panic!() };
        assert_eq!((ba.starting_sequence, ba.bitmap), (1, 0b0101));
        mac.on_phy_event(&RxEvent::NdpReceived { sample_index: 0, body, metrics: metrics() }, now, &mut out);
        mac.poll(now, &mut out);
        assert_eq!(out.iter().filter(|e| matches!(e, MacEvent::TxComplete { acked: true, .. })).count(), 4);
        assert!(!out.iter().any(|e| matches!(e, MacEvent::TxDropped { .. })));
        // B delivered each frame exactly once, in order.
        let delivered: Vec<&Vec<u8>> = out_b
            .iter()
            .filter_map(|e| match e {
                MacEvent::EthReceived(f) => Some(f),
                _ => None,
            })
            .collect();
        assert_eq!(delivered.len(), 4);
    }

    #[test]
    fn unacknowledged_ampdu_retries_then_drops_every_mpdu() {
        let mut cfg = MacConfig::new(A);
        cfg.max_retries = 1;
        cfg.ack_timeout_us = 1000;
        let mut mac = Mac::new(cfg);
        let mut out = Vec::new();
        for i in 0..3 {
            mac.enqueue_eth(&eth_frame(B, A, 40 + i)).unwrap();
        }
        let mut now = 0;
        let mut transmissions = 0;
        for _ in 0..100_000 {
            if let Some(MacAction::Transmit { .. }) = mac.poll(now, &mut out) {
                transmissions += 1;
            }
            if out.iter().filter(|e| matches!(e, MacEvent::TxDropped { .. })).count() == 3 {
                break;
            }
            now += 500;
        }
        assert_eq!(transmissions, 2, "initial + 1 retry, each carrying the whole batch");
        assert_eq!(out.iter().filter(|e| matches!(e, MacEvent::TxDropped { reason: "retry limit", .. })).count(), 3);
        assert!(mac.poll(now + 1_000_000, &mut out).is_none());
    }

    #[test]
    fn aggregation_is_off_for_broadcast_legacy_ack_and_ampdu_1() {
        // Broadcast: one frame per PPDU.
        let mut mac = Mac::new(MacConfig::new(A));
        let mut out = Vec::new();
        mac.enqueue_eth(&eth_frame(BCAST, A, 30)).unwrap();
        mac.enqueue_eth(&eth_frame(BCAST, A, 31)).unwrap();
        let mut now = 0;
        let Some(MacAction::Transmit { txv, .. }) = drain_tx(&mut mac, &mut now, &mut out) else { panic!() };
        assert!(!txv.aggregation);
        assert_eq!(mac.queued(), 1);
        // Legacy Ack cannot cover an A-MPDU.
        let mut cfg = MacConfig::new(A);
        cfg.ndp_ack = false;
        let mut mac = Mac::new(cfg);
        mac.enqueue_eth(&eth_frame(B, A, 30)).unwrap();
        mac.enqueue_eth(&eth_frame(B, A, 31)).unwrap();
        let Some(MacAction::Transmit { txv, .. }) = drain_tx(&mut mac, &mut now, &mut out) else { panic!() };
        assert!(!txv.aggregation);
        assert_eq!(mac.queued(), 1);
        // ampdu_max_mpdus = 1 disables packing.
        let mut cfg = MacConfig::new(A);
        cfg.ampdu_max_mpdus = 1;
        let mut mac = Mac::new(cfg);
        mac.enqueue_eth(&eth_frame(B, A, 30)).unwrap();
        mac.enqueue_eth(&eth_frame(B, A, 31)).unwrap();
        let Some(MacAction::Transmit { txv, psdu }) = drain_tx(&mut mac, &mut now, &mut out) else { panic!() };
        assert!(!txv.aggregation);
        assert!(matches!(frame::parse(&psdu).unwrap(), ParsedFrame::Data { tid: None, .. }));
        assert_eq!(mac.queued(), 1);
    }

    #[test]
    fn a_batch_that_does_not_fit_one_ppdu_is_sent_in_pieces() {
        // MCS 0: a 511-symbol PPDU carries ~1660 octets; eight 300-octet
        // frames need two PPDUs. All are delivered without any retry.
        let mut cfg = MacConfig::new(A);
        cfg.mcs = 0;
        let mut mac = Mac::new(cfg);
        let mut mac_b = Mac::new(MacConfig::new(B));
        let (mut out, mut out_b) = (Vec::new(), Vec::new());
        for i in 0..8 {
            mac.enqueue_eth(&eth_frame(B, A, 300 + i)).unwrap();
        }
        let mut now = 0;
        let mut ppdus = 0;
        while let Some(MacAction::Transmit { txv, psdu }) = drain_tx(&mut mac, &mut now, &mut out) {
            ppdus += 1;
            assert!(ppdus < 10);
            let body = deliver_ampdu(&mut mac_b, &txv, &psdu, &[], now, &mut out_b).expect("response");
            mac.on_phy_event(&RxEvent::NdpReceived { sample_index: 0, body, metrics: metrics() }, now, &mut out);
        }
        assert!(ppdus >= 2, "{ppdus} PPDUs");
        assert_eq!(out.iter().filter(|e| matches!(e, MacEvent::TxComplete { acked: true, retries: 0, .. })).count(), 8);
        assert_eq!(out_b.iter().filter(|e| matches!(e, MacEvent::EthReceived(_))).count(), 8);
    }

    #[test]
    fn identification_precedes_data_repeats_and_closes() {
        let mut cfg = MacConfig::new(A);
        cfg.ident = IdentConfig { callsign: Some("w1aw".into()), info: "FN31".into(), interval_us: 600_000_000, end_idle_us: 30_000_000 };
        cfg.ack_enabled = false;
        let mut mac = Mac::new(cfg);
        let mut out = Vec::new();
        mac.enqueue_eth(&eth_frame(B, A, 40)).unwrap();
        let mut now = 0u64;
        // 1. The very first transmission is the identification, broadcast at MCS 0.
        let Some(MacAction::Transmit { txv, psdu }) = drain_tx(&mut mac, &mut now, &mut out) else { panic!() };
        assert_eq!(txv.mcs, 0);
        assert_eq!(txv.response_indication, ResponseIndication::None);
        let ParsedFrame::Data { dest, body, tid, .. } = frame::parse(&psdu).unwrap() else { panic!() };
        assert_eq!(dest, BCAST);
        assert_eq!(tid, None);
        assert_eq!(ident::parse_body(&body).as_deref(), Some("DE W1AW FN31"));
        assert!(out.iter().any(|e| matches!(e, MacEvent::IdentSent { text } if text == "DE W1AW FN31")));
        // 2. Then the data.
        let Some(MacAction::Transmit { psdu, .. }) = drain_tx(&mut mac, &mut now, &mut out) else { panic!() };
        assert!(matches!(frame::parse(&psdu).unwrap(), ParsedFrame::Data { dest, .. } if dest == B));
        // 3. Continuous traffic: no identification for 10 minutes, then one.
        let mut idents = 0;
        let t0 = now;
        while now < t0 + 11 * 60_000_000 {
            mac.enqueue_eth(&eth_frame(B, A, 40)).unwrap();
            let Some(MacAction::Transmit { psdu, .. }) = drain_tx(&mut mac, &mut now, &mut out) else { panic!() };
            if let ParsedFrame::Data { body, .. } = frame::parse(&psdu).unwrap() {
                if ident::parse_body(&body).is_some() {
                    idents += 1;
                    assert!(now - t0 >= 600_000_000 - 1_000_000, "identified early at {}", now - t0);
                }
            }
            now += 5_000_000;
        }
        assert_eq!(idents, 1, "one repeat in 11 minutes of traffic");
        // 4. Traffic stops: 30 s later the closing identification goes out
        //    on its own, and nothing after that.
        let Some(MacAction::Transmit { psdu, .. }) = drain_tx(&mut mac, &mut now, &mut out) else {
            // data frame from the loop above may still be pending; consume
            panic!()
        };
        let _ = psdu;
        now += 31_000_000;
        let Some(MacAction::Transmit { psdu, .. }) = drain_tx(&mut mac, &mut now, &mut out) else { panic!("no closing identification") };
        let ParsedFrame::Data { body, .. } = frame::parse(&psdu).unwrap() else { panic!() };
        assert!(ident::parse_body(&body).is_some());
        now += 120_000_000;
        assert!(mac.poll(now, &mut out).is_none());
        // 5. A station without a call sign never identifies.
        let mut quiet = Mac::new(MacConfig::new(A));
        quiet.enqueue_eth(&eth_frame(BCAST, A, 40)).unwrap();
        let Some(MacAction::Transmit { psdu, .. }) = drain_tx(&mut quiet, &mut now, &mut out) else { panic!() };
        let ParsedFrame::Data { body, .. } = frame::parse(&psdu).unwrap() else { panic!() };
        assert!(ident::parse_body(&body).is_none());
        // 6. A receiver reports a heard identification instead of forwarding it.
        let mut mac_b = Mac::new(MacConfig::new(B));
        let mut out_b = Vec::new();
        let id = frame::build_data(BCAST, A, 77, false, 0, &ident::body("W1AW", ""));
        mac_b.on_phy_event(&psdu_event(&TxVector::default(), &id), now, &mut out_b);
        assert!(out_b.iter().any(|e| matches!(e, MacEvent::IdentReceived { src, text } if *src == A && text == "DE W1AW")));
        assert!(!out_b.iter().any(|e| matches!(e, MacEvent::EthReceived(_))));
    }

    #[test]
    fn good_neighbour_filter_both_directions() {
        // The test helper builds IPv4 frames: blocked by the policy on the
        // way out (reported to the caller) and on the way in (event).
        let mut cfg = MacConfig::new(A);
        cfg.filter = FilterConfig::good_neighbor();
        let mut mac = Mac::new(cfg.clone());
        assert!(matches!(mac.enqueue_eth(&eth_frame(B, A, 40)), Err(MacError::Filtered("IPv4"))));
        let mut now = 0;
        let mut out = Vec::new();
        assert!(drain_tx(&mut mac, &mut now, &mut out).is_none());
        assert_eq!(mac.filtered(), (1, 0));
        // An IPv6 Babel packet passes.
        let mut v6 = Vec::new();
        v6.extend_from_slice(&B);
        v6.extend_from_slice(&A);
        v6.extend_from_slice(&0x86DDu16.to_be_bytes());
        v6.extend_from_slice(&[0x60, 0, 0, 0, 0, 12, 17, 64]);
        v6.extend_from_slice(&[0xfe, 0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]);
        v6.extend_from_slice(&[0xff, 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 6]);
        v6.extend_from_slice(&[0x1a, 0x28, 0x1a, 0x28, 0, 12, 0, 0, 1, 2, 3, 4]);
        mac.enqueue_eth(&v6).unwrap();
        assert!(drain_tx(&mut mac, &mut now, &mut out).is_some());
        // Ingress: an IPv4 frame from the air is dropped with an event, the
        // IPv6 one delivered.
        let mut cfg_b = MacConfig::new(B);
        cfg_b.filter = FilterConfig::good_neighbor();
        let mut mac_b = Mac::new(cfg_b);
        let mut out_b = Vec::new();
        let data4 = frame::build_data(B, A, 1, false, 0, &eth::to_body(0x0800, &[0x45; 30]));
        mac_b.on_phy_event(&psdu_event(&TxVector::default(), &data4), 0, &mut out_b);
        assert!(out_b.iter().any(|e| matches!(e, MacEvent::Filtered { reason: "IPv4" })));
        assert!(!out_b.iter().any(|e| matches!(e, MacEvent::EthReceived(_))));
        let data6 = frame::build_data(B, A, 2, false, 0, &eth::to_body(0x86DD, &v6[14..]));
        mac_b.on_phy_event(&psdu_event(&TxVector::default(), &data6), 0, &mut out_b);
        assert!(out_b.iter().any(|e| matches!(e, MacEvent::EthReceived(_))));
        assert_eq!(mac_b.filtered(), (0, 1));
        // Library default: no filtering.
        let mut plain = Mac::new(MacConfig::new(A));
        plain.enqueue_eth(&eth_frame(B, A, 40)).unwrap();
    }
}
