//! Streaming S1G receiver: push samples in, get events out.
//!
//! Pipeline: STF autocorrelation detect (coarse CFO) → LTF cross-correlation
//! timing (fine CFO) → LTF channel estimate → SIG decode → per-symbol
//! equalize/track → Viterbi → descramble → PSDU. All FFT windows back off
//! [`BACKOFF`] samples into the preceding GI (ISI-safe; the common linear
//! phase is absorbed by the channel estimate since every window uses the
//! same offset).

pub mod chanest;
pub mod decode;
pub mod sync;

use crate::ofdm::{self, DATA_SUBCARRIER_INDICES, SIG_SUBCARRIER_INDICES};
use crate::params::{self, SAMPLE_RATE_HZ};
use crate::vector::RxVector;
use crate::{pilots, sig, Complex32};
use chanest::Equalizer;
use decode::DataDecoder;

/// Receiver tuning knobs.
#[derive(Debug, Clone)]
pub struct RxConfig {
    /// STF autocorrelation detection threshold (0..1).
    pub detect_threshold: f32,
    /// Emit `PpduStart` events (CCA-style hook for a MAC).
    pub emit_ppdu_start: bool,
}

impl Default for RxConfig {
    fn default() -> Self {
        Self { detect_threshold: 0.55, emit_ppdu_start: true }
    }
}

/// Per-PPDU receive quality metrics.
#[derive(Debug, Clone, PartialEq)]
pub struct RxMetrics {
    pub snr_db: f32,
    pub cfo_hz: f32,
    pub evm_db: f32,
    pub rssi_dbfs: f32,
}

#[derive(Debug, Clone, PartialEq)]
pub enum RxErrorKind {
    /// SIG failed CRC / structural checks (or false detection).
    SigInvalid,
    /// SIG decoded but requests an unsupported mode.
    Unsupported(&'static str),
    /// Stream ended mid-PPDU (reported by [`Receiver::finish`]).
    Truncated,
}

#[derive(Debug, Clone, PartialEq)]
pub enum RxEvent {
    /// Preamble detected. `sample_index` counts samples ever pushed.
    PpduStart { sample_index: u64, coarse_cfo_hz: f32 },
    /// SIG decoded — PPDU parameters known.
    SigDecoded { sample_index: u64, rxvector: RxVector },
    /// NDP CMAC PPDU received (37-bit body, LSB = B0) [23.3.11].
    NdpReceived { sample_index: u64, body: u64, metrics: RxMetrics },
    /// Full PSDU decoded (bit errors possible — the MAC FCS is the final
    /// arbiter).
    PsduReceived { sample_index: u64, rxvector: RxVector, psdu: Vec<u8>, metrics: RxMetrics },
    /// PPDU reception aborted.
    Error { sample_index: u64, kind: RxErrorKind },
}

/// FFT-window backoff into the preceding GI, samples.
const BACKOFF: u64 = 2;
/// Overlap re-scanned across `process` calls so a detection run split by a
/// chunk boundary is still found.
const SCAN_OVERLAP: u64 = 112;

enum State {
    Search,
    LtfSync { trig: u64, coarse_cfo: f32 },
    Sig { anchor: u64, cfo: f32, eq: Equalizer, rssi: f32 },
    Data(Box<DataState>),
}

struct DataState {
    anchor: u64,
    cfo: f32,
    eq: Equalizer,
    rxv: RxVector,
    dec: DataDecoder,
    n: usize,
    rssi: f32,
}

/// Streaming receiver state machine.
pub struct Receiver {
    cfg: RxConfig,
    buf: Vec<Complex32>,
    /// Absolute index of `buf[0]`.
    buf_abs: u64,
    /// Next absolute scan position (Search state).
    scan_abs: u64,
    total_in: u64,
    state: State,
}

impl Receiver {
    pub fn new(cfg: RxConfig) -> Self {
        Self { cfg, buf: Vec::new(), buf_abs: 0, scan_abs: 0, total_in: 0, state: State::Search }
    }

    /// Drop any in-progress PPDU state and re-arm detection.
    pub fn reset(&mut self) {
        self.state = State::Search;
        self.scan_abs = self.total_in;
        self.trim(self.total_in);
    }

    fn rel(&self, abs: u64) -> usize {
        (abs - self.buf_abs) as usize
    }

    fn end_abs(&self) -> u64 {
        self.buf_abs + self.buf.len() as u64
    }

    fn trim(&mut self, keep_from: u64) {
        let keep = keep_from.saturating_sub(8).max(self.buf_abs);
        let n = self.rel(keep.min(self.end_abs()));
        if n > 0 {
            self.buf.drain(..n);
            self.buf_abs += n as u64;
        }
    }

    /// Extract `len` samples starting at absolute `abs`, derotated by `cfo`
    /// with phase reference `ref_abs`. None if not fully buffered yet.
    fn extract(&self, abs: u64, len: usize, cfo: f32, ref_abs: u64) -> Option<Vec<Complex32>> {
        if abs < self.buf_abs || abs + len as u64 > self.end_abs() {
            return None;
        }
        let start = self.rel(abs);
        let w = -2.0 * std::f64::consts::PI * cfo as f64 / SAMPLE_RATE_HZ;
        Some(
            self.buf[start..start + len]
                .iter()
                .enumerate()
                .map(|(i, &v)| {
                    let t = (abs + i as u64) as i64 - ref_abs as i64;
                    v * Complex32::from_polar(1.0, (w * t as f64) as f32)
                })
                .collect(),
        )
    }

    fn fft_window(&self, payload_abs: u64, cfo: f32, ref_abs: u64) -> Option<ofdm::FreqSymbol> {
        let t = self.extract(payload_abs - BACKOFF, 64, cfo, ref_abs)?;
        Some(ofdm::fft_symbol(&t))
    }

    fn rssi_dbfs(&self, from_abs: u64, len: usize) -> f32 {
        match self.extract(from_abs, len, 0.0, from_abs) {
            Some(s) => {
                let p: f32 = s.iter().map(|v| v.norm_sqr()).sum::<f32>() / len as f32;
                10.0 * p.max(1e-12).log10()
            }
            None => -100.0,
        }
    }

    /// Push samples (2 MS/s), collecting events.
    pub fn process(&mut self, samples: &[Complex32], events: &mut Vec<RxEvent>) {
        self.buf.extend_from_slice(samples);
        self.total_in += samples.len() as u64;
        loop {
            let progressed = match &mut self.state {
                State::Search => self.step_search(events),
                State::LtfSync { .. } => self.step_ltf(events),
                State::Sig { .. } => self.step_sig(events),
                State::Data(_) => self.step_data(events),
            };
            if !progressed {
                break;
            }
        }
    }

    /// Signal end-of-stream: emits `Truncated` if a PPDU was in progress.
    pub fn finish(&mut self, events: &mut Vec<RxEvent>) {
        if !matches!(self.state, State::Search) {
            events.push(RxEvent::Error { sample_index: self.total_in, kind: RxErrorKind::Truncated });
        }
        self.reset();
    }

    fn rearm(&mut self, scan_from: u64) {
        self.state = State::Search;
        self.scan_abs = scan_from;
    }

    fn step_search(&mut self, events: &mut Vec<RxEvent>) -> bool {
        let from = self.scan_abs.max(self.buf_abs);
        let from_rel = self.rel(from.min(self.end_abs()));
        match sync::detect_stf(&self.buf, from_rel, self.cfg.detect_threshold) {
            Some((pos, coarse_cfo)) => {
                let trig = self.buf_abs + pos as u64;
                if self.cfg.emit_ppdu_start {
                    events.push(RxEvent::PpduStart { sample_index: trig, coarse_cfo_hz: coarse_cfo });
                }
                self.state = State::LtfSync { trig, coarse_cfo };
                true
            }
            None => {
                // Remember how far we scanned (minus overlap), trim, stop.
                let scanned_to = self.end_abs().saturating_sub(SCAN_OVERLAP);
                if scanned_to > self.scan_abs {
                    self.scan_abs = scanned_to;
                }
                self.trim(self.scan_abs);
                false
            }
        }
    }

    fn step_ltf(&mut self, _events: &mut Vec<RxEvent>) -> bool {
        let (trig, coarse) = match self.state {
            State::LtfSync { trig, coarse_cfo } => (trig, coarse_cfo),
            _ => unreachable!(),
        };
        // Need the rest of STF + GI2 + 2×LTS + margin. The search span
        // tolerates a trigger up to ~160 samples before the true STF start.
        const SPAN: usize = 480;
        let Some(slice) = self.extract(trig, SPAN, coarse, trig) else {
            return false;
        };
        let Some(r) = sync::ltf_sync(&slice, 0, SPAN - 128, coarse) else {
            // False alarm: resume scanning shortly after the trigger.
            self.rearm(trig + 64);
            return true;
        };
        let anchor = trig + r.lts_start as u64;
        let cfo = r.cfo_hz;
        let l1 = self.fft_window(anchor, cfo, anchor).expect("buffered");
        let l2 = self.fft_window(anchor + 64, cfo, anchor).expect("buffered");
        let est = chanest::estimate(&l1, &l2);
        let rssi = self.rssi_dbfs(trig, 128);
        self.state = State::Sig { anchor, cfo, eq: Equalizer::new(est), rssi };
        true
    }

    fn step_sig(&mut self, events: &mut Vec<RxEvent>) -> bool {
        let (anchor, cfo) = match &self.state {
            State::Sig { anchor, cfo, .. } => (*anchor, *cfo),
            _ => unreachable!(),
        };
        // SIG symbols' payloads start at anchor + 128 + 16 + n·80.
        let Some(w2) = self.fft_window(anchor + 128 + 16 + 80, cfo, anchor) else {
            return false;
        };
        let w1 = self.fft_window(anchor + 128 + 16, cfo, anchor).expect("buffered");
        let State::Sig { eq, rssi, .. } = std::mem::replace(&mut self.state, State::Search) else {
            unreachable!()
        };
        let e1 = eq.equalize(&w1, &SIG_SUBCARRIER_INDICES, &pilots::sig_pilots(0));
        let e2 = eq.equalize(&w2, &SIG_SUBCARRIER_INDICES, &pilots::sig_pilots(1));
        let sig_end = anchor + 288;
        match sig::decode(&e1.data, &e2.data, &e1.csi, &e2.csi) {
            Ok(sig::SigContent::Normal(fields)) => match fields.to_rxvector() {
                Ok(mut rxv) => {
                    rxv.rssi_dbfs = rssi;
                    let p = params::mcs_params(rxv.mcs).expect("validated");
                    events.push(RxEvent::SigDecoded { sample_index: anchor, rxvector: rxv.clone() });
                    let dec = DataDecoder::new(p, rxv.n_sym, rxv.psdu_length);
                    self.state = State::Data(Box::new(DataState { anchor, cfo, eq, rxv, dec, n: 0, rssi }));
                }
                Err(e) => {
                    events.push(RxEvent::Error {
                        sample_index: anchor,
                        kind: RxErrorKind::Unsupported(unsupported_reason(e)),
                    });
                    self.rearm(sig_end);
                }
            },
            Ok(sig::SigContent::Ndp { body }) => {
                let metrics = RxMetrics {
                    snr_db: eq.estimate().snr_db(),
                    cfo_hz: cfo,
                    evm_db: f32::NAN,
                    rssi_dbfs: rssi,
                };
                events.push(RxEvent::NdpReceived { sample_index: sig_end, body, metrics });
                self.rearm(sig_end);
            }
            Err(_) => {
                events.push(RxEvent::Error { sample_index: anchor, kind: RxErrorKind::SigInvalid });
                // Re-arm AT the anchor: if this was a false sync, a real
                // preamble may begin only slightly later and must not be
                // skipped (LTF/Data don't autocorrelate at the STF period,
                // so re-triggering on the same signal is not a risk).
                self.rearm(anchor);
            }
        }
        true
    }

    fn step_data(&mut self, events: &mut Vec<RxEvent>) -> bool {
        let (payload, anchor, cfo) = match &self.state {
            State::Data(ds) => (ds.anchor + 288 + 16 + 80 * ds.n as u64, ds.anchor, ds.cfo),
            _ => unreachable!(),
        };
        let Some(w) = self.fft_window(payload, cfo, anchor) else { return false };
        let State::Data(mut ds) = std::mem::replace(&mut self.state, State::Search) else {
            unreachable!()
        };
        let e = ds.eq.equalize(&w, &DATA_SUBCARRIER_INDICES, &pilots::data_pilots(ds.n));
        let result = ds.dec.push_symbol(&e.data, &e.csi);
        ds.n += 1;
        match result {
            None => {
                // Trim consumed symbols to bound memory on long PPDUs.
                let consumed = ds.anchor + 288 + 80 * (ds.n as u64 - 1);
                self.state = State::Data(ds);
                self.trim(consumed);
                true
            }
            Some(r) => {
                let end = ds.anchor + 288 + 80 * ds.rxv.n_sym as u64;
                let mut rxv = ds.rxv;
                rxv.scrambler_seed = r.scrambler_seed;
                let metrics = RxMetrics {
                    snr_db: ds.eq.estimate().snr_db(),
                    cfo_hz: ds.cfo,
                    evm_db: r.evm_db,
                    rssi_dbfs: ds.rssi,
                };
                events.push(RxEvent::PsduReceived { sample_index: end, rxvector: rxv, psdu: r.psdu, metrics });
                self.rearm(end);
                true
            }
        }
    }
}

fn unsupported_reason(e: crate::error::PhyError) -> &'static str {
    match e {
        crate::error::PhyError::Unsupported(s) => s,
        crate::error::PhyError::InvalidMcs(_) => "invalid MCS",
        _ => "malformed SIG",
    }
}
