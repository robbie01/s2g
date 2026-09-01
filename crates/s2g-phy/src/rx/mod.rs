//! Streaming S1G receiver: push samples in, get events out — the PHY
//! receive procedure of 23.3.20 (Figure 23-53) for a 2 MHz STA.
//!
//! Pipeline: energy detect (CCA) → STF autocorrelation detect (coarse CFO)
//! → LTF cross-correlation timing (fine CFO) → LTF channel estimate, RSSI,
//! RCPI → SIG format discrimination (S1G_SHORT vs S1G_LONG) and decode →
//! PHY-RXSTART → per-symbol equalize / pilot tracking / timing-drift
//! tracking → (Viterbi | LDPC) → descramble → PSDU → PHY-RXEND. Any failure
//! emits `RxEnd` with the spec status, holds CCA BUSY for the predicted PPDU
//! duration when one is known, and re-arms the detector.
//!
//! All FFT windows back off [`BACKOFF`] samples into the preceding GI
//! (ISI-safe; the common linear phase is absorbed by the channel estimate
//! since every window uses the same offset), and the window position is
//! advanced/retarded as pilot phase slopes reveal sampling-clock drift.

pub mod chanest;
pub mod decode;
pub mod sync;

use crate::ofdm::{self, SIG_SUBCARRIER_INDICES};
use crate::params::{self, rf, SAMPLE_RATE_HZ};
use crate::vector::{PreambleType, RxVector};
use crate::{pilots, sig, Complex32};
use chanest::Equalizer;
use decode::DataDecoder;

/// Receiver tuning knobs.
#[derive(Debug, Clone)]
pub struct RxConfig {
    /// STF autocorrelation detection threshold (0..1).
    pub detect_threshold: f32,
    /// Emit `PpduStart` events (carrier-sense hook for a MAC).
    pub emit_ppdu_start: bool,
    /// Emit `Cca` events.
    pub emit_cca: bool,
    /// CCA channel classification (type 2 = 3 dB higher thresholds).
    pub cca_type: rf::CcaType,
    /// Calibration: dBm = dBFS + `cal_offset_db`. With the default 0.0 the
    /// dBm thresholds below act directly on dBFS values, which is the only
    /// sane behaviour for an uncalibrated SDR front end.
    pub cal_offset_db: f32,
    /// Energy-detect threshold, dBm [Table 23-37/38: −72 dBm at 2 MHz].
    pub ed_threshold_dbm: f32,
    /// Consecutive Data symbols without signal before declaring
    /// `CarrierLost`.
    pub carrier_lost_symbols: usize,
    /// LDPC decoder iteration cap.
    pub max_ldpc_iterations: usize,
    /// Track sampling-clock drift and move the FFT window accordingly.
    pub timing_tracking: bool,
}

impl Default for RxConfig {
    fn default() -> Self {
        Self {
            detect_threshold: 0.55,
            emit_ppdu_start: true,
            emit_cca: true,
            cca_type: rf::CcaType::Type1,
            cal_offset_db: 0.0,
            ed_threshold_dbm: rf::ED_THRESHOLD_2MHZ_DBM,
            carrier_lost_symbols: 4,
            max_ldpc_iterations: 30,
            timing_tracking: true,
        }
    }
}

/// Per-PPDU receive quality metrics.
#[derive(Debug, Clone, PartialEq)]
pub struct RxMetrics {
    pub snr_db: f32,
    pub cfo_hz: f32,
    pub evm_db: f32,
    pub rssi_dbfs: f32,
    /// Sampling-clock drift accumulated over the PPDU, samples (positive =
    /// the transmitter's clock is slower than ours).
    pub timing_drift_samples: f32,
    /// LDPC codewords that failed to converge (0 for BCC).
    pub ldpc_failures: usize,
}

/// Why CCA reports BUSY [23.3.18.5].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CcaReason {
    /// Any signal above the energy-detect threshold (aCCATime window).
    EnergyDetect,
    /// Start of an S1G PPDU detected (preamble).
    PreambleDetect,
    /// A valid SIG/SIG-A predicted the PPDU duration; BUSY is held for it.
    PpduHold,
}

/// PHY-RXEND.indication status [23.3.20].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RxEndStatus {
    NoError,
    /// SIG/SIG-A CRC failure or Reserved SIG Indication.
    FormatViolation,
    /// Valid SIG/SIG-A for a mode this receiver does not decode.
    UnsupportedRate(&'static str),
    /// Signal vanished during the Data field.
    CarrierLost,
    /// Stream ended mid-PPDU (reported by [`Receiver::finish`]).
    Truncated,
}

#[derive(Debug, Clone, PartialEq)]
#[allow(clippy::large_enum_variant)] // PsduReceived carries the RXVECTOR + PSDU by value on purpose
pub enum RxEvent {
    /// PHY-CCA.indication: `busy` changed. `hold_us` is the predicted
    /// remaining busy time when known (PPDU hold), else 0.
    Cca { sample_index: u64, busy: bool, reason: Option<CcaReason>, hold_us: u32 },
    /// Preamble detected. `sample_index` counts samples ever pushed.
    PpduStart { sample_index: u64, coarse_cfo_hz: f32 },
    /// PHY-RXSTART.indication: SIG/SIG-A valid, RXVECTOR known. Issued for
    /// supported *and* unsupported modes (the latter is followed at once by
    /// `RxEnd(UnsupportedRate)`).
    RxStart { sample_index: u64, rxvector: RxVector },
    /// NDP CMAC PPDU received (37-bit body, LSB = B0) [23.3.11].
    NdpReceived { sample_index: u64, body: u64, metrics: RxMetrics },
    /// Full PSDU decoded (bit errors possible — the MAC FCS is the final
    /// arbiter). Followed by `RxEnd(NoError)`.
    PsduReceived { sample_index: u64, rxvector: RxVector, psdu: Vec<u8>, metrics: RxMetrics },
    /// PHY-RXEND.indication.
    RxEnd { sample_index: u64, status: RxEndStatus },
}

/// FFT-window backoff into the preceding GI, samples. With a 16-sample GI
/// this tolerates a channel delay spread of up to 16 − 6 = 10 samples
/// (5 µs) together with ±6 samples of timing error before ISI appears; the
/// timing tracker keeps the residual error well inside that.
const BACKOFF: u64 = 6;
/// Overlap re-scanned across `process` calls so a detection run split by a
/// chunk boundary is still found.
const SCAN_OVERLAP: u64 = 112;
/// Energy-detect window: aCCATime (40 µs) = 80 samples.
const ED_WINDOW: u64 = 80;
/// Samples per microsecond at the native rate.
const SAMPLES_PER_US: u64 = 2;

enum State {
    Search,
    LtfSync { trig: u64, coarse_cfo: f32 },
    Sig { anchor: u64, cfo: f32, eq: Equalizer, rssi: f32 },
    Data(Box<DataState>),
}

/// Sampling-clock drift tracker: filters the per-symbol timing offset seen
/// by the pilots and steps the FFT window by whole samples. The filtered
/// residual (not the noisy per-symbol measurement) is what the equalizer
/// removes as a linear phase.
struct TimingTracker {
    filtered: f32,
    shift: i64,
    total: f32,
}

impl TimingTracker {
    fn new() -> Self {
        Self { filtered: 0.0, shift: 0, total: 0.0 }
    }

    /// Feed one symbol's timing-offset estimate (samples, relative to the
    /// current window). Returns the residual offset to correct for *this*
    /// symbol; the window shift for later symbols is in `shift`.
    fn update(&mut self, offset: f32) -> f32 {
        const ALPHA: f32 = 1.0 / 12.0;
        const STEP_AT: f32 = 0.7;
        self.filtered += ALPHA * (offset - self.filtered);
        let applied = self.filtered;
        if self.filtered > STEP_AT {
            self.shift += 1;
            self.filtered -= 1.0;
            self.total += 1.0;
        } else if self.filtered < -STEP_AT {
            self.shift -= 1;
            self.filtered += 1.0;
            self.total -= 1.0;
        }
        applied
    }

    fn drift(&self) -> f32 {
        self.total + self.filtered
    }
}

/// Second-order common-phase-error loop: predicts the next symbol's CPE
/// from the previous one plus a phase rate (residual CFO), then corrects
/// with the measurement. Smooths the 4-pilot estimate at low SNR while
/// following a linearly growing phase without lag.
struct PhaseTracker {
    cpe: f32,
    rate: f32,
    init: bool,
}

impl PhaseTracker {
    fn new() -> Self {
        Self { cpe: 0.0, rate: 0.0, init: false }
    }

    fn update(&mut self, measured: f32) -> f32 {
        const K1: f32 = 0.5;
        const K2: f32 = 0.08;
        if !self.init {
            self.init = true;
            self.cpe = measured;
            return measured;
        }
        let pred = self.cpe + self.rate;
        let mut err = measured - pred;
        while err > core::f32::consts::PI {
            err -= 2.0 * core::f32::consts::PI;
        }
        while err < -core::f32::consts::PI {
            err += 2.0 * core::f32::consts::PI;
        }
        self.cpe = pred + K1 * err;
        self.rate += K2 * err;
        self.cpe
    }
}

struct DataState {
    anchor: u64,
    cfo: f32,
    eq: Equalizer,
    rxv: RxVector,
    dec: DataDecoder,
    n: usize,
    rssi: f32,
    end: u64,
    timing: TimingTracker,
    phase: PhaseTracker,
    lost_run: usize,
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
    // ---- CCA ----
    /// Next energy-detect window start (absolute, multiple of ED_WINDOW).
    ed_abs: u64,
    ed_busy: bool,
    /// CCA BUSY held until this absolute sample (predicted PPDU end).
    hold_until: Option<u64>,
    /// A preamble has been detected and the PPDU is being received.
    in_ppdu: bool,
    cca_busy: bool,
}

impl Receiver {
    pub fn new(cfg: RxConfig) -> Self {
        Self {
            cfg,
            buf: Vec::new(),
            buf_abs: 0,
            scan_abs: 0,
            total_in: 0,
            state: State::Search,
            ed_abs: 0,
            ed_busy: false,
            hold_until: None,
            in_ppdu: false,
            cca_busy: false,
        }
    }

    /// Drop any in-progress PPDU state and re-arm detection.
    pub fn reset(&mut self) {
        self.state = State::Search;
        self.scan_abs = self.total_in;
        self.in_ppdu = false;
        self.hold_until = None;
        self.trim(self.total_in);
    }

    /// Current CCA state.
    pub fn cca_busy(&self) -> bool {
        self.cca_busy
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

    fn power_dbfs(&self, from_abs: u64, len: usize) -> f32 {
        match self.extract(from_abs, len, 0.0, from_abs) {
            Some(s) => {
                let p: f32 = s.iter().map(|v| v.norm_sqr()).sum::<f32>() / len as f32;
                10.0 * p.max(1e-12).log10()
            }
            None => -100.0,
        }
    }

    fn dbm(&self, dbfs: f32) -> f32 {
        dbfs + self.cfg.cal_offset_db
    }

    // ---------------------------------------------------------------- CCA

    /// Recompute the combined CCA state and emit on change.
    fn update_cca(&mut self, at: u64, reason: Option<CcaReason>, events: &mut Vec<RxEvent>) {
        if let Some(until) = self.hold_until {
            if at >= until {
                self.hold_until = None;
            }
        }
        let busy = self.ed_busy || self.in_ppdu || self.hold_until.is_some();
        if busy != self.cca_busy {
            self.cca_busy = busy;
            if self.cfg.emit_cca {
                let hold_us = match self.hold_until {
                    Some(u) if busy => ((u.saturating_sub(at)) / SAMPLES_PER_US) as u32,
                    _ => 0,
                };
                let reason = if busy {
                    reason.or(if self.hold_until.is_some() {
                        Some(CcaReason::PpduHold)
                    } else if self.in_ppdu {
                        Some(CcaReason::PreambleDetect)
                    } else {
                        Some(CcaReason::EnergyDetect)
                    })
                } else {
                    None
                };
                events.push(RxEvent::Cca { sample_index: at, busy, reason, hold_us });
            }
        }
    }

    /// Energy detection over aCCATime windows of newly arrived samples.
    fn step_ed(&mut self, events: &mut Vec<RxEvent>) {
        self.ed_abs = self.ed_abs.max(self.buf_abs);
        while self.ed_abs + ED_WINDOW <= self.end_abs() {
            let p = self.dbm(self.power_dbfs(self.ed_abs, ED_WINDOW as usize));
            let busy = p > self.cfg.ed_threshold_dbm;
            self.ed_abs += ED_WINDOW;
            if busy != self.ed_busy {
                self.ed_busy = busy;
                self.update_cca(self.ed_abs, Some(CcaReason::EnergyDetect), events);
            }
        }
        // Hold expiry is time-driven even when nothing else changes.
        if self.hold_until.is_some_and(|u| self.end_abs() >= u) {
            let at = self.hold_until.unwrap();
            self.update_cca(at, None, events);
        }
    }

    // ---------------------------------------------------------- pipeline

    /// Push samples (2 MS/s), collecting events.
    pub fn process(&mut self, samples: &[Complex32], events: &mut Vec<RxEvent>) {
        self.buf.extend_from_slice(samples);
        self.total_in += samples.len() as u64;
        self.step_ed(events);
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

    /// Signal end-of-stream: emits `RxEnd(Truncated)` if a PPDU was in
    /// progress.
    pub fn finish(&mut self, events: &mut Vec<RxEvent>) {
        if !matches!(self.state, State::Search) {
            events.push(RxEvent::RxEnd { sample_index: self.total_in, status: RxEndStatus::Truncated });
        }
        self.reset();
        self.update_cca(self.total_in, None, events);
    }

    /// Leave the PPDU (at sample `at`) and resume searching at `scan_from`;
    /// CCA BUSY is held until `hold_until` if given, and a `Cca` update with
    /// the predicted hold time is emitted even when CCA was already BUSY so
    /// the MAC learns the duration.
    fn rearm(&mut self, at: u64, scan_from: u64, hold_until: Option<u64>, events: &mut Vec<RxEvent>) {
        self.state = State::Search;
        self.scan_abs = scan_from;
        self.in_ppdu = false;
        match hold_until {
            Some(h) if h > at => {
                if self.hold_until.is_none_or(|cur| h > cur) {
                    self.hold_until = Some(h);
                }
                self.cca_busy = true;
                if self.cfg.emit_cca {
                    events.push(RxEvent::Cca {
                        sample_index: at,
                        busy: true,
                        reason: Some(CcaReason::PpduHold),
                        hold_us: ((h - at) / SAMPLES_PER_US) as u32,
                    });
                }
            }
            _ => self.update_cca(at, None, events),
        }
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
                self.in_ppdu = true;
                self.update_cca(trig, Some(CcaReason::PreambleDetect), events);
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

    fn step_ltf(&mut self, events: &mut Vec<RxEvent>) -> bool {
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
            self.rearm(trig + 64, trig + 64, None, events);
            return true;
        };
        let anchor = trig + r.lts_start as u64;
        let cfo = r.cfo_hz;
        let l1 = self.fft_window(anchor, cfo, anchor).expect("buffered");
        let l2 = self.fft_window(anchor + 64, cfo, anchor).expect("buffered");
        let est = chanest::estimate(&l1, &l2);
        // RSSI/RCPI over LTF1 (the two LTS periods) [23.3.18.6/7].
        let rssi = self.power_dbfs(anchor, 128);
        self.state = State::Sig { anchor, cfo, eq: Equalizer::new(est), rssi };
        true
    }

    fn fill_measurements(&self, rxv: &mut RxVector, eq: &Equalizer, rssi_dbfs: f32) {
        rxv.rssi_dbfs = rssi_dbfs;
        rxv.rssi = ((rssi_dbfs + 127.5) * 2.0).round().clamp(0.0, 255.0) as u8;
        rxv.rcpi_dbm = self.dbm(rssi_dbfs);
        rxv.rcpi = rf::rcpi_encode(rxv.rcpi_dbm);
        rxv.snr_db = eq.estimate().mean_tone_snr_db();
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
        let e1 = eq.equalize(&w1, &SIG_SUBCARRIER_INDICES, &pilots::PILOT_INDICES, &pilots::sig_pilots(0));
        let e2 = eq.equalize(&w2, &SIG_SUBCARRIER_INDICES, &pilots::PILOT_INDICES, &pilots::sig_pilots(1));
        let (ptype, _conf) = sig::detect_preamble_type(&e2.data);
        let sig_end = anchor + 288;
        // PPDU start (first STF sample) for duration bookkeeping.
        let ppdu_start = anchor - 192;
        let ppdu_end = |rxv: &RxVector| ppdu_start + rxv.ppdu_duration_us() as u64 * SAMPLES_PER_US;
        match sig::decode(&e1.data, &e2.data, &e1.csi, &e2.csi, ptype) {
            Ok(sig::SigContent::Ndp { body }) => {
                let metrics = RxMetrics {
                    snr_db: eq.estimate().snr_db(),
                    cfo_hz: cfo,
                    evm_db: f32::NAN,
                    rssi_dbfs: rssi,
                    timing_drift_samples: 0.0,
                    ldpc_failures: 0,
                };
                events.push(RxEvent::NdpReceived { sample_index: sig_end, body, metrics });
                self.rearm(sig_end, sig_end, None, events);
            }
            Ok(content) => match content.verdict().expect("non-NDP") {
                sig::SigVerdict::Supported(mut rxv) => {
                    self.fill_measurements(&mut rxv, &eq, rssi);
                    let p = params::mcs_params(rxv.mcs).expect("validated");
                    let end = ppdu_end(&rxv);
                    events.push(RxEvent::RxStart { sample_index: anchor, rxvector: rxv.clone() });
                    let mut eq = eq;
                    if rxv.smoothing {
                        // The LTF windows start BACKOFF samples early, which
                        // the estimate carries as a known linear phase.
                        eq.smooth(-2.0 * core::f32::consts::PI * BACKOFF as f32 / 64.0);
                    }
                    let dec = DataDecoder::new(p, &rxv, self.cfg.max_ldpc_iterations);
                    self.state = State::Data(Box::new(DataState {
                        anchor,
                        cfo,
                        eq,
                        rxv,
                        dec,
                        n: 0,
                        rssi,
                        end,
                        timing: TimingTracker::new(),
                        phase: PhaseTracker::new(),
                        lost_run: 0,
                    }));
                }
                sig::SigVerdict::Unsupported(mut rxv, why) => {
                    self.fill_measurements(&mut rxv, &eq, rssi);
                    let end = ppdu_end(&rxv);
                    events.push(RxEvent::RxStart { sample_index: anchor, rxvector: rxv });
                    events.push(RxEvent::RxEnd { sample_index: anchor, status: RxEndStatus::UnsupportedRate(why) });
                    // Wait out the PPDU with CCA BUSY [Fig 23-53 End-of-Wait].
                    self.rearm(sig_end, end, Some(end), events);
                }
                sig::SigVerdict::Reserved(_) => {
                    events.push(RxEvent::RxEnd { sample_index: anchor, status: RxEndStatus::FormatViolation });
                    self.rearm(sig_end, anchor, None, events);
                }
            },
            Err(_) => {
                events.push(RxEvent::RxEnd { sample_index: anchor, status: RxEndStatus::FormatViolation });
                // CCA stays BUSY only while energy detect says so (release
                // when the level drops below min-sensitivity + 20 dB, which
                // is the ED threshold at 2 MHz). Re-arm AT the anchor: if
                // this was a false sync, a real preamble may begin only
                // slightly later and must not be skipped.
                self.rearm(sig_end, anchor, None, events);
            }
        }
        true
    }

    fn step_data(&mut self, events: &mut Vec<RxEvent>) -> bool {
        let (payload, anchor, cfo, n, shift) = match &self.state {
            State::Data(ds) => (
                (ds.anchor as i64 + 288 + 16 + 80 * ds.n as i64 + ds.timing.shift) as u64,
                ds.anchor,
                ds.cfo,
                ds.n,
                ds.timing.shift,
            ),
            _ => unreachable!(),
        };
        let _ = shift;
        let Some(w) = self.fft_window(payload, cfo, anchor) else { return false };
        let State::Data(mut ds) = std::mem::replace(&mut self.state, State::Search) else {
            unreachable!()
        };
        let tp = ds.rxv.traveling_pilots;
        let positions = pilots::pilot_positions(n, tp);
        let expected = pilots::data_pilots(n, tp);
        let indices = pilots::data_subcarriers(n, tp);

        // ---- Carrier-lost detection [23.3.20] ----
        let est = ds.eq.estimate();
        let p_sym = chanest::used_tone_power(&w);
        let lost = p_sym < 0.15 * est.signal_power && p_sym < 3.0 * est.noise_var;
        ds.lost_run = if lost { ds.lost_run + 1 } else { 0 };
        if ds.lost_run >= self.cfg.carrier_lost_symbols {
            events.push(RxEvent::RxEnd { sample_index: payload, status: RxEndStatus::CarrierLost });
            let end = ds.end;
            // Wait for the intended end of the PSDU before CCA IDLE.
            self.rearm(payload, end, Some(end), events);
            return true;
        }

        // ---- Pilot tracking: CPE loop + timing-drift filter ----
        let hint = self.cfg.timing_tracking.then(|| chanest::slope_for_timing_offset(ds.timing.filtered));
        let m = ds.eq.measure_pilots(&w, &positions, &expected, hint);
        let cpe = ds.phase.update(m.cpe);
        let slope = if self.cfg.timing_tracking {
            let residual = if m.quality > 0.5 { ds.timing.update(m.timing_offset_samples()) } else { ds.timing.filtered };
            chanest::slope_for_timing_offset(residual)
        } else {
            m.slope
        };
        let e = ds.eq.apply(&w, &indices, cpe, slope);
        if tp {
            ds.eq.track_pilots(&w, &positions, &expected, cpe, slope, 0.5);
        }

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
                let end = ds.end;
                let mut rxv = ds.rxv;
                rxv.scrambler_seed = r.scrambler_seed;
                let metrics = RxMetrics {
                    snr_db: ds.eq.estimate().snr_db(),
                    cfo_hz: ds.cfo,
                    evm_db: r.evm_db,
                    rssi_dbfs: ds.rssi,
                    timing_drift_samples: ds.timing.drift(),
                    ldpc_failures: r.ldpc_failures,
                };
                events.push(RxEvent::PsduReceived { sample_index: end, rxvector: rxv, psdu: r.psdu, metrics });
                events.push(RxEvent::RxEnd { sample_index: end, status: RxEndStatus::NoError });
                self.rearm(end, end, None, events);
                true
            }
        }
    }
}

impl Receiver {
    /// Predicted PPDU-end sample for a detected PPDU starting at LTS
    /// `anchor` (exposed for tests).
    #[doc(hidden)]
    pub fn ppdu_end_sample(anchor: u64, rxv: &RxVector) -> u64 {
        anchor - 192 + rxv.ppdu_duration_us() as u64 * SAMPLES_PER_US
    }
}

/// Unused-format guard for the `PreambleType` import (kept for readers).
#[allow(dead_code)]
fn _preamble_type_marker(_: PreambleType) {}
