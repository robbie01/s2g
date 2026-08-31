//! SDR hardware abstraction. `s1g-phy` never sees these traits — apps wire
//! PHY ↔ SDR together — but every backend (Pluto today, others later)
//! implements this interface so apps are backend-agnostic.
//!
//! Sample convention: interleaved-in-time `Complex<f32>` normalized to
//! roughly ±1.0 full scale, at [`StreamConfig::sample_rate_hz`].

use num_complex::Complex;
use thiserror::Error;

pub type Complex32 = Complex<f32>;

#[derive(Debug, Error)]
pub enum SdrError {
    #[error("device not found or unreachable: {0}")]
    NotFound(String),
    #[error("configuration rejected: {0}")]
    Config(String),
    #[error("stream error: {0}")]
    Stream(String),
    #[error("backend error: {0}")]
    Backend(String),
}

/// Radio/stream configuration requested by the application.
#[derive(Debug, Clone)]
pub struct StreamConfig {
    /// RF center frequency in Hz (this project runs at 1.25 GHz).
    pub center_freq_hz: f64,
    /// Baseband complex sample rate in Hz at the device boundary.
    pub sample_rate_hz: f64,
    /// Analog RF bandwidth hint in Hz (backend may round).
    pub rf_bandwidth_hz: f64,
}

/// Gain control for RX.
#[derive(Debug, Clone, Copy)]
pub enum RxGain {
    /// Backend AGC.
    Auto,
    /// Manual gain in dB.
    Manual(f64),
}

/// Receiver stream: pull complex samples.
pub trait SdrRx {
    /// Blocking read into `buf`; returns samples written (>0) or an error.
    fn recv(&mut self, buf: &mut [Complex32]) -> Result<usize, SdrError>;
    /// Actual device sample rate (after any backend rounding).
    fn sample_rate_hz(&self) -> f64;
}

/// Transmitter stream: push complex samples (blocking until enqueued).
pub trait SdrTx {
    fn send(&mut self, samples: &[Complex32]) -> Result<(), SdrError>;
    fn sample_rate_hz(&self) -> f64;
    /// Flush/let the hardware drain (best effort).
    fn flush(&mut self) -> Result<(), SdrError>;
}

/// A device that can open RX and/or TX streams. RX-only backends simply
/// return `Unsupported` from `open_tx`.
pub trait SdrDevice {
    type Rx: SdrRx;
    type Tx: SdrTx;

    fn open_rx(&mut self, cfg: &StreamConfig, gain: RxGain) -> Result<Self::Rx, SdrError>;
    fn open_tx(&mut self, cfg: &StreamConfig, tx_gain_db: f64) -> Result<Self::Tx, SdrError>;
}
