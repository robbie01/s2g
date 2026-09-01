//! IEEE 802.11ah (S1G) PHY baseband implementation.
//!
//! Scope: 2 MHz bandwidth, long GI (8 µs), 1 spatial stream, BCC, S1G_SHORT
//! preamble, SU PPDUs, MCS 0–8. Pure DSP — no SDR dependencies. All sample
//! I/O is `Complex<f32>` at the native 2 MS/s rate.
//!
//! * TX entry point: [`tx::Transmitter`] — PSDU bytes + [`vector::TxVector`] → IQ samples.
//! * RX entry point: [`rx::Receiver`] — streaming IQ samples → [`rx::RxEvent`]s.
//!
//! Constants and MCS tables derive from IEEE 802.11-2024 Clause 23; every
//! table in [`params`] cites its clause section.

pub mod bits;
pub mod bcc;
pub mod error;
pub mod interleaver;
pub mod mapping;
pub mod ofdm;
pub mod params;
pub mod pilots;
pub mod preamble;
pub mod rx;
pub mod scrambler;
pub mod sig;
pub mod tx;
pub mod vector;

pub type Complex32 = num_complex::Complex<f32>;

pub use error::PhyError;
pub use rx::{Receiver, RxEvent};
pub use tx::Transmitter;
pub use vector::{RxVector, TxVector};
