//! IEEE 802.11ah (S1G) PHY baseband implementation.
//!
//! Scope: 2 MHz bandwidth, 1 spatial stream, long or short GI, BCC or
//! LDPC, fixed or traveling pilots, S1G_SHORT and S1G_LONG (SU) preambles,
//! NDP CMAC PPDUs, MCS 0-8 and 11. The receiver identifies (for CCA and
//! RID) every other mode the SIG / SIG-A can signal and implements the PHY
//! receive procedure of 23.3.20: CCA, RSSI/RCPI, PHY-RXSTART / PHY-RXEND
//! statuses, carrier-lost handling. Pure DSP, no SDR dependencies. All
//! sample I/O is `Complex<f32>` at the native 2 MS/s.
//!
//! * TX entry point: [`tx::Transmitter`]: PSDU bytes + [`vector::TxVector`] -> IQ samples.
//! * RX entry point: [`rx::Receiver`]: streaming IQ samples -> [`rx::RxEvent`]s.
//! * Conformance helpers: [`conformance`] (spectral flatness, EVM, mask).
//! * Simulation helpers: [`sim`] (deterministic RNG, channel impairments).
//!
//! Constants and MCS tables derive from IEEE 802.11-2024 Clause 23; every
//! table in [`params`] cites its clause section.

pub mod bits;
pub mod bcc;
pub mod conformance;
pub mod error;
pub mod interleaver;
pub mod ldpc;
pub mod mapping;
pub mod ofdm;
pub mod params;
pub mod pilots;
pub mod preamble;
pub mod rx;
pub mod scrambler;
pub mod sig;
pub mod sim;
pub mod tx;
pub mod vector;

pub type Complex32 = num_complex::Complex<f32>;

pub use error::PhyError;
pub use rx::{Receiver, RxEndStatus, RxEvent};
pub use tx::Transmitter;
pub use vector::{Coding, PreambleType, RxVector, TxVector};
