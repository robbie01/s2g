//! PHY error types.

use thiserror::Error;

#[derive(Debug, Error, Clone, PartialEq, Eq)]
pub enum PhyError {
    #[error("invalid MCS index {0} for 2 MHz / 1 spatial stream")]
    InvalidMcs(u8),
    #[error("PSDU length {len} out of range for this PPDU configuration: {reason}")]
    InvalidLength { len: usize, reason: &'static str },
    #[error("unsupported feature: {0}")]
    Unsupported(&'static str),
    #[error("invalid TXVECTOR: {0}")]
    InvalidTxVector(&'static str),
}
