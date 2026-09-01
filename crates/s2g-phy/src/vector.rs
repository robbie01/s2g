//! TXVECTOR / RXVECTOR analogues (IEEE 802.11-2024 23.2.2, Table 23-1),
//! restricted to 2 MHz / 1 SS / BCC / S1G_SHORT SU PPDUs.
//!
//! These are the PHY↔MAC contract. The future (OCB/non-BSS) MAC constructs a
//! [`TxVector`] per PPDU and receives an [`RxVector`] with each decoded PSDU.

/// Guard interval selection. Only `Long` (8 µs) is implemented in v1; the
/// enum exists so the API doesn't change when short GI is added.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum GuardInterval {
    #[default]
    Long,
    Short,
}

/// SIG-2 Response Indication field (what the transmitter expects SIFS after
/// this PPDU; used by third parties for deferral) [Table 23-12].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ResponseIndication {
    #[default]
    None,
    Ndp,
    Normal,
    Long,
}

impl ResponseIndication {
    pub fn to_bits(self) -> u8 {
        match self {
            ResponseIndication::None => 0,
            ResponseIndication::Ndp => 1,
            ResponseIndication::Normal => 2,
            ResponseIndication::Long => 3,
        }
    }
    pub fn from_bits(v: u8) -> Self {
        match v & 3 {
            0 => ResponseIndication::None,
            1 => ResponseIndication::Ndp,
            2 => ResponseIndication::Normal,
            _ => ResponseIndication::Long,
        }
    }
}

/// Per-PPDU transmit parameters (TXVECTOR analogue).
#[derive(Debug, Clone, PartialEq)]
pub struct TxVector {
    /// S1G MCS index; valid for 2 MHz / 1 SS: 0..=8 or 11.
    pub mcs: u8,
    /// Guard interval (v1: must be `Long`).
    pub gi: GuardInterval,
    /// True when the PSDU is an A-MPDU. Mandatory for PSDUs > 511 octets
    /// (SIG Length is 9 bits). Changes SIG Length semantics to symbols.
    pub aggregation: bool,
    /// SIG-2 Response Indication.
    pub response_indication: ResponseIndication,
    /// Scrambler initial state (1..=127). `None` → pseudo-random per PPDU.
    pub scrambler_seed: Option<u8>,
    /// SIG-1 Smoothing bit (channel-smoothing recommendation to receivers).
    pub smoothing: bool,
    /// Traveling pilots (Doppler mode). v1: must be false.
    pub traveling_pilots: bool,
    /// SIG-1 Uplink Indication: 1 = PPDU addressed to an AP [Table 23-12].
    /// For OCB/non-BSS operation leave false.
    pub uplink_indication: bool,
    /// BSS COLOR (0..=7), carried in SIG-1 ID bits when
    /// `uplink_indication` is false.
    pub color: u8,
    /// Partial AID: 0..=511 when `uplink_indication`, else 0..=63.
    pub partial_aid: u16,
}

impl Default for TxVector {
    fn default() -> Self {
        Self {
            mcs: 0,
            gi: GuardInterval::Long,
            aggregation: false,
            response_indication: ResponseIndication::None,
            scrambler_seed: None,
            smoothing: true,
            traveling_pilots: false,
            uplink_indication: false,
            color: 0,
            partial_aid: 0,
        }
    }
}

/// Per-PPDU receive parameters and measurements (RXVECTOR analogue).
#[derive(Debug, Clone, PartialEq)]
pub struct RxVector {
    pub mcs: u8,
    pub gi: GuardInterval,
    pub aggregation: bool,
    pub response_indication: ResponseIndication,
    pub smoothing: bool,
    pub traveling_pilots: bool,
    pub uplink_indication: bool,
    /// COLOR when `uplink_indication` is false (0 otherwise).
    pub color: u8,
    pub partial_aid: u16,
    /// PSDU length in octets derived from the SIG (Eq 23-81 when
    /// aggregated).
    pub psdu_length: usize,
    /// Number of Data-field OFDM symbols.
    pub n_sym: usize,
    /// Recovered scrambler seed (RXVECTOR SCRAMBLER_OR_CRC); 0 until the
    /// SERVICE field has been decoded.
    pub scrambler_seed: u8,
    /// RSSI relative to full scale, dB (absolute calibration is the SDR
    /// layer's business). Filled by the receiver.
    pub rssi_dbfs: f32,
}
