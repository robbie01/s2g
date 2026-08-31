//! Full TX chain: TXVECTOR + PSDU → S1G_SHORT 2 MHz PPDU samples at 2 MS/s
//! [23.3.4.3, 23.3.4.6.1; digest coding-chain §11].
//!
//! PPDU = STF ‖ LTF1 ‖ SIG ‖ Data. Data field: SERVICE(8×0) + PSDU bits
//! (LSB-first per octet) + pad → scramble → append 6 zero tail bits →
//! BCC encode/puncture → per-symbol interleave → map → pilots → OFDM with
//! 8 µs GI. Each field has unit average power before the output `amplitude`
//! scaling (default 0.25 ≈ −12 dBFS, leaving OFDM PAPR headroom in a ±1.0
//! full-scale DAC).

use crate::bits::bytes_to_bits;
use crate::error::PhyError;
use crate::ofdm::{self, DATA_SUBCARRIER_INDICES};
use crate::params::{self, McsParams, N_GI_LONG, N_SERVICE, N_SYM_MAX, N_TAIL, N_TONE_DATA, PSDU_MAX_NO_AGG};
use crate::vector::{GuardInterval, TxVector};
use crate::{bcc, interleaver, mapping, pilots, preamble, scrambler, sig, Complex32};

/// Transmitter for 2 MHz S1G_SHORT PPDUs.
pub struct Transmitter {
    /// Output amplitude scale applied to the unit-power waveform.
    pub amplitude: f32,
}

impl Default for Transmitter {
    fn default() -> Self {
        Self { amplitude: 0.25 }
    }
}

/// Number of Data-field OFDM symbols [Eq 23-79/23-80 specialized:
/// ceil((8·LEN + 14) / N_DBPS)].
pub fn n_sym(mcs: u8, psdu_len: usize, aggregation: bool) -> Result<usize, PhyError> {
    let p = params::mcs_params(mcs)?;
    let n = (8 * psdu_len + N_SERVICE + N_TAIL).div_ceil(p.n_dbps);
    if aggregation {
        if n > N_SYM_MAX {
            return Err(PhyError::InvalidLength { len: psdu_len, reason: "needs more than 511 symbols" });
        }
    } else if psdu_len == 0 || psdu_len > PSDU_MAX_NO_AGG {
        return Err(PhyError::InvalidLength { len: psdu_len, reason: "non-aggregated PSDU must be 1..=511 octets" });
    }
    Ok(n)
}

/// PPDU airtime in µs [Eq 23-74 specialized: 240 + 40·N_SYM].
pub fn txtime_us(mcs: u8, psdu_len: usize, aggregation: bool) -> Result<u32, PhyError> {
    Ok(240 + 40 * n_sym(mcs, psdu_len, aggregation)? as u32)
}

fn pick_seed() -> u8 {
    // Pseudo-random nonzero 7-bit seed [Table 17-7]; entropy from the clock.
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.subsec_nanos())
        .unwrap_or(12345);
    (nanos % 127) as u8 + 1
}

impl Transmitter {
    pub fn new() -> Self {
        Self::default()
    }

    /// Generate the complete PPDU waveform at 2 MS/s.
    pub fn generate(&self, txv: &TxVector, psdu: &[u8]) -> Result<Vec<Complex32>, PhyError> {
        if txv.gi != GuardInterval::Long {
            return Err(PhyError::Unsupported("short GI"));
        }
        let p: &McsParams = params::mcs_params(txv.mcs)?;
        let nsym = n_sym(txv.mcs, psdu.len(), txv.aggregation)?;
        let length_field = if txv.aggregation { nsym as u16 } else { psdu.len() as u16 };
        let fields = sig::SigFields::from_txvector(txv, length_field)?;
        let seed = match txv.scrambler_seed {
            Some(s) if (1..=127).contains(&s) => s,
            Some(_) => return Err(PhyError::InvalidTxVector("scrambler seed must be 1..=127")),
            None => pick_seed(),
        };

        // ---- Data-field bits [coding-chain §11] ----
        let n_pad = nsym * p.n_dbps - 8 * psdu.len() - N_SERVICE - N_TAIL;
        let mut bits = Vec::with_capacity(nsym * p.n_dbps);
        bits.extend_from_slice(&[0u8; N_SERVICE]);
        bits.extend(bytes_to_bits(psdu));
        bits.extend(std::iter::repeat_n(0u8, n_pad));
        scrambler::scramble_in_place(seed, &mut bits);
        bits.extend_from_slice(&[0u8; N_TAIL]); // unscrambled zero tail
        debug_assert_eq!(bits.len(), nsym * p.n_dbps);
        let coded = bcc::encode(&bits, p.rate);
        debug_assert_eq!(coded.len(), nsym * p.n_cbps);

        // ---- Waveform assembly ----
        let scale = 1.0 / (N_TONE_DATA as f32).sqrt();
        let mut out = Vec::with_capacity(480 + nsym * 80);
        out.extend(preamble::stf_time());
        out.extend(preamble::ltf1_time());
        out.extend(sig::encode(&fields));
        for n in 0..nsym {
            let ilv = interleaver::interleave(&coded[n * p.n_cbps..(n + 1) * p.n_cbps], p.n_bpscs);
            let tones = mapping::map(&ilv, p.modulation);
            let sym = ofdm::assemble_freq_symbol(&DATA_SUBCARRIER_INDICES, &tones, &pilots::data_pilots(n));
            out.extend(ofdm::to_time_domain(&sym, N_GI_LONG, scale));
        }
        for v in &mut out {
            *v *= self.amplitude;
        }
        Ok(out)
    }

    /// Generate an NDP CMAC PPDU (no Data field): STF ‖ LTF1 ‖ SIG carrying
    /// the 37-bit body [23.3.11]. 480 samples (240 µs).
    pub fn generate_ndp(&self, body_37: u64) -> Result<Vec<Complex32>, PhyError> {
        if body_37 >> 37 != 0 {
            return Err(PhyError::InvalidTxVector("NDP body exceeds 37 bits"));
        }
        let mut out = Vec::with_capacity(480);
        out.extend(preamble::stf_time());
        out.extend(preamble::ltf1_time());
        out.extend(sig::encode_ndp(body_37));
        for v in &mut out {
            *v *= self.amplitude;
        }
        Ok(out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn n_sym_matches_digest_examples() {
        // MCS 2, 100 octets: ceil(814/78) = 11 [coding-chain sanity #10].
        assert_eq!(n_sym(2, 100, false).unwrap(), 11);
        // MCS 0, 100 octets: ceil(814/26) = 32; TXTIME 240+1280 = 1520 µs
        // [vectors-format sanity #8].
        assert_eq!(n_sym(0, 100, false).unwrap(), 32);
        assert_eq!(txtime_us(0, 100, false).unwrap(), 1520);
    }

    #[test]
    fn length_limits() {
        assert!(n_sym(0, 0, false).is_err());
        assert!(n_sym(0, 512, false).is_err());
        assert!(n_sym(0, 512, true).is_ok());
        // Aggregated cap: MCS0 N_DBPS=26, 511 symbols → max ~1659 octets.
        assert!(n_sym(0, 1659, true).is_ok());
        assert!(n_sym(0, 1700, true).is_err());
    }

    #[test]
    fn waveform_shape_and_determinism() {
        let tx = Transmitter::new();
        for mcs in params::valid_mcs() {
            let psdu: Vec<u8> = (0..137u32).map(|i| (i * 7 + mcs as u32) as u8).collect();
            let txv = TxVector { mcs, scrambler_seed: Some(93), ..Default::default() };
            let w = tx.generate(&txv, &psdu).unwrap();
            let nsym = n_sym(mcs, psdu.len(), false).unwrap();
            assert_eq!(w.len(), 480 + 80 * nsym, "MCS {mcs}");
            // Deterministic with a fixed seed.
            let w2 = tx.generate(&txv, &psdu).unwrap();
            assert_eq!(w, w2);
        }
    }

    #[test]
    fn seed_changes_data_not_length() {
        let tx = Transmitter::new();
        let psdu = [0xAAu8; 64];
        let a = tx
            .generate(&TxVector { scrambler_seed: Some(1), ..Default::default() }, &psdu)
            .unwrap();
        let b = tx
            .generate(&TxVector { scrambler_seed: Some(77), ..Default::default() }, &psdu)
            .unwrap();
        assert_eq!(a.len(), b.len());
        // Preamble + SIG identical; data differs.
        assert_eq!(&a[..480], &b[..480]);
        assert_ne!(&a[480..], &b[480..]);
    }

    #[test]
    fn ndp_waveform() {
        let tx = Transmitter::new();
        let w = tx.generate_ndp(0x1F_0000_0001).unwrap();
        assert_eq!(w.len(), 480);
        assert!(tx.generate_ndp(1u64 << 37).is_err());
    }
}
