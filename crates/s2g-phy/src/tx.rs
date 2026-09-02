//! Full TX chain: TXVECTOR + PSDU → 2 MHz PPDU samples at 2 MS/s
//! [23.3.4.3, 23.3.4.6.1; digest coding-chain §11].
//!
//! PPDU = STF ‖ LTF1 ‖ SIG ‖ Data (S1G_SHORT) or STF ‖ LTF1 ‖ SIG-A ‖
//! D-STF ‖ D-LTF ‖ SIG-B ‖ Data (S1G_LONG SU, 1 STS). Data symbols carry
//! an 8 µs GI, or 4 µs from the second symbol on with short GI. Data field:
//!
//! * **BCC**: SERVICE(8×0) + PSDU bits (LSB-first per octet) + pad →
//!   scramble → append 6 zero tail bits → BCC encode/puncture → per-symbol
//!   interleave → map → pilots → OFDM with 8 µs GI.
//! * **LDPC** [23.3.9.4.4]: SERVICE + PSDU + pad (no tail) → scramble
//!   everything → LDPC PPDU encoding (codeword selection, shortening,
//!   puncturing/repetition, possibly one extra symbol) → map → LDPC tone
//!   map → pilots → OFDM.
//!
//! Pilots are fixed or traveling per TXVECTOR [23.3.9.10]. Each field has
//! unit average power before the output `amplitude` scaling (default 0.25 ≈
//! −12 dBFS, leaving OFDM PAPR headroom in a ±1.0 full-scale DAC).

use crate::bits::bytes_to_bits;
use crate::error::PhyError;
use crate::ldpc::PpduParams;
use crate::params::{self, McsParams, N_GI_LONG, N_GI_SHORT, N_SERVICE, N_SYM_MAX, N_TAIL, N_TONE_DATA, PSDU_MAX_NO_AGG};
use crate::vector::{self, Coding, GuardInterval, PreambleType, TxVector};
use crate::{bcc, interleaver, mapping, ofdm, pilots, preamble, scrambler, sig, Complex32};

/// Transmitter for 2 MHz single-stream PPDUs.
pub struct Transmitter {
    /// Output amplitude scale applied to the unit-power waveform.
    pub amplitude: f32,
}

impl Default for Transmitter {
    fn default() -> Self {
        Self { amplitude: 0.25 }
    }
}

/// Data-field geometry chosen by the TX chain for a PSDU.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DataGeometry {
    /// Symbols actually transmitted.
    pub n_sym: usize,
    /// N_SYM,init (LDPC) — equals `n_sym` for BCC.
    pub n_sym_init: usize,
    /// LDPC Extra OFDM Symbol flag.
    pub ldpc_extra: bool,
    /// Value of the SIG Length field (octets or `n_sym`).
    pub length_field: u16,
}

/// Choose the Data-field geometry for `psdu_len` octets [Eq 23-79/23-80
/// (BCC), 23-46/23-47 + 19.3.11.7.5 (LDPC)].
pub fn data_geometry(mcs: u8, psdu_len: usize, aggregation: bool, coding: Coding) -> Result<DataGeometry, PhyError> {
    let p = params::mcs_params(mcs)?;
    if !aggregation && (psdu_len == 0 || psdu_len > PSDU_MAX_NO_AGG) {
        return Err(PhyError::InvalidLength { len: psdu_len, reason: "non-aggregated PSDU must be 1..=511 octets" });
    }
    if aggregation && psdu_len == 0 {
        return Err(PhyError::InvalidLength { len: 0, reason: "empty A-MPDU" });
    }
    let (n_sym, n_sym_init, ldpc_extra) = match coding {
        Coding::Bcc => {
            let n = (8 * psdu_len + N_SERVICE + N_TAIL).div_ceil(p.n_dbps);
            (n, n, false)
        }
        Coding::Ldpc => {
            let n_init = (8 * psdu_len + N_SERVICE).div_ceil(p.n_dbps);
            let lp = PpduParams::new(n_init, p.n_dbps, p.n_cbps, p.rate);
            (lp.n_sym, n_init, lp.extra_symbol)
        }
    };
    if n_sym > N_SYM_MAX {
        return Err(PhyError::InvalidLength { len: psdu_len, reason: "needs more than 511 symbols" });
    }
    let length_field = if aggregation { n_sym as u16 } else { psdu_len as u16 };
    Ok(DataGeometry { n_sym, n_sym_init, ldpc_extra, length_field })
}

/// Number of Data-field OFDM symbols for a BCC PPDU [Eq 23-79/23-80
/// specialized: ceil((8·LEN + 14) / N_DBPS)].
pub fn n_sym(mcs: u8, psdu_len: usize, aggregation: bool) -> Result<usize, PhyError> {
    Ok(data_geometry(mcs, psdu_len, aggregation, Coding::Bcc)?.n_sym)
}

/// PPDU airtime in µs for a BCC S1G_SHORT / long-GI PPDU [Eq 23-74
/// specialized: 240 + 40·N_SYM].
pub fn txtime_us(mcs: u8, psdu_len: usize, aggregation: bool) -> Result<u32, PhyError> {
    txtime_us_coded(mcs, psdu_len, aggregation, Coding::Bcc)
}

/// PPDU airtime in µs for either coding (S1G_SHORT, long GI).
pub fn txtime_us_coded(mcs: u8, psdu_len: usize, aggregation: bool, coding: Coding) -> Result<u32, PhyError> {
    Ok(params::T_PREAMBLE_US + params::T_SYML_US * data_geometry(mcs, psdu_len, aggregation, coding)?.n_sym as u32)
}

/// PSDU capacity in octets of an aggregated PPDU that must carry at least
/// `min_octets`: the MAC pads its A-MPDU to exactly this size [23.3.9.4.3.2,
/// 23.3.9.4.4.2; Eq 23-81/23-82].
pub fn aggregated_capacity(mcs: u8, min_octets: usize, coding: Coding) -> Result<usize, PhyError> {
    let p = params::mcs_params(mcs)?;
    let g = data_geometry(mcs, min_octets, true, coding)?;
    Ok(match coding {
        Coding::Bcc => (g.n_sym * p.n_dbps - N_SERVICE - N_TAIL) / 8,
        Coding::Ldpc => (g.n_sym_init * p.n_dbps - N_SERVICE) / 8,
    })
}

fn pick_seed() -> u8 {
    // Pseudo-random nonzero 7-bit seed [Table 17-7]; entropy from the clock.
    let nanos = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.subsec_nanos()).unwrap_or(12345);
    (nanos % 127) as u8 + 1
}

/// What the PHY reports back after building a PPDU (PHY-TXEND.confirm
/// SCRAMBLER_OR_CRC plus the geometry).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TxInfo {
    pub scrambler_seed: u8,
    pub geometry: DataGeometry,
    pub txtime_us: u32,
}

impl Transmitter {
    pub fn new() -> Self {
        Self::default()
    }

    /// Generate the complete PPDU waveform at 2 MS/s.
    pub fn generate(&self, txv: &TxVector, psdu: &[u8]) -> Result<Vec<Complex32>, PhyError> {
        Ok(self.generate_with_info(txv, psdu)?.0)
    }

    /// Generate the PPDU and report the scrambler seed and geometry used.
    pub fn generate_with_info(&self, txv: &TxVector, psdu: &[u8]) -> Result<(Vec<Complex32>, TxInfo), PhyError> {
        let p: &McsParams = params::mcs_params(txv.mcs)?;
        let geom = data_geometry(txv.mcs, psdu.len(), txv.aggregation, txv.fec_coding)?;
        let sig_wave = match txv.preamble_type {
            PreambleType::S1gShort => sig::encode(&sig::SigFields::from_txvector(txv, geom.length_field, geom.ldpc_extra)?),
            PreambleType::S1gLong => sig::encode_sig_a_su(&sig::SigASu::from_txvector(txv, geom.length_field, geom.ldpc_extra)?),
        };
        let seed = match txv.scrambler_seed {
            Some(s) if (1..=127).contains(&s) => s,
            Some(_) => return Err(PhyError::InvalidTxVector("scrambler seed must be 1..=127")),
            None => pick_seed(),
        };

        // ---- Data-field coded bits ----
        let coded: Vec<u8> = match txv.fec_coding {
            Coding::Bcc => {
                let n_pad = geom.n_sym * p.n_dbps - 8 * psdu.len() - N_SERVICE - N_TAIL;
                let mut bits = Vec::with_capacity(geom.n_sym * p.n_dbps);
                bits.extend_from_slice(&[0u8; N_SERVICE]);
                bits.extend(bytes_to_bits(psdu));
                bits.extend(std::iter::repeat_n(0u8, n_pad));
                scrambler::scramble_in_place(seed, &mut bits);
                bits.extend_from_slice(&[0u8; N_TAIL]); // unscrambled zero tail
                debug_assert_eq!(bits.len(), geom.n_sym * p.n_dbps);
                bcc::encode(&bits, p.rate)
            }
            Coding::Ldpc => {
                let n_pad = geom.n_sym_init * p.n_dbps - 8 * psdu.len() - N_SERVICE;
                let mut bits = Vec::with_capacity(geom.n_sym_init * p.n_dbps);
                bits.extend_from_slice(&[0u8; N_SERVICE]);
                bits.extend(bytes_to_bits(psdu));
                bits.extend(std::iter::repeat_n(0u8, n_pad));
                scrambler::scramble_in_place(seed, &mut bits); // pad bits scrambled too
                let lp = PpduParams::new(geom.n_sym_init, p.n_dbps, p.n_cbps, p.rate);
                debug_assert_eq!(lp.n_sym, geom.n_sym);
                lp.encode_data(&bits)
            }
        };
        debug_assert_eq!(coded.len(), geom.n_sym * p.n_cbps);

        // ---- Waveform assembly ----
        let scale = 1.0 / (N_TONE_DATA as f32).sqrt();
        let mut out = Vec::with_capacity(720 + geom.n_sym * 80);
        out.extend(preamble::stf_time());
        out.extend(preamble::ltf1_time());
        out.extend(sig_wave);
        if txv.preamble_type == PreambleType::S1gLong {
            // Beam-changeable portion [23.3.8.2.3.3]: D-STF, one D-LTF
            // (1 STS) and SIG-B, which for an SU PPDU repeats D-LTF1.
            out.extend(preamble::dstf_time());
            out.extend(preamble::dltf_time());
            out.extend(preamble::dltf_time());
        }
        // Data pilots: p_{n+2} for S1G_SHORT [Eq 23-55] and for S1G_LONG SU
        // [Eq 23-56, z(n) = n + 2] alike.
        let tp = txv.traveling_pilots;
        for n in 0..geom.n_sym {
            let sym_bits = &coded[n * p.n_cbps..(n + 1) * p.n_cbps];
            let tones = match txv.fec_coding {
                Coding::Bcc => mapping::map(&interleaver::interleave(sym_bits, p.n_bpscs), p.modulation),
                Coding::Ldpc => ofdm::ldpc_tone_map(&mapping::map(sym_bits, p.modulation)),
            };
            let sym = ofdm::assemble_freq_symbol(
                &pilots::data_subcarriers(n, tp),
                &tones,
                &pilots::pilot_positions(n, tp),
                &pilots::data_pilots(n, tp),
            );
            // Short GI starts with the second Data symbol [Eq 23-58].
            let gi = if n > 0 && txv.gi == GuardInterval::Short { N_GI_SHORT } else { N_GI_LONG };
            out.extend(ofdm::to_time_domain(&sym, gi, scale));
        }
        for v in &mut out {
            *v *= self.amplitude;
        }
        let txtime_us = vector::ppdu_duration_us(txv.preamble_type, txv.gi, geom.n_sym);
        debug_assert_eq!(out.len() as u32, 2 * txtime_us);
        let info = TxInfo { scrambler_seed: seed, geometry: geom, txtime_us };
        Ok((out, info))
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
    fn ldpc_geometry() {
        // MCS 0, 100 octets, LDPC: N_SYM,init = ceil(808/26) = 32 and the
        // PPDU encoding process adds an extra symbol (see ldpc tests).
        let g = data_geometry(0, 100, false, Coding::Ldpc).unwrap();
        assert_eq!(g.n_sym_init, 32);
        assert_eq!(g.n_sym, 33);
        assert!(g.ldpc_extra);
        assert_eq!(g.length_field, 100);
        let ga = data_geometry(0, 100, true, Coding::Ldpc).unwrap();
        assert_eq!(ga.length_field, 33);
        assert_eq!(txtime_us_coded(0, 100, false, Coding::Ldpc).unwrap(), 240 + 33 * 40);
        // Capacity for the MAC: N_SYM,init·N_DBPS − 8 bits.
        assert_eq!(aggregated_capacity(0, 100, Coding::Ldpc).unwrap(), (32 * 26 - 8) / 8);
        assert_eq!(aggregated_capacity(0, 100, Coding::Bcc).unwrap(), (32 * 26 - 14) / 8);
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
            for coding in [Coding::Bcc, Coding::Ldpc] {
                let psdu: Vec<u8> = (0..137u32).map(|i| (i * 7 + mcs as u32) as u8).collect();
                let txv = TxVector { mcs, fec_coding: coding, scrambler_seed: Some(93), ..Default::default() };
                let (w, info) = tx.generate_with_info(&txv, &psdu).unwrap();
                let g = data_geometry(mcs, psdu.len(), false, coding).unwrap();
                assert_eq!(w.len(), 480 + 80 * g.n_sym, "MCS {mcs} {coding:?}");
                assert_eq!(info.scrambler_seed, 93);
                assert_eq!(info.geometry, g);
                // Deterministic with a fixed seed.
                let w2 = tx.generate(&txv, &psdu).unwrap();
                assert_eq!(w, w2);
            }
        }
    }

    #[test]
    fn long_preamble_and_short_gi_shapes() {
        let tx = Transmitter::new();
        let psdu = [0x3Cu8; 90];
        let n = n_sym(3, 90, false).unwrap();
        let long = tx
            .generate_with_info(
                &TxVector { mcs: 3, preamble_type: PreambleType::S1gLong, scrambler_seed: Some(4), ..Default::default() },
                &psdu,
            )
            .unwrap();
        assert_eq!(long.0.len(), 720 + 80 * n);
        assert_eq!(long.1.txtime_us, 360 + 40 * n as u32);
        // Omni portion up to the SIG-A shares STF/LTF1 with S1G_SHORT.
        let short = tx.generate(&TxVector { mcs: 3, scrambler_seed: Some(4), ..Default::default() }, &psdu).unwrap();
        assert_eq!(&long.0[..320], &short[..320]);
        assert_ne!(&long.0[320..480], &short[320..480]);
        // D-LTF and SIG-B are identical symbols.
        assert_eq!(&long.0[560..640], &long.0[640..720]);
        let sgi = tx
            .generate_with_info(&TxVector { mcs: 3, gi: GuardInterval::Short, scrambler_seed: Some(4), ..Default::default() }, &psdu)
            .unwrap();
        assert_eq!(sgi.0.len(), 480 + 80 + 72 * (n - 1));
        assert_eq!(sgi.1.txtime_us, 240 + 40 + 36 * (n as u32 - 1));
        // The first Data symbol is unchanged by the GI choice; the SIG differs.
        assert_ne!(&sgi.0[320..480], &short[320..480]);
        assert_eq!(&sgi.0[480..560], &short[480..560]);
    }

    #[test]
    fn seed_changes_data_not_length() {
        let tx = Transmitter::new();
        let psdu = [0xAAu8; 64];
        let a = tx.generate(&TxVector { scrambler_seed: Some(1), ..Default::default() }, &psdu).unwrap();
        let b = tx.generate(&TxVector { scrambler_seed: Some(77), ..Default::default() }, &psdu).unwrap();
        assert_eq!(a.len(), b.len());
        // Preamble + SIG identical; data differs.
        assert_eq!(&a[..480], &b[..480]);
        assert_ne!(&a[480..], &b[480..]);
    }

    #[test]
    fn traveling_pilots_change_only_data_symbols() {
        let tx = Transmitter::new();
        let psdu = [0x5Au8; 64];
        let a = tx.generate(&TxVector { scrambler_seed: Some(9), ..Default::default() }, &psdu).unwrap();
        let b = tx.generate(&TxVector { scrambler_seed: Some(9), traveling_pilots: true, ..Default::default() }, &psdu).unwrap();
        assert_eq!(a.len(), b.len());
        assert_eq!(&a[..320], &b[..320]); // STF + LTF1
        assert_ne!(&a[320..480], &b[320..480]); // SIG carries the TP bit
        assert_ne!(&a[480..], &b[480..]);
        // Unit power preserved (within the 1.5× pilot boost on 4 of 56 tones).
        let p: f32 = b[480..].iter().map(|v| v.norm_sqr()).sum::<f32>() / (b.len() - 480) as f32;
        let expect = 0.25f32 * 0.25 * (52.0 + 4.0 * 2.25) / 56.0;
        assert!((p / expect - 1.0).abs() < 0.1, "power {p} vs {expect}");
    }

    #[test]
    fn ndp_waveform() {
        let tx = Transmitter::new();
        let w = tx.generate_ndp(0x1F_0000_0001).unwrap();
        assert_eq!(w.len(), 480);
        assert!(tx.generate_ndp(1u64 << 37).is_err());
    }
}
