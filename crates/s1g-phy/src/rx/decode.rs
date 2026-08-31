//! Data-field bit-level decoding: LLRs → deinterleave → Viterbi →
//! descramble → PSDU bytes [inverse of digest coding-chain §11].

use crate::bits::bits_to_bytes;
use crate::params::{McsParams, N_SERVICE, N_TAIL};
use crate::{bcc, interleaver, mapping, scrambler, Complex32};

pub struct DataDecoder {
    mcs: &'static McsParams,
    n_sym: usize,
    psdu_length: usize,
    llrs: Vec<f32>,
    fed: usize,
    evm_num: f64,
    evm_den: f64,
}

pub struct DecodeResult {
    pub psdu: Vec<u8>,
    /// Recovered scrambler seed (SERVICE field B0..B6).
    pub scrambler_seed: u8,
    /// Data-field EVM in dB (hard-decision reference).
    pub evm_db: f32,
}

impl DataDecoder {
    pub fn new(mcs: &'static McsParams, n_sym: usize, psdu_length: usize) -> Self {
        Self {
            mcs,
            n_sym,
            psdu_length,
            llrs: Vec::with_capacity(n_sym * mcs.n_cbps),
            fed: 0,
            evm_num: 0.0,
            evm_den: 0.0,
        }
    }

    /// Feed one equalized symbol (52 tones + CSI in data-subcarrier order).
    /// Returns `Some(result)` after the final symbol.
    pub fn push_symbol(&mut self, data: &[Complex32], csi: &[f32]) -> Option<DecodeResult> {
        debug_assert!(self.fed < self.n_sym);
        let sym_llrs = mapping::demap_llrs(data, csi, self.mcs.modulation);
        // Hard-decision EVM against the re-mapped nearest constellation point.
        let hard: Vec<u8> = sym_llrs.iter().map(|&l| if l > 0.0 { 0 } else { 1 }).collect();
        let nearest = mapping::map(&hard, self.mcs.modulation);
        for (y, x) in data.iter().zip(&nearest) {
            self.evm_num += (y - x).norm_sqr() as f64;
            self.evm_den += x.norm_sqr() as f64;
        }
        self.llrs.extend(interleaver::deinterleave_llrs(&sym_llrs, self.mcs.n_bpscs));
        self.fed += 1;
        if self.fed < self.n_sym {
            return None;
        }

        let n_info = self.n_sym * self.mcs.n_dbps;
        let mut bits = bcc::viterbi_decode(&self.llrs, self.mcs.rate, n_info);
        // Descramble everything but the 6 tail bits; recover the seed from
        // the first 7 bits (SERVICE B0..B6 are zero before scrambling).
        let seed = scrambler::recover_seed(&bits[..7]).unwrap_or(1);
        let scr_len = n_info - N_TAIL;
        scrambler::scramble_in_place(seed, &mut bits[..scr_len]);
        let psdu_bits = &bits[N_SERVICE..N_SERVICE + 8 * self.psdu_length];
        let evm = (self.evm_num / self.evm_den.max(1e-30)) as f32;
        Some(DecodeResult {
            psdu: bits_to_bytes(psdu_bits),
            scrambler_seed: seed,
            evm_db: 10.0 * evm.max(1e-9).log10(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bits::bytes_to_bits;
    use crate::params;
    use crate::pilots;

    /// Build the TX-side symbol stream inline (per digest coding-chain §11)
    /// and verify the decoder inverts it exactly.
    #[test]
    fn decodes_synthetic_tx_stream() {
        for mcs_idx in [0u8, 2, 4, 7, 8, 11] {
            let p = params::mcs_params(mcs_idx).unwrap();
            let psdu: Vec<u8> = (0..97u32).map(|i| (i * 13 + mcs_idx as u32) as u8).collect();
            let seed = 45u8;
            let n_sym = (8 * psdu.len() + 14).div_ceil(p.n_dbps);
            let n_pad = n_sym * p.n_dbps - 8 * psdu.len() - 14;
            let mut bits = vec![0u8; 8];
            bits.extend(bytes_to_bits(&psdu));
            bits.extend(std::iter::repeat_n(0u8, n_pad));
            crate::scrambler::scramble_in_place(seed, &mut bits);
            bits.extend_from_slice(&[0; 6]);
            let coded = crate::bcc::encode(&bits, p.rate);

            let mut dec = DataDecoder::new(p, n_sym, psdu.len());
            let csi = vec![1.0f32; 52];
            let mut result = None;
            for n in 0..n_sym {
                let ilv = crate::interleaver::interleave(&coded[n * p.n_cbps..(n + 1) * p.n_cbps], p.n_bpscs);
                let tones = crate::mapping::map(&ilv, p.modulation);
                // pilots don't enter the decoder; data tones only
                let _ = pilots::data_pilots(n);
                result = dec.push_symbol(&tones, &csi);
                if n + 1 < n_sym {
                    assert!(result.is_none());
                }
            }
            let r = result.expect("decode complete");
            assert_eq!(r.psdu, psdu, "MCS {mcs_idx}");
            assert_eq!(r.scrambler_seed, seed);
            assert!(r.evm_db < -80.0, "clean stream EVM {}", r.evm_db);
        }
    }
}
