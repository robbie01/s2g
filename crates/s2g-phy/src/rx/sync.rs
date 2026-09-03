//! Packet detection and synchronization (implementation-defined; the spec
//! only defines the waveform).
//!
//! Detection: normalized autocorrelation at the 16-sample STF period over a
//! 32-sample window, requiring a run of above-threshold positions (plateau).
//! Coarse CFO from the autocorrelation angle (unambiguous to ±62.5 kHz).
//! Timing: cross-correlation against the known 64-sample LTS period, using
//! the two-repetition structure of LTF1; fine CFO from the phase between
//! the two repetitions (±15.6 kHz, applied after coarse correction).

use crate::params::SAMPLE_RATE_HZ;
use crate::preamble;
use crate::Complex32;

/// STF autocorrelation lag (samples) and averaging window.
const LAG: usize = 16;
const WIN: usize = 32;
/// Consecutive above-threshold positions required to declare detection.
const RUN: usize = 48;

/// Result of LTF synchronization.
#[derive(Debug, Clone, Copy)]
pub struct SyncResult {
    /// Offset within the examined slice of the first sample of the first
    /// LTS period (i.e. LTF1 start + 32).
    pub lts_start: usize,
    /// Total CFO estimate in Hz (coarse + fine).
    pub cfo_hz: f32,
    /// Peak cross-correlation quality (0..1, normalized).
    pub quality: f32,
}

/// Scan `buf` for an STF plateau starting at `from`. Returns
/// `Some((position, coarse_cfo_hz))` where `position` is the start of the
/// detected run, or `None` (with all positions up to
/// `buf.len().saturating_sub(WIN + LAG)` exhausted).
pub fn detect_stf(buf: &[Complex32], from: usize, threshold: f32) -> Option<(usize, f32)> {
    if buf.len() < WIN + LAG + 1 {
        return None;
    }
    let last = buf.len() - WIN - LAG;
    if from > last {
        return None;
    }
    // Sliding sums. The half-period (lag-8) correlation is tracked as a
    // guard: the STF's occupied tones (multiples of 4) cancel exactly at
    // lag 8 through any LTI channel, while DC offset (LO leakage) and CW
    // interferers correlate at every lag. Requiring low lag-8 correlation
    // rejects those without touching real preambles.
    let mut c = Complex32::new(0.0, 0.0);
    let mut c8 = Complex32::new(0.0, 0.0);
    let mut e = 0.0f32;
    for i in 0..WIN {
        c += buf[from + i] * buf[from + i + LAG].conj();
        c8 += buf[from + i] * buf[from + i + LAG / 2].conj();
        e += buf[from + i + LAG].norm_sqr();
    }
    let mut run = 0usize;
    let mut acc = Complex32::new(0.0, 0.0);
    let mut n = from;
    loop {
        let m = c.norm() / e.max(1e-12);
        let m8 = c8.norm() / e.max(1e-12);
        if m > threshold && m8 < 0.7 * m && e > 1e-9 {
            run += 1;
            acc += c;
            if run >= RUN {
                let start = n + 1 - run;
                let cfo = -acc.arg() / (2.0 * core::f32::consts::PI * LAG as f32 / SAMPLE_RATE_HZ as f32);
                return Some((start, cfo));
            }
        } else {
            run = 0;
            acc = Complex32::new(0.0, 0.0);
        }
        if n >= last {
            return None;
        }
        c += buf[n + WIN] * buf[n + WIN + LAG].conj() - buf[n] * buf[n + LAG].conj();
        c8 += buf[n + WIN] * buf[n + WIN + LAG / 2].conj() - buf[n] * buf[n + LAG / 2].conj();
        e += buf[n + WIN + LAG].norm_sqr() - buf[n + LAG].norm_sqr();
        n += 1;
    }
}

/// Locate LTF1 in `buf` (already coarse-CFO corrected), searching LTS-start
/// candidates in `[search_from, search_to)`. Requires
/// `buf.len() >= search_to + 128`.
pub fn ltf_sync(buf: &[Complex32], search_from: usize, search_to: usize, coarse_cfo_hz: f32) -> Option<SyncResult> {
    let reference = preamble::ltf_period();
    let ref_energy: f32 = reference.iter().map(|v| v.norm_sqr()).sum();
    let mut best = (0usize, 0.0f32);
    for n in search_from..search_to {
        if n + 128 > buf.len() {
            break;
        }
        let mut m = 0.0f32;
        for rep in 0..2 {
            let mut xc = Complex32::new(0.0, 0.0);
            let mut en = 0.0f32;
            for i in 0..64 {
                let s = buf[n + rep * 64 + i];
                xc += s * reference[i].conj();
                en += s.norm_sqr();
            }
            m += xc.norm_sqr() / (en * ref_energy).max(1e-12);
        }
        if m > best.1 {
            best = (n, m);
        }
    }
    let (n0, quality) = best;
    // quality is the sum of two normalized-correlation-squared terms (max 2).
    if quality < 0.5 {
        return None;
    }
    // Fine CFO from the two repetitions.
    let mut c = Complex32::new(0.0, 0.0);
    for i in 0..64 {
        c += buf[n0 + i] * buf[n0 + 64 + i].conj();
    }
    let fine = -c.arg() / (2.0 * core::f32::consts::PI * 64.0 / SAMPLE_RATE_HZ as f32);
    Some(SyncResult {
        lts_start: n0,
        cfo_hz: coarse_cfo_hz + fine,
        quality: quality / 2.0,
    })
}

/// Result of the mid-packet detector.
#[derive(Debug, Clone, Copy)]
pub struct MidPacket {
    /// Peak guard-interval correlation, normalized (0..1).
    pub metric: f32,
    /// Symbol period that matched: 80 samples (long GI) or 72 (short GI).
    pub symbol_len: usize,
}

/// Mid-packet PPDU detection [23.3.18.5.3.1, aCCAMidTime]: an S1G PPDU
/// whose preamble was missed still shows the cyclic-prefix structure of
/// its Data symbols. Correlates each sample with the one 64 samples later
/// (the useful symbol length), sums over the guard interval and over the
/// symbols in `buf` (≥ 212 µs = 424 samples), and looks for one symbol
/// phase where that correlation is coherent. Tones (CW, DC) and the STF/LTF
/// correlate at every phase, so a peak is only accepted when it clearly
/// exceeds the mean over phases. Tries the long-GI (80-sample) and
/// short-GI (72-sample) symbol periods.
pub fn detect_mid_packet(buf: &[Complex32]) -> Option<MidPacket> {
    const LAG: usize = 64;
    if buf.len() < 3 * 80 {
        return None;
    }
    let mean = buf.iter().sum::<Complex32>() / buf.len() as f32;
    let x: Vec<Complex32> = buf.iter().map(|&v| v - mean).collect();
    let mut best: Option<MidPacket> = None;
    for (sym, gi) in [(80usize, 16usize), (72, 8)] {
        let last = x.len() - gi - LAG;
        let mut cph = vec![Complex32::new(0.0, 0.0); sym];
        let mut eph = vec![0.0f32; sym];
        // Sliding sums of the gi-sample correlation and energy.
        let mut c = Complex32::new(0.0, 0.0);
        let mut e = 0.0f32;
        for i in 0..gi {
            c += x[i] * x[i + LAG].conj();
            e += 0.5 * (x[i].norm_sqr() + x[i + LAG].norm_sqr());
        }
        for k in 0..last {
            cph[k % sym] += c;
            eph[k % sym] += e;
            c += x[k + gi] * x[k + gi + LAG].conj() - x[k] * x[k + LAG].conj();
            e += 0.5 * (x[k + gi].norm_sqr() + x[k + gi + LAG].norm_sqr() - x[k].norm_sqr() - x[k + LAG].norm_sqr());
        }
        let m: Vec<f32> = cph.iter().zip(&eph).map(|(c, e)| c.norm() / e.max(1e-12)).collect();
        let peak = m.iter().cloned().fold(0.0f32, f32::max);
        let avg = m.iter().sum::<f32>() / m.len() as f32;
        if peak > 0.5 && peak > 2.0 * avg && best.is_none_or(|b| peak > b.metric) {
            best = Some(MidPacket { metric: peak, symbol_len: sym });
        }
    }
    best
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::preamble::{ltf1_time, stf_time};

    /// Shift `buf` by `-cfo_hz`, phase zero at index 0.
    fn derotate(buf: &[Complex32], cfo_hz: f32) -> Vec<Complex32> {
        let w = -2.0 * core::f32::consts::PI * cfo_hz / SAMPLE_RATE_HZ as f32;
        buf.iter().enumerate().map(|(i, &v)| v * Complex32::from_polar(1.0, w * i as f32)).collect()
    }

    fn add_cfo(buf: &[Complex32], cfo_hz: f32) -> Vec<Complex32> {
        derotate(buf, -cfo_hz)
    }

    /// Zero-mean uniform noise. A raw LCG has lattice correlation at the
    /// STF lag and falsely triggers the detector; splitmix64 does not.
    fn noise(n: usize, amp: f32, seed: u64) -> Vec<Complex32> {
        let mut rng = crate::sim::Rng(seed);
        (0..n).map(|_| Complex32::new((rng.uniform() * 2.0 - 1.0) * amp, (rng.uniform() * 2.0 - 1.0) * amp)).collect()
    }

    fn preamble_stream(cfo: f32, lead: usize) -> (Vec<Complex32>, usize) {
        let mut v = noise(lead, 0.02, 7);
        let start = v.len();
        v.extend(stf_time());
        v.extend(ltf1_time());
        // SIG-ish filler so LTF windows exist.
        v.extend(noise(200, 0.3, 9));
        let v = add_cfo(&v, cfo);
        (v, start)
    }

    #[test]
    fn detects_stf_and_coarse_cfo() {
        for cfo in [-40e3f32, -11e3, 0.0, 17e3, 40e3] {
            let (v, start) = preamble_stream(cfo, 300);
            let (pos, est) = detect_stf(&v, 0, 0.6).expect("detect");
            // The plateau can begin slightly before the STF proper (partial
            // window overlap), so allow a small early margin.
            assert!(pos + 24 >= start && pos < start + 120, "pos {pos} vs start {start}");
            assert!((est - cfo).abs() < 400.0, "cfo est {est} vs {cfo}");
        }
    }

    #[test]
    fn no_false_detect_on_noise() {
        let v = noise(4000, 0.5, 3);
        assert!(detect_stf(&v, 0, 0.6).is_none());
    }

    #[test]
    fn no_false_detect_on_dc_or_cw() {
        // Constant DC (LO leakage) correlates at every lag; the lag-8 guard
        // must reject it.
        let dc = vec![Complex32::new(1e-3, 1e-3); 4000];
        assert!(detect_stf(&dc, 0, 0.55).is_none());
        // Same for a slow CW tone.
        let cw: Vec<Complex32> = (0..4000)
            .map(|i| Complex32::from_polar(0.1, 2.0 * core::f32::consts::PI * 0.01 * i as f32))
            .collect();
        assert!(detect_stf(&cw, 0, 0.55).is_none());
        // And a real preamble on top of a DC offset still detects.
        let (v, start) = preamble_stream(5e3, 400);
        let with_dc: Vec<Complex32> = v.iter().map(|&s| s + Complex32::new(2e-3, -1e-3)).collect();
        let (pos, _) = detect_stf(&with_dc, 0, 0.55).expect("detect with DC");
        assert!(pos + 24 >= start && pos < start + 120);
    }

    #[test]
    fn mid_packet_detector() {
        use crate::vector::TxVector;
        use crate::Transmitter;
        let tx = Transmitter::new();
        let psdu: Vec<u8> = (0..300u32).map(|i| (i * 11) as u8).collect();
        let wave = tx.generate(&TxVector { mcs: 3, scrambler_seed: Some(5), ..Default::default() }, &psdu).unwrap();
        // Data symbols only (preamble cut off), with noise and CFO.
        let data: Vec<Complex32> = add_cfo(&wave[480 + 160..480 + 160 + 424], 20e3)
            .iter()
            .zip(noise(424, 0.05, 4))
            .map(|(&s, n)| s + n)
            .collect();
        let d = detect_mid_packet(&data).expect("long-GI data symbols");
        assert_eq!(d.symbol_len, 80);
        assert!(d.metric > 0.7, "{}", d.metric);
        // Short-GI symbols: 8-sample cyclic prefix + 64-sample payload.
        let mut sgi = Vec::new();
        for n in 0..7 {
            let payload = &wave[480 + n * 80 + 16..480 + n * 80 + 80];
            sgi.extend_from_slice(&payload[56..]);
            sgi.extend_from_slice(payload);
        }
        let d = detect_mid_packet(&sgi[..424]).expect("short-GI data symbols");
        assert_eq!(d.symbol_len, 72);
        // Noise, a CW tone, DC and the STF (which correlates at every
        // phase) must not trigger it.
        assert!(detect_mid_packet(&noise(424, 0.3, 9)).is_none());
        let cw: Vec<Complex32> = (0..424).map(|i| Complex32::from_polar(0.3, 0.07 * i as f32)).collect();
        assert!(detect_mid_packet(&cw).is_none());
        let dc = vec![Complex32::new(0.2, -0.1); 424];
        assert!(detect_mid_packet(&dc).is_none());
        let mut stf = stf_time();
        stf.extend(stf_time());
        stf.extend(stf_time());
        assert!(detect_mid_packet(&stf[..424]).is_none());
    }

    #[test]
    fn ltf_sync_finds_exact_timing() {
        for cfo in [-38e3f32, 0.0, 25e3] {
            let (v, start) = preamble_stream(cfo, 250);
            let (pos, coarse) = detect_stf(&v, 0, 0.6).unwrap();
            let corrected = derotate(&v, coarse);
            let true_lts = start + 160 + 32;
            let r = ltf_sync(&corrected, pos, pos + 300, coarse).expect("ltf");
            assert_eq!(r.lts_start, true_lts, "cfo {cfo}");
            assert!((r.cfo_hz - cfo).abs() < 100.0, "fine cfo {} vs {}", r.cfo_hz, cfo);
            assert!(r.quality > 0.8);
        }
    }
}
