//! End-to-end TX → channel → RX tests for every valid MCS and both codings,
//! with impairments: AWGN, CFO (beyond the spec's ±20 ppm at 1.25 GHz),
//! timing offsets (integer + fractional), sampling-clock offset, amplitude
//! scaling, chunked streaming, multiple PPDUs, truncation, NDP, traveling
//! pilots, S1G_LONG preambles, CCA and the PHY-RXEND status codes.

use s2g_phy::params::valid_mcs;
use s2g_phy::rx::{CcaReason, Receiver, RxConfig, RxEndStatus, RxEvent};
use s2g_phy::sig::{self, SigASu};
use s2g_phy::vector::{Coding, PreambleType, ResponseIndication, TxVector};
use s2g_phy::{preamble, Complex32, Transmitter};

struct Rng(u64);

impl Rng {
    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    fn uniform(&mut self) -> f32 {
        ((self.next_u64() >> 32) as f32 / (1u64 << 31) as f32) - 1.0
    }
    /// Approximately Gaussian (sum of uniforms), unit variance per call.
    fn gauss(&mut self) -> f32 {
        let s: f32 = (0..6).map(|_| self.uniform()).sum();
        s / (6.0f32 / 3.0).sqrt() // var of sum = 6·(1/3) = 2 → scale to 1
    }
    fn bytes(&mut self, n: usize) -> Vec<u8> {
        (0..n).map(|_| (self.next_u64() >> 40) as u8).collect()
    }
}

fn awgn(sig: &[Complex32], snr_db: f32, rng: &mut Rng) -> Vec<Complex32> {
    let p: f32 = sig.iter().map(|v| v.norm_sqr()).sum::<f32>() / sig.len() as f32;
    let nv = p / 10f32.powf(snr_db / 10.0);
    let s = (nv / 2.0).sqrt();
    sig.iter().map(|&v| v + Complex32::new(rng.gauss() * s, rng.gauss() * s)).collect()
}

fn apply_cfo(sig: &[Complex32], cfo_hz: f32) -> Vec<Complex32> {
    let w = 2.0 * std::f64::consts::PI * cfo_hz as f64 / 2.0e6;
    sig.iter()
        .enumerate()
        .map(|(i, &v)| v * Complex32::from_polar(1.0, (w * i as f64) as f32))
        .collect()
}

/// Fractional delay via linear interpolation (crude but adequate for tests).
fn frac_delay(sig: &[Complex32], mu: f32) -> Vec<Complex32> {
    (0..sig.len().saturating_sub(1))
        .map(|i| sig[i] * (1.0 - mu) + sig[i + 1] * mu)
        .collect()
}

fn lead_noise(n: usize, amp: f32, rng: &mut Rng) -> Vec<Complex32> {
    (0..n)
        .map(|_| Complex32::new(rng.gauss(), rng.gauss()) * amp * std::f32::consts::FRAC_1_SQRT_2)
        .collect()
}

fn silence(n: usize) -> Vec<Complex32> {
    vec![Complex32::new(0.0, 0.0); n]
}

fn run_rx_cfg(stream: &[Complex32], chunk: usize, cfg: RxConfig) -> Vec<RxEvent> {
    let mut rx = Receiver::new(cfg);
    let mut ev = Vec::new();
    for c in stream.chunks(chunk.max(1)) {
        rx.process(c, &mut ev);
    }
    rx.finish(&mut ev);
    ev
}

fn run_rx(stream: &[Complex32], chunk: usize) -> Vec<RxEvent> {
    run_rx_cfg(stream, chunk, RxConfig::default())
}

fn psdus(events: &[RxEvent]) -> Vec<&Vec<u8>> {
    events
        .iter()
        .filter_map(|e| match e {
            RxEvent::PsduReceived { psdu, .. } => Some(psdu),
            _ => None,
        })
        .collect()
}

/// An aggregated PPDU carries the PSDU *capacity* of its symbols, so the
/// receiver returns the sent octets followed by the transmitter's padding.
fn carries(got: &[u8], sent: &[u8]) -> bool {
    got.len() >= sent.len() && got[..sent.len()] == sent[..]
}

fn ends(events: &[RxEvent]) -> Vec<&RxEndStatus> {
    events
        .iter()
        .filter_map(|e| match e {
            RxEvent::RxEnd { status, .. } => Some(status),
            _ => None,
        })
        .collect()
}

#[test]
fn all_mcs_ideal_channel_both_codings() {
    let tx = Transmitter::new();
    let mut rng = Rng(1);
    for coding in [Coding::Bcc, Coding::Ldpc] {
        for mcs in valid_mcs() {
            let psdu = rng.bytes(200);
            let txv = TxVector { mcs, fec_coding: coding, ..Default::default() };
            let wave = tx.generate(&txv, &psdu).unwrap();
            let mut stream = lead_noise(500, 1e-4, &mut rng);
            stream.extend(&wave);
            stream.extend(lead_noise(300, 1e-4, &mut rng));
            let ev = run_rx(&stream, 1024);
            let got = psdus(&ev);
            assert_eq!(got.len(), 1, "MCS {mcs} {coding:?}: {ev:?}");
            assert_eq!(got[0], &psdu, "MCS {mcs} {coding:?}");
            let start = ev.iter().find_map(|e| match e {
                RxEvent::RxStart { rxvector, .. } => Some(rxvector.clone()),
                _ => None,
            });
            let r = start.expect("RxStart");
            assert_eq!(r.fec_coding, coding);
            assert_eq!(r.preamble_type, PreambleType::S1gShort);
            assert!(ends(&ev).contains(&&RxEndStatus::NoError));
        }
    }
}

#[test]
fn awgn_25db_all_mcs_both_codings() {
    let tx = Transmitter::new();
    let mut rng = Rng(2);
    for coding in [Coding::Bcc, Coding::Ldpc] {
        for mcs in valid_mcs() {
            // 256/1024-QAM need more than 25 dB — test those at 35 dB.
            let snr = if mcs >= 8 { 35.0 } else { 25.0 };
            let mut ok = 0;
            let trials = 4;
            for _ in 0..trials {
                let psdu = rng.bytes(150);
                let txv = TxVector { mcs, fec_coding: coding, ..Default::default() };
                let wave = tx.generate(&txv, &psdu).unwrap();
                let mut stream = silence(300);
                stream.extend(&wave);
                stream.extend(silence(300));
                let noisy = awgn(&stream, snr, &mut rng);
                let ev = run_rx(&noisy, 777);
                if psdus(&ev).first().is_some_and(|p| *p == &psdu) {
                    ok += 1;
                }
            }
            assert_eq!(ok, trials, "MCS {mcs} {coding:?} at {snr} dB");
        }
    }
}

#[test]
fn ldpc_gains_over_bcc_at_low_snr() {
    // MCS 1 (QPSK 1/2), 700-octet aggregated PPDUs near the sensitivity
    // limit: LDPC must not do worse than BCC (it typically does noticeably
    // better) and must mostly succeed.
    let tx = Transmitter::new();
    let mut rng = Rng(77);
    let trials = 12;
    let mut summary = Vec::new();
    for snr in [5.0f32, 6.5, 8.0, 10.0] {
        let mut ok = [0usize; 2];
        for (ci, coding) in [Coding::Bcc, Coding::Ldpc].into_iter().enumerate() {
            for _ in 0..trials {
                let psdu = rng.bytes(700);
                let txv = TxVector { mcs: 1, fec_coding: coding, aggregation: true, ..Default::default() };
                let wave = tx.generate(&txv, &psdu).unwrap();
                let mut stream = silence(300);
                stream.extend(&wave);
                stream.extend(silence(300));
                let noisy = awgn(&stream, snr, &mut rng);
                let ev = run_rx(&noisy, 2048);
                if psdus(&ev).first().is_some_and(|p| carries(p, &psdu)) {
                    ok[ci] += 1;
                }
            }
        }
        eprintln!("MCS 1 @ {snr} dB: BCC {}/{trials}, LDPC {}/{trials}", ok[0], ok[1]);
        summary.push((snr, ok));
    }
    // With 12 trials per point a one-PPDU difference is noise; LDPC must be
    // within that of BCC everywhere and solid at 8 dB.
    for (snr, ok) in &summary {
        assert!(ok[1] + 1 >= ok[0], "@ {snr} dB LDPC {} vs BCC {} successes of {trials}", ok[1], ok[0]);
    }
    let at8 = summary.iter().find(|(s, _)| *s == 8.0).unwrap().1;
    assert!(at8[1] >= trials * 3 / 4, "LDPC only {} of {trials} at 8 dB", at8[1]);
}

#[test]
fn cfo_robustness() {
    // ±20 ppm at 1.25 GHz = ±25 kHz per side; test well beyond.
    let tx = Transmitter::new();
    let mut rng = Rng(3);
    for cfo in [-55e3f32, -25e3, -3e3, 0.0, 7e3, 25e3, 55e3] {
        for mcs in [0u8, 4, 8] {
            let psdu = rng.bytes(120);
            let txv = TxVector { mcs, ..Default::default() };
            let wave = tx.generate(&txv, &psdu).unwrap();
            let mut stream = lead_noise(400, 1e-3, &mut rng);
            stream.extend(&wave);
            stream.extend(silence(200));
            let shifted = apply_cfo(&stream, cfo);
            let noisy = awgn(&shifted, 30.0, &mut rng);
            let ev = run_rx(&noisy, 500);
            let got = psdus(&ev);
            assert_eq!(got.len(), 1, "cfo {cfo} mcs {mcs}: {ev:?}");
            assert_eq!(got[0], &psdu, "cfo {cfo} mcs {mcs}");
            let m = ev.iter().find_map(|e| match e {
                RxEvent::PsduReceived { metrics, .. } => Some(metrics.clone()),
                _ => None,
            });
            let m = m.unwrap();
            assert!((m.cfo_hz - cfo).abs() < 300.0, "cfo est {} vs {cfo}", m.cfo_hz);
        }
    }
}

#[test]
fn timing_and_amplitude_robustness() {
    let tx = Transmitter::new();
    let mut rng = Rng(4);
    for (lead, mu, amp) in [(0usize, 0.0f32, 1.0f32), (17, 0.3, 0.05), (301, 0.5, 3.0), (999, 0.9, 0.2)] {
        let psdu = rng.bytes(64);
        let txv = TxVector { mcs: 6, ..Default::default() };
        let wave = tx.generate(&txv, &psdu).unwrap();
        let mut stream = silence(lead);
        stream.extend(wave.iter().map(|&v| v * amp));
        stream.extend(silence(200));
        let d = frac_delay(&stream, mu);
        let noisy = awgn(&d, 28.0, &mut rng);
        let ev = run_rx(&noisy, 333);
        assert_eq!(psdus(&ev).first().map(|p| *p == &psdu), Some(true), "lead {lead} mu {mu} amp {amp}: {ev:?}");
    }
}

/// Static two-path channel: direct path plus an echo `delay` samples later.
fn two_path(sig: &[Complex32], delay: usize, gain: Complex32) -> Vec<Complex32> {
    (0..sig.len())
        .map(|i| sig[i] + if i >= delay { sig[i - delay] * gain } else { Complex32::new(0.0, 0.0) })
        .collect()
}

#[test]
fn sampling_clock_offset_max_length_ppdu() {
    // A 511-symbol PPDU (20.7 ms) with ±40 ppm clock mismatch (both ends at
    // the ±20 ppm tolerance of 23.3.17.3), CFO and a 2 µs echo. The FFT
    // window would drift ~1.6 samples over the PPDU; the tracker keeps it
    // inside the ISI-free part of the guard interval.
    let tx = Transmitter::new();
    let mut rng = Rng(5);
    let echo = Complex32::new(0.4, -0.2);
    for coding in [Coding::Bcc, Coding::Ldpc] {
        // MCS 0: 1659 octets → 511 symbols (BCC); LDPC needs one fewer
        // to stay ≤ 511 with the extra symbol.
        let len = if coding == Coding::Bcc { 1659 } else { 1600 };
        let psdu = rng.bytes(len);
        let txv = TxVector { mcs: 0, fec_coding: coding, aggregation: true, ..Default::default() };
        let wave = tx.generate(&txv, &psdu).unwrap();
        assert!(wave.len() >= 480 + 80 * 490, "{}", wave.len());
        for ppm in [-40.0f64, 40.0] {
            let mut stream = silence(400);
            stream.extend(&wave);
            stream.extend(silence(400));
            let stretched = s2g_dsp::apply_sfo_ppm(&two_path(&stream, 4, echo), ppm);
            let shifted = apply_cfo(&stretched, 12_000.0);
            let noisy = awgn(&shifted, 20.0, &mut rng);
            let ev = run_rx(&noisy, 4096);
            let got = psdus(&ev);
            assert_eq!(got.len(), 1, "{coding:?} ppm {ppm}: ends {:?}", ends(&ev));
            assert!(carries(got[0], &psdu), "{coding:?} ppm {ppm}");
            let m = ev
                .iter()
                .find_map(|e| match e {
                    RxEvent::PsduReceived { metrics, .. } => Some(metrics.clone()),
                    _ => None,
                })
                .unwrap();
            // Expected drift over ~41 000 samples at 40 ppm ≈ 1.6 samples;
            // a fast receiver clock (+ppm) makes the signal appear late.
            let expect = ppm as f32 * 1e-6 * (wave.len() as f32);
            assert!(
                (m.timing_drift_samples - expect).abs() < 0.6,
                "{coding:?} ppm {ppm}: drift {} vs {expect}",
                m.timing_drift_samples
            );
        }
    }
    // Far beyond spec (−200 ppm ≈ 8 samples of drift) the tracker is what
    // keeps the window out of the echo's ISI: with tracking off the same
    // stream must fail, with it on it must decode.
    let psdu = rng.bytes(1659);
    let txv = TxVector { mcs: 0, aggregation: true, ..Default::default() };
    let wave = tx.generate(&txv, &psdu).unwrap();
    let mut stream = silence(400);
    stream.extend(&wave);
    stream.extend(silence(400));
    let stretched = s2g_dsp::apply_sfo_ppm(&two_path(&stream, 6, echo), -200.0);
    let noisy = awgn(&stretched, 20.0, &mut rng);
    let ev_off = run_rx_cfg(&noisy, 4096, RxConfig { timing_tracking: false, ..Default::default() });
    assert!(psdus(&ev_off).first().is_none_or(|p| !carries(p, &psdu)), "decoded without tracking?");
    let ev_on = run_rx(&noisy, 4096);
    assert_eq!(psdus(&ev_on).first().map(|p| carries(p, &psdu)), Some(true), "{:?}", ends(&ev_on));
}

#[test]
fn traveling_pilots_roundtrip_with_impairments() {
    let tx = Transmitter::new();
    let mut rng = Rng(6);
    for coding in [Coding::Bcc, Coding::Ldpc] {
        for mcs in [0u8, 3, 7, 11] {
            let snr = if mcs >= 8 { 35.0 } else { 27.0 };
            let psdu = rng.bytes(300);
            let txv = TxVector { mcs, fec_coding: coding, traveling_pilots: true, ..Default::default() };
            let wave = tx.generate(&txv, &psdu).unwrap();
            let mut stream = silence(300);
            stream.extend(&wave);
            stream.extend(silence(300));
            let shifted = apply_cfo(&stream, -9_000.0);
            let noisy = awgn(&frac_delay(&shifted, 0.4), snr, &mut rng);
            let ev = run_rx(&noisy, 900);
            let start = ev.iter().find_map(|e| match e {
                RxEvent::RxStart { rxvector, .. } => Some(rxvector.clone()),
                _ => None,
            });
            assert!(start.is_some_and(|r| r.traveling_pilots), "TP bit not signalled");
            assert_eq!(psdus(&ev).first().map(|p| *p == &psdu), Some(true), "MCS {mcs} {coding:?}: {:?}", ends(&ev));
        }
    }
}

#[test]
fn traveling_pilots_track_a_drifting_channel() {
    // A frequency-selective channel whose echo grows during a long PPDU:
    // with traveling pilots every tone's estimate is refreshed every 14
    // symbols, so the constellation stays clean (lower EVM) and the PPDU
    // decodes; fixed pilots can only follow the common phase.
    let tx = Transmitter::new();
    let mut rng = Rng(8);
    let psdu = rng.bytes(1500); // MCS 2: ~154 symbols = 11 pilot periods
    let make = |tp: bool| {
        let txv = TxVector { mcs: 2, aggregation: true, traveling_pilots: tp, scrambler_seed: Some(41), ..Default::default() };
        tx.generate(&txv, &psdu).unwrap()
    };
    // Time-varying two-tap channel: echo amplitude ramps 0 → 0.6.
    let channel = |w: &[Complex32]| -> Vec<Complex32> {
        let n = w.len();
        let mut out = vec![Complex32::new(0.0, 0.0); n];
        for i in 0..n {
            let a = 0.6 * (i as f32 / n as f32);
            out[i] = w[i] + if i >= 3 { w[i - 3] * Complex32::new(a, -0.4 * a) } else { Complex32::new(0.0, 0.0) };
        }
        out
    };
    let mut evm = [0.0f32; 2];
    let mut ok = [false; 2];
    for (i, tp) in [false, true].into_iter().enumerate() {
        let mut stream = silence(300);
        stream.extend(channel(&make(tp)));
        stream.extend(silence(300));
        let noisy = awgn(&stream, 30.0, &mut rng);
        let ev = run_rx(&noisy, 2000);
        ok[i] = psdus(&ev).first().is_some_and(|p| carries(p, &psdu));
        evm[i] = ev
            .iter()
            .find_map(|e| match e {
                RxEvent::PsduReceived { metrics, .. } => Some(metrics.evm_db),
                _ => None,
            })
            .unwrap_or(0.0);
    }
    assert!(ok[1], "traveling pilots should track the drifting channel");
    assert!(evm[1] < evm[0] - 3.0, "TP EVM {} dB should beat fixed-pilot EVM {} dB", evm[1], evm[0]);
}

#[test]
fn chunked_feeding_extremes() {
    let tx = Transmitter::new();
    let mut rng = Rng(9);
    let psdu = rng.bytes(90);
    let txv = TxVector { mcs: 2, ..Default::default() };
    let wave = tx.generate(&txv, &psdu).unwrap();
    let mut stream = silence(250);
    stream.extend(&wave);
    stream.extend(silence(250));
    let noisy = awgn(&stream, 30.0, &mut rng);
    for chunk in [1usize, 7, 64, 79, 80, 81, 1000, 100_000] {
        let ev = run_rx(&noisy, chunk);
        assert_eq!(psdus(&ev), vec![&psdu], "chunk {chunk}: {ev:?}");
    }
}

#[test]
fn back_to_back_ppdus() {
    let tx = Transmitter::new();
    let mut rng = Rng(10);
    let mut stream = silence(100);
    let mut expect = Vec::new();
    for (i, mcs) in [0u8, 5, 11, 1, 8].into_iter().enumerate() {
        let psdu = rng.bytes(40 + 30 * i);
        let coding = if i % 2 == 0 { Coding::Bcc } else { Coding::Ldpc };
        let txv = TxVector { mcs, fec_coding: coding, ..Default::default() };
        stream.extend(tx.generate(&txv, &psdu).unwrap());
        // Gap of a few symbols (or none for one pair).
        stream.extend(silence(if i == 2 { 0 } else { 160 }));
        expect.push(psdu);
    }
    stream.extend(silence(300));
    let noisy = awgn(&stream, 30.0, &mut rng);
    let ev = run_rx(&noisy, 512);
    let got: Vec<Vec<u8>> = psdus(&ev).into_iter().cloned().collect();
    assert_eq!(got, expect);
    assert_eq!(ends(&ev).iter().filter(|s| ***s == RxEndStatus::NoError).count(), 5);
}

#[test]
fn carrier_lost_then_recovery() {
    let tx = Transmitter::new();
    let mut rng = Rng(11);
    let psdu1 = rng.bytes(400);
    let psdu2 = rng.bytes(50);
    let w1 = tx.generate(&TxVector { mcs: 0, ..Default::default() }, &psdu1).unwrap();
    let w2 = tx.generate(&TxVector { mcs: 3, ..Default::default() }, &psdu2).unwrap();
    let cut = 480 + 80 * 20; // 20 of ~124 data symbols
    let mut stream = silence(200);
    stream.extend(&w1[..cut]);
    // Signal vanishes: carrier lost. The PHY waits out the intended end of
    // PPDU 1 (CCA BUSY) before it will look for PPDU 2.
    stream.extend(silence(w1.len() - cut + 200));
    stream.extend(&w2);
    stream.extend(silence(300));
    let noisy = awgn(&stream, 30.0, &mut rng);
    let ev = run_rx(&noisy, 300);
    let st = ends(&ev);
    assert!(st.contains(&&RxEndStatus::CarrierLost), "{st:?}");
    assert_eq!(psdus(&ev), vec![&psdu2], "{st:?}");
    let holds: Vec<(u64, u32)> = ev
        .iter()
        .filter_map(|e| match e {
            RxEvent::Cca { sample_index, busy: true, reason: Some(CcaReason::PpduHold), hold_us } => {
                Some((*sample_index, *hold_us))
            }
            _ => None,
        })
        .collect();
    assert_eq!(holds.len(), 1, "{holds:?}");
    // Hold runs from the point of loss to the intended PPDU end.
    let intended_end = 200 + w1.len() as u64;
    let predicted_end = holds[0].0 + 2 * holds[0].1 as u64;
    assert!((predicted_end as i64 - intended_end as i64).abs() <= 8, "{predicted_end} vs {intended_end}");
}

#[test]
fn truncated_stream_reports_truncated() {
    let tx = Transmitter::new();
    let mut rng = Rng(12);
    let psdu = rng.bytes(400);
    let w = tx.generate(&TxVector { mcs: 0, ..Default::default() }, &psdu).unwrap();
    let mut stream = silence(200);
    stream.extend(&w[..480 + 80 * 30]);
    let ev = run_rx(&awgn(&stream, 30.0, &mut rng), 256);
    assert!(ends(&ev).contains(&&RxEndStatus::Truncated), "{:?}", ends(&ev));
    assert!(psdus(&ev).is_empty());
}

#[test]
fn format_violation_on_corrupted_sig() {
    let tx = Transmitter::new();
    let mut rng = Rng(13);
    let psdu = rng.bytes(60);
    let mut w = tx.generate(&TxVector { mcs: 2, ..Default::default() }, &psdu).unwrap();
    // Wreck the second SIG symbol.
    for v in &mut w[400..480] {
        *v = Complex32::new(rng.gauss(), rng.gauss()) * 0.25;
    }
    let mut stream = silence(200);
    stream.extend(&w);
    stream.extend(silence(300));
    let ev = run_rx(&awgn(&stream, 30.0, &mut rng), 256);
    assert!(ends(&ev).contains(&&RxEndStatus::FormatViolation), "{:?}", ends(&ev));
    assert!(psdus(&ev).is_empty());
    assert!(!ev.iter().any(|e| matches!(e, RxEvent::RxStart { .. })));
}

#[test]
fn aggregated_ppdu_both_codings() {
    let tx = Transmitter::new();
    let mut rng = Rng(14);
    for coding in [Coding::Bcc, Coding::Ldpc] {
        let psdu = rng.bytes(1300);
        let txv = TxVector { mcs: 7, fec_coding: coding, aggregation: true, ..Default::default() };
        let wave = tx.generate(&txv, &psdu).unwrap();
        let mut stream = silence(200);
        stream.extend(&wave);
        stream.extend(silence(200));
        let noisy = awgn(&apply_cfo(&stream, 4000.0), 28.0, &mut rng);
        let ev = run_rx(&noisy, 2048);
        let start = ev.iter().find_map(|e| match e {
            RxEvent::RxStart { rxvector, .. } => Some(rxvector.clone()),
            _ => None,
        });
        let r = start.expect("RxStart");
        assert!(r.aggregation);
        assert_eq!(r.fec_coding, coding);
        // PSDU capacity (the TX pads nothing: the MAC would) may exceed the
        // PSDU we sent; the receiver returns exactly psdu_length octets.
        let got = psdus(&ev);
        assert_eq!(got.len(), 1, "{coding:?}: {:?}", ends(&ev));
        assert_eq!(&got[0][..psdu.len()], &psdu[..], "{coding:?}");
        assert_eq!(got[0].len(), r.psdu_length);
    }
}

#[test]
fn ndp_roundtrip_with_impairments() {
    let tx = Transmitter::new();
    let mut rng = Rng(15);
    let body = 0x000A_5A5A_5A5B_u64 & ((1 << 37) - 1);
    let w = tx.generate_ndp(body).unwrap();
    let mut stream = silence(300);
    stream.extend(&w);
    stream.extend(silence(300));
    let noisy = awgn(&apply_cfo(&stream, -21_000.0), 22.0, &mut rng);
    let ev = run_rx(&noisy, 128);
    let got: Vec<u64> = ev
        .iter()
        .filter_map(|e| match e {
            RxEvent::NdpReceived { body, .. } => Some(*body),
            _ => None,
        })
        .collect();
    assert_eq!(got, vec![body], "{ev:?}");
}

/// Build an S1G_LONG PPDU preamble (STF ‖ LTF1 ‖ SIG-A) followed by a
/// stand-in for D-STF/D-LTF/SIG-B/Data of the signalled duration.
fn s1g_long_ppdu(fields: &SigASu, rng: &mut Rng) -> (Vec<Complex32>, usize) {
    let mut w = Vec::new();
    w.extend(preamble::stf_time());
    w.extend(preamble::ltf1_time());
    w.extend(sig::encode_sig_a_su(fields));
    let rxv = match fields.verdict() {
        sig::SigVerdict::Unsupported(r, _) => r,
        other => panic!("{other:?}"),
    };
    let total = rxv.ppdu_duration_us() as usize * 2;
    while w.len() < total {
        w.push(Complex32::new(rng.gauss(), rng.gauss()) * 0.7);
    }
    (w.into_iter().map(|v| v * 0.25).collect(), total)
}

#[test]
fn s1g_long_sig_a_is_decoded_and_cca_held() {
    let tx = Transmitter::new();
    let mut rng = Rng(16);
    let fields = SigASu {
        stbc: false,
        uplink_indication: false,
        bandwidth: 0,
        nsts: 1, // 2 space-time streams → N_LTF = 2
        id: (9 << 3) | 2,
        short_gi: false,
        ldpc: false,
        ldpc_extra: true,
        mcs: 4,
        beam_change_or_smoothing: true,
        aggregation: true,
        length: 25,
        response_indication: ResponseIndication::Normal,
        traveling_pilots: false,
    };
    let (long, total) = s1g_long_ppdu(&fields, &mut rng);
    let psdu = rng.bytes(80);
    let short = tx.generate(&TxVector { mcs: 1, ..Default::default() }, &psdu).unwrap();
    let mut stream = silence(300);
    stream.extend(&long);
    stream.extend(silence(100));
    stream.extend(&short);
    stream.extend(silence(400));
    let noisy = awgn(&apply_cfo(&stream, 15_000.0), 25.0, &mut rng);
    let ev = run_rx(&noisy, 700);
    let starts: Vec<_> = ev
        .iter()
        .filter_map(|e| match e {
            RxEvent::RxStart { rxvector, .. } => Some(rxvector.clone()),
            _ => None,
        })
        .collect();
    assert_eq!(starts.len(), 2, "{ev:?}");
    let r = &starts[0];
    assert_eq!(r.preamble_type, PreambleType::S1gLong);
    assert_eq!(r.num_sts, 2);
    assert_eq!(r.mcs, 4);
    assert_eq!(r.partial_aid, 9);
    assert_eq!(r.color, 2);
    assert_eq!(r.response_indication, ResponseIndication::Normal);
    assert_eq!(r.n_sym, 25);
    assert_eq!(r.ppdu_duration_us() as usize * 2, total);
    assert!(ends(&ev).iter().any(|s| matches!(s, RxEndStatus::UnsupportedRate(_))), "{:?}", ends(&ev));
    // CCA was held for the remainder of the long PPDU.
    let hold = ev.iter().find_map(|e| match e {
        RxEvent::Cca { busy: true, reason: Some(CcaReason::PpduHold), hold_us, .. } => Some(*hold_us),
        _ => None,
    });
    let hold = hold.expect("PpduHold");
    let expect = r.ppdu_duration_us() - 240;
    assert!((hold as i64 - expect as i64).abs() <= 8, "hold {hold} vs {expect}");
    // …and the S1G_SHORT PPDU right after it decoded normally.
    assert_eq!(psdus(&ev), vec![&psdu]);
}

#[test]
fn cca_energy_detect_and_ppdu_hold() {
    let tx = Transmitter::new();
    let mut rng = Rng(17);
    let psdu = rng.bytes(100);
    let w = tx.generate(&TxVector { mcs: 0, ..Default::default() }, &psdu).unwrap();
    let mut stream = silence(800);
    stream.extend(&w);
    stream.extend(silence(800));
    // Uncalibrated: thresholds apply to dBFS; the PPDU at −12 dBFS is far
    // above −72, the −80 dBFS noise floor is below.
    let ev = run_rx(&awgn(&stream, 60.0, &mut rng), 400);
    let cca: Vec<(u64, bool)> = ev
        .iter()
        .filter_map(|e| match e {
            RxEvent::Cca { sample_index, busy, .. } => Some((*sample_index, *busy)),
            _ => None,
        })
        .collect();
    assert!(cca.len() >= 2, "{cca:?}");
    assert!(cca[0].1, "first transition must be BUSY: {cca:?}");
    assert!(cca[0].0 <= 800 + 80, "busy late: {cca:?}");
    let last = cca.last().unwrap();
    assert!(!last.1, "must end IDLE: {cca:?}");
    let ppdu_end = 800 + w.len() as u64;
    assert!(last.0 >= ppdu_end && last.0 <= ppdu_end + 160, "idle at {} vs end {ppdu_end}", last.0);
    assert_eq!(psdus(&ev), vec![&psdu]);
    // Pure energy (a CW tone) with no preamble: BUSY by energy detect only.
    let cw: Vec<Complex32> = (0..3000).map(|i| Complex32::from_polar(0.2, 0.05 * i as f32)).collect();
    let mut s2 = silence(400);
    s2.extend(cw);
    s2.extend(silence(400));
    let ev2 = run_rx(&s2, 500);
    assert!(ev2.iter().any(|e| matches!(e, RxEvent::Cca { busy: true, reason: Some(CcaReason::EnergyDetect), .. })));
    assert!(!ev2.iter().any(|e| matches!(e, RxEvent::RxStart { .. })));
}

#[test]
fn rxvector_measurements() {
    let tx = Transmitter::new();
    let mut rng = Rng(18);
    let psdu = rng.bytes(64);
    let w = tx.generate(&TxVector { mcs: 2, scrambler_seed: Some(88), ..Default::default() }, &psdu).unwrap();
    let mut stream = silence(300);
    stream.extend(w.iter().map(|&v| v * 0.5)); // −18 dBFS
    stream.extend(silence(300));
    let cfg = RxConfig { cal_offset_db: -30.0, ..Default::default() };
    let ev = run_rx_cfg(&awgn(&stream, 20.0, &mut rng), 512, cfg);
    let r = ev
        .iter()
        .find_map(|e| match e {
            RxEvent::PsduReceived { rxvector, .. } => Some(rxvector.clone()),
            _ => None,
        })
        .expect("psdu");
    assert_eq!(r.scrambler_seed, 88);
    assert!((r.rssi_dbfs + 18.0).abs() < 1.5, "rssi {}", r.rssi_dbfs);
    assert!((r.rcpi_dbm + 48.0).abs() < 1.5, "rcpi {}", r.rcpi_dbm);
    assert_eq!(r.rcpi, s2g_phy::params::rf::rcpi_encode(r.rcpi_dbm));
    assert!(r.rssi > 200 && r.rssi < 230, "rssi code {}", r.rssi);
    assert!((r.snr_db - 20.0).abs() < 3.0, "snr {}", r.snr_db);
    assert_eq!(r.length, 64);
}

#[test]
fn tx_spectral_occupancy() {
    // FFT of data-symbol payloads: energy confined to tones −28..28, DC null.
    let tx = Transmitter::new();
    let psdu = vec![0x5Au8; 100];
    let wave = tx.generate(&TxVector { mcs: 8, scrambler_seed: Some(51), ..Default::default() }, &psdu).unwrap();
    let nsym = (wave.len() - 480) / 80;
    let mut used = vec![0.0f64; 64];
    for n in 0..nsym {
        let payload = &wave[480 + n * 80 + 16..480 + n * 80 + 80];
        let f = s2g_phy::ofdm::fft_symbol(payload);
        for a in 0..64 {
            used[a] += f[a].norm_sqr() as f64;
        }
    }
    let dc = used[32];
    let occupied: f64 = (4..61).filter(|&a| a != 32).map(|a| used[a]).sum();
    let guards: f64 = (0..4).chain(61..64).map(|a| used[a]).sum();
    assert!(dc < occupied * 1e-6, "DC leakage");
    assert!(guards < occupied * 1e-6, "guard-tone leakage");
}

#[test]
fn spectral_mask_after_interpolation() {
    // Interpolate 2× twice (8 MS/s) to examine the full ±3 MHz mask.
    let tx = Transmitter::new();
    let mut rng = Rng(19);
    let psdu = rng.bytes(500);
    let wave = tx.generate(&TxVector { mcs: 5, aggregation: false, ..Default::default() }, &psdu).unwrap();
    let mut up4 = Vec::new();
    let mut i1 = s2g_dsp::HalfbandInterp2::new();
    i1.process(&wave, &mut up4);
    i1.process(&silence(64), &mut up4);
    let mut up8 = Vec::new();
    let mut i2 = s2g_dsp::HalfbandInterp2::new();
    i2.process(&up4, &mut up8);
    i2.process(&silence(64), &mut up8);
    let r = s2g_phy::conformance::spectral_mask(&up8, 8.0e6);
    let worst = r.bins.iter().min_by(|a, b| (a.2 - a.1).partial_cmp(&(b.2 - b.1)).unwrap()).unwrap();
    assert!(r.pass, "mask violated: worst margin {} dB at {} MHz (psd {} dBr, mask {} dBr)", r.worst_margin_db, worst.0, worst.1, worst.2);
}

