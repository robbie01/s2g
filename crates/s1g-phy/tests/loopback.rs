//! End-to-end TX → channel → RX tests for every valid MCS, with impairments:
//! AWGN, CFO (beyond the spec's ±20 ppm at 1.25 GHz), timing offsets
//! (integer + fractional), amplitude scaling, chunked streaming, multiple
//! PPDUs, truncation, NDP.

use s1g_phy::params::valid_mcs;
use s1g_phy::rx::{Receiver, RxConfig, RxErrorKind, RxEvent};
use s1g_phy::vector::TxVector;
use s1g_phy::{Complex32, Transmitter};

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

fn run_rx(stream: &[Complex32], chunk: usize) -> Vec<RxEvent> {
    let mut rx = Receiver::new(RxConfig::default());
    let mut events = Vec::new();
    for c in stream.chunks(chunk.max(1)) {
        rx.process(c, &mut events);
    }
    rx.finish(&mut events);
    events
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

#[test]
fn all_mcs_ideal_channel() {
    let tx = Transmitter::new();
    let mut rng = Rng(1);
    for mcs in valid_mcs() {
        for len in [1usize, 40, 511] {
            let psdu = rng.bytes(len);
            let txv = TxVector { mcs, ..Default::default() };
            let wave = tx.generate(&txv, &psdu).unwrap();
            let mut stream = lead_noise(500, 1e-4, &mut rng);
            stream.extend(&wave);
            stream.extend(lead_noise(300, 1e-4, &mut rng));
            let ev = run_rx(&stream, 4096);
            let got = psdus(&ev);
            assert_eq!(got.len(), 1, "MCS {mcs} len {len}: events {ev:?}");
            assert_eq!(got[0], &psdu, "MCS {mcs} len {len}");
        }
    }
}

#[test]
fn awgn_25db_all_mcs() {
    let tx = Transmitter::new();
    let mut rng = Rng(2);
    for mcs in valid_mcs() {
        // MCS 11 (1024-QAM) needs more than 25 dB — test it at 35 dB.
        let snr = if mcs >= 8 { 35.0 } else { 25.0 };
        let psdu = rng.bytes(120);
        let txv = TxVector { mcs, ..Default::default() };
        let wave = tx.generate(&txv, &psdu).unwrap();
        let mut stream = lead_noise(400, 1e-3, &mut rng);
        stream.extend(&wave);
        stream.extend(lead_noise(200, 1e-3, &mut rng));
        let noisy = awgn(&stream, snr, &mut rng);
        let got_events = run_rx(&noisy, 1000);
        let got = psdus(&got_events);
        assert_eq!(got.len(), 1, "MCS {mcs} @ {snr} dB: {got_events:?}");
        assert_eq!(got[0], &psdu, "MCS {mcs} @ {snr} dB");
        // Metrics sanity.
        if let Some(RxEvent::PsduReceived { metrics, .. }) =
            got_events.iter().find(|e| matches!(e, RxEvent::PsduReceived { .. }))
        {
            assert!(metrics.snr_db > snr - 8.0 && metrics.snr_db < snr + 8.0, "MCS {mcs} snr metric {}", metrics.snr_db);
            assert!(metrics.evm_db < -14.0, "MCS {mcs} evm {}", metrics.evm_db);
        }
    }
}

#[test]
fn cfo_robustness() {
    let tx = Transmitter::new();
    let mut rng = Rng(3);
    let psdu = rng.bytes(200);
    for cfo in [-40e3f32, -12.5e3, 7e3, 40e3] {
        let txv = TxVector { mcs: 3, ..Default::default() };
        let wave = tx.generate(&txv, &psdu).unwrap();
        let mut stream = lead_noise(600, 1e-3, &mut rng);
        stream.extend(&wave);
        stream.extend(lead_noise(200, 1e-3, &mut rng));
        let shifted = apply_cfo(&stream, cfo);
        let noisy = awgn(&shifted, 25.0, &mut rng);
        let ev = run_rx(&noisy, 512);
        let got = psdus(&ev);
        assert_eq!(got.len(), 1, "cfo {cfo}: {ev:?}");
        assert_eq!(got[0], &psdu, "cfo {cfo}");
        if let Some(RxEvent::PsduReceived { metrics, .. }) =
            ev.iter().find(|e| matches!(e, RxEvent::PsduReceived { .. }))
        {
            assert!((metrics.cfo_hz - cfo).abs() < 300.0, "cfo est {} vs {cfo}", metrics.cfo_hz);
        }
    }
}

#[test]
fn timing_and_amplitude_robustness() {
    let tx = Transmitter::new();
    let mut rng = Rng(4);
    let psdu = rng.bytes(90);
    for (lead, mu, amp) in [(137usize, 0.0f32, 1.0f32), (911, 0.3, 0.1), (250, 0.7, 0.05), (64, 0.5, 0.5)] {
        let txv = TxVector { mcs: 4, ..Default::default() };
        let wave = tx.generate(&txv, &psdu).unwrap();
        let mut stream = lead_noise(lead, 1e-4, &mut rng);
        stream.extend(wave.iter().map(|&v| v * amp));
        stream.extend(lead_noise(300, 1e-4, &mut rng));
        let delayed = if mu > 0.0 { frac_delay(&stream, mu) } else { stream };
        let noisy = awgn(&delayed, 30.0, &mut rng);
        let ev = run_rx(&noisy, 333);
        let got = psdus(&ev);
        assert_eq!(got.len(), 1, "lead {lead} mu {mu} amp {amp}: {ev:?}");
        assert_eq!(got[0], &psdu, "lead {lead} mu {mu} amp {amp}");
    }
}

#[test]
fn chunked_feeding_extremes() {
    let tx = Transmitter::new();
    let mut rng = Rng(5);
    let psdu = rng.bytes(60);
    let wave = tx.generate(&TxVector { mcs: 1, ..Default::default() }, &psdu).unwrap();
    let mut stream = lead_noise(300, 1e-4, &mut rng);
    stream.extend(&wave);
    stream.extend(lead_noise(200, 1e-4, &mut rng));
    for chunk in [1usize, 7, 4096] {
        let got_events = run_rx(&stream, chunk);
        let got = psdus(&got_events);
        assert_eq!(got.len(), 1, "chunk {chunk}");
        assert_eq!(got[0], &psdu, "chunk {chunk}");
    }
}

#[test]
fn back_to_back_ppdus() {
    let tx = Transmitter::new();
    let mut rng = Rng(6);
    let p1 = rng.bytes(30);
    let p2 = rng.bytes(200);
    let w1 = tx.generate(&TxVector { mcs: 0, ..Default::default() }, &p1).unwrap();
    let w2 = tx.generate(&TxVector { mcs: 7, ..Default::default() }, &p2).unwrap();
    let mut stream = lead_noise(200, 1e-4, &mut rng);
    stream.extend(&w1);
    // SIFS-ish gap of 320 samples (160 µs).
    stream.extend(lead_noise(320, 1e-4, &mut rng));
    stream.extend(&w2);
    stream.extend(lead_noise(200, 1e-4, &mut rng));
    let ev = run_rx(&stream, 2048);
    let got = psdus(&ev);
    assert_eq!(got.len(), 2, "{ev:?}");
    assert_eq!(got[0], &p1);
    assert_eq!(got[1], &p2);
}

#[test]
fn truncated_ppdu_then_recovery() {
    let tx = Transmitter::new();
    let mut rng = Rng(7);
    let p1 = rng.bytes(300);
    let p2 = rng.bytes(50);
    let w1 = tx.generate(&TxVector { mcs: 0, ..Default::default() }, &p1).unwrap();
    let w2 = tx.generate(&TxVector { mcs: 2, ..Default::default() }, &p2).unwrap();
    let mut stream = lead_noise(100, 1e-4, &mut rng);
    stream.extend(&w1[..w1.len() / 2]); // cut mid-Data
    stream.extend(lead_noise(400, 1e-4, &mut rng));
    stream.extend(&w2);
    stream.extend(lead_noise(200, 1e-4, &mut rng));

    let mut rx = Receiver::new(RxConfig::default());
    let mut ev = Vec::new();
    for c in stream.chunks(1024) {
        rx.process(c, &mut ev);
    }
    // The truncated PPDU stalls waiting for samples; the reset below (or more
    // input) must not prevent the second PPDU from decoding... feed a reset:
    // in live streaming the samples keep flowing, so instead verify: second
    // PPDU already decoded? If not, it's because RX is stuck waiting for
    // PPDU-1 symbols that never arrive — the stream itself contains PPDU 2's
    // preamble inside what RX believes is PPDU 1's Data field.
    // A real receiver only re-arms via the Length timeout. We accept the
    // loss of PPDU 2 here but REQUIRE: finish() reports truncation and a
    // subsequent stream decodes cleanly.
    rx.finish(&mut ev);
    assert!(
        ev.iter().any(|e| matches!(e, RxEvent::Error { kind: RxErrorKind::Truncated, .. })),
        "{ev:?}"
    );
    let mut ev2 = Vec::new();
    let mut tail = lead_noise(100, 1e-4, &mut rng);
    tail.extend(&w2);
    tail.extend(lead_noise(100, 1e-4, &mut rng));
    rx.process(&tail, &mut ev2);
    rx.finish(&mut ev2);
    assert_eq!(psdus(&ev2).len(), 1);
    assert_eq!(psdus(&ev2)[0], &p2);
}

#[test]
fn aggregated_ppdu() {
    let tx = Transmitter::new();
    let mut rng = Rng(8);
    // Aggregation: PSDU fills the symbol capacity (MAC-side padding rule).
    // MCS 5, pick n_sym = 40 → capacity floor((40·208−14)/8) = 1038 octets.
    let psdu = rng.bytes(1038);
    let txv = TxVector { mcs: 5, aggregation: true, ..Default::default() };
    let wave = tx.generate(&txv, &psdu).unwrap();
    let mut stream = lead_noise(300, 1e-4, &mut rng);
    stream.extend(&wave);
    stream.extend(lead_noise(300, 1e-4, &mut rng));
    let ev = run_rx(&stream, 4096);
    let got = psdus(&ev);
    assert_eq!(got.len(), 1, "{ev:?}");
    assert_eq!(got[0].len(), 1038);
    assert_eq!(got[0], &psdu);
    // RXVECTOR should say aggregated with n_sym = 40.
    let rxv = ev
        .iter()
        .find_map(|e| match e {
            RxEvent::PsduReceived { rxvector, .. } => Some(rxvector),
            _ => None,
        })
        .unwrap();
    assert!(rxv.aggregation);
    assert_eq!(rxv.n_sym, 40);
}

#[test]
fn ndp_roundtrip_with_impairments() {
    let tx = Transmitter::new();
    let mut rng = Rng(9);
    let body: u64 = 0x0A5A_5A5A_5A & ((1 << 37) - 1);
    let wave = tx.generate_ndp(body).unwrap();
    let mut stream = lead_noise(400, 1e-3, &mut rng);
    stream.extend(&wave);
    stream.extend(lead_noise(400, 1e-3, &mut rng));
    let noisy = awgn(&apply_cfo(&stream, -20e3), 20.0, &mut rng);
    let ev = run_rx(&noisy, 777);
    let got: Vec<u64> = ev
        .iter()
        .filter_map(|e| match e {
            RxEvent::NdpReceived { body, .. } => Some(*body),
            _ => None,
        })
        .collect();
    assert_eq!(got, vec![body], "{ev:?}");
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
        let f = s1g_phy::ofdm::fft_symbol(payload);
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
