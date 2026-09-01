//! Two full MAC+PHY nodes talking over a simulated air interface:
//! Ethernet in on one side, Ethernet out on the other, ACKs and retries
//! through the real waveform with noise and CFO.

use num_complex::Complex;
use s2g_mac::{Mac, MacAction, MacConfig, MacEvent};
use s2g_phy::rx::{Receiver, RxConfig, RxEvent};
use s2g_phy::Transmitter;

type C32 = Complex<f32>;

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    fn gauss(&mut self) -> f32 {
        let s: f32 = (0..6)
            .map(|_| ((self.next() >> 32) as f32 / (1u64 << 31) as f32) - 1.0)
            .sum();
        s / (2.0f32).sqrt()
    }
}

struct Node {
    mac: Mac,
    tx: Transmitter,
    rx: Receiver,
    events: Vec<MacEvent>,
}

impl Node {
    fn new(addr: [u8; 6], mcs: u8, ack: bool) -> Self {
        let mut cfg = MacConfig::new(addr);
        cfg.mcs = mcs;
        cfg.ack_enabled = ack;
        cfg.ack_timeout_us = 30_000;
        cfg.max_retries = 4;
        Self {
            mac: Mac::new(cfg),
            tx: Transmitter::new(),
            rx: Receiver::new(RxConfig::default()),
            events: Vec::new(),
        }
    }

    /// Receive a waveform: PHY → MAC events.
    fn hear(&mut self, wave: &[C32], now_us: u64) {
        let mut phy_events: Vec<RxEvent> = Vec::new();
        self.rx.process(wave, &mut phy_events);
        // Pad with silence so trailing state flushes.
        let silence = vec![C32::new(1e-4, 0.0); 600];
        self.rx.process(&silence, &mut phy_events);
        for ev in &phy_events {
            self.mac.on_phy_event(ev, now_us, &mut self.events);
        }
    }
}

fn channel(wave: &[C32], snr_db: f32, cfo_hz: f32, rng: &mut Rng) -> Vec<C32> {
    let p: f32 = wave.iter().map(|v| v.norm_sqr()).sum::<f32>() / wave.len() as f32;
    let sigma = (p / 10f32.powf(snr_db / 10.0) / 2.0).sqrt();
    let w = 2.0 * std::f64::consts::PI * cfo_hz as f64 / 2.0e6;
    let mut out: Vec<C32> = vec![C32::new(1e-4, 1e-4); 300];
    out.extend(wave.iter().enumerate().map(|(i, &v)| {
        v * C32::from_polar(1.0, (w * i as f64) as f32) + C32::new(rng.gauss() * sigma, rng.gauss() * sigma)
    }));
    out.extend(std::iter::repeat_n(C32::new(1e-4, -1e-4), 300));
    out
}

const A: [u8; 6] = [2, 0, 0, 0, 0, 0xA];
const B: [u8; 6] = [2, 0, 0, 0, 0, 0xB];

fn eth(dest: [u8; 6], src: [u8; 6], n: usize) -> Vec<u8> {
    let mut f = Vec::new();
    f.extend_from_slice(&dest);
    f.extend_from_slice(&src);
    f.extend_from_slice(&0x0800u16.to_be_bytes());
    f.extend((0..n).map(|i| (i * 7) as u8));
    f
}

/// Run both nodes for up to `max_ms` virtual milliseconds; `drop_tx` returns
/// true when a given transmission (by 0-based index) should be lost in the
/// air.
fn run(a: &mut Node, b: &mut Node, max_ms: u64, mut drop_tx: impl FnMut(usize) -> bool) {
    fn step(
        me: &mut Node,
        peer: &mut Node,
        now: u64,
        rng: &mut Rng,
        tx_count: &mut usize,
        drop_tx: &mut impl FnMut(usize) -> bool,
    ) {
        if let Some(MacAction::Transmit { txv, psdu }) = me.mac.poll(now, &mut me.events) {
            let idx = *tx_count;
            *tx_count += 1;
            let wave = me.tx.generate(&txv, &psdu).expect("phy tx");
            if !drop_tx(idx) {
                let air = channel(&wave, 25.0, 9_000.0, rng);
                peer.hear(&air, now);
            }
        }
    }
    let mut rng = Rng(0xABCD);
    let mut tx_count = 0usize;
    for i in 0..max_ms * 2 {
        let now = i * 500;
        step(a, b, now, &mut rng, &mut tx_count, &mut drop_tx);
        step(b, a, now, &mut rng, &mut tx_count, &mut drop_tx);
    }
}

#[test]
fn unicast_large_frame_with_ack() {
    let mut a = Node::new(A, 5, true);
    let mut b = Node::new(B, 5, true);
    let frame = eth(B, A, 1400); // forces A-MPDU aggregation
    a.mac.enqueue_eth(&frame).unwrap();
    run(&mut a, &mut b, 200, |_| false);

    let delivered: Vec<_> = b
        .events
        .iter()
        .filter_map(|e| match e {
            MacEvent::EthReceived(f) => Some(f),
            _ => None,
        })
        .collect();
    assert_eq!(delivered.len(), 1, "B events: {:?}", b.events);
    assert_eq!(delivered[0], &frame);
    assert!(
        a.events.iter().any(|e| matches!(e, MacEvent::TxComplete { acked: true, retries: 0, .. })),
        "A events: {:?}",
        a.events
    );
}

#[test]
fn broadcast_no_ack() {
    let mut a = Node::new(A, 2, true);
    let mut b = Node::new(B, 2, true);
    let frame = eth([0xff; 6], A, 200);
    a.mac.enqueue_eth(&frame).unwrap();
    run(&mut a, &mut b, 100, |_| false);
    assert!(b.events.iter().any(|e| matches!(e, MacEvent::EthReceived(f) if f == &frame)));
    assert!(a.events.iter().any(|e| matches!(e, MacEvent::TxComplete { acked: false, .. })));
    // No ACK should ever have been sent by B.
    assert_eq!(b.mac.poll(u64::MAX / 2, &mut b.events), None);
}

#[test]
fn lost_first_transmission_retried() {
    let mut a = Node::new(A, 3, true);
    let mut b = Node::new(B, 3, true);
    let frame = eth(B, A, 300);
    a.mac.enqueue_eth(&frame).unwrap();
    // Drop the very first over-the-air transmission (A's initial data PPDU).
    run(&mut a, &mut b, 400, |idx| idx == 0);

    let delivered: Vec<_> = b.events.iter().filter(|e| matches!(e, MacEvent::EthReceived(_))).collect();
    assert_eq!(delivered.len(), 1, "B events: {:?}", b.events);
    assert!(
        a.events
            .iter()
            .any(|e| matches!(e, MacEvent::TxComplete { acked: true, retries, .. } if *retries >= 1)),
        "A events: {:?}",
        a.events
    );
}
