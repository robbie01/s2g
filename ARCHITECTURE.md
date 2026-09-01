# s2g — IEEE 802.11ah (S1G) PHY in Rust

A modular implementation of the IEEE 802.11-2024 Clause 23 S1G PHY, targeting the
ADALM-Pluto SDR at a **nonstandard carrier of 1250 MHz**.

## Scope (v1)

| Aspect | Choice | Rationale |
|---|---|---|
| Bandwidth | 2 MHz only | requirement |
| Guard interval | Long GI (8 µs) only | requirement |
| MCS | 0–8 and 11 (all valid for 2 MHz / 1 SS; 9/10/12 are "Not valid" per Table 23-46) | requirement ("all MCSes") |
| Spatial streams | 1 | Pluto is 1×1 |
| Coding | BCC (mandatory mode) | LDPC is optional in the spec; module boundaries allow adding it |
| Preamble | S1G_SHORT | the ≥2 MHz SU format |
| PPDU | SU data PPDUs + NDP CMAC PPDUs (37-bit body) | MAC (OCB/non-BSS) comes next |

## Crate layout

```
crates/
  s2g-phy        # pure DSP, no SDR deps. TX: PSDU -> IQ samples @2 MS/s.
                 # RX: streaming state machine, IQ samples -> events (SIG, PSDU, errors).
  s2g-mac        # OCB (non-BSS) MAC: 802.11 data/ACK frames, wildcard BSSID, FCS,
                 # A-MPDU, LLC/SNAP, CSMA/backoff, ACK/retry, dedup. IO-free,
                 # clock-injected engine driven by PHY events; testable standalone.
  s2g-dsp        # rate conversion + generic DSP (halfband 2x resamplers) used at the SDR boundary.
  s2g-sdr        # hardware abstraction: SdrTx / SdrRx / SdrDevice traits. No hardware deps.
  s2g-sdr-pluto  # PlutoSDR backend: pure-Rust iiod network-protocol client (TCP 30431,
                 # docs/iiod-protocol.md) — no native libiio dependency. Optional feature of s2g-tools.
  s2g-tools      # binaries: s2g-tx, s2g-rx, s2g-sim (loopback with impairments),
                 # s2g-node (NIC <-> MAC <-> PHY <-> Pluto), file I/O (.cf32),
                 # Nic trait: TAP via tappers (unix, feature "tap") or Ethernet-over-UDP.
```

Dependency direction (arrows = "depends on"):

```
s2g-tools -> s2g-phy, s2g-dsp, s2g-sdr, [s2g-sdr-pluto]
s2g-sdr-pluto -> s2g-sdr
s2g-phy -> (num-complex, rustfft only)
```

A future RX-only system on a different SDR = `s2g-phy` + `s2g-sdr` + a new backend crate.
A future MAC sits on top of `s2g-phy`'s TXVECTOR/RXVECTOR-shaped API (`vector.rs`) and the
`RxEvent` stream; PHY characteristics needed for MAC timing (SIFS, slot time) are exported
from `s2g_phy::params`.

## Sample-rate plan

The PHY natively runs at **2 MS/s** (64-pt FFT, 31.25 kHz spacing; long-GI symbol = 80
samples = 40 µs). The AD9363 cannot stream below ~2.08 MS/s, so the Pluto runs at
**4 MS/s** and `s2g-dsp` halfband-resamples 2×: interpolate on TX, decimate on RX. The
resampler lives outside `s2g-phy`, so a different SDR that runs at 2 MS/s natively (or any
other integer relation) plugs in without touching the PHY.

## RX pipeline (inside s2g-phy)

Push-based: `Receiver::process(&mut self, &[Complex32], &mut Vec<RxEvent>)`. Internally:
STF autocorrelation detect -> coarse CFO -> LTF cross-correlation timing + fine CFO ->
channel estimate -> SIG decode (BPSK/QBPSK, rate-1/2 BCC, CRC) -> per-symbol pilot phase
tracking -> LLR demap -> deinterleave -> soft Viterbi -> descramble -> PSDU out with
metrics (SNR, CFO, EVM). Any failure emits `RxEvent::Error` and re-arms the detector.

## Spec digest

Implementation-grade notes distilled from IEEE 802.11-2024 live in `docs/spec-digest/`
(committed source of truth for constants; every table cites clause/page).
