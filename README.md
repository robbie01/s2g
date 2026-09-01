# s2g — IEEE 802.11ah (S1G) PHY in Rust for PlutoSDR

**s2g** ("sub-2 GHz") is the working name of this mode — a pun on S1G, since this
deployment runs in the 24 cm band rather than sub-1 GHz.

A from-scratch, modular implementation of the IEEE 802.11-2024 Clause 23 **S1G PHY**:
2 MHz bandwidth, long GI (8 µs), single spatial stream, **BCC and LDPC**, **fixed and
traveling pilots**, S1G_SHORT preamble, **all valid MCSes (0–8 and 11**, i.e. BPSK½ …
256-QAM¾ and 1024-QAM¾; MCS 9/10/12 are "Not valid" at 2 MHz/1SS per Table 23-46**)**,
NDP CMAC PPDUs, and the mandatory receive procedures of 23.3.20: CCA, RSSI/RCPI,
S1G_LONG SIG-A detection/decoding, PHY-RXSTART/RXEND statuses, carrier-lost handling.
Runs on an ADALM-Pluto at a **nonstandard 1250 MHz carrier** — the carrier is just a
tuning parameter; the baseband is band-agnostic. 1 MHz operation is deliberately out of
scope.

See `ARCHITECTURE.md` for the crate layout, the scope table and design decisions, and
`docs/spec-digest/` for the implementation-grade spec notes (with clause/page citations)
that every constant in the code traces back to.

## Build & test

```sh
cargo build --release            # everything, incl. Pluto backend (pure Rust, no libiio)
cargo test --workspace           # 145 tests: golden vectors, LDPC matrices, roundtrips, full loopback,
                                 # SFO/echo channels, CCA/RXEND behaviour, two-node MAC exchanges
```

## Tools

```sh
# Simulate: PER vs SNR for every MCS with CFO + timing offset impairments
target/release/s2g-sim --count 100 --snr-db "3,6,9,12,15,18,21,25,30,36"
# …with LDPC, traveling pilots, a 40 ppm clock offset and a 2 µs echo
target/release/s2g-sim --ldpc --traveling-pilots --sfo-ppm 40 --echo-delay 4 --mcs 3

# Generate a waveform file (GNU Radio-compatible .cf32), 3 PPDUs at MCS 4
target/release/s2g-tx --mcs 4 --random 200 --count 3 --out wave.cf32 --out-rate 4e6
target/release/s2g-tx --mcs 4 --ldpc --traveling-pilots --random 200 --out wave.cf32
target/release/s2g-tx --ndp 0x0000000002 --out ndp.cf32        # an NDP CMAC PPDU

# Decode a waveform file (native 2 MS/s, or 4 MS/s with ×2 decimation); prints CCA,
# RXSTART (full RXVECTOR incl. RSSI/RCPI/SNR), RXEND statuses, PSDUs
target/release/s2g-rx --in wave.cf32 --rate 4e6 --cal-offset-db -30

# Live TX on a Pluto at 1250 MHz (device streams at 4 MS/s, ×2 interpolated)
target/release/s2g-tx --uri 192.168.2.1 --mcs 2 --hex "dead beef 0102" --count 10

# Live RX on a Pluto at 1250 MHz
target/release/s2g-rx --uri 192.168.2.1 --gain auto
```

The Pluto backend speaks the **iiod network protocol directly** (TCP 30431, the same
path libiio's `ip:` backend uses) — no native libiio install needed on the host. Reach
the Pluto at its usual USB-network address `192.168.2.1`.

## Hardware notes

- The AD9363 can't stream at 2 MS/s, so the radio runs at **4 MS/s** and `s2g-dsp`
  halfband-resamples ×2 in software (TX interpolate / RX decimate).
- 1250 MHz is inside the AD9363 tuning range but **outside every S1G regulatory band —
  transmit into a dummy load / cable / shielded box unless you're licensed for that
  spectrum.**
- Frequency accuracy: the RX tolerates ≳ ±55 kHz CFO (±44 ppm at 1.25 GHz) and ±40 ppm
  sampling-clock offset over a maximum-length PPDU, comfortably above the spec's ±20 ppm
  per-end budget, so a stock Pluto TCXO on both ends is fine.
- CCA / RCPI thresholds are specified in dBm; pass `--cal-offset-db` (dBm = dBFS +
  offset, measured for your gain setting) to `s2g-rx` / `s2g-node`. Uncalibrated (0),
  the thresholds simply act on dBFS.

## Networking: OCB MAC + TAP (`s2g-node`)

`s2g-mac` implements a nonstandard-where-it-matters OCB (non-BSS) MAC: 802.11 Data
frames with the wildcard BSSID, FCS, sequence numbers + dedup, RFC 1042 LLC/SNAP for
Ethernet payloads, spec-format A-MPDU aggregation for frames over 511 octets, CSMA
with DIFS + exponential backoff gated by PHY CCA, NAV and **RID** (response indication
deferral), and acknowledgement via **NDP Ack / NDP BlockAck** CMAC PPDUs (Ack ID from
the scrambler seed + FCS exactly as 23.3.12 specifies; legacy Ack frames selectable).
Frames above `--rts-threshold` are protected by RTS → **NDP CTS**. None of this needs a
BSS or association; the one OCB liberty is deriving the 9-bit partial AID from the MAC
address. Timeouts are relaxed to SDR-latency scale; real SIFS needs hardware
timestamping. The engine is IO-free and clock-injected — fully unit-tested plus
two-node over-the-air simulation tests (NDP Ack, NDP BlockAck, RTS/NDP CTS, retries).

`s2g-node` wires NIC ↔ MAC ↔ PHY ↔ Pluto:

```sh
# Linux / macOS / *BSD: a real L2 TAP interface (build with the tap feature)
cargo build --release --features tap
sudo target/release/s2g-node --tap s2g0 --uri 192.168.2.1 --mcs 2 --ldpc --rts-threshold 300
# then: ip addr add 10.99.0.1/24 dev s2g0   (etc. on each node)

# Windows (no L2 TAP backend yet): Ethernet-over-UDP NIC instead
target\release\s2g-node.exe --udp 127.0.0.1:5001 --uri 192.168.2.1
```

The `Nic` trait in `s2g-tools` keeps the attachment point pluggable; TAP is via the
cross-platform `tappers` crate (Linux/macOS/FreeBSD/OpenBSD/NetBSD — all cross-checked
to compile). Two nodes need distinct `--mac` addresses (default is randomized).

## Status / roadmap

- [x] TX chain: preamble (STF/LTF1), SIG (CRC-4, QBPSK), scrambler, BCC + puncturing,
      interleaver, LDPC (Annex F codes, 19.3.11.7.5 encoding process, tone mapper),
      constellation mapping, fixed/traveling pilots, OFDM assembly — all MCSes
- [x] RX chain: energy detect / CCA, STF detect, coarse/fine CFO, LTF timing + channel
      estimate (+ smoothing), RSSI/RCPI/SNR, S1G_SHORT vs S1G_LONG discrimination, SIG and
      SIG-A (SU/MU) decode with reserved-bit checks, PHY-RXSTART/RXEND statuses
      (FormatViolation / UnsupportedRate / CarrierLost) with RXTIME hold, pilot CPE loop +
      sampling-clock-drift tracking, soft Viterbi / layered min-sum LDPC, descrambling,
      full RXVECTOR + metrics
- [x] TX conformance measurements: spectral flatness, EVM vs Table 23-34, 2 MHz spectral
      mask, DC leakage
- [x] NDP CMAC PPDU TX/RX; NDP CTS / Ack / BlockAck frame bodies (bitmap protection)
- [x] PlutoSDR TX/RX backend (pure-Rust iiod client) at arbitrary carrier
- [x] OCB MAC: data/RTS/ACK frames, A-MPDU, CSMA with PHY CCA + NAV + RID, NDP responses,
      retries, dedup
- [x] `s2g-node`: TAP (Unix) / UDP (Windows) network interface over the radio
- [ ] Windows L2 TAP backend (tap-windows6); hardware-timestamped SIFS/ACK timing
- [ ] S1G_LONG Data-field reception (optional for a ≤ 2 MHz STA), other NDP CMAC types
      (PS-Poll, Paging, Probe Request), short GI, 1/4/8/16 MHz, multi-stream/STBC —
      all optional (or 1 MHz: skipped by choice); module boundaries chosen so they slot in
