# s2g — IEEE 802.11ah (S1G) PHY in Rust for PlutoSDR

A from-scratch, modular implementation of the IEEE 802.11-2024 Clause 23 **S1G PHY**:
2 MHz bandwidth, long GI (8 µs), single spatial stream, BCC, S1G_SHORT preamble,
**all valid MCSes (0–8 and 11**, i.e. BPSK½ … 256-QAM¾ and 1024-QAM¾; MCS 9/10/12 are
"Not valid" at 2 MHz/1SS per Table 23-46**)**, plus NDP CMAC PPDUs (needed later by the
MAC for control frames). Runs on an ADALM-Pluto at a **nonstandard 1250 MHz carrier** —
the carrier is just a tuning parameter; the baseband is band-agnostic.

See `ARCHITECTURE.md` for the crate layout and design decisions, and
`docs/spec-digest/` for the implementation-grade spec notes (with clause/page citations)
that every constant in the code traces back to.

## Build & test

```sh
cargo build --release            # everything, incl. Pluto backend (pure Rust, no libiio)
cargo test --workspace           # 71 tests: golden vectors, roundtrips, full loopback
```

## Tools

```sh
# Simulate: PER vs SNR for every MCS with CFO + timing offset impairments
target/release/s1g-sim --count 100 --snr-db "3,6,9,12,15,18,21,25,30,36"

# Generate a waveform file (GNU Radio-compatible .cf32), 3 PPDUs at MCS 4
target/release/s1g-tx --mcs 4 --random 200 --count 3 --out wave.cf32 --out-rate 4e6

# Decode a waveform file (native 2 MS/s, or 4 MS/s with ×2 decimation)
target/release/s1g-rx --in wave.cf32 --rate 4e6

# Live TX on a Pluto at 1250 MHz (device streams at 4 MS/s, ×2 interpolated)
target/release/s1g-tx --uri 192.168.2.1 --mcs 2 --hex "dead beef 0102" --count 10

# Live RX on a Pluto at 1250 MHz
target/release/s1g-rx --uri 192.168.2.1 --gain auto
```

The Pluto backend speaks the **iiod network protocol directly** (TCP 30431, the same
path libiio's `ip:` backend uses) — no native libiio install needed on the host. Reach
the Pluto at its usual USB-network address `192.168.2.1`.

## Hardware notes

- The AD9363 can't stream at 2 MS/s, so the radio runs at **4 MS/s** and `s1g-dsp`
  halfband-resamples ×2 in software (TX interpolate / RX decimate).
- 1250 MHz is inside the AD9363 tuning range but **outside every S1G regulatory band —
  transmit into a dummy load / cable / shielded box unless you're licensed for that
  spectrum.**
- Frequency accuracy: the RX tolerates ≳ ±40 kHz CFO (±32 ppm at 1.25 GHz), comfortably
  above the spec's ±20 ppm budget, so a stock Pluto TCXO on both ends is fine.

## Networking: OCB MAC + TAP (`s1g-node`)

`s1g-mac` implements a nonstandard-where-it-matters OCB (non-BSS) MAC: 802.11 Data
frames with the wildcard BSSID, FCS, sequence numbers + dedup, RFC 1042 LLC/SNAP for
Ethernet payloads, spec-format A-MPDU aggregation for frames over 511 octets, CSMA
with DIFS + exponential backoff, and optional ACK/retry for unicast (timeouts relaxed
to SDR-latency scale; real SIFS needs hardware timestamping). The engine is IO-free
and clock-injected — fully unit-tested plus a two-node over-the-air simulation test.

`s1g-node` wires NIC ↔ MAC ↔ PHY ↔ Pluto:

```sh
# Linux / macOS / *BSD: a real L2 TAP interface (build with the tap feature)
cargo build --release --features tap
sudo target/release/s1g-node --tap s1g0 --uri 192.168.2.1 --mcs 2
# then: ip addr add 10.99.0.1/24 dev s1g0   (etc. on each node)

# Windows (no L2 TAP backend yet): Ethernet-over-UDP NIC instead
target\release\s1g-node.exe --udp 127.0.0.1:5001 --uri 192.168.2.1
```

The `Nic` trait in `s1g-tools` keeps the attachment point pluggable; TAP is via the
cross-platform `tappers` crate (Linux/macOS/FreeBSD/OpenBSD/NetBSD — all cross-checked
to compile). Two nodes need distinct `--mac` addresses (default is randomized).

## Status / roadmap

- [x] TX chain: preamble (STF/LTF1), SIG (CRC-4, QBPSK), scrambler, BCC + puncturing,
      interleaver, constellation mapping, pilots, OFDM assembly — all MCSes
- [x] RX chain: STF detect, coarse/fine CFO, LTF timing + channel estimate, SIG decode,
      pilot phase/slope tracking, soft Viterbi, descrambling, RXVECTOR + metrics
      (SNR/CFO/EVM/RSSI)
- [x] NDP CMAC PPDU TX/RX (37-bit body PHY transport for MAC control)
- [x] PlutoSDR TX/RX backend (pure-Rust iiod client) at arbitrary carrier
- [x] OCB MAC: data/ACK frames, A-MPDU, CSMA/backoff, retries, dedup
- [x] `s1g-node`: TAP (Unix) / UDP (Windows) network interface over the radio
- [ ] Windows L2 TAP backend (tap-windows6); hardware-timestamped SIFS/ACK timing
- [ ] Short GI, LDPC, 1/4/8/16 MHz, multi-stream: out of scope for v1, module
      boundaries chosen so they slot in
