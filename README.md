# s2g — IEEE 802.11ah (S1G) PHY in Rust for PlutoSDR

**s2g** ("sub-2 GHz") is the working name of this mode — a pun on S1G, since this
deployment runs in the 24 cm band rather than sub-1 GHz.

A from-scratch, modular implementation of the IEEE 802.11-2024 Clause 23 **S1G PHY**:
2 MHz bandwidth, single spatial stream, **long and short GI**, **BCC and LDPC**, **fixed and
traveling pilots**, **S1G_SHORT and S1G_LONG (SU) preambles**, **all valid MCSes (0–8 and 11**, i.e. BPSK½ …
256-QAM¾ and 1024-QAM¾; MCS 9/10/12 are "Not valid" at 2 MHz/1SS per Table 23-46**)**,
NDP CMAC PPDUs, and the mandatory receive procedures of 23.3.20: CCA, RSSI/RCPI,
SIG/SIG-A decoding, PHY-RXSTART/RXEND statuses, carrier-lost handling. Transmission defaults
to S1G_SHORT with the 8 µs GI (best range); short GI and S1G_LONG are per-PPDU TXVECTOR
options, signalled in the SIG like the MCS, so a receiver parked on one channel takes both.
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

## Validation on real captures

`s2g-rx` replays SigMF / ci16 / cf32 recordings at any sample rate (`--rate`, `--shift-hz`),
parses the MAC frames (`--mac`) and writes them to a PCAP (`--pcap`). Run against
[Daniel Estévez's 35 s baby-monitor capture](https://destevez.net/2025/01/decoding-ieee-802-11ah/)
(a commercial HaLow chip at 866 MHz, 2 MHz channel, Pluto at 3.84 MS/s):

```sh
target/release/s2g-rx --in baby-monitor.sigmf --mac --quiet --pcap s2g.pcap
python scripts/compare_pcap.py 802_11_ah.pcap s2g.pcap
```

| PPDUs | s2g result | vs. ground-truth PCAP |
|---|---|---|
| 1469 S1G_SHORT (MCS 1–2, BCC, some with traveling pilots) | 1469 decoded, FCS valid | byte-exact match for all 1467 non-Data frames (223 RTS, 51 Action, 71 S1G Beacons, 1122 +HTC-wrapped CTS/BlockAck) plus 2 valid frames the reference decoder missed |
| 1072 S1G_LONG (MCS 0–7 data, aggregated, traveling pilots, 17 with short GI) | 1072 decoded; 1279 data MPDUs FCS-valid | byte-exact match for 1276 of the 1278 FCS-valid data frames (the reference PCAP also carries 18 frames flagged bad-FCS — radiotap 0x40 — that neither decoder recovered) plus 3 valid data frames the reference missed; all 11 short-GI frames it flags (radiotap 0x80) are among the matches |
| SIG CRC failures | 2 in 35 s | — |

The chip rounds non-aggregated PSDU lengths up to a multiple of 4 octets and pads after the
FCS; `frame::locate_mpdu` tolerates that.

Two further captures from [sigidwiki](https://www.sigidwiki.com/wiki/802.11ah) (a HaLow
router, SDR# WAV at 20 MS/s centred on 862.005 MHz, so the 864 / 866 MHz channels sit
2 / 4 MHz off-centre and the receiver decimates 10× past a strong adjacent channel):

```sh
target/release/s2g-rx --in baseband_862004550Hz_09-28-46_19-07-2026.wav --shift-hz 2.0e6 --mac --quiet
```

| Capture | Result |
|---|---|
| "router looking for client", 864 MHz, 6 s | 102/102 PSDUs FCS-valid (MCS 0 A-MPDUs with traveling pilots, Action No Ack) |
| same capture, 866 MHz channel | 93/93 FCS-valid |
| "15 MB transfer", 866 MHz, 8 s | 263/263 FCS-valid (119 RTS, 129 wrapped CTS/BlockAck, 5 S1G Beacons, 6 Action No Ack, 4 Action) + 109 S1G_LONG data PPDUs identified |

`scripts/mega_get.py` fetches the Mega-hosted files.

The [imec Sub-GHz IQ dataset](https://github.com/JaronFontaine/Sub-GHz-IQ-signals-dataset)
(RTL-SDR at 2.048 MS/s over coax, `.mat` files with an `IQ_samples` vector) has ten 2 MHz
802.11ah captures of 4 s each (`*_chan2_*`; the `mcs0`/`mcs7` in the file names is not what
the device sent — every PPDU is MCS 2, 280-octet QoS Data):

```sh
python scripts/nextcloud_zip_filter.py "https://cloud.ilabt.imec.be/public.php/dav/files/bqXtdp9QsfXLbb3/864/80211ah?accept=zip" chan2 mat/
python scripts/convert_mat.py mat/*chan2*.mat
target/release/s2g-rx --in mat/80211ah_mcs0_chan2_g0.0dB_att10dB_freq864.0MHz_0.cf32 --rate 2.048e6 --mac --quiet
```

| Result over the ten files | |
|---|---|
| PPDUs with valid SIG | 15 663 (about 400 per second) |
| MPDUs with valid FCS | 15 505 (99.0 %) |
| Remaining failures | RTL-SDR stream discontinuities mid-PPDU (a sudden half-sample timing jump visible with `S2G_TRACE=1`); the tracker now snaps to such jumps, which recovers most of them |
| Chip quirk handled | about 1 in 128 PPDUs is scrambled with the all-zero seed, which the standard forbids; the receiver treats it as "no scrambling" |

`S2G_TRACE=1` prints per-symbol pilot tracking (timing offset, CPE, pilot coherence, symbol
power) for any decode.

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
Ethernet payloads, S-MPDU aggregation for frames over 511 octets, PV1 (short header)
frame reception, CSMA with DIFS/EIFS + exponential backoff gated by PHY CCA, NAV and
**RID** (response indication deferral), and acknowledgement via **NDP Ack** CMAC PPDUs
(Ack ID from the scrambler seed + FCS exactly as 23.3.12 specifies; NDP BlockAck for
received multi-MPDU A-MPDUs; legacy Ack frames selectable).
Frames above `--rts-threshold` are protected by RTS → **NDP CTS**. None of this needs a
BSS or association; the one OCB liberty is deriving the 9-bit partial AID from the MAC
address. Timeouts are relaxed to SDR-latency scale; real SIFS needs hardware
timestamping. The engine is IO-free and clock-injected — fully unit-tested plus
two-node over-the-air simulation tests (NDP Ack, NDP BlockAck, RTS/NDP CTS, retries).

### A-MPDUs, S-MPDUs and the partial AID (background for non-RF readers)

*Aggregation.* An 802.11 PHY frame (PPDU) can carry several MAC frames back to back: an
**A-MPDU** is a list of `[4-byte delimiter][MAC frame][pad]` records inside one PPDU, like
a length-prefixed record stream. In S1G the PHY header's length field is only 9 bits, so any
MAC frame over 511 octets *must* travel this way even if it is alone — the aggregation bit
just switches the length units from octets to OFDM symbols. The standard then says what a
real multi-frame A-MPDU may contain: QoS Data frames (they carry a traffic ID and an ack
policy), acknowledged with a **BlockAck** bitmap covering all of them, which in turn needs a
Block Ack agreement negotiated beforehand (ADDBA) — a stateful handshake this OCB MAC does
not do.

*What s2g does.* A frame over 511 octets is sent as an **S-MPDU**: an A-MPDU whose single
record has the EOF bit set in its delimiter. The standard (10.12.8) defines an S-MPDU as
"the rules of a non-aggregated frame apply": any MAC frame that is valid on its own is valid
inside it, no Block Ack agreement is needed, and it is acknowledged with an ordinary
(NDP) Ack. So plain Data frames inside our aggregated PSDUs are conformant, and a standard
receiver deaggregates them with its normal A-MPDU parser. (Earlier versions sent the same
thing with EOF = 0 and expected an NDP BlockAck; that was the deviation.)

*Partial AID.* When an S1G station associates with an AP it is assigned an **AID** (a small
integer, like a session id). The 9-bit "partial AID" in the PHY header and in an NDP CTS
is derived from it, so receivers can tell early whether a PPDU is for them. There is no
association in OCB and hence no AID, so s2g hashes the MAC address into those 9 bits
(`ndp::ocb_partial_aid`). Both ends of an s2g link compute the same value, but a standard
station would not, so only the RA field of our NDP CTS frames is affected.

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
- [x] RX chain: CCA (energy detect within aCCATime, preamble detect, mid-packet detect
      within aCCAMidTime, predicted-duration hold incl. reserved SIG indications), STF
      detect, coarse/fine CFO, LTF timing + channel estimate (+ smoothing), RSSI/RCPI/SNR,
      S1G_SHORT vs S1G_LONG discrimination, SIG and SIG-A (SU/MU) decode with reserved-bit
      checks, PHY-RXSTART/RXEND statuses (FormatViolation / UnsupportedRate / CarrierLost)
      with RXTIME hold, pilot CPE loop + sampling-clock-drift tracking with jump detection,
      soft Viterbi / layered min-sum LDPC, descrambling, full RXVECTOR + metrics
- [x] TX conformance measurements: spectral flatness, EVM vs Table 23-34, 2 MHz spectral
      mask, DC leakage
- [x] NDP CMAC PPDU TX/RX; NDP CTS / Ack / BlockAck frame bodies (bitmap protection)
- [x] PlutoSDR TX/RX backend (pure-Rust iiod client) at arbitrary carrier
- [x] OCB MAC: data/RTS/ACK frames, S-MPDU, PV1 reception, CSMA with PHY CCA + NAV + RID +
      EIFS, NDP responses, retries, dedup
- [x] `s2g-node`: TAP (Unix) / UDP (Windows) network interface over the radio
- [ ] Windows L2 TAP backend (tap-windows6); hardware-timestamped SIFS/ACK timing
- [x] S1G_LONG SU Data-field reception and short-GI reception (both optional for a ≤ 2 MHz
      STA; validated on the baby-monitor capture) — also available on TX via `TxVector`
- [ ] Other NDP CMAC types (PS-Poll, Paging, Probe Request), 1/4/8/16 MHz, multi-stream/STBC —
      all optional (or 1 MHz: skipped by choice); module boundaries chosen so they slot in
