# s2g: IEEE 802.11ah (S1G) PHY in Rust

A modular implementation of the IEEE 802.11-2024 Clause 23 S1G PHY, targeting the
ADALM-Pluto SDR at a **nonstandard carrier of 1250 MHz**.

## Scope

| Aspect | Choice | Notes |
|---|---|---|
| Bandwidth | 2 MHz only | 1 MHz (mandatory for a compliant S1G STA) deliberately ignored; 4/8/16 MHz PPDUs are identified from the SIG BW field for CCA/RID only |
| Guard interval | Long GI (8 µs) is the TX default; **long and short GI are received** (short GI on TX via `TxVector::gi`) | Short GI applies from the second Data symbol on [Eq 23-58]; those symbols use a 3-sample FFT-window backoff, compensated as a known timing offset |
| MCS | 0–8 and 11 (all valid for 2 MHz / 1 SS) | Mandatory floor is MCS 0–2 (non-AP) / 0–7 (AP) [4.3.14.1] |
| Spatial streams | 1 | Pluto is 1×1; multi-stream/STBC PPDUs are identified from SIG for CCA/RID |
| Coding | BCC (mandatory) **and LDPC** (optional) | Annex F matrices, 19.3.11.7.5 shortening/puncturing/repetition, LDPC extra symbol, tone mapper |
| Pilots | Fixed (mandatory) **and traveling** (optional, Table 23-23) | Traveling pilots also drive per-tone channel tracking on RX |
| Preamble | S1G_SHORT TX/RX (default); **S1G_LONG SU TX/RX** (1 STS): SIG-A, D-STF, D-LTF1, SIG-B (= D-LTF1 for SU), Data with p_{n+2} pilots [23.3.8.2.3.3, Eq 23-56] | MU and multi-stream S1G_LONG PPDUs are identified from SIG-A for CCA/RID → `RxEnd(UnsupportedRate)` with a complete RXVECTOR |
| PPDU | SU data PPDUs + NDP CMAC PPDUs | NDP CTS / Ack / BlockAck bodies are built and parsed in `s2g-mac` |
| PHY procedures (23.3.20) | CCA (energy detect, preamble detect, predicted-duration hold), RSSI/RCPI/SNR in the RXVECTOR, PHY-RXSTART, PHY-RXEND statuses (NoError / FormatViolation / UnsupportedRate / CarrierLost), RXTIME wait-out | Thresholds in dBm via a calibration offset (`RxConfig::cal_offset_db`) |
| RX tracking | Pilot CPE loop, sampling-clock-drift tracking with FFT-window stepping, channel smoothing when the SIG recommends it | ±20 ppm per end (23.3.17.3) handled with margin |
| TX conformance | `conformance` module: spectral flatness (Table 23-33), EVM vs Table 23-34, 2 MHz spectral mask (Fig 23-40), DC leakage | Measurements on baseband streams; the mask test runs on the 8 MS/s interpolated waveform |
| Host attachment | TAP on Unix (tappers), tap-windows6 on Windows, Ethernet-over-UDP anywhere; virtual Pluto for hardware-free two-node runs | `s2g-tools/src/nic.rs`, `wintap.rs`, `bin/s2g-virtual-pluto.rs` |
| MAC | OCB (no BSS); A-MPDU packing of QoS Data MPDUs with NDP BlockAck selective retransmission, NDP Ack / NDP CTS responses, RTS protection, RID, NAV, PHY-driven CCA, station identification frames (EtherType 0x88B5), stateless good-neighbor frame filter, per-peer rate control (`rate.rs`: EWMA success per MCS and guard interval, one-step MCS probing with back-off, short-GI and LDPC probing, retry walk-down, reliability floor priced against the response timeout, SNR- and delay-spread-bounded) | The only OCB liberty in the NDP path is the 9-bit partial AID derived from the MAC address; rate adaptation is implementation-defined in 802.11 |

## Crate layout

```
crates/
  s2g-phy        # pure DSP, no SDR deps. TX: PSDU -> IQ samples @2 MS/s (BCC/LDPC, fixed/traveling pilots).
                 # RX: streaming state machine, IQ samples -> events (CCA, RXSTART, PSDU, NDP, RXEND).
                 # conformance: TX measurements. ldpc: Annex F codes + PPDU encoding process.
  s2g-mac        # OCB (non-BSS) MAC: 802.11 data/RTS/ACK frames, wildcard BSSID, FCS, A-MPDU,
                 # NDP CMAC frames (CTS/Ack/BlockAck), LLC/SNAP, CSMA/backoff with CCA+NAV+RID,
                 # ACK/retry, dedup. IO-free, clock-injected engine driven by PHY events.
  s2g-dsp        # rate conversion + generic DSP: halfband 2x resamplers, windowed-sinc
                 # arbitrary-ratio resampler (sampling-clock-offset simulation).
  s2g-sdr        # hardware abstraction: SdrTx / SdrRx / SdrDevice traits. No hardware deps.
  s2g-sdr-pluto  # PlutoSDR backend: pure-Rust iiod network-protocol client (TCP 30431,
                 # docs/iiod-protocol.md); no native libiio dependency. Optional feature of s2g-tools.
  s2g-tools      # binaries: s2g-tx, s2g-rx, s2g-sim (loopback with impairments incl. SFO/echo),
                 # s2g-node (NIC <-> MAC <-> PHY <-> Pluto), file I/O (.cf32),
                 # Nic trait: TAP via tappers (unix, feature "tap") or Ethernet-over-UDP,
                 # pcap.rs: radiotap PCAP to a file, stdout or a pipe Wireshark reads live.
```

Dependency direction (arrows = "depends on"):

```
s2g-tools -> s2g-phy, s2g-mac, s2g-dsp, s2g-sdr, [s2g-sdr-pluto]
s2g-mac -> s2g-phy
s2g-sdr-pluto -> s2g-sdr
s2g-phy -> s2g-dsp (plus num-complex, rustfft)
```

A future RX-only system on a different SDR = `s2g-phy` + `s2g-sdr` + a new backend crate.
The MAC sits on top of `s2g-phy`'s TXVECTOR/RXVECTOR-shaped API (`vector.rs`) and the
`RxEvent` stream; PHY characteristics needed for MAC timing (SIFS, slot time, NDPTxTime)
and the RF thresholds (CCA levels, sensitivity, RCPI encoding, EVM limits, spectral mask)
are exported from `s2g_phy::params`.

## Sample-rate plan

The PHY natively runs at **2 MS/s** (64-pt FFT, 31.25 kHz spacing; long-GI symbol = 80
samples = 40 µs). The AD9363 cannot stream below ~2.08 MS/s, so the Pluto runs at
**4 MS/s** and `s2g-dsp` halfband-resamples 2×: interpolate on TX, decimate on RX. The
resampler lives outside `s2g-phy`, so a different SDR that runs at 2 MS/s natively (or any
other integer relation) plugs in without touching the PHY.

## RX pipeline (inside s2g-phy)

Push-based: `Receiver::process(&mut self, &[Complex32], &mut Vec<RxEvent>)`. Internally:
energy detect over aCCATime windows (CCA) -> STF autocorrelation detect -> coarse CFO ->
LTF cross-correlation timing + fine CFO -> LTF channel estimate, RSSI, RCPI, SNR ->
SIG format discrimination (QBPSK on symbol 2 = S1G_SHORT, BPSK = S1G_LONG) -> SIG /
SIG-A decode (rate-1/2 BCC, CRC-4, reserved-bit checks) -> verdict (supported /
unsupported / reserved) -> `RxStart` -> per-symbol pilot measurement -> CPE loop +
timing-drift filter (window stepping) -> optional traveling-pilot channel refresh ->
equalize -> LLR demap -> (deinterleave + soft Viterbi | LDPC tone demap + layered
min-sum) -> descramble -> PSDU + metrics -> `RxEnd(NoError)`. Any failure emits
`RxEnd` with the spec status, holds CCA BUSY for the predicted PPDU duration when a
valid SIG gave one (unsupported mode, carrier lost), and re-arms the detector. `Cca`
events carry the predicted hold time so the MAC can defer without its own guesswork.

## MAC (s2g-mac)

`Mac` consumes `RxEvent`s and Ethernet frames and produces `MacAction::Transmit` /
`MacAction::TransmitNdp`. Unicast data solicits an NDP Ack (NDP BlockAck for A-MPDUs)
with RESPONSE_INDICATION = NDP Response; the MAC chooses the scrambler seed so it can
predict the Ack ID (`ndp::ack_id`) / BlockAck ID. RTS above `rts_threshold` elicits an
NDP CTS. Every received PPDU updates the RID from its RESPONSE_INDICATION [10.3.2.5];
Duration fields of frames for others set the NAV; CCA comes straight from the PHY.

## Spec digest

Implementation-grade notes distilled from IEEE 802.11-2024 live in `docs/spec-digest/`
(committed source of truth for constants; every table cites clause/page). Matrix
prototypes for the LDPC codes were transcribed from Annex F (Tables F-1..F-3) and are
verified by an H·cᵀ = 0 test for every code.
