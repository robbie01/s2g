# Risk and validation register

What in this implementation has been proven against something outside this
repository, what has only been shown to be self-consistent, and what cannot be
checked without hardware. Keep this current: when a capture or a device
validates (or breaks) a row, say so here with the date.

Hardware situation (2026-09): no HaLow hardware in the lab. Validation so far
comes from three third-party captures of commercial 2 MHz HaLow devices (see
README "Validation on real captures") and from simulation. Many commercial
chips do not use NDP CMAC PPDUs or LDPC, so those paths may stay unverified
for a while even with a dev kit in hand.

## Legend

- **External** — matched against a signal or decoder we did not write.
- **Self-consistent** — TX and RX agree with each other and with the spec as
  we read it; a systematic misreading would be invisible.
- **Unmeasurable here** — needs the actual radio / test equipment.

## PHY

| Feature | Status | Evidence / gap |
|---|---|---|
| S1G_SHORT preamble, SIG, BCC MCS 0–2, fixed pilots | External | Byte-exact frames from three chips (baby monitor, HaLow router, imec dataset) |
| Traveling pilots (RX) | External | Baby-monitor Action frames and router MCS 0 A-MPDUs use them; all decoded |
| Traveling pilots (TX) | Self-consistent | Our RX decodes our TX; positions transcribed from Table 23-23 |
| MCS 3–7 (16-QAM … 64-QAM 5/6) | External | Baby-monitor S1G_LONG data PPDUs (MCS 3–7) decode byte-exact (2026-09-02) |
| MCS 8, 11 (256/1024-QAM) | Self-consistent | Loopback only; no capture uses them. Sensitivity simulation meets Table 23-35 with margin |
| LDPC (all rates, all block lengths) | Self-consistent | Annex F matrices verified by H·cᵀ = 0; PPDU process (shortening / puncturing / repetition / extra symbol / tone mapper) round-trips. No external LDPC S1G signal exists in our captures. `scripts/matlab_vectors.m` generates reference waveforms if WLAN Toolbox is available |
| S1G_LONG SU Data reception (D-LTF1/SIG-B estimate merged with LTF1, Eq 23-56 pilots) | External | Baby-monitor capture (2026-09-02): 1276 of 1278 FCS-valid data frames byte-exact vs the reference PCAP; the 18 frames the reference flags bad-FCS are the only other misses |
| Short-GI reception | External | All 11 FCS-valid short-GI PPDUs in the baby-monitor capture (radiotap flag 0x80) decode byte-exact; 3-sample window backoff on short-GI symbols |
| Short-GI / S1G_LONG transmission | Self-consistent | Loopback with CFO, echo and ±30 ppm SFO; TX defaults stay S1G_SHORT + 8 µs GI |
| Sampling-clock tracking | External (weak) | Real captures show 9 ppm tracked and RTL-SDR sample-drop jumps snapped; ±40 ppm only in simulation |
| CCA energy / preamble / mid-packet detect | Self-consistent | Thresholds are in dBm and depend on `cal_offset_db`; nothing checks the calibration. The mid-packet (guard-interval correlation) detector is tested on our own waveform only; its false-alarm rate on real interference is unmeasured |
| RSSI / RCPI / SNR | Unmeasurable here | Monotonic and encoded per Table 9-215; absolute dBm accuracy (±5 dB) needs a calibrated source |
| Carrier-lost, FormatViolation, UnsupportedRate, RXTIME wait-out | Self-consistent | Loopback tests; real captures exercised FormatViolation and UnsupportedRate |
| Spectral mask / flatness / EVM | Unmeasurable here | Measured on the baseband stream only (passes); the Pluto's RF chain is not included |
| Receiver sensitivity, adjacent-channel rejection, max input | Unmeasurable here | Simulation only (10 dB NF assumed) |
| NDP CMAC PPDU (PHY transport) | Self-consistent | 37-bit body round-trips; no capture contained an NDP PPDU |

## MAC

| Feature | Status | Evidence / gap |
|---|---|---|
| Data / RTS / Ack / A-MPDU wire formats, FCS, LLC/SNAP | External | Frames from real chips parse and re-serialise byte-exact |
| NDP Ack / NDP BlockAck / NDP CTS bodies, Ack ID derivation | Self-consistent | Layouts transcribed from 23.3.12; the three recorded chips all use +HTC-wrapped CTS/BlockAck frames instead (allowed when link adaptation is negotiated), so no external check exists |
| RID, NAV, EIFS, PHY-driven CCA deferral | Self-consistent | Unit tests; timing values from Tables 23-41 / 10-3 |
| PV1 reception | Self-consistent | Layout from 9.8; no PV1 frame seen in any capture |
| Padded-PSDU tolerance (`locate_mpdu`) | External | Baby monitor pads non-aggregated PSDUs to 4-octet multiples |
| Scrambler seed 0 tolerance | External | imec dataset device uses the all-zero seed ~1/128 of the time |
| A-MPDU packing, NDP BlockAck bitmap, selective retry | Self-consistent | Unit tests with corrupted MPDUs and a two-node PHY simulation; the 16-bit bitmap and SSN semantics follow 23.3.12.2.6.2 |
| Per-peer rate control | Self-consistent | Unit tests with synthetic success ceilings and a two-node PHY simulation at 30 dB / 11 dB SNR; the SNR hint assumes a roughly symmetric link and the probe/back-off constants are untuned against real fading |

## Deliberate deviations from the standard

| Deviation | Why | Consequence |
|---|---|---|
| OCB (no BSS): no association, beacons, TIM, BSS max idle, RAW, power save | Project goal | A standard S1G STA would not talk to this MAC without an AP; two s2g nodes talk to each other |
| Partial AID for NDP CTS derived from the MAC address | No AID without association | Only matters for the RA field of our own NDP CTS |
| PV1 frames addressed by AID (SID) are dropped | No association ⇒ no AID table | PV1 QoS Data with full MAC addresses (type 3) is delivered; SID-addressed frames are received, FCS-checked and discarded |
| A-MPDUs without a block ack agreement | No ADDBA exchange in OCB | s2g peers acknowledge them with NDP BlockAck; a standard STA would not. Set `--ampdu 1` (S-MPDUs only) when talking to other vendors |
| Station identification as a broadcast Data frame with EtherType 0x88B5 | Part 97 needs an in-the-clear call sign; 802.11 has no field for it | Plain ASCII, readable in any capture; adds one MCS 0 frame per 10 minutes of traffic |
| DIFS-based DCF access, backoff redrawn instead of frozen | Simplicity | Slightly unfair against standard EDCA stations |
| Response timeouts of ~150 ms instead of SIFS-scale | Buffered SDR streaming | Throughput bound; interop with a real SIFS-timed peer is not possible without hardware timestamping |
| 1 MHz / S1G_1M not implemented | User decision | Mandatory for a compliant S1G STA; 1 MHz devices are invisible to this receiver |
| CCA thresholds interpreted on dBFS unless `cal_offset_db` is set | No calibration data | Uncalibrated, energy detect triggers at −72 dBFS |

## How to retire a risk

- **LDPC / traveling pilots TX**: run `scripts/matlab_vectors.m` (WLAN Toolbox) and decode the `.cf32` files with `s2g-rx --mac`; or capture any device that advertises LDPC (Morse Micro MM61xx modules do).
- **NDP CMAC**: needs a device that sends NDP Ack/CTS; Newracom NRC7292-based modules reportedly do — capture with a Pluto at 3.84 MS/s and look for `NdpReceived` in `s2g-rx` output.
- **MCS 8, 11**: any dev kit with rate control disabled, or MATLAB vectors.
- **RF conformance**: spectrum analyser on the Pluto output; the `conformance` module then applies to a captured loopback as well.
- **Calibration**: inject a known-level tone, set `--cal-offset-db` so that `rcpi_dbm` matches.
