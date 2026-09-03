# s2g: IEEE 802.11ah (S1G) PHY in Rust for PlutoSDR

**s2g** ("sub-2 GHz") is the working name of this mode, a pun on S1G, since this
deployment runs in the 24 cm band rather than sub-1 GHz.

A from-scratch, modular implementation of the IEEE 802.11-2024 Clause 23 **S1G PHY**:
2 MHz bandwidth, single spatial stream, **long and short GI**, **BCC and LDPC**, **fixed and
traveling pilots**, **S1G_SHORT and S1G_LONG (SU) preambles**, **all valid MCSes (0–8 and 11**, i.e. BPSK½ …
256-QAM¾ and 1024-QAM¾; MCS 9/10/12 are "Not valid" at 2 MHz/1SS per Table 23-46**)**,
NDP CMAC PPDUs, and the mandatory receive procedures of 23.3.20: CCA, RSSI/RCPI,
SIG/SIG-A decoding, PHY-RXSTART/RXEND statuses, carrier-lost handling. Transmission defaults
to S1G_SHORT with the 8 µs GI (best range); short GI and S1G_LONG are per-PPDU TXVECTOR
options, signalled in the SIG like the MCS, so a receiver parked on one channel takes both.
Runs on an ADALM-Pluto at a **nonstandard 1250 MHz carrier**; the carrier is only a
tuning parameter and the baseband is band-agnostic. 1 MHz operation is deliberately out of
scope.

See `ARCHITECTURE.md` for the crate layout, the scope table and design decisions, and
`docs/spec-digest/` for the implementation-grade spec notes (with clause/page citations)
that every constant in the code traces back to.

## Build & test

```sh
cargo build --release            # everything, incl. Pluto backend (pure Rust, no libiio)
cargo test --workspace           # golden vectors, LDPC matrices, roundtrips, full loopback,
                                 # SFO/echo channels, CCA/RXEND behavior, two-node MAC exchanges
```

## Tools

```sh
# Simulate: PER vs SNR for every MCS with CFO + timing offset impairments
target/release/s2g-sim --count 100 --snr-db "3,6,9,12,15,18,21,25,30,36"
# …with LDPC, traveling pilots, a 40 ppm clock offset and a 2 µs echo
target/release/s2g-sim --ldpc --traveling-pilots --sfo-ppm 40 --echo-delay 4 --mcs 3

# Generate a waveform file (GNU Radio-compatible .cf32), 3 PPDUs at MCS 4
target/release/s2g-tx --mcs 4 --random 200 --count 3 --out wave.cf32 --out-rate-hz 4e6
target/release/s2g-tx --mcs 4 --ldpc --traveling-pilots --random 200 --out wave.cf32
target/release/s2g-tx --ndp 0x0000000002 --out ndp.cf32        # an NDP CMAC PPDU

# Decode a waveform file (native 2 MS/s, or 4 MS/s with ×2 decimation); prints CCA,
# RXSTART (full RXVECTOR incl. RSSI/RCPI/SNR), RXEND statuses, PSDUs
target/release/s2g-rx --in wave.cf32 --rate-hz 4e6 --cal-offset-db -30

# Live TX on a Pluto at 1250 MHz (device streams at 4 MS/s, ×2 interpolated)
target/release/s2g-tx --uri 192.168.2.1 --mcs 2 --hex "dead beef 0102" --count 10

# Live RX on a Pluto at 1250 MHz
target/release/s2g-rx --uri 192.168.2.1 --rx-gain auto
```

The Pluto backend speaks the **iiod network protocol directly** (TCP 30431, the same
path libiio's `ip:` backend uses); no native libiio install is needed on the host. Reach
the Pluto at its usual USB-network address `192.168.2.1`.

## Validation on real captures

`s2g-rx` replays SigMF / ci16 / cf32 recordings at any sample rate (`--rate-hz`, `--shift-hz`),
parses the MAC frames (`--mac`) and writes them to a radiotap PCAP (`--pcap`; bad FCS
flagged). Run against
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
router, SDR# WAV at 20 MS/s centered on 862.005 MHz, so the 864 / 866 MHz channels sit
2 / 4 MHz off-center and the receiver decimates 10× past a strong adjacent channel):

```sh
target/release/s2g-rx --in baseband_862004550Hz_09-28-46_19-07-2026.wav --shift-hz 2.0e6 --mac --quiet
```

| Capture | Result |
|---|---|
| "router looking for client", first 6 s, 864 MHz | 42/42 PSDUs FCS-valid (MCS 0 Action No Ack); `scripts/validate_captures.py` asserts 42 |
| same capture, 866 MHz channel | 42/42 FCS-valid |
| "15 MB transfer", 866 MHz, 8 s | 263/263 S1G_SHORT FCS-valid (119 RTS, 129 wrapped CTS/BlockAck, 5 S1G Beacons, 6 Action No Ack, 4 Action); the 109 S1G_LONG data PPDUs (MCS 6/7, 7 with short GI) now decode, but this recording's noise floor leaves them only 14–24 dB SNR: 4 of the 7 short MCS 6 MPDUs pass FCS, the 482-symbol MCS 7 transfers at 15 dB do not (64-QAM 5/6 needs ~23 dB) |

`scripts/mega_get.py` fetches the Mega-hosted files.

The [imec Sub-GHz IQ dataset](https://github.com/JaronFontaine/Sub-GHz-IQ-signals-dataset)
(RTL-SDR at 2.048 MS/s over coax, `.mat` files with an `IQ_samples` vector) has ten 2 MHz
802.11ah captures of 4 s each (`*_chan2_*`; the `mcs0`/`mcs7` in the file names is not what
the device sent — every PPDU is MCS 2, 280-octet QoS Data):

```sh
python scripts/nextcloud_zip_filter.py "https://cloud.ilabt.imec.be/public.php/dav/files/bqXtdp9QsfXLbb3/864/80211ah?accept=zip" chan2 mat/
python scripts/convert_mat.py mat/*chan2*.mat
target/release/s2g-rx --in mat/80211ah_mcs0_chan2_g0.0dB_att10dB_freq864.0MHz_0.cf32 --rate-hz 2.048e6 --mac --quiet
```

| Result over the ten files | |
|---|---|
| PPDUs with valid SIG | 15 663 (about 400 per second) |
| MPDUs with valid FCS | 15 505 (99.0 %) |
| Remaining failures | RTL-SDR stream discontinuities mid-PPDU (a sudden half-sample timing jump visible with `S2G_TRACE=1`); the tracker snaps to such jumps and recovers most of them |
| Chip quirk handled | about 1 in 128 PPDUs is scrambled with the all-zero seed, which the standard forbids; the receiver treats it as "no scrambling" |

`S2G_TRACE=1` prints per-symbol pilot tracking (timing offset, CPE, pilot coherence, symbol
power) for any decode.

## Windows TAP

`s2g-node --tap` on Windows opens an OpenVPN **tap-windows6** adapter (ComponentId
`tap0901`, shown as "TAP-Windows Adapter V9"): install the driver from the OpenVPN
installer or the standalone tap-windows package, run the node from an elevated prompt,
name the adapter with `--tap "TAP-Windows Adapter V9"` if you have several, and give it an
address with `netsh interface ip set address "TAP-Windows Adapter V9" static 10.44.0.1
255.255.255.0`. The node sets the adapter's media status to connected and exchanges raw
Ethernet frames through overlapped `ReadFile`/`WriteFile`. WireGuard's Wintun does not
work for this: it is a layer-3 TUN, and the MAC carries Ethernet frames. Without any
TAP driver, `--udp` remains available on every platform.

## Testing without hardware

- `scripts/validate_captures.py` decodes the three recordings above and fails if any
  documented count drops (`--quick` skips the slow 20 MS/s WAVs). Run it before and after
  touching the PHY.
- `s2g-virtual-pluto` is an iiod server hosting virtual AD9363 radios coupled over a
  simulated air interface: path loss, noise, per-radio oscillator error, propagation delay,
  real-time pacing and iiod-style TX back-pressure. `s2g-node --uri 127.0.0.1:31431` talks to
  it exactly as it would to a Pluto, so the whole streaming path runs on one machine.
  `scripts/virtual_link.py` starts two radios 20 ppm apart, two nodes on UDP NICs, pushes
  IPv6/UDP frames between ULA addresses (what the good-neighbor filter lets through) in
  both directions and prints delivery, each direction's one-way latency measured while the
  frames are in flight, the reported peer offset (±20 ppm to three digits), the response
  delay the MAC learned and the rate-control decisions; `--max-median-ms` makes the latency
  a pass/fail criterion and `--log FILE` keeps every line the three processes printed. On
  this machine 20 frames per direction 50 ms apart arrive with a median of about 30 ms
  and a worst case of about 70 ms (three SDR pipeline crossings); the same 20 frames as
  one burst (`--spacing-ms 0`) need two 16-MPDU A-MPDU cycles, about 90 ms for the last
  frame. The server reproduces iiod's READBUF framing exactly (see `docs/iiod-protocol.md`).

## Hardware notes

**Carrier offset at 1250 MHz.** Two ±25 ppm Pluto crystals can put the peer 62 kHz
away, right at the ±62.5 kHz the STF autocorrelation can resolve. The receiver therefore
tries the coarse estimate and its ±125 kHz aliases at LTF sync and keeps the one whose
LTF correlation peaks, which gives a capture range of about ±187 kHz (±150 ppm). Still,
trim the crystals: `s2g-node` prints every peer's offset in Hz and ppm whenever its rate
changes; multiply the Pluto's `xo_correction` (nominally 40000000) by (1 + ppm/1e6) on
one node and pass it with `--xo-correction`.

**CCA calibration.** The CCA thresholds are dBm values from the standard, and an SDR
knows only dBFS. Without `--cal-offset-db` the receiver measures its own noise floor
(quietest 20 ms of the last second) and assumes it sits at −104 dBm, which places energy
detect 32 dB above the floor and gives RCPI a plausible scale. Both tools print the
measured floor and the derived offset. Pass `--cal-offset-db` when you have a calibrated
source; energy detect reports IDLE until the floor has been measured.

- The AD9363 can't stream at 2 MS/s, so the radio runs at **4 MS/s** and `s2g-dsp`
  halfband-resamples ×2 in software (TX interpolate / RX decimate).
- 1250 MHz is inside the AD9363 tuning range but **outside every S1G regulatory band:
  transmit into a dummy load / cable / shielded box unless you're licensed for that
  spectrum.**
- Frequency accuracy: the RX tolerates ≳ ±55 kHz CFO (±44 ppm at 1.25 GHz) and ±40 ppm
  sampling-clock offset over a maximum-length PPDU, comfortably above the spec's ±20 ppm
  per-end budget, so a stock Pluto TCXO on both ends is fine.
- CCA / RCPI thresholds are specified in dBm; pass `--cal-offset-db` (dBm = dBFS +
  offset, measured for your gain setting) to `s2g-rx` / `s2g-node`. Uncalibrated (0),
  the thresholds simply act on dBFS.

## Networking: OCB MAC + TAP (`s2g-node`)

`s2g-mac` implements an OCB (non-BSS) MAC, nonstandard where the spec assumes a BSS: 802.11 Data
frames with the wildcard BSSID, FCS, sequence numbers + dedup, RFC 1042 LLC/SNAP for
Ethernet payloads, **A-MPDU packing** (queued frames for one peer travel as QoS Data
MPDUs in one PPDU, up to `--ampdu` of them (16 by default, the width of the NDP BlockAck
bitmap), acknowledged by one NDP BlockAck whose
bitmap retransmits only what was lost), S-MPDU aggregation for lone frames over 511
octets, PV1 (short header) frame reception, CSMA with DIFS/EIFS + exponential backoff gated by PHY CCA, NAV and
**RID** (response indication deferral), and acknowledgment via **NDP Ack** CMAC PPDUs
(Ack ID from the scrambler seed + FCS exactly as 23.3.12 specifies; NDP BlockAck for
received multi-MPDU A-MPDUs; legacy Ack frames selectable).
Frames above `--rts-threshold` are protected by RTS → **NDP CTS**. None of this needs a
BSS or association; the one OCB liberty is deriving the 9-bit partial AID from the MAC
address. Response waits are on the SDR-latency scale: `--ack-timeout-ms` is only a ceiling, the
MAC measures how long acknowledgments actually take (about 30 ms through two SDR
pipelines on the virtual link) and waits a little longer than that; real SIFS needs
hardware timestamping. The engine is IO-free and clock-injected: unit-tested plus
two-node over-the-air simulation tests (NDP Ack, NDP BlockAck, RTS/NDP CTS, retries,
rate control).

**Per-peer rate control** (`s2g_mac::rate`, on by default in `s2g-node`; `--fixed-mcs`
turns it off, `--min-mcs`/`--max-mcs` bound it): every unicast destination gets its own
MCS. The controller keeps a smoothed success probability per MCS from the
acknowledgments (an attempt counts when it delivered at least what one PPDU at the
next-lower rate would have carried, so a lone frame must get through while a big A-MPDU
may lose an MPDU or two), uses the highest MCS still above a reliability floor, probes one
step up every few frames (with exponential back-off after failed probes, re-armed when the
peer's SNR clearly rises), steps down on retries from the rate that failed, and bounds
probing with the SNR the PHY measures on whatever it hears from that peer (frames, NDP
Acks, NDP CTS) against a table of what this PHY was measured to need per MCS in the
receiver's own units (`s2g-sim --report-snr`). A rate that has been flawless may still
probe above that bound, rarely, when the airtime the next rate would save outweighs one
failure, which big A-MPDUs can and single small frames cannot. Broadcasts stay at
`--mcs`. 802.11 leaves rate adaptation to the implementation, so nothing here is
spec-constrained. The constants come from `crates/s2g-mac/tests/rate_sim.rs`, a
link-level simulation over static, shadowed, fading and stepped channels with the PHY's
own PER curves and this link's turnaround and timeout costs: the defaults reach 99 % of
the best fixed rate on average and 80 % in the worst scenario, where the previous
constants gave 96 % and 72 %
(`cargo test -p s2g-mac --release --test rate_sim -- --nocapture`; `sweep` is the ignored
test behind the numbers).

### Amateur-radio operation: identification and no encryption

With `--callsign` set, the MAC transmits a station identification frame in the clear
before the first data frame, at least every `--id-interval-min` (default 10) minutes while
traffic flows, and once more 30 s after the last data frame, which is how 47 CFR 97.119
reads for a packet link. The frame is a broadcast 802.11 Data frame at MCS 0 whose
LLC/SNAP EtherType is 0x88B5 (IEEE 802a local experimental 1) and whose payload is plain
ASCII, `DE <CALLSIGN> [free text]` (`--id-info` supplies the text, e.g. a grid square).
Any monitor-mode capture shows the call sign in the packet bytes without a dissector.
Heard identifications are reported as `id heard from …` and never forwarded to the TAP.
Without `--callsign` the node prints a warning and identifies nothing, which is only
appropriate for unlicensed-band testing.

s2g never encrypts or scrambles the meaning of a frame (the PHY scrambler is a
standard, self-synchronizing whitening sequence, not encryption). Under Part 97 the
upper layers you run over the link must not either; that is the operator's call, not
something the software can enforce.

### Good-neighbor filter

`s2g-node` runs a stateless frame filter (`s2g_mac::filter`) on every frame headed for
the air and, by default, on every frame received from it. No connection tracking: each
Ethernet frame is judged on its own, tunnels included. The default policy drops

| Dropped | Why |
|---|---|
| TCP or UDP with source or destination port 22, 443, 853, 465, 993, 995, 3389, 500, 4500, 51820 or 8443 | SSH, HTTPS/QUIC, DNS over TLS, SMTPS, IMAPS, POP3S, RDP, IKE/IPsec NAT-T, WireGuard, HTTPS alternate: encrypted transports, which Part 97 forbids |
| Anything that is not IPv6 (IPv4, ARP, RARP, VLAN tags, LLDP, PPPoE, EAPOL, 802.3/LLC such as STP) | The link is IPv6-only; this also silences DHCP, IGMP, NetBIOS over IPv4 and the rest of the IPv4 background noise in one rule |
| IPv4 inside a tunnel: next header 4 (4in6), GRE with protocol 0x0800 or a bridged frame carrying IPv4/ARP, VXLAN with an inner IPv4 frame, IPv4 inside ESP-NULL | Same rule, seen through the encapsulation |
| IPv6 whose destination is not link-local (fe80::/10), multicast (ff00::/8) or ULA (fc00::/7) | The mesh is a private IPv6 island; nothing on it should be addressed to the global Internet. The inner packet of a tunnel may go anywhere, only the outer destination must be a mesh scope |
| ICMPv6 Router Solicitation, Router Advertisement, Redirect | SLAAC is off; addresses are static and routes come from Babel |
| DHCPv6 (UDP 546/547) | Same reason |
| MLD (ICMPv6 130 to 132, 143) and Multicast Router Discovery (151 to 153) | A radio link has no snooping switch, multicast is flooded anyway, and hosts otherwise report every group join; MRD exists only to let such switches find multicast routers |
| mDNS, LLMNR, SSDP/UPnP, WS-Discovery, NetBIOS, SMB, NAT-PMP/PCP | LAN discovery chatter, discussed below |
| ESP that is not recognizably ESP-NULL | Encrypted payloads |

Everything else passes: neighbor solicitation and advertisement (needed to resolve
link-local addresses on the link), ICMPv6 echo and errors, Babel on UDP 6696, DNS, NTP,
HTTP, OSPFv3, AH, and anything unlisted. Non-first IPv6 fragments cannot be inspected and
are let through. The identification frames are never filtered. Above the EtherType level
this is a deny list: an unknown protocol on an unknown port passes.

**ESP-NULL.** IPsec with NULL encryption (RFC 2410) is authentication without secrecy,
which Part 97 allows and which is exactly the right tool for an authenticated tunnel over
the mesh. A stateless filter cannot read the SA, so it uses the RFC 5879 heuristic: for
each usual ICV length (12, 16, 24, 32 octets, and none) the ESP trailer must show the
RFC 4303 default padding 1, 2, 3, … and a Next Header whose payload parses (an IPv6 or
IPv4 header, a plausible TCP/UDP/ICMPv6 header, or a dummy packet). A match is treated as
ESP-NULL and the payload is filtered like any other packet, so a global inner destination
is fine but TCP 443 inside the tunnel is still dropped. Anything else is treated as
encrypted. Every implementation uses the default padding, but an ESP-NULL stack with
random or TFC padding would be misclassified; `--allow-esp` passes all ESP if that ever
matters. AH (next header 51) is passed as an extension header and is the simpler choice
when only authentication is wanted.

**Tunnels.** IPv6-in-IPv6, GRE (including transparent Ethernet bridging), VXLAN and
ESP-NULL are followed up to three levels deep and the inner packet gets the same rules
with the destination-scope check relaxed. Tunnels inside UDP the filter does not know
(L2TP, Geneve, IP-in-UDP variants) are not looked into; WireGuard and IKE are blocked by
port.

**LAN discovery chatter, and why it is a hard call.** These protocols exist to find
things on a LAN, and over a 2 MHz link every multicast frame is a broadcast PPDU at MCS 0
that nobody acknowledges: a 300-octet mDNS packet costs about 4 ms of air, so a laptop
that re-announces five services and answers every query it hears can burn a few percent
of the channel doing nothing useful.

- **mDNS (UDP 5353, ff02::fb)**: Bonjour/Avahi. Every host answers every query for every
  service type it offers, re-announces at 80/85/90/95 % of each record's TTL, sends
  goodbye packets, and clients re-query each service type separately. It is also the
  protocol most likely to be *useful* for peer discovery in a gossip or epidemic scheme,
  because it is already there and every OS speaks it. The compromise, not implemented
  yet, would be to pass only records under a chosen name, say `_mesh._udp.local`, and
  drop the rest: the DNS question and answer sections are plain to parse. Until then the
  rule is all or nothing (`--allow-discovery`). Babel already announces every node's
  presence with Hellos every few seconds on ff02::1:6, which is usually the discovery a
  mesh needs.
- **LLMNR (UDP 5355, ff02::1:3)**: Windows' fallback name resolution. One multicast query
  per unresolved name per interface, unicast answers, low volume, but it leaks every
  mistyped hostname and every internal name a Windows box tries to reach.
- **SSDP/UPnP (UDP 1900, ff02::c)**: M-SEARCH bursts every few minutes from Windows and
  media players, NOTIFY alive/byebye from every UPnP device, around a kilobyte each. Zero
  use over a mesh.
- **WS-Discovery (UDP 3702, ff02::c)**: Windows Network Discovery and WSD printing. Bursts
  of Probe/Hello/Bye on interface up and on a timer.
- **NetBIOS (137 to 139)** and **SMB (445)**: NetBIOS is IPv4 in practice and listed for
  completeness; SMB 3 encrypts by default and is a common worm vector.
- **NAT-PMP/PCP (5350/5351)**: asks a gateway for port mappings; there is no gateway.

**Other noise left alone.** ICMPv6 Node Information queries (139/140), NTP broadcast,
and any host's own keepalives. Block them with `--block-port` where they have a port.

Knobs: `--no-filter`, `--filter-egress-only`, `--block-port N` / `--allow-port N`,
`--allow-ipv4`, `--allow-global`, `--allow-router-discovery`, `--allow-dhcpv6`,
`--allow-mld`, `--allow-discovery`, `--allow-esp`. The node prints the policy at startup
and logs each (direction, reason) the first time it fires and then at most every 30 s
with a count. The library default (`MacConfig::new`) is no filtering; the node turns the
policy on.

### A-MPDUs, S-MPDUs and the partial AID (background for non-RF readers)

*Aggregation.* An 802.11 PHY frame (PPDU) can carry several MAC frames back to back: an
**A-MPDU** is a list of `[4-byte delimiter][MAC frame][pad]` records inside one PPDU, like
a length-prefixed record stream. In S1G the PHY header's length field is only 9 bits, so any
MAC frame over 511 octets *must* travel this way even if it is alone; the aggregation bit
switches the length units from octets to OFDM symbols. The standard then says what a
real multi-frame A-MPDU may contain: QoS Data frames (they carry a traffic ID and an ack
policy), acknowledged with a **BlockAck** bitmap covering all of them, which in turn needs a
Block Ack agreement negotiated beforehand (ADDBA), a stateful handshake this OCB MAC does
not do.

*What s2g does.* A frame over 511 octets is sent as an **S-MPDU**: an A-MPDU whose single
record has the EOF bit set in its delimiter. The standard (10.12.8) defines an S-MPDU as
"the rules of a non-aggregated frame apply": any MAC frame that is valid on its own is valid
inside it, no Block Ack agreement is needed, and it is acknowledged with an ordinary
(NDP) Ack. So plain Data frames inside s2g's aggregated PSDUs are conformant, and a standard
receiver deaggregates them with its normal A-MPDU parser.

*Partial AID.* When an S1G station associates with an AP it is assigned an **AID** (a small
integer, like a session id). The 9-bit "partial AID" in the PHY header and in an NDP CTS
is derived from it, so receivers can tell early whether a PPDU is for them. There is no
association in OCB and hence no AID, so s2g hashes the MAC address into those 9 bits
(`ndp::ocb_partial_aid`). Both ends of an s2g link compute the same value, but a standard
station would not, so only the RA field of s2g's NDP CTS frames is affected.

`s2g-node` wires NIC ↔ MAC ↔ PHY ↔ Pluto:

```sh
# Linux / macOS / *BSD: a real L2 TAP interface (build with the tap feature)
cargo build --release --features tap
sudo target/release/s2g-node --tap s2g0 --uri 192.168.2.1 --mcs 2 --ldpc --rts-threshold 300
# then: ip addr add 10.99.0.1/24 dev s2g0   (etc. on each node)

# Windows: tap-windows6 from an elevated prompt (see above), or the Ethernet-over-UDP NIC
target\release\s2g-node.exe --udp 127.0.0.1:5001 --uri 192.168.2.1
```

The `Nic` trait in `s2g-tools` keeps the attachment point pluggable; TAP is via the
cross-platform `tappers` crate (Linux/macOS/FreeBSD/OpenBSD/NetBSD). Two nodes need distinct `--mac` addresses (default is randomized).

### Frames in Wireshark

`--pcap PATH` records every MPDU the PHY decodes (bad FCS flagged) and every MPDU the
node transmits behind a radiotap header: the S1G field (PPDU format, guard interval, MCS,
response indication, RSSI), the TX flags field on transmitted frames, one reference
number shared by the subframes of an A-MPDU. PATH is a file, `-` (standard output; node
messages go to stderr), a Windows named pipe `\\.\pipe\NAME` or an existing FIFO.
Wireshark reads a pipe live and can detach and attach again at any time; start the node
first.

```sh
# Windows
target\release\s2g-node.exe --udp 127.0.0.1:5001 --uri 192.168.2.1 --pcap \\.\pipe\s2g
"C:\Program Files\Wireshark\Wireshark.exe" -k -i \\.\pipe\s2g
# Unix
mkfifo /tmp/s2g.pcap
sudo target/release/s2g-node --tap s2g0 --uri 192.168.2.1 --pcap /tmp/s2g.pcap &
wireshark -k -i /tmp/s2g.pcap
# any platform, through standard output
target/release/s2g-node --udp 127.0.0.1:5001 --uri 192.168.2.1 --pcap - | wireshark -k -i -
```

NDP CMAC PPDUs (NDP Ack, NDP BlockAck, NDP CTS) carry no MPDU and do not appear;
`--verbose` prints them. Display filters: `radiotap.txflags` for transmitted frames,
`radiotap.flags.badfcs == 1` for failed FCS (Wireshark's own FCS column stays
"unverified" unless its 802.11 preference "Validate the FCS checksum" is on). `s2g-rx
--pcap` writes the same format, so a recording or a receive-only Pluto shows in Wireshark
the same way.

## Status / roadmap

- [x] TX chain: preamble (STF/LTF1), SIG (CRC-4, QBPSK), scrambler, BCC + puncturing,
      interleaver, LDPC (Annex F codes, 19.3.11.7.5 encoding process, tone mapper),
      constellation mapping, fixed/traveling pilots, OFDM assembly: all MCSes
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
- [x] OCB MAC: data/RTS/ACK frames, A-MPDU packing with NDP BlockAck selective retry,
      S-MPDU, PV1 reception, CSMA with PHY CCA + NAV + RID + EIFS, NDP responses, retries,
      dedup, per-peer adaptive MCS (probing + SNR-bounded), amateur station identification,
      stateless good-neighbor frame filter (IPv6-only, no RA/RS/DHCPv6, no port 22/443)
- [x] `s2g-node`: TAP (Unix via tappers, Windows via tap-windows6) / UDP network interface
      over the radio
- [x] Virtual Pluto (iiod server with simulated air) and a two-node end-to-end script;
      capture regression script
- [ ] Hardware-timestamped SIFS/ACK timing
- [x] S1G_LONG SU Data-field reception and short-GI reception (both optional for a ≤ 2 MHz
      STA; validated on the baby-monitor capture); also available on TX via `TxVector`
- [ ] Other NDP CMAC types (PS-Poll, Paging, Probe Request), 1/4/8/16 MHz, multi-stream/STBC:
      all optional (or 1 MHz: skipped by choice); module boundaries chosen so they slot in

## License

AGPL-3.0-or-later (see `LICENSE`). Copyright (C) 2026 Robert B. Langer.
