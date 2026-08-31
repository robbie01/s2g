# S1G PHY digest — TX/RX procedures and RF requirements (2 MHz, 1 SS, LGI, BCC, S1G_SHORT, SU)

Scope: IEEE Std 802.11-2024, Clause 23 (S1G / 802.11ah PHY), sections 23.3.17–23.3.20 plus the
supporting constants (Table 23-5, Table 23-11, 23.4.3, 23.4.4) needed to make this self-contained.
Everything is specialized to **2 MHz channel bandwidth (CBW2), 1 spatial stream (NSTS = NSS = 1,
no STBC), long GI (8 µs), BCC coding, S1G_SHORT preamble, SU PPDU**. Other bandwidths / formats
(1 MHz S1G_1M, S1G_LONG, duplicate modes, 4/8/16 MHz, LDPC, MU, short GI) are noted where they
appear but not expanded. Page numbers are the PDF page of `80211-2024.pdf` (printed page number is
PDF page − 1). All numeric tables below were verified against the rendered PDF pages, not just the
text extraction.

Conventions used throughout:

- Subcarrier indices are physical OFDM tone indices, DC = 0, range −32..+31 for the 64-point
  2 MHz FFT. Data+pilot tones occupy −28..−1, +1..+28 (N_ST = 56) [Table 23-5, p3757].
- dBr = dB relative to the maximum spectral density of the signal [23.3.17.1, p3824].
- All primitive names (`PHY-TXSTART.request` etc.) are the PHY SAP API; they map 1:1 onto the
  state machine events an SDR PHY driver must implement.

---

## 1. Key constants used in this digest

From Table 23-5 (timing-related constants), CBW2 column [23.3.6, pp3756–3758]:

| Constant | Value (2 MHz) | Meaning |
|---|---|---|
| N_SD | 52 | data subcarriers per OFDM symbol |
| N_SP | 4 | pilot subcarriers per OFDM symbol |
| N_ST | 56 | N_SD + N_SP (total used tones) |
| N_SR | 28 | highest used subcarrier index |
| ΔF | 31.25 kHz | subcarrier spacing |
| T_DFT | 32 µs | IDFT/DFT period (= 1/ΔF; 64 samples @ 2 Msps) |
| T_GI | 8 µs | long guard interval (= T_DFT/4; 16 samples) |
| T_GIS | 4 µs | short GI (out of scope) |
| T_SYML | 40 µs | long-GI OFDM symbol (= T_DFT + T_GI; 80 samples) |
| T_SYMS | 36 µs | short-GI symbol (out of scope) |
| T_STF | 80 µs = 2·T_SYML | STF duration (≥2 MHz formats) |
| T_LTF1 | 80 µs = 2·T_DFT + T_GI2 (T_GI2 = 16 µs) | LTF1 duration |
| T_LTF | 40 µs | each additional LTF symbol (LTF2..LTF_NLTF) |
| T_SIG | 80 µs = 2·T_SYML | SIG field duration (S1G_SHORT, 2 symbols) |
| N_service | 8 | SERVICE field bits |
| N_tail | 6 | tail bits per BCC encoder |

From Table 23-11 [23.3.8.2.2.4, p3796]: N_LTF as a function of N_STS: {1→1, 2→2, 3→4, 4→4}.
**For SISO, N_LTF = 1** (no LTF2..N symbols; the S1G_SHORT PPDU is
STF(2 sym) | LTF1(2 sym) | SIG(2 sym) | Data) [23.3.2, Fig 23-1, p3741].

### 1.1 2 MHz, N_SS = 1 MCS set (Table 23-46, pp3858–3859; verified against PDF)

| MCS | Mod | R | N_BPSCS | N_CBPS | N_DBPS | N_ES | Data rate LGI (kb/s) |
|---|---|---|---|---|---|---|---|
| 0 | BPSK | 1/2 | 1 | 52 | 26 | 1 | 650.0 |
| 1 | QPSK | 1/2 | 2 | 104 | 52 | 1 | 1300.0 |
| 2 | QPSK | 3/4 | 2 | 104 | 78 | 1 | 1950.0 |
| 3 | 16-QAM | 1/2 | 4 | 208 | 104 | 1 | 2600.0 |
| 4 | 16-QAM | 3/4 | 4 | 208 | 156 | 1 | 3900.0 |
| 5 | 64-QAM | 2/3 | 6 | 312 | 208 | 1 | 5200.0 |
| 6 | 64-QAM | 3/4 | 6 | 312 | 234 | 1 | 5850.0 |
| 7 | 64-QAM | 5/6 | 6 | 312 | 260 | 1 | 6500.0 |
| 8 | 256-QAM | 3/4 | 8 | 416 | 312 | 1 | 7800.0 |
| 9 | — | Not valid at 2 MHz / 1 SS | | | | | |
| 10 | — | Not valid (1 MHz only: BPSK 1/2 with 2× repetition) | | | | | |
| 11 | 1024-QAM | 3/4 | 10 | 520 | 390 | 1 | 9750.0 |
| 12 | — | Not valid at 2 MHz / 1 SS | | | | | |

Mandatory support: an AP supports MCS 0–7 (1 SS) on every supported width; a non-AP STA supports
MCS 0–2 (1 SS) at 1 and 2 MHz. MCS 8/9 and 11/12 (when valid) are optional [23.5, pp3855–3856].

---

## 2. PHY transmit procedure [23.3.19, pp3839–3842]

### 2.1 Primitive sequence (S1G_SHORT SU, the API contract)

The typical procedure (Figure 23-45, p3839; verified) is:

1. MAC (optionally, earlier) issues `PHY-CONFIG.request(PHYCONFIG_VECTOR)` to set CCA type,
   group/partial-AID filters (see §3.5). The PHY is tuned to the operating frequency via the PLME
   [p3840].
2. MAC checks `PHY-CCA.indication` state (medium idle) — the MAC, not the PHY, gates channel
   access [p3840].
3. MAC → `PHY-TXSTART.request(TXVECTOR)`. PHY enters the transmit state and **starts transmitting
   the preamble immediately**, using the TXVECTOR parameters (Table 23-1): FORMAT=S1G,
   PREAMBLE_TYPE=S1G_SHORT_PREAMBLE, CH_BANDWIDTH=CBW2, MCS, LENGTH, AGGREGATION,
   GI_TYPE=LONG_GI, FEC_CODING=BCC_CODING, STBC=0, NUM_STS=1, NDP_INDICATION=0,
   PARTIAL_AID, UPLINK_INDICATION, COLOR, TXPWR_LEVEL, … [pp3840–3841].
4. PHY → `PHY-TXSTART.confirm` to MAC (issued after preamble transmission starts) [p3840].
5. While the preamble is on air, the PHY starts scrambling + BCC encoding of the Data field
   (SERVICE field + PSDU, per 23.3.2/23.3.3). Data octets flow MAC→PHY as a series of
   `PHY-DATA.request(DATA)` (one octet each), each acknowledged by `PHY-DATA.confirm` [p3840].
6. 0–7 PHY pad bits are appended to the PSDU so the coded PSDU is an integer number of OFDM
   symbols; then the 6 BCC tail bits ("Tail Bits" block in Fig 23-45) [p3840; state machine
   PADDING & TAIL box p3842].
7. MAC → `PHY-TXEND.request` terminates PSDU transmission (normal termination occurs after the
   final bit of the last PSDU octet, according to N_SYM from 23.4.3; the request may also
   prematurely terminate at any stage). PHY → `PHY-TXEND.confirm` [p3841, and the "A" bubble in
   Fig 23-49, p3842].
8. When the PPDU transmission completes, the PHY entity **enters the receive state** [p3842].
9. GI insertion: the (long) GI is inserted in every data OFDM symbol [p3841].

Field-by-field on-air content for S1G_SHORT (Fig 23-45 / Fig 23-49, pp3839, 3842):
STF, LTF1, then **SIG: 2 symbols, coded OFDM, QBPSK, rate 1/2** (both SIG symbols are
QBPSK-rotated — this is what distinguishes S1G_SHORT from S1G_LONG, whose SIG-A1 is QBPSK and
SIG-A2 is BPSK), then Training Symbols (LTF2..LTF_NLTF; **absent for SISO**), then Data with
"Coded OFDM, MCS indicated in SIG".

### 2.2 Transmit state machine (Figure 23-49, p3842; described in text)

States and transitions (SU, FORMAT=S1G, S1G_SHORT_PREAMBLE, NDP_INDICATION=0 path):

- **Initialize / Set TX parameters** — on `PHY-TXSTART.request(TXVECTOR)`. Branch on
  FORMAT/PREAMBLE_TYPE/NDP_INDICATION (S1G_DUP_1M/2M branches "not explained"; S1G_LONG,
  S1G_1M, and NDP branches exist).
- **TX PREAMBLE** — for our branch: TX STF → TX LTF1 → TX SIG (QBPSK) → TX Training Symbols
  (none for SISO).
- **TX DATA (setup)** — "Use MCS and number of space-time streams set by TXVECTOR. 8-bit SERVICE
  field prepended, padding and tail bits (BCC only) appended to PSDU."
- **SETUP PSDU TX** — issue `PHY-TXSTART.confirm`; set symbol count = N_SYM (for NDP: number of
  SIG symbols).
- **TX PSDU OCTET** — on each `PHY-DATA.request(DATA)`: get octet from MAC, scramble, encode,
  buffer; issue `PHY-DATA.confirm`. Loop while more octets and buffer < one symbol.
- Decision **Last Symbol?**: No + buffer holds a symbol's worth (or last octet received) → **TX
  SYMBOL**. Yes → **PADDING & TAIL**: add PHY padding bits, scramble, encode, buffer; encode &
  buffer tail bits (BCC only) → TX SYMBOL.
- **TX SYMBOL → Decrement Symbol**: decrement symbol count; if count > 0 loop back to
  TX PSDU OCTET/TX SYMBOL; if count = 0 → **Switch RX state** (PHY returns to receive).
- At **any** stage, a received `PHY-TXEND.request` jumps to the termination point "A"
  (→ Switch RX state), i.e. transmission can be cut off asynchronously.

NDP CMAC PPDU differences (out of scope for data TX but part of the same machine)
[23.3.19, p3842]: no SERVICE field, no PHY pad bits, no scrambling, PSDU maps into the SIG field
(1-bit NDP Indication prepended, CRC + tail bits appended in SIG).

### 2.3 TXTIME (PLME-TXTIME.confirm) — S1G_SHORT [23.4.3, Eq (23-73)/(23-74), p3852]

Long GI (Eq 23-74):

```text
TXTIME = T_PREAMBLE + T_SIG + T_LTF*(N_LTF - 1) + T_SYML*N_SYM
T_PREAMBLE = T_STF + T_LTF1
```

**2 MHz / SISO / LGI: TXTIME = 80 + 80 + 80 + 0 + 40*N_SYM = 240 + 40*N_SYM µs.**
(Short-GI variant Eq (23-73) noted, out of scope. For an NDP, N_SYM = 0 [p3853].)

N_SYM for BCC SU (identical to Eq (23-65) used on receive; see §3.4).

---

## 3. PHY receive procedure [23.3.20, pp3842–3850]

### 3.1 Reception flow (S1G_SHORT, SU) — Figure 23-51 (p3844, verified) plus text

Phase A — **CS/CCA state** (before/while the preamble arrives):

1. On detecting a PPDU whose preamble overlaps the primary 2 MHz channel (≥2 MHz BSS) — or
   primary 1 MHz in a 1 MHz BSS — the PHY measures receive signal strength (RSSI is measured over
   the STF/LTF1) and issues **`PHY-CCA.indication(BUSY, channel-list)`** as the initial indication
   of reception, per the CCA sensitivity rules of 23.3.18.5 (§5.4 below). channel-list ∈
   {primary1, primary2, secondary2, secondary4, secondary8} subsets [p3843].
   A PPDU that does **not** overlap the primary channel never generates
   `PHY-RXSTART.indication` [p3843].
2. The PHY receives the training symbols and searches for/decodes **SIG** (S1G_SHORT: 2 QBPSK
   symbols; the QBPSK-vs-BPSK rotation of symbol 2 discriminates S1G_SHORT from S1G_LONG —
   state "Detect SIG: determine type of SIG field", Fig 23-53) to obtain PPDU duration,
   modulation, coding type and rate [p3843].

Phase B — SIG evaluation and outcomes [pp3843–3846]:

- **SIG CRC fails** → no `PHY-RXSTART.indication` (nor `PHY-RXEARLYSIG.indication`); PHY issues
  **`PHY-RXEND.indication(FormatViolation)`** and sets `PHY-CCA.indication(IDLE)` only **when the
  receive level drops below (minimum-MCS sensitivity + 20 dB)** — for 2 MHz that threshold is
  −92 + 20 = **−72 dBm** [pp3843, 3845].
- **SIG CRC valid**: the PHY predicts the PPDU duration RXTIME (Eq 23-69, §3.4) and maintains
  `PHY-CCA.indication(BUSY, channel-list)` for that duration — for supported modes, unsupported
  modes, and "Reserved SIG Indication" alike. Two sub-cases [pp3844–3845]:
  - The SIG's ID/COLOR matches this STA (Uplink Indication = 1 and ID = own BSS PBSSID, or
    Uplink Indication = 0 and COLOR = own AP's COLOR): hold BUSY for RXTIME unconditionally.
  - No match: hold BUSY for RXTIME **if the reception meets the minimum CCA sensitivity level of
    23.3.18.5.3**.
  - *Reserved SIG (or SIG-A) Indication* is defined as: Reserved bits equal to 0, or a field
    combination not valid per 23.3.8.2.2.5 / 23.3.8.2.3.2.5 / 23.3.8.3.5, or an S1G-MCS/NSTS
    combination not in 23.5, or any other bit combination not corresponding to a defined mode
    → treated as **`PHY-RXEND.indication(FormatViolation)`** [pp3845–3846].
- **Unsupported mode** in SIG (valid CRC, but a mode this receiver doesn't implement, e.g. an
  optional MCS) → `PHY-RXSTART.indication(RXVECTOR)` is issued **then**
  **`PHY-RXEND.indication(UnsupportedRate)`** (Fig 23-53 shows RXSTART-then-RXEND for the
  unsupported-mode exit) [pp3845–3846].
- **Supported mode** → PHY receives the remaining training symbols (none for SISO), then issues
  **`PHY-RXSTART.indication(RXVECTOR)`** (RXVECTOR per Table 23-1, §3.3). If
  dot11TimingMsmtActivated, RXVECTOR carries RX_START_OF_FRAME_OFFSET [p3846].
  - S1G_SHORT/S1G_1M with **NDP Indication = 1** in SIG (NDP CMAC PPDU): the PHY generates
    `PHY-CCA.indication(IDLE)` and returns to RX IDLE without issuing
    `PHY-RXSTART.indication` (the "NDP MAC frame" check in Fig 23-53) [p3846].
  - (S1G_LONG only, noted: after valid SIG-A, SU PPDUs don't require SIG-B decoding →
    `PHY-RXSTART.indication` directly; MU PPDUs decode SIG-B after a
    `PHY-RXEARLYSIG.indication`; SIG-B CRC fail → FormatViolation; SIG-B unsupported →
    UnsupportedRate [p3846].)
- **Filtering** (optional, §3.5): a filtered-out PPDU produces `PHY-RXSTART.indication(RXVECTOR)`
  then **`PHY-RXEND.indication(Filtered)`** [pp3846–3847].

Phase C — **RX state** (Data field):

- Data symbols are decoded and descrambled; received PSDU bits are assembled into octets and
  delivered via repeated **`PHY-DATA.indication(DATA)`**. Final bits that don't form a complete
  octet are pad bits and are discarded [p3849].
- RCPI is measured (over STF or LTF fields) and reported [23.3.18.7, p3838].
- After the final PSDU octet (+ possible pad/tail bits):
  **`PHY-RXEND.indication(NoError, RXVECTOR)`**, then **`PHY-CCA.indication(IDLE)`**, and return
  to RX IDLE (Fig 23-51 ordering: PHY-DATA.indication…, PHY-RXEND.indication(NoError,RXVECTOR),
  PHY-CCA.indication(IDLE)) [pp3844, 3849].
- **Carrier lost** during PSDU reception → **`PHY-RXEND.indication(CarrierLost)`**; the PHY then
  *waits out the remainder of RXTIME* before setting `PHY-CCA.indication(IDLE)` and returning to
  RX IDLE [p3849].

### 3.2 Receive state machine (Figure 23-53, p3845 — verified; transitions in text)

- **RX IDLE / CS-CCA**: on signal detection meeting a CCA condition, set
  `PHY-CCA.indication(BUSY, primary)` → **Detect SIG** (determine SIG field type).
  - Branch S1G_SHORT/S1G_1M → **RX SIG** (receive + test CRC).
  - Branch S1G_LONG → **RX SIG-A** (out of scope; symmetric structure with SIG-B sub-path).
- **RX SIG → Evaluate SIG** on CRC OK. On CRC Fail → `PHY_RXEND.indication(FormatViolation)`,
  then wait until receive level < (min-MCS sensitivity + 20 dB) before
  `PHY-CCA.indication(IDLE)` → RX IDLE.
- **Evaluate SIG** (check contents for supported mode):
  - Reserved SIG Indication → `PHY_RXEND.indication(FormatViolation)` → End-of-Wait path.
  - Unsupported mode → `PHY_RXSTART.indication(RXVECTOR)` then
    `PHY_RXEND.indication(UnsupportedRate)` → End-of-Wait path.
  - Supported mode → **Check contents for NDP MAC frame**: Yes → (deliver NDP body;
    `PHY-CCA.indication(IDLE)`, no RXSTART) → RX IDLE. No → **Determine if PPDU is filtered**.
- **Filter check** (based on PHYCONFIG_VECTOR): Filtered out →
  `PHY_RXSTART.indication(RXVECTOR)` then `PHY_RXEND.indication(Filtered)` → End-of-Wait path.
  Not filtered → **Setup PSDU RX**.
- **Setup PSDU RX**: set N_symbol = N_SYM; issue `PHY_RXSTART.indication(RXVECTOR)` →
  **RX Symbol**.
- **RX Symbol**: Valid signal → **Decode Symbol** (decode + descramble, bit removing if needed;
  `PHY_DATA.indication(DATA)`; decrement N_symbol). N_symbol > 0 → RX Symbol;
  N_symbol = 0 → **End of PSDU RX**: RxEndStatus = (NoError, RXVECTOR) → set
  `PHY-CCA.indication(IDLE)` → RX IDLE.
  Carrier lost → **Signal Not Valid**: RxEndStatus = (CarrierLost, Null), i.e.
  `PHY_RXEND.indication(CarrierLost)` → **Decrement Time** ("wait for intended end of PSDU based
  on RXTIME"); Time = 0 → End of Wait → `PHY_CCA.indication(IDLE)` → RX IDLE.
- **End-of-Wait rule** (annotation): for unsupported modes, Reserved SIG/SIG-A/SIG-B indication,
  SIG-B CRC failure, or filtered PPDU, set `PHY_CCA.indication(IDLE)` when the predicted duration
  based on RXTIME has elapsed. For SIG/SIG-A CRC failure, set `PHY_CCA.indication(IDLE)` when the
  receive level drops below (minimum modulation-and-coding-rate sensitivity + 20 dB).

Implementation summary of error exits:

| Condition | Primitives | CCA release |
|---|---|---|
| SIG CRC fail | RXEND(FormatViolation) | when RX level < min-sens + 20 dB (−72 dBm @ 2 MHz) |
| Reserved SIG indication | RXEND(FormatViolation) | after RXTIME elapses |
| Unsupported MCS/mode | RXSTART(RXVECTOR) + RXEND(UnsupportedRate) | after RXTIME elapses |
| Filtered PPDU | RXSTART(RXVECTOR) + RXEND(Filtered) | after RXTIME elapses |
| Carrier lost mid-PSDU | RXEND(CarrierLost) | after RXTIME elapses |
| Success | RXSTART(RXVECTOR), PHY-DATA.indication×n, RXEND(NoError,RXVECTOR) | immediately after PPDU end |

### 3.3 RXVECTOR population (Table 23-1, pp3732–3739; S1G_SHORT SU relevant subset)

Set from decoded SIG + measurements:

- FORMAT = S1G; PREAMBLE_TYPE = S1G_SHORT_PREAMBLE [p3732].
- MCS: integer, range 0–9, 11, 12 for ≥2 MHz (from SIG MCS field) [p3736].
- CH_BANDWIDTH: CBW1/2/4/8/16 (from preamble/SIG BW detection) [p3737].
- LENGTH: octets in PSDU if NOT_AGGREGATED; number of symbols if AGGREGATED (from SIG Length
  field) [p3737].
- PSDU_LENGTH: octets; computed per 23.4.3 rules (§3.4); 0 indicates an S1G NDP [p3738].
- AGGREGATION: AGGREGATED / NOT_AGGREGATED (SIG Aggregation bit) [p3733].
- GI_TYPE: LONG_GI / SHORT_GI (SIG Short GI bit) [p3735].
- FEC_CODING: BCC_CODING / LDPC_CODING (SIG Coding bit) [p3735].
- STBC: 0/1; NUM_STS: 0–4 in RXVECTOR [pp3735, 3738].
- SMOOTHING: 1 if frequency-domain smoothing recommended [p3733].
- NDP_INDICATION: 0/1 [p3732].
- PARTIAL_AID (0–511 if UPLINK_INDICATION=1 else 0–63), UPLINK_INDICATION, COLOR (0–7)
  [pp3738–3740].
- RESPONSE_INDICATION: 0 = No Response, 1 = NDP Response, 2 = Normal Response, 3 = Long Response
  [p3739]. TRAVELING_PILOTS: 0/1 (optional in RXVECTOR) [p3739].
- RSSI: 0–255, measured over the LTF, monotonically increasing with RX power; the most recently
  measured RSSI value is included in `PHY-RXSTART.indication(RXVECTOR)` [pp3736, 3843;
  23.3.18.6, p3838].
- RCPI: received RF power in dBm over STF or LTF fields, averaged over active RX chains,
  accuracy ±5 dB (95% CI); if measured over the STF of a 1 MHz MCS10 PPDU, report 3 dB less
  (that STF is boosted 3 dB) [23.3.18.7, p3838].
- SNR: per-spatial-stream 8-bit average SNR (sum of per-tone dB SNR / number of tones) [p3735].
- SCRAMBLER_OR_CRC: for NDP_INDICATION=0, the 7-bit scrambler init [B0:B6] of the SERVICE field
  prior to descrambling; for NDP=1, the calculated 4-bit SIG CRC [p3740].
- RX_START_OF_FRAME_OFFSET (only if dot11TimingMsmtActivated): 0..2^32−1 in 10 ns units, offset
  from preamble arrival at the antenna port to the issuing of RXSTART [pp3739, 3846].

### 3.4 N_SYM, RXTIME, PSDU_LENGTH equations (BCC) [pp3847–3849; verified in PDF]

**N_SYM** (BCC; Eq 23-65, p3847 — same as the transmit-side Eq (23-79)/(23-80) in 23.4.3):

```text
if Aggregation == 1:   N_SYM = LENGTH                      # LENGTH is in symbols
else:                  N_SYM = m_STBC * ceil( (8*LENGTH + N_service + N_tail*N_ES)
                                              / (m_STBC * N_DBPS) )
# m_STBC = 1 (no STBC). 2 MHz/1SS: N_service=8, N_tail=6, N_ES=1
# => N_SYM = ceil( (8*LENGTH + 14) / N_DBPS )
```

(LDPC: Eq (23-66): N_SYM = LENGTH when aggregated; Eq (23-67)/(23-68) with the LDPC Extra OFDM
Symbol SIG bit otherwise — noted, out of scope [p3848].)

**RXTIME** — S1G_SHORT / S1G_1M (Eq 23-69, p3848, transcribed exactly from the PDF):

```text
RXTIME(µs) = T_DSTF + T_LTF*(N_LTF − 1) + N_SYM*T_SYML                      , Short GI = 0
RXTIME(µs) = T_DSTF + T_LTF*(N_LTF − 1) + T_SYML + (N_SYM − 1)*T_SYMS      , Short GI = 1
```

Specialized (2 MHz, SISO, LGI): T_DSTF = 40, N_LTF = 1 →
**RXTIME = 40 + 40·N_SYM µs**.

> ⚠ Editorial caution: Eq (23-69) literally uses T_DSTF, which Table 23-5 defines only for the
> S1G_LONG D-STF (and marks N/A for CBW1), while S1G_SHORT/S1G_1M have no D-STF. Comparing with
> Eq (23-70) — where RXTIME = T_DSTF + N_LTF·T_DLTF + T_SIG-B + N_SYM·T_SYML exactly equals the
> *remaining* PPDU duration measured from the end of SIG-A — RXTIME is a *residual* duration used
> to hold CCA BUSY / wait out errored PPDUs, not the full PPDU duration (which is TXTIME,
> §2.3). Under that reading the S1G_SHORT residual after SIG should be
> (N_LTF−1)·T_LTF + N_SYM·T_SYML, and the extra 40 µs "T_DSTF" term acts as a one-symbol margin.
> For an SDR implementation the safe behavior is: hold CCA BUSY until the PPDU end computed from
> the *full* TXTIME anchored at the detected PPDU start (240 + 40·N_SYM µs from preamble start),
> which is consistent with 40 + 40·N_SYM µs measured from the end of the 240 µs
> STF+LTF1+SIG minus one 40 µs decode symbol. Flagged in Gaps.

**RXTIME** — S1G_LONG (Eq 23-70, p3849, for completeness):
`RXTIME = T_DSTF + N_LTF*T_DLTF + T_SIG-B + N_SYM*T_SYML` (LGI).

**PSDU_LENGTH returned in RXVECTOR** [p3849]:

```text
if Aggregation == 0:  PSDU_LENGTH = LENGTH                 # octets, directly from SIG
if Aggregation == 1 (BCC, Eq 23-71):
    PSDU_LENGTH = floor( (N_SYM*N_DBPS − N_service − N_tail*N_ES) / 8 )
                = floor( (N_SYM*N_DBPS − 14) / 8 )         # 2 MHz / 1 SS
# LDPC (Eq 23-72): floor((N_SYM*N_DBPS − N_service)/8) — noted, out of scope
# NDP sounding / NDP CMAC with Aggregation=1: PSDU_LENGTH = 0
```

### 3.5 PPDU filtering by GID/PARTIAL_AID [pp3846–3847]

The PHY may (optionally) filter PPDUs using SIG fields (GID, MU[0-3] NSTS,
UPLINK_INDICATION, ID) against the PHYCONFIG_VECTOR
(GROUP_ID_MANAGEMENT, PARTIAL_AID_LIST_GID00/63, LISTEN_TO_GID00/63,
CCA_SENSITIVITY_TYPE — definitions in 23.2.5, pp3741–3742). It shall NOT filter when any of:

- g = 0 ∧ LISTEN_TO_GID00 ∧ partial AID ∈ PARTIAL_AID_LIST_GID00
- g = 63 ∧ LISTEN_TO_GID63 ∧ partial AID ∈ PARTIAL_AID_LIST_GID63
- 1 ≤ g ≤ 62 ∧ MembershipStatusInGroupID[g] = 1 ∧ nSTS[UserPositionInGroupID[g]] > 0

where for an SU PPDU g = 0 if UPLINK_INDICATION = 1, else g = 63. Otherwise it *may* filter →
`PHY-RXEND.indication(Filtered)`.

---

## 4. S1G transmit specification [23.3.17, pp3823–3830]

### 4.1 Transmit spectrum mask — 2 MHz PPDU [23.3.17.1, p3824; figure 23-40 p3824]

Two components; the **overall mask at each offset = max(interim mask in dBr → dBm, absolute
limit)**:

- Interim mask (dBr, relative to max PSD of the signal), symmetric around the channel center:

| |f| offset (MHz) | Level |
|---|---|
| 0 → 0.9 | 0 dBr (0 dBr bandwidth = 1.8 MHz) |
| 0.9 → 1.1 | linear in dB from 0 dBr to −20 dBr |
| 1.1 → 2.0 | linear in dB from −20 dBr to −28 dBr |
| 2.0 → 3.0 | linear in dB from −28 dBr to −40 dBr |
| ≥ 3.0 | −40 dBr |

- Absolute limit: transmit spectrum shall not exceed max(interim mask, **−43 dBm/MHz**) at any
  offset (i.e. the −43 dBm/MHz line floors the mask when the −40 dBr level would fall below it).
- Measurement: 10 kHz resolution bandwidth, 100 Hz video bandwidth [p3824].
- The mask does not apply to RF LO leakage (that is bounded separately, §4.4) [p3823].
- Regulatory masks apply in addition [p3823].

Pseudocode for the test limit at offset f (MHz), given measured peak PSD P_max (dBm/MHz):

```text
def mask_dBr(f):
    a = abs(f)
    if a <= 0.9:  return 0.0
    if a <= 1.1:  return lerp(a, 0.9, 0.0, 1.1, -20.0)
    if a <= 2.0:  return lerp(a, 1.1, -20.0, 2.0, -28.0)
    if a <= 3.0:  return lerp(a, 2.0, -28.0, 3.0, -40.0)
    return -40.0
limit_dBm_per_MHz(f) = max(P_max + mask_dBr(f), -43.0)
```

(Other widths, noted only: 1 MHz — 0 dBr bw 0.9 MHz, −20 dBr @0.6, −28 @1.0, −40 @1.5, floor
−40 dBm/MHz; 4 MHz — 3.8/2.1/4/6, floor −46; 8 MHz — 7.8/4.1/8/12, floor −49; 16 MHz —
15.8/8.1/16/24, floor −49 [pp3823–3826].)

### 4.2 Spectral flatness — 2 MHz normal mode [23.3.17.2 + Table 23-33, pp3826–3827; verified]

Measured on **BPSK-modulated PPDUs**; E_i,avg = average constellation energy of subcarrier i;
averaging is **in the linear domain** over the averaging set; spatial mapping must be flat;
conducted (cable) test.

- Averaging subcarrier indices: **−16..−1 and +1..+16** (32 tones).
- Tested set 1 = −16..−1, +1..+16: each tone's E_i,avg within **±4 dB** of the average.
- Tested set 2 = −28..−17 and +17..+28 (24 tones): within **+4 / −6 dB** of the average.

```text
avg = mean_linear(E[i] for i in [-16..-1, 1..16])
for i in [-16..-1, 1..16]:        assert -4 <= 10*log10(E[i]/avg) <= +4
for i in [-28..-17, 17..28]:      assert -6 <= 10*log10(E[i]/avg) <= +4
```

### 4.3 Center frequency / symbol clock tolerance [23.3.17.3, p3826]

- **Symbol clock frequency tolerance: ±20 ppm.**
- **Transmit center frequency tolerance: ±20 ppm.**
- All TX antennas/frequency segments derive both from the **same reference oscillator**.
- (Channel center frequency: fc [MHz] = ChannelStartingFrequency + 0.5 × ChannelCenterChannelNumber,
  per Annex E [23.3.14, p3823].)

### 4.4 TX center frequency (LO) leakage [23.3.17.4.2, p3828]

With P = transmit power per antenna (dBm), 31.25 kHz resolution BW:

- LO at the center of the transmitted PPDU BW: power at the center ≤ average per-subcarrier
  power = **P − 10·log10(N_ST)** → 2 MHz: **P − 10·log10(56) ≈ P − 17.5 dB**.
- LO not at the center of the PPDU BW: leakage must fall within one RBW of a 2 MHz
  channelization boundary (1 MHz boundary where 2 MHz channelization is not permitted) and
  ≤ **max(P − 27 dB, −15 dBm)**.
- Specified per antenna.

### 4.5 Transmitter constellation error (EVM) [23.3.17.4.3, Table 23-34, pp3828–3829; verified]

Relative constellation RMS error (averaged over subcarriers, PPDUs, spatial streams per
Equation (19-89)) shall not exceed, mapped onto 2 MHz/1SS MCS indices:

| MCS (2 MHz/1SS) | Modulation, rate | EVM limit (dB) |
|---|---|---|
| — (1 MHz MCS10) | BPSK 1/2, 2× rep | −4 |
| 0 | BPSK 1/2 | −5 |
| 1 | QPSK 1/2 | −10 |
| 2 | QPSK 3/4 | −13 |
| 3 | 16-QAM 1/2 | −16 |
| 4 | 16-QAM 3/4 | −19 |
| 5 | 64-QAM 2/3 | −22 |
| 6 | 64-QAM 3/4 | −25 |
| 7 | 64-QAM 5/6 | −27 |
| 8 | 256-QAM 3/4 | −30 |
| — (MCS9, invalid @2 MHz/1SS) | 256-QAM 5/6 | −32 |
| 11 | 1024-QAM 3/4 | −35 (−32 with amplitude-drift compensation disabled) |
| — (MCS12, invalid @2 MHz/1SS) | 1024-QAM 5/6 | −35 / −32 |

1024-QAM rule: ≤ −35 dB with amplitude-drift compensation enabled in the test equipment; ≤ −32 dB
with it disabled. All other constellations: the table values regardless of drift compensation
[p3829]. Test uses N_SS = N_STS (no STBC), each TX port cabled to one analyzer port [p3828].

**EVM test method** [23.3.17.4.4, pp3829–3830]: sample at ≥ signal BW; then
(a) detect PPDU start; (b) detect STF→LTF1 transition, establish fine timing; (c) estimate
coarse+fine CFO; (d) de-rotate; (e) per LTF symbol: FFT, estimate phase from pilots, de-rotate;
(f) estimate channel per subcarrier/stream; (g) per data symbol: FFT, pilot-phase de-rotation,
zero-forcing equalization with the estimated channel; (h) per data tone: distance to nearest
constellation point; (i) average per Eq (19-89). **≥ 20 PPDUs, each ≥ 16 data OFDM symbols,
random data.**

Equation (19-89) [19.3.18.7.4, p3475], as code:

```text
# N_f frames; per frame i_f: N_SYM(P) symbols, N_SS streams, N_SD data tones; P0 = average
# constellation power
err_f  = sum over is, iss, isc of ((I - I0)^2 + (Q - Q0)^2)
norm_f = N_SYM_f * N_SS * N_SD * P0
Error_RMS = (1/N_f) * sum over frames of sqrt(err_f / norm_f)
EVM_dB = 20*log10(Error_RMS)   # compare against the table limit
```

### 4.6 Time of departure accuracy [23.3.17.5, p3830]

RMS accuracy of TIME_OF_DEPARTURE ≤ 80 ns; test per Annex T with
MULTICHANNEL_SAMPLING_RATE = ((f_H − f_L)/2 MHz + 1) × 2×10^6 sample/s for CBW2;
FIRST/SECOND transition fields = STF/LTF1; the 80 ns THRESH applies for CBW16 (unspecified
otherwise). (Optional feature; low priority for SDR.)

---

## 5. S1G receiver specification [23.3.18, pp3831–3838]

All of 23.3.18.1–23.3.18.3 apply exactly to our scope: **non-STBC, 8 µs GI, BCC, S1G PPDU**
[pp3831–3832].

### 5.1 Minimum input sensitivity — 2 MHz PPDU [Table 23-35, p3831; verified against PDF]

Pass criterion: **PER < 10% at PSDU length 256 octets** at the given input level:

| MCS (2 MHz/1SS) | Modulation, rate | Min sensitivity (dBm) |
|---|---|---|
| 0 | BPSK 1/2 | −92 |
| 1 | QPSK 1/2 | −89 |
| 2 | QPSK 3/4 | −87 |
| 3 | 16-QAM 1/2 | −84 |
| 4 | 16-QAM 3/4 | −80 |
| 5 | 64-QAM 2/3 | −76 |
| 6 | 64-QAM 3/4 | −75 |
| 7 | 64-QAM 5/6 | −74 |
| 8 | 256-QAM 3/4 | −69 |
| (256-QAM 5/6, invalid) | | (−67) |
| 11 | 1024-QAM 3/4 | −64 |
| (1024-QAM "3/4" [sic], last row — presumed 5/6, invalid @2 MHz/1SS) | | (−62) |

(1 MHz column, for reference: MCS10 −98, and each halving of BW relaxes… actually each doubling
of BW tightens by… see sanity check: every column step of +1 BW doubling = +3 dB. BPSK 1/2:
1 MHz −95, 2 MHz −92, 4 MHz −89, 8 MHz −86, 16 MHz −83.)

Note (verified in the PDF): the last two rows of Table 23-35 both print "1024-QAM 3/4"; by the
progression of every other table (23-34, 23-36, MCS tables) the final row is the 5/6 rate. Both
1024-QAM rows are moot at 2 MHz/1SS except MCS 11 (3/4) = −64 dBm.

### 5.2 Adjacent / nonadjacent channel rejection [23.3.18.2/23.3.18.3, Table 23-36, pp3831–3832]

Method: desired signal at **sensitivity + 3 dB**; raise a conformant, unsynchronized S1G
interferer (≥50% duty cycle) of the same width W until PER = 10% @ 256-octet PSDU. Adjacent:
interferer center **W MHz** away (W = 2 for us). Nonadjacent: **≥ 2·W MHz** away. Rejection =
interferer power − desired power. 2/4/8/16 MHz measurement required only where the regulatory
domain permits that band plan [p3832].

2/4/8/16 MHz channel columns (identical values), mapped to 2 MHz/1SS MCS:

| MCS | Modulation, rate | ACR (dB) | Non-ACR (dB) |
|---|---|---|---|
| 0 | BPSK 1/2 | 16 | 32 |
| 1 | QPSK 1/2 | 13 | 29 |
| 2 | QPSK 3/4 | 11 | 27 |
| 3 | 16-QAM 1/2 | 8 | 24 |
| 4 | 16-QAM 3/4 | 4 | 20 |
| 5 | 64-QAM 2/3 | 0 | 16 |
| 6 | 64-QAM 3/4 | −1 | 15 |
| 7 | 64-QAM 5/6 | −2 | 14 |
| 8 | 256-QAM 3/4 | −7 | 9 |
| (256-QAM 5/6) | | (−9) | (7) |
| 11 | 1024-QAM 3/4 | −12 | 4 |
| (1024-QAM 5/6) | | (−14) | (2) |

(1 MHz-only MCS10 row: ACR 19 dB, non-ACR 35 dB.) Note Non-ACR = ACR + 16 dB on every row.

### 5.3 Receiver maximum input level [23.3.18.4, p3833]

PER ≤ 10% @ 256-octet PSDU at **−30 dBm** input per antenna, for any baseband S1G modulation.

### 5.4 CCA sensitivity [23.3.18.5, pp3833–3838]

Thresholds compare against the signal level **at each receiving antenna** [p3833].
Channels are classified **type 1** (protective, lower thresholds) or **type 2** (higher reuse,
thresholds 3 dB higher), per Annex E "CCA Level Classification"; selected via
`PHY-CONFIG.request(CCA_SENSITIVITY_TYPE = TYPE_1 | TYPE_2 | TYPE_2_WIDEBAND)` [pp3833–3834].

Timing constants [Table 23-41, p3855]: **aCCATime < 40 µs** (start-of-PPDU detection and
energy detection windows), **aCCAMidTime = 212 µs** (mid-PPDU detection window). All
"detected" conditions require **> 90% detection probability** within the stated window.

#### 5.4.1 BUSY(primary1) — signals in the primary 1 MHz channel [23.3.18.5.3.1, p3834]

| Condition | Type 1 | Type 2 | Window |
|---|---|---|---|
| Start of S1G_1M (or dup S1G_1M) PPDU in primary 1 MHz | ≥ −98 dBm | ≥ −89 dBm | aCCATime |
| Any S1G PPDU within primary 1 MHz (mid-packet) | ≥ −89 dBm | ≥ −86 dBm | aCCAMidTime |
| **Any signal** (energy detect), both types | > −75 dBm | > −75 dBm | aCCATime |

While these hold, do not issue BUSY for {primary2/secondary*} until the PPDU-indicated duration
ends or the conditions clear.

#### 5.4.2 BUSY(primary2) — 2 MHz operating width (our case) [23.3.18.5.3.1 + Tables 23-37/38, pp3834–3836]

Issued when the primary1 conditions are absent and, in an otherwise idle operating channel:

| Condition | Type 1 | Type 2 | Window |
|---|---|---|---|
| S1G_1M PPDU in the nonprimary 1 MHz half of the primary 2 MHz | ≥ −89 dBm | ≥ −86 dBm | aCCAMidTime |
| ≥2 MHz S1G PPDU within the primary 2 MHz (mid-packet) | ≥ −89 dBm | ≥ −86 dBm | aCCAMidTime |
| **Start of a 2 MHz S1G_SHORT/S1G_LONG (or duplicate) PPDU in the primary 2 MHz** [Tables 23-37/38] | ≥ **−92 dBm** | ≥ **−89 dBm** | aCCATime |
| **Any signal** (energy detect) in the primary 2 MHz, both types | > **−72 dBm** | > **−72 dBm** | aCCATime |

(Tables 23-37/38 rows for wider operating widths, noted only: start of 4 MHz PPDU −89/−86 dBm;
8 MHz −86/−83; 16 MHz −83/−80. Table 23-39, for type 2 with intended 8/16 MHz wideband access
per 10.23.2.6: primary1 uses −86 dBm everywhere, and the 2/4/8/16 MHz start-detect levels
become −86/−83/−80/−77 dBm [23.3.18.5.3.2, pp3835–3836].)

Partial AID/COLOR match rule: on detecting an S1G_SHORT/S1G_LONG PPDU whose SIG Partial AID /
COLOR matches the STA's partial AID or BSSID, issue BUSY(primary2) for the remaining PPDU
duration from its preamble [pp3835–3836].

#### 5.4.3 Secondary channels (≥4 MHz operating widths; out of scope, headline numbers) [23.3.18.5.4, pp3836–3837]

secondary2: ED −72 dBm (aCCATime, both types; sticky while exceeded); 2 MHz S1G PPDU −86 (t1) /
−82 (t2) dBm within aCCAMidTime. secondary4: ED −69 dBm; 4 MHz or any 2 MHz sub −86/−82 dBm.
secondary8: ED −66 dBm; 8 MHz −83 (t1)/−79 (t2); 4 or 2 MHz subchannels −86/−82 dBm.

### 5.5 RSSI and RCPI [23.3.18.6/7, p3838]

- RSSI: computed during the (D-)LTFs; monotonically increasing function of RX power (relative
  measure, 0–255 in RXVECTOR).
- RCPI: RF power in the channel over STF or LTF fields, dBm, ±5 dB (95% CI), averaged over
  active receive chains; 1 MHz MCS10 STF-based measurement reports measured − 3 dB.

---

## 6. PHY characteristics (timing deadlines) [23.4.4, Table 23-41, p3855; verified]

| Characteristic | Value |
|---|---|
| aSlotTime | 52 µs [also 23.3.15, p3823] |
| aSIFSTime | 160 µs |
| aCCATime | < 40 µs |
| aRxPHYStartDelay | 600 µs (S1G_1M); **280 µs (S1G_SHORT and S1G_LONG)** |
| aRxTxTurnaroundTime, aTxPHYDelay, aRxPHYDelay, aRxTxSwitchTime, aTxRampOnTime, aMACProcessingDelay | implementation dependent (10.3.7) |
| aAirPropagationTime | 6 µs |
| aCCAMidTime | 212 µs |
| aPPDUMaxTime | 27 920 µs (S1G_1M MCS10, 511-octet PSDU) |
| aPSDUMaxLengthWithNoAggregation | 511 octets (SIG Length field limit, Aggregation=0) |
| aPSDUMaxLength | 797 159 octets (16 MHz, MCS9, 4 SS, 511 symbols) |

Implications for the PHY API: `PHY-RXSTART.indication` must be delivered within
aRxPHYStartDelay = 280 µs of PPDU start for S1G_SHORT (= 240 µs of STF+LTF1+SIG on air + 40 µs
decode allowance); CCA BUSY must assert within < 40 µs of a detectable PPDU start / energy;
mid-packet detection gets 212 µs.

---

## 7. Test-assertion cheat sheet (2 MHz / 1 SS / LGI / BCC / S1G_SHORT / SU)

TX:
- Spectrum: 0 dBr ≤ ±0.9 MHz; −20 dBr @ ±1.1; −28 dBr @ ±2.0; −40 dBr @ ≥ ±3.0; dB-linear
  interpolation; floor −43 dBm/MHz; RBW 10 kHz / VBW 100 Hz.
- Flatness (BPSK PPDUs): tones ±1..±16 within ±4 dB of linear mean over ±1..±16;
  tones ±17..±28 within +4/−6 dB of the same mean.
- CFO and symbol clock: ±20 ppm, common reference.
- LO leakage: center ⇒ ≤ P − 17.5 dB (31.25 kHz RBW); off-center ⇒ ≤ max(P−27, −15 dBm).
- EVM (dB): MCS0..8 = −5, −10, −13, −16, −19, −22, −25, −27, −30; MCS11 = −35 (−32 w/o drift
  comp). ≥20 PPDUs, ≥16 data symbols, random data, ZF equalizer, pilot phase tracking.
- TXTIME = 240 + 40·N_SYM µs; N_SYM = ceil((8·LENGTH + 14)/N_DBPS) (non-aggregated).

RX:
- Sensitivity (dBm, PER<10% @256 B): MCS0..8 = −92, −89, −87, −84, −80, −76, −75, −74, −69;
  MCS11 = −64. Max input −30 dBm.
- ACR (desired @ sens+3 dB, 2 MHz offset): 16, 13, 11, 8, 4, 0, −1, −2, −7 (MCS0..8), −12
  (MCS11); non-ACR (≥4 MHz offset) = ACR + 16.
- CCA @2 MHz operating width: preamble start-detect BUSY within <40 µs at −92 dBm (type 1) /
  −89 dBm (type 2); ED BUSY within <40 µs above −72 dBm; mid-packet S1G detect within 212 µs at
  −89/−86 dBm; hold BUSY for RXTIME after valid SIG (even unsupported/reserved); CRC-fail
  release when level < −72 dBm.
- RXSTART deadline 280 µs from PPDU start.

---

## 8. SANITY CHECKS (numeric identities verified)

1. N_CBPS = N_SD·N_BPSCS: 52·1=52, 52·2=104, 52·4=208, 52·6=312, 52·8=416, 52·10=520 ✓
   (matches Table 23-46). N_DBPS = N_CBPS·R for every row (e.g. 312·5/6 = 260) ✓.
2. N_ST = N_SD + N_SP = 52 + 4 = 56 ✓ (Table 23-5 NOTE). N_SR = 28 = highest tone; occupied BW
   = 2·28·31.25 kHz = 1.75 MHz < 1.8 MHz 0-dBr mask bandwidth ✓.
3. T_SYML = T_DFT + T_GI = 32 + 8 = 40 µs; 64 samples · (1/2 MHz) = 32 µs = 1/ΔF ✓.
4. Data rate = N_DBPS/T_SYML: MCS0 26/40 µs = 650 kb/s; MCS8 312/40 µs = 7.8 Mb/s; MCS11
   390/40 µs = 9.75 Mb/s — all equal Table 23-46 ✓.
5. S1G_SHORT preamble on air = T_STF + T_LTF1 + T_SIG = 80+80+80 = 240 µs; aRxPHYStartDelay
   280 µs = 240 + 40 (one symbol of decode latency); S1G_1M: 160+160+240 = 560 + 40 = 600 µs ✓
   (both rows of Table 23-41 reproduce exactly).
6. aPPDUMaxTime: S1G_1M MCS10 (N_DBPS=6), 511 octets ⇒ N_SYM = ceil((8·511+14)/6) = 684;
   TXTIME = 560 + 684·40 = 27 920 µs ✓ (Table 23-41 NOTE 1).
7. Sensitivity scales +3 dB per BW doubling in every Table 23-35 row (e.g. BPSK1/2:
   −95/−92/−89/−86/−83) ✓; 2 MHz MCS0 sensitivity (−92 dBm) equals the type-1 2 MHz CCA
   preamble-detect level (−92 dBm, Table 23-37) ✓.
8. Non-adjacent rejection = adjacent rejection + 16 dB on every Table 23-36 row ✓.
9. CRC-fail CCA release threshold (min-MCS sensitivity + 20 dB) = −92 + 20 = −72 dBm = the
   2 MHz energy-detect threshold ✓ (consistent design).
10. Flatness index sets: |{−16..−1,+1..+16}| = 32, |{−28..−17,+17..+28}| = 24; 32+24 = 56 = N_ST ✓.
11. aCCAMidTime (212 µs) < 240 µs preamble+SIG duration, so mid-packet detection window closes
    before SIG decode completes on a freshly-started PPDU — the two mechanisms are disjoint ✓
    (aCCATime < 40 µs < aSlotTime 52 µs ✓).
12. Type-2 thresholds sit 3 dB above type-1 for every paired CCA condition (−92→−89, −89→−86,
    −98→−89 excepted: that pair is 9 dB, as printed) ✓ against PDF pp3834–3835.
13. EVM ladder decreases monotonically with MCS order and matches the HT ladder (Table 19-22:
    −5…−27) for the shared constellations ✓.
