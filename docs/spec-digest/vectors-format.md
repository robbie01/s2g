# S1G PHY Digest — Service Interface, PPDU Formats, TXTIME/Padding Math

**Scope of this digest**: IEEE 802.11-2024 Clause 23 (S1G / 802.11ah PHY), specialized to
**2 MHz bandwidth, 1 spatial stream (SISO), BCC coding, Long GI (8 µs), S1G_SHORT preamble, SU PPDU, no STBC, no MU, no beamforming**.
Out-of-scope variants (1/4/8/16 MHz, LDPC, short GI, S1G_LONG/S1G_1M, MU, STBC, NDP-CMAC) are noted where they exist but not expanded.

Citations are `[subclause, pPDFpage]` where the page is the PDF page number (spec-printed page is PDF page − 1). All numeric tables and equations below were verified visually against the PDF (pages rendered as images), not just the text extraction.

---

## 1. Timing constants (Table 23-5, Table 23-6)

### 1.1 Data field / general constants — 2 MHz column of Table 23-5 [23.3.6, p3757–3758]

| Symbol | Value (CBW2) | Meaning |
|---|---|---|
| `N_SD` | **52** | data subcarriers per OFDM symbol (Data field) |
| `N_SP` | **4** | pilot subcarriers per OFDM symbol (Data field) |
| `N_ST` | **56** | total useful subcarriers = N_SD + N_SP |
| `N_SR` | **28** | highest occupied subcarrier index (Data field occupies k = −28..−1, +1..+28) |
| `ΔF` | **31.25 kHz** | subcarrier spacing (all S1G BWs) |
| `T_DFT` | **32 µs** = 1/ΔF | IDFT/DFT period (64-point FFT at 2 MHz sample rate) |
| `T_GI` | **8 µs** = T_DFT/4 | long guard interval |
| `T_GI2` | **16 µs** | double guard interval (used in LTF1) |
| `T_GIS` | **4 µs** = T_DFT/8 | short GI (out of scope) |
| `T_SYML` | **40 µs** = T_DFT + T_GI | OFDM symbol, long GI |
| `T_SYMS` | **36 µs** = T_DFT + T_GIS | OFDM symbol, short GI (out of scope) |
| `T_SYM` | T_SYML or T_SYMS per GI used | |
| `T_STF` | **80 µs** = 2 × T_SYML | STF field duration (≥2 MHz) |
| `T_LTF1` | **80 µs** = 2 × T_DFT + T_GI2 | first LTF field duration (≥2 MHz): GI2(16) + LTS(32) + LTS(32) |
| `T_LTF` | **40 µs** = T_SYML | each of LTF2..LTF_NLTF |
| `T_SIG` | **80 µs** = 2 × T_SYML | SIG field duration (≥2 MHz, S1G_SHORT) |
| `N_service` | **8** | bits in SERVICE field |
| `N_tail` | **6** | tail bits per BCC encoder |

(1 MHz column differs — T_STF=160 µs, T_LTF1=160 µs, T_SIG=240 µs, N_SD=24 etc.; S1G_LONG-only rows T_DSTF/T_DLTF/T_SIG-A/T_SIG-B = 40/40/80/40 µs — out of scope.) [Table 23-5, p3757–3758]

### 1.2 SIG-field-specific constants (Table 23-6) — SIG of S1G_SHORT at ≥2 MHz [23.3.6, p3759]

The SIG field does **not** use the Data-field tone plan. Per 2 MHz subchannel it reuses the classic 11a 64-FFT layout:

| Symbol | Value | Meaning |
|---|---|---|
| `N_SD` (SIG) | **48** | data subcarriers per SIG symbol per 2 MHz subchannel |
| `N_SP` (SIG) | **4** | pilots per SIG symbol per 2 MHz subchannel |
| `N_ST` (SIG) | **52** | total useful subcarriers per 2 MHz subchannel |
| `N_SR` (SIG) | **26** (CBW2) | highest subcarrier index → SIG occupies k = −26..−1, +1..+26 |
| `T_SYM` (SIG) | **40 µs** (long GI always; short/double GI never used in SIG) |
| `T_SIG` | **80 µs** = 2 symbols |

So: **Data symbols use tones ±1..±28 (56 tones), SIG symbols use tones ±1..±26 (52 tones)**; DC (k=0) is always null. [Table 23-6, p3759; 23.3.7, p3761 for the −28..+28 statement]

---

## 2. TXVECTOR / RXVECTOR parameters (Table 23-1) [23.2.2, p3733–3740]

Legend: **TX/RX** = present in TXVECTOR / RXVECTOR (Y present, N not, O optional, MU per-user array). Rows below are restricted to what exists for the baseline configuration (FORMAT=S1G, CH_BANDWIDTH=CBW2, PREAMBLE_TYPE=S1G_SHORT_PREAMBLE, SU); parameters that are "Not present" under these conditions are listed at the end.

| Parameter | TX | RX | Allowed values / semantics (baseline config) | Lands in SIG bits? |
|---|---|---|---|---|
| `FORMAT` | Y | Y | Enum: `S1G` (ours), `S1G_DUP_1M`, `S1G_DUP_2M` [p3733] | No (implied by preamble structure) |
| `PREAMBLE_TYPE` | Y | Y | For FORMAT=S1G & CBW2: `S1G_SHORT_PREAMBLE` (23.3.8.2.2) or `S1G_LONG_PREAMBLE` (23.3.8.2.3). Ours: SHORT. [p3733] | No (RX detects via QBPSK rotation of SIG symbols, see §5.3) |
| `NDP_INDICATION` | Y | Y | 1 = PPDU is an NDP CMAC PPDU (23.3.12); **0 for normal data PPDUs (ours)** [p3733] | Yes — SIG-2 B13 |
| `SMOOTHING` | Y | Y | 1 = frequency-domain smoothing recommended for channel estimation, else 0. Present for S1G_SHORT (the "Otherwise" branch of the table). [p3734] | Yes — SIG-1 B23 |
| `AGGREGATION` | Y | Y | Enum `AGGREGATED` (PSDU is an A-MPDU) / `NOT_AGGREGATED`. NOTE: PSDUs > 511 octets **must** be sent AGGREGATED (Length field is 9 bits). [p3734; Table 23-12 NOTE, p3774] | Yes — SIG-2 B0 |
| `N_TX` | Y | N | Number of transmit chains. Ours: 1. [p3735] | No — processing only |
| `RCPI` | N | Y | RX power measure over STF/LTF (23.3.18.7) [p3735] | RX-only measurement |
| `SNR` | N | Y | Per-spatial-stream SNR array, 8-bit [p3735] | RX-only measurement |
| `FEC_CODING` | Y | Y | Enum `BCC_CODING` (ours) / `LDPC_CODING` [p3735] | Yes — SIG-1 B17 (Coding) |
| `STBC` | Y | Y | 0 = no STBC, N_STS=N_SS (**ours: 0**); 1 = STBC, N_STS=2·N_SS [p3736] | Yes — SIG-1 B1 |
| `GI_TYPE` | Y | Y | Enum `LONG_GI` (ours: all Data symbols 8 µs GI) / `SHORT_GI` (first Data symbol 8 µs, rest 4 µs) [p3736; 23.3.4.6.1 p, p3754] | Yes — SIG-1 B16 (Short GI) |
| `TXPWR_LEVEL` | Y | N | 1..N/2 index into dot11TxPowerLevelExtended [p3736] | No — analog only |
| `RSSI` | N | Y | 0–255, monotonic in RX power, measured during LTF [p3736] | RX-only |
| `MCS` | Y(MU/SU) | Y | For S1G ≥2 MHz: integer **0–9, 11, 12**; at 2 MHz/1SS the *valid* subset is **0–8 and 11** (9, 10, 12 "Not valid", see §7) [p3736; Table 23-46] | Yes — SIG-1 B19–B22 (4 bits) |
| `REC_MCS` | N | O | MCS the receiver recommends (same range) [p3736] | No — carried by MAC feedback |
| `CH_BANDWIDTH` | Y | Y | Enum CBW1/CBW2/CBW4/CBW8/CBW16 for FORMAT=S1G. Ours: **CBW2** [p3737] | Yes — SIG-1 B3–B4 (BW) |
| `LENGTH` | Y | Y | **If AGGREGATION=AGGREGATED: PPDU duration in number of SYMBOLS in the PSDU. If NOT_AGGREGATED: number of OCTETS in the PSDU.** (This is the value in the SIG Length field.) [p3737, verified against PDF] | Yes — SIG-2 B1–B9 (9 bits) |
| `APEP_LENGTH` | Y(MU/SU) | O | Present only when AGGREGATED. 0 = NDP; >0 = octets in the A-MPDU pre-EOF padding (10.12.2); drives N_SYM via Eq (23-79) [p3737] | No — only via N_SYM in Length |
| `PSDU_LENGTH` | Y | Y | TXVECTOR (SU): >0 & NOT_AGGREGATED → number of octets in PSDU; 0 → NDP. RXVECTOR: value computed per 23.4.3 (Eq 23-81). [p3738, verified against PDF] | Indirectly — equals Length when Aggregation=0 |
| `NUM_STS` | Y(MU/SU) | Y | 1–4 per user in TX (0–4 in RX). Ours: **1** [p3738] | Yes — SIG-1 B5–B6 (Nsts = NUM_STS−1) |
| `PARTIAL_AID` | Y | Y | Present for S1G+CBW≥2+SU. Range **0–511 if UPLINK_INDICATION=1**, **0–63 if UPLINK_INDICATION=0** [p3738] | Yes — inside SIG-1 B7–B15 (ID) |
| `NUM_USERS` | Y | N | Ours (SU): **1** [p3739] | No |
| `RESPONSE_INDICATION` | Y | Y | **0 = No Response, 1 = NDP Response, 2 = Normal Response, 3 = Long Response** (type of PPDU expected SIFS after this one, see 10.3.2.5) [p3739] | Yes — SIG-2 B10–B11 |
| `TRAVELING_PILOTS` | Y | O | 1 = traveling pilots used in the PPDU, 0 = regular pilot locations [p3739] | Yes — SIG-2 B12 |
| `TIME_OF_DEPARTURE_REQUESTED` | O | N | Boolean; ToD measurement request [p3739] | No |
| `RX_START_OF_FRAME_OFFSET` | N | Y | 0..2^32−1, 10 ns units, preamble-arrival→primitive-issue offset (only if dot11TimingMsmtActivated) [p3739] | RX-only |
| `UPLINK_INDICATION` | Y | Y | Present when NDP_INDICATION=0, FORMAT=S1G, CBW≠1. **1 = PPDU addressed to an AP, 0 otherwise** (10.21) [p3740] | Yes — SIG-1 B2 |
| `COLOR` | Y | Y | Present when UPLINK_INDICATION=0, NDP_INDICATION=0, SU, CBW≠1. **0–7**, BSS color (10.21) [p3740] | Yes — inside SIG-1 B7–B9 when Uplink Indication=0 |
| `SCRAMBLER_OR_CRC` | N | Y | RX-only. For NDP_INDICATION=0: the 7-bit scrambler initialization [B0:B6] of the SERVICE field recovered prior to descrambling. (For NDP: the 4-bit SIG CRC.) [p3740] | It *is* the descrambled seed; TX seed goes in SERVICE field, not SIG |

**Not present in the baseline configuration** (conditions unmet): `MU_SU` (long preamble only), `NDP_CMAC_PPDU_BODY` (NDP only), `SECTOR_ID`, `EXPANSION_MAT`, `CHAN_MAT`, `DELTA_SNR` (long-preamble/sounding), `USER_POSITION`, `GROUP_ID` (MU only), `BEAM_CHANGE` (long preamble only). [Table 23-1, p3733–3740]

**Scrambler initialization**: There is no TXVECTOR scrambler parameter. The SERVICE field (Table 23-20) is 8 bits: B0–B6 "Scrambler Initialization" **set to 0** (i.e., transmitted as zeros; the actual pseudorandom nonzero initial state of the self-synchronizing scrambler of 17.3.5.5 is recovered by the receiver from these 7 bits after descrambling with state 0), B7 reserved (0). Bit 0 transmitted first. [23.3.9.2, p3794; 23.3.9.3 → 17.3.5.5]

---

## 3. Effect of CH_BANDWIDTH (Table 23-2) [23.2.3, p3741]

For FORMAT=S1G, CH_BANDWIDTH=CBW2: *"The STA transmits an S1G PPDU of 2 MHz bandwidth. If the operating channel width is wider than 2 MHz, then the transmission shall use the primary 2 MHz channel."* Rows exist for CBW1/4/8/16 (analogous, on primary subchannels) and for the duplicate formats S1G_DUP_2M / S1G_DUP_1M with per-subchannel phase-rotation patterns (Eq 23-7..23-13) — out of scope. For a standalone 2 MHz-only device, CBW2 simply means: the whole 2 MHz channel, no rotation needed at 2 MHz (single subchannel). [Table 23-2, p3741]

---

## 4. S1G PPDU formats [23.3.2, p3743–3745]

Three formats: **S1G_SHORT**, **S1G_LONG**, **S1G_1M** [p3743].
S1G_SHORT is used for **SU transmission at 2/4/8/16 MHz**: the baseline case.

### 4.1 S1G_SHORT layout (Figure 23-1) [p3743, verified against PDF]

```
| STF | LTF1        | SIG | LTF2 … LTF_NLTF | Data          |
  2sym  2sym          2sym  1 sym per LTF     N_SYM symbols
        └ GI2|LTS|LTS ┘
```

Per-field durations at 2 MHz, long GI, N_STS = 1 (⇒ N_LTF = 1, so **no LTF2..LTF_NLTF fields at all**):

| Field | Duration | Structure |
|---|---|---|
| STF | 80 µs | 2 × 40 µs symbols (each: 8 µs GI + 32 µs) |
| LTF1 | 80 µs | 16 µs GI2 + 32 µs LTS + 32 µs LTS |
| SIG | 80 µs | 2 × 40 µs symbols (SIG-1 then SIG-2), QBPSK, long GI always |
| LTF2..LTF_NLTF | 0 µs (N_LTF=1) | would be 40 µs each for N_STS>1 |
| Data | N_SYM × 40 µs | long GI |

Total preamble+SIG for the baseline config = **240 µs**.

`N_LTF` vs `N_STS` (Table 23-11) [23.3.8.2.2.4, p3762]:

| N_STS | 1 | 2 | 3 | 4 |
|---|---|---|---|---|
| N_LTF | **1** | 2 | 4 | 4 |

In an S1G NDP the Data field is absent. [23.3.2, p3745]

(S1G_LONG: STF, LTF1, SIG-A ×2, then beam-changeable D-STF, D-LTF×N_LTF, SIG-B, Data — out of scope. S1G_1M: 4-sym STF, 4-sym LTF1, 6-sym SIG — out of scope.) [Figures 23-2/23-3, p3744]

---

## 5. Transmitter processing [23.3.3, p3745–3746; 23.3.4.3, p3749–3751; 23.3.4.6.1, p3753–3754]

### 5.1 Block list (subset used per field) [23.3.3, p3745]

PHY padding → Scrambler → BCC encoder parser → FEC (BCC) encoder → Stream parser → (Segment parser: 16 MHz only, bypassed) → BCC interleaver → Constellation mapper → Pilot insertion → (STBC: bypassed) → CSD per STS → Spatial mapper (Q) → IDFT → GI insertion → windowing. **N_ES = 1 always for BCC in S1G** (single encoder). [p3745–3746]

The general flow equals the corresponding Clause 21 (VHT) figures with "20 MHz"→"2 MHz": Figure 21-10 is the BCC Data-field chain for 2 MHz SU. [p3745–3746]

### 5.2 Data field construction, BCC, SU (23.3.4.6.1) [p3753–3754]

1. Build SERVICE field (8 bits, §2 note) and append PSDU [23.3.9.2].
2. **PHY padding** (23.3.9.4.3.2, see §8.3): append N_PAD pad bits (value arbitrary 0/1).
3. **Scramble** SERVICE+PSDU+pad (17.3.5.5 scrambler).
4. **BCC encode** (single encoder), replacing the last 6 scrambled bits' positions: 6 zero tail bits are appended *after* scrambling, then encoded; rate per MCS via puncturing (21.3.10.5.3 by reference) [23.3.9.4.3.1, p3795].
5. Stream parser (identity for 1 SS), segment parser bypassed (<16 MHz).
6. **BCC interleave** (23.3.9.8).
7. **Constellation map** BPSK/QPSK/16/64/256/1024-QAM (23.3.9.9).
8. STBC bypassed; **pilot insertion** (23.3.9.10); CSD per STS (identity for 1 STS, cyclic shift 0); spatial mapping Q (identity, 1×1); phase rotation (none needed at plain 2 MHz); IDFT;
9. **GI**: LONG_GI ⇒ every Data symbol gets 8 µs GI. (SHORT_GI would be 8 µs on the first Data symbol, 4 µs on the rest — out of scope.) Windowing per 17.3.2.5.
10. Upconvert to channel center frequency. [p3754]

### 5.3 SIG field construction (23.3.4.3.3) [p3750–3751]

1. Assemble the 48 uncoded SIG bits from TXVECTOR (layout in §6): 24 bits SIG-1 + 24 bits SIG-2, including reserved bits, computed 4-bit CRC, and 6 tail bits (zero).
2. BCC encode at R=1/2 (17.3.5.6) → 96 coded bits. No puncturing.
3. Interleave per 17.3.5.7 (the 48-bit-per-symbol 11a interleaver, N_CBPS=48, N_BPSC=1), each 48-bit symbol block independently.
4. **Constellation: QBPSK for BOTH symbols** — all 96 BPSK points rotated 90° counter-clockwise (bit b → j·(2b−1)); first 48 → SIG-1 symbol, second 48 → SIG-2 symbol. This rotation is what lets a receiver distinguish S1G_SHORT from S1G_1M and S1G_LONG. [23.3.8.2.2.5, p3772 Fig 23-9 verified: both SIG-1 and SIG-2 constellations lie on the Q axis; NOTE p3775]
5. Pilot insertion per 17.3.5.10 (4 pilots at k = ±7, ±21, 11a polarity).
6. Map by first column of P_HTLTF (scalar +1 for 1 STS); CSD (0); Q (identity); IDFT; **8 µs GI** per symbol; window; upconvert. [p3750–3751]

STF/LTF1 construction highlights (sequences belong to another digest): STF symbols get 8 µs GI each; LTF1 gets one 16 µs GI2 before its two 32 µs LTS periods. [23.3.4.3.1–23.3.4.3.2, p3749–3750]

---

## 6. SIG field bit layout (S1G_SHORT, ≥2 MHz) — Table 23-12, Figures 23-7/23-8 [23.3.8.2.2.5, p3772–3774, verified against PDF]

48 uncoded bits total; SIG-1 (24 bits, B0 first into the BCC encoder / first in time) then SIG-2 (24 bits). SIG-1 is transmitted before SIG-2.

### SIG-1

| Bits | Field | Width | Value (baseline config in **bold**) |
|---|---|---|---|
| B0 | Reserved | 1 | **1** on transmit (see 23.3.20 on receive) |
| B1 | STBC | 1 | **0** (no STBC) |
| B2 | Uplink Indication | 1 | 1 if PPDU addressed to an AP, else 0 (= TXVECTOR UPLINK_INDICATION) |
| B3–B4 | BW | 2 | **0 = CBW2**, 1 = CBW4, 2 = CBW8, 3 = CBW16 (0 also for S1G_DUP_2M) |
| B5–B6 | Nsts | 2 | **0 = 1 STS**, 1 = 2, 2 = 3, 3 = 4 |
| B7–B15 | ID | 9 | If Uplink Indication = 1 (or not present): B7–B15 = PARTIAL_AID (0–511). If Uplink Indication = 0: **B7–B9 = COLOR (BSS identifier, 0–7), B10–B15 = PARTIAL_AID (0–63)** |
| B16 | Short GI | 1 | **0** (long GI) |
| B17 | Coding | 1 | **0 = BCC**, 1 = LDPC |
| B18 | LDPC Extra | 1 | If Coding=1: LDPC extra-symbol flag (21.3.10.5.4). **If Coding=0 this field is set to 1.** |
| B19–B22 | MCS | 4 | MCS index (0–8, 11 valid at 2 MHz/1SS) |
| B23 | Smoothing | 1 | 1 = channel smoothing recommended |

### SIG-2

| Bits | Field | Width | Value |
|---|---|---|---|
| B0 | Aggregation | 1 | 1 = PSDU is A-MPDU. (Must be 1 whenever PSDU > 511 octets, per 10.12.5.) |
| B1–B9 | Length | 9 | **Aggregation=0: number of octets in the PSDU (= PSDU_LENGTH, 1..511). Aggregation=1: N_SYM (number of Data OFDM symbols, from 23.4.3, ≤511).** |
| B10–B11 | Response Indication | 2 | 0 No / 1 NDP / 2 Normal / 3 Long response expected SIFS after this PPDU |
| B12 | Traveling Pilots | 1 | 1 = traveling pilots, 0 = regular pilot locations |
| B13 | NDP Indication | 1 | **0** (not an NDP CMAC PPDU) |
| B14–B17 | CRC | 4 | see below |
| B18–B23 | Tail | 6 | **0** (BCC trellis termination) |

### CRC-4 over SIG bits [23.3.8.2.2.6, p3775, verified against PDF]

Protects bits 0–37 of the ≥2 MHz SIG (i.e., SIG-1 B0..B23 followed by SIG-2 B0..B13, 38 bits; N=37):

```
crc(D) = ones_complement( (M(D) ⊕ I(D)) · D^4 mod G(D) )
G(D) = D^4 + D + 1
M(D) = m0·D^N + m1·D^(N-1) + ... + mN·D^0     (m0 = SIG-1 B0)
I(D) = D^N + D^(N-1) + D^(N-2) + D^(N-3)       (XOR 1 into the first 4 message bits)
crc(D) = c3·D^3 + c2·D^2 + c1·D + c0 ; the CRC field is transmitted c3 first (SIG-2 B14 = c3).
```

Pseudocode (shift-register form): init 4-bit register to `1111`; for each of the 38 bits (SIG-1 B0 first): `fb = bit XOR reg[3]; reg = (reg << 1) & 0xF; reg[0] = fb; if fb: reg ^= 0b0010` (the D term); after all bits, CRC = ones-complement of reg; output c3=reg[3] first.

---

## 7. MCS table — 2 MHz, N_SS = 1 (Table 23-46) [23.5, p3858–3859, verified against PDF]

N_ES = 1 for all rows. N_SD = 52, N_SP = 4.

| MCS | Modulation | R | N_BPSCS | N_CBPS | N_DBPS | Rate @ 8 µs GI (kb/s) | Rate @ 4 µs GI |
|---|---|---|---|---|---|---|---|
| 0 | BPSK | 1/2 | 1 | 52 | 26 | 650.0 | 722.2 |
| 1 | QPSK | 1/2 | 2 | 104 | 52 | 1300.0 | 1444.4 |
| 2 | QPSK | 3/4 | 2 | 104 | 78 | 1950.0 | 2166.7 |
| 3 | 16-QAM | 1/2 | 4 | 208 | 104 | 2600.0 | 2888.9 |
| 4 | 16-QAM | 3/4 | 4 | 208 | 156 | 3900.0 | 4333.3 |
| 5 | 64-QAM | 2/3 | 6 | 312 | 208 | 5200.0 | 5777.8 |
| 6 | 64-QAM | 3/4 | 6 | 312 | 234 | 5850.0 | 6500.0 |
| 7 | 64-QAM | 5/6 | 6 | 312 | 260 | 6500.0 | 7222.2 |
| 8 | 256-QAM | 3/4 | 8 | 416 | 312 | 7800.0 | 8666.7 |
| 9 | **Not valid** | | | | | | |
| 10 | **Not valid** (MCS 10 exists only at 1 MHz/1SS: BPSK 1/2 with 2× repetition) | | | | | | |
| 11 | 1024-QAM | 3/4 | 10 | 520 | 390 | 9750 | 10833.3 |
| 12 | **Not valid** at 2 MHz/1SS (valid at 1 MHz) | | | | | | |

Support rules [23.5, p3856–3857]: MCS 0–7 (1SS) mandatory for an AP on every supported width; a non-AP STA shall support at least MCS 0–2 (1SS) at 1 & 2 MHz; MCS 8/9 optional, MCS 11/12 optional, 4 µs GI optional. 1 & 2 MHz with 1SS mandatory; everything else optional.

---

## 8. TXTIME, N_SYM, PSDU_LENGTH, and padding [23.4.3, p3853–3855, all equations verified against PDF]

### 8.1 TXTIME — S1G_SHORT

Long GI, Equation (23-74):

```
TXTIME = T_PREAMBLE + T_SIG + T_LTF·(N_LTF − 1) + T_SYML·N_SYM
T_PREAMBLE = T_STF + T_LTF1
```

(Short GI, Eq (23-73), for reference: same for N_SYM=0; for N_SYM>0 the Data term is `T_SYML + T_SYMS·(N_SYM−1)` — first Data symbol keeps the 8 µs GI.)

**Specialized (2 MHz, 1 SS ⇒ N_LTF = 1, long GI):**

```
TXTIME [µs] = 80 + 80 + 80 + 40·N_SYM  =  240 + 40·N_SYM
```

For an NDP: N_SYM = 0 ⇒ TXTIME = 240 µs. [p3853–3854]

### 8.2 N_SYM — SU PPDU, BCC (Equations 23-79 / 23-80)

If the SIG Aggregation subfield = 1 (Eq 23-79):

```
N_SYM = m_STBC * ceil( (8*APEP_LENGTH + N_service + N_tail*N_ES) / (m_STBC * N_DBPS) )
```

Otherwise (Aggregation = 0, Eq 23-80):

```
N_SYM = m_STBC * ceil( (8*PSDU_LENGTH + N_service + N_tail*N_ES) / (m_STBC * N_DBPS) )
```

where `m_STBC = 2` when STBC is used, **1 otherwise (ours)**; `N_ES = 1`; `N_service = 8`; `N_tail = 6`.

**Specialized (no STBC, BCC, 2 MHz/1SS):**

```
N_SYM = ceil( (8*LEN + 14) / N_DBPS )        // LEN = PSDU_LENGTH or APEP_LENGTH octets
```

(LDPC N_SYM is per 23.3.9.4.4 — out of scope.) [p3854]

### 8.3 PHY padding (BCC) [23.3.9.4.3.2, p3795]

```
N_PAD = N_SYM * N_DBPS − 8*PSDU_LENGTH − N_service − N_tail*N_ES
```

- **Aggregation = 0 (ours, typical)**: MAC does no padding; PHY appends N_PAD pad bits (each may be 0 or 1) after the PSDU. SIG Length = PSDU_LENGTH in octets.
- **Aggregation = 1**: MAC first fills the PSDU to the maximum whole-octet capacity (PSDU_LENGTH from Eq 23-81 below, using A-MPDU EOF padding per 10.12.2/10.12.5); PHY then appends only `N_PAD mod 8` bits (< 8). SIG Length = N_SYM.
- In both cases: scramble (SERVICE + PSDU + pad), then append the N_tail·N_ES = 6 zero tail bits *after* scrambling (tail bits are not scrambled).

### 8.4 PSDU_LENGTH returned by PLME-TXTIME.confirm / derived at RX (Equation 23-81, BCC SU)

```
PSDU_LENGTH = floor( (N_SYM * N_DBPS − N_service − N_tail*N_ES) / 8 )
            = floor( (N_SYM * N_DBPS − 14) / 8 )                       // specialized
```

For an NDP, PSDU_LENGTH = 0. (LDPC variant Eq 23-82 uses N_SYM,init and no tail term.) [p3855]

### 8.5 Receiver derivation of PSDU length and N_SYM from SIG (putting it together)

Given decoded SIG fields (MCS → N_DBPS from §7; Short GI bit → T_SYM; Nsts → N_LTF; STBC → m_STBC):

- **Aggregation = 0**: `PSDU_LENGTH = Length` (octets, 1–511; 0 ⇒ NDP). `N_SYM = m_STBC*ceil((8*Length + 14)/(m_STBC*N_DBPS))` (Eq 23-80). Receive exactly N_SYM Data symbols; after descrambling, deliver the first PSDU_LENGTH octets after the SERVICE field.
- **Aggregation = 1**: `N_SYM = Length` (symbols, ≤511). `PSDU_LENGTH = floor((N_SYM*N_DBPS − 14)/8)` (Eq 23-81); the A-MPDU EOF/pad subframes delimit real MPDUs inside.
- Data-field end time = TXTIME per §8.1; the PHY-RXEND timing and Length consistency are what bound aPPDUMaxTime.

---

## 9. PHY characteristics (Table 23-41) [23.4.4, p3856, verified against PDF]

| Characteristic | Value |
|---|---|
| `aSlotTime` | **52 µs** (also normative in 23.3.15, p3824: "The slot time for the S1G PHY shall be 52 µs.") |
| `aSIFSTime` | **160 µs** |
| `aCCATime` | **< 40 µs** |
| `aRxPHYStartDelay` | **280 µs** for S1G_SHORT (and S1G_LONG); 600 µs for S1G_1M |
| `aRxTxTurnaroundTime` | implementation dependent (10.3.7) |
| `aTxPHYDelay`, `aRxPHYDelay`, `aRxTxSwitchTime`, `aTxRampOnTime`, `aMACProcessingDelay` | implementation dependent (10.3.7) |
| `aAirPropagationTime` | **6 µs** |
| `aCCAMidTime` | **212 µs** |
| `aPPDUMaxTime` | **27 920 µs** (bound set by 1 MHz MCS10 1SS with 511-octet PSDU) |
| `aPSDUMaxLengthWithNoAggregation` | **511 octets** (9-bit Length field, Aggregation=0) |
| `aPSDUMaxLength` | **797 159 octets** (16 MHz MCS9 4SS bound: 511 data symbols) |

Derived for the MAC: SlotTime = aCCATime + aRxTxTurnaroundTime + aAirPropagationTime + aMACProcessingDelay budget = 52 µs; SIFS = 160 µs; typical PIFS = SIFS + slot = 212 µs (= aCCAMidTime), DIFS = SIFS + 2·slot = 264 µs.

Max PSDU at 2 MHz/1SS with Aggregation=1: N_SYM ≤ 511 ⇒ e.g. MCS 11: floor((511·390−14)/8) = 24 909 octets.

---

## 10. Channelization [23.3.14, p3823–3824, verified against PDF]

S1G operates in sub-1 GHz channels (700 MHz – 1 GHz) defined in Annex E. Channel center frequency:

```
f_c [MHz] = ChannelStartingFrequency + f_separation × ChannelCenterChannelNumber
f_separation = 0.5 MHz
```

Primary-channel center: `f_c,primary [MHz] = ChannelStartingFrequency + 0.5 × PrimaryChannelNumber` where PrimaryChannelNumber is the subchannel index of the primary 1/2 MHz channel within the overall bandwidth. ChannelStartingFrequency and the channel-number sets are region/operating-class specific (Annex E Table E-5; "Channel spacing" column = S1G bandwidth). [p3824]

**US band (902–928 MHz), ChannelStartingFrequency = 0.902 GHz** [Table E-5, PDF p5664]:

- 2 MHz channels (S1G operating class 2, global class 69): channel numbers **2, 6, 10, 14, 18, 22, 26, 30, 34, 38, 42, 46, 50** → f_c = 902 + 0.5·n: **903, 905, 907, … , 927 MHz**. (CCA type 1 for ch 2, 38, 42, 46, 50; type 2 for 6–34.)
- 1 MHz channels (class 1): odd numbers 1–51 → 902.5 … 927.5 MHz.

**Out-of-band note for this project**: running at 1250 MHz is outside every Annex E class; the formula extrapolates trivially (e.g., with ChannelStartingFrequency = 902 MHz, "channel 696" ⇒ 902 + 0.5·696 = 1250 MHz), but no valid S1G channel number exists there — just program the synthesizer to f_c = 1250 MHz and keep the baseband identical.

## 11. Slot time [23.3.15, p3824]

"The slot time for the S1G PHY shall be 52 µs." (Consistent with Table 23-41.)

---

## SANITY CHECKS (verified numerically)

1. **Tone counts**: N_ST = N_SD + N_SP: Data 56 = 52+4 ✓; SIG 52 = 48+4 ✓. Data occupies ±1..±28 = 56 tones = N_ST ✓ (N_SR=28); SIG occupies ±1..±26 = 52 tones ✓ (N_SR=26 at CBW2).
2. **N_CBPS = N_SD × N_BPSCS** for every valid 2 MHz/1SS MCS: 52·1=52, 52·2=104, 52·4=208, 52·6=312, 52·8=416, 52·10=520 ✓.
3. **N_DBPS = N_CBPS × R**: 52/2=26, 104/2=52, 104·3/4=78, 208/2=104, 208·3/4=156, 312·2/3=208, 312·3/4=234, 312·5/6=260, 416·3/4=312, 520·3/4=390 ✓ (all integers).
4. **Data rate = N_DBPS / 40 µs** (long GI): MCS0 26/40µs = 650 kb/s ✓; MCS7 260/40µs = 6500 kb/s ✓; MCS11 390/40µs = 9750 kb/s ✓ — matches Table 23-46 exactly. Short-GI column = N_DBPS/36 µs (MCS0: 26/36µs = 722.2 kb/s ✓).
5. **Symbol/field durations**: T_SYML = 32+8 = 40 µs ✓; T_STF = 2·40 = 80 µs ✓; T_LTF1 = 2·32+16 = 80 µs ✓; T_SIG = 2·40 = 80 µs ✓; preamble+SIG (1 STS) = 240 µs ✓; consistent with aRxPHYStartDelay = 280 µs (= 240 µs + one 40 µs symbol of decision latency).
6. **SIG bit budget**: SIG-1 widths 1+1+1+2+2+9+1+1+1+4+1 = 24 ✓; SIG-2 widths 1+9+2+1+1+4+6 = 24 ✓; 48 uncoded bits → R=1/2 → 96 coded = 2 symbols × 48 data tones × 1 bit (BPSK) ✓. CRC covers 24+14 = 38 bits = bits 0–37 ✓ (N=37).
7. **Length field capacity**: 9 bits ⇒ max 511 = aPSDUMaxLengthWithNoAggregation ✓ (octets) and the 511-data-symbol cap behind aPSDUMaxLength ✓.
8. **TXTIME example** (MCS0, 100-octet PSDU, Agg=0, LGI): N_SYM = ceil((800+14)/26) = ceil(31.31) = 32; TXTIME = 240 + 40·32 = 1520 µs. Round-trip: floor((32·26−14)/8) = floor(818/8) = 102 ≥ 100 ✓ (Eq 23-81 upper-bounds the loadable octets; N_PAD = 32·26−800−14 = 18 bits ✓ 0 ≤ N_PAD < N_DBPS).
9. **aPPDUMaxTime cross-check** (1 MHz MCS10, N_DBPS=6 with 2× rep, 511 octets): N_SYM = ceil((8·511+14)/6) = 684; TXTIME(S1G_1M, LGI) = T_STF(160) + T_LTF1(160) + T_SIG(240) + 40·684 = 27 920 µs ✓ matches Table 23-41 NOTE 1.
10. **US channel math**: ch 2 → 902+1 = 903 MHz; ch 50 → 902+25 = 927 MHz; 13 channels of 2 MHz spaced 2 MHz spanning 902–928 ✓.
