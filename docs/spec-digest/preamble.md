# S1G_SHORT Preamble (2 MHz) + NDP Preamble — Implementation Digest

Scope: IEEE 802.11-2024 Clause 23 (S1G PHY). 2 MHz channel, 1 spatial stream (N_SS = N_STS = 1),
1 transmit chain (N_TX = 1), long GI, BCC, SU, no STBC, no beamforming (Q_k = 1), S1G_SHORT
format. All cross-references into Clauses 17/19/21 are resolved and inlined. Page numbers are PDF
page numbers of 80211-2024.pdf (printed page = PDF page − 1). All numeric tables/equations below
were verified against the rendered PDF pages, not just the text extraction.

---

## 1. Conventions and global constants

- Complex baseband; transmitted RF = Re{r(t)·exp(j2πf_c t)} [23.3.7, Eq 23-1, p3762].
- 2 MHz PPDU: 64 subcarriers, Δ_F = 31.25 kHz; signal occupies subcarriers −28…−1 and +1…+28;
  k = 0 is DC (unused) [23.3.7, p3761].
- **All numeric fields are transmitted in unsigned format, LSB first** [23.3.8.1, p3768].
- Timing constants (CBW2 column, Table 23-5) [23.3.6, pp3757-3758]:

  | Constant | Value | Samples @ 2 MS/s |
  |---|---|---|
  | N_SD (Data field) | 52 | — |
  | N_SP (Data field) | 4 | — |
  | N_ST (Data field) | 56 | — |
  | N_SR (Data field / LTF) | 28 | — |
  | Δ_F | 31.25 kHz | — |
  | T_DFT | 32 µs | 64 |
  | T_GI | 8 µs | 16 |
  | T_GI2 | 16 µs | 32 |
  | T_GIS (short GI; unused here) | 4 µs | 8 |
  | T_SYML | 40 µs | 80 |
  | T_STF | 80 µs (= 2·T_SYML) | 160 |
  | T_LTF1 | 80 µs (= 2·T_DFT + T_GI2) | 160 |
  | T_LTF (LTF2…) | 40 µs | 80 |
  | T_SIG | 80 µs (= 2·T_SYML) | 160 |
  | N_service | 8, N_tail | 6 | — |

- SIG-specific timing constants (Table 23-6) [23.3.6, p3759]: for the SIG field in S1G_SHORT at
  any BW ≥ 2 MHz, per-2 MHz-subchannel N_SD = 48, N_SP = 4, N_ST = 52; N_SR = 26 for CBW2.
  Short/double GI is never used for SIG (always T_GI = 8 µs, T_SYM = 40 µs, T_SIG = 80 µs).
- Tone scaling factors N_Tone^Field (Table 23-8, 2 MHz column) [23.3.7, pp3764-3765]:
  STF = 12, LTF1 = 56, SIG = 52, LTF2~LTFN = 56, Data = 56. GI per field: STF none (built into
  the periodic waveform), LTF1 = T_GI2, SIG = T_GI, LTF2~N = T_GI.
- Normalization N_Norm = N_STS,total = 1 for our case (S1G_SHORT) [23.3.7, p3765].
- Phase rotation Υ_k,BW: **for CBW2, Υ_k,2 = 1 for all k** (Eq 23-6) [23.3.7, p3767]. (4/8/16 MHz
  rotate per-2MHz-subchannel, Eqs 23-7…23-9 — out of scope.)
- Cyclic shift (Table 23-10) [23.3.8.2.2.2, p3769]: T_CS(n) for N_STS,total = 1 is **0 µs**.
  (2 streams: {0, −4} µs; 3: {0, −4, −2}; 4: {0, −4, −2, −6} — note only.)
- Spatial mapping Q_k: for 1 SS / 1 TX direct mapping, Q_k = 1 [23.3.10, 23.3.9.11].
- P_HTLTF (Eq 19-27) [19.3.9.4.6, p3439]:

  ```
  P_HTLTF = [ 1 -1  1  1
              1  1 -1  1
              1  1  1 -1
             -1  1  1  1 ]
  ```
  First column [P_HTLTF]_{*,1} = [1, 1, 1, −1]^T. With 1 SS only [P_HTLTF]_{1,1} = **1**, so every
  P_HTLTF factor below drops out.
- Windowing w_T(t): rectangular pulse of the field duration, optionally with raised-cosine
  transitions of length T_TR to smooth boundaries (example only, Eq 17-4; implementation-defined,
  constrained by the spectral mask) [17.3.2.5, pp3348-3349].
- Field timing boundaries (Eq 23-2, Figure 23-5) [23.3.7, pp3762-3763]:
  t_LTF1 = T_STF = 80 µs; t_SIG = 160 µs; t_LTF2 = 240 µs; t_Data = 240 + (N_LTF−1)·40 µs.
  For 1 SS: **preamble = STF(80) + LTF1(80) + SIG(80) = 240 µs = 480 samples @ 2 MS/s**, Data
  starts at t = 240 µs. (PLME TXTIME uses 240 µs preamble for S1G_SHORT [23.4.3, p~3843].)

### S1G_SHORT PPDU structure (Figure 23-1) [23.3.2, p3743]

```
| STF (2 sym) | LTF1 = GI2+LTS+LTS (2 sym) | SIG (2 sym) | LTF2..LTF_NLTF (1 sym each) | Data |
```
N_LTF from N_STS (Table 23-11) [23.3.8.2.2.4, p3770]: N_STS 1→1, 2→2, 3→4, 4→4.
**For 1 SS, N_LTF = 1: the LTF2…LTFN block is absent.**

---

## 2. STF [23.3.8.2.2.3, p3769]

### 2.1 Frequency-domain sequence (2 MHz)

The 2 MHz STF sequence is the 20 MHz L-STF of Eq (19-8) [19.3.9.3.3, p3428] (identical BPSK
pattern to Eq 17-8, but with QPSK normalization sqrt(1/2)):

```
S_{-26..26} = sqrt(1/2) * {
  0, 0, 1+j, 0, 0, 0, -1-j, 0, 0, 0, 1+j, 0, 0, 0, -1-j, 0, 0, 0, -1-j, 0, 0, 0, 1+j, 0, 0, 0,
  0,
  0, 0, 0, -1-j, 0, 0, 0, -1-j, 0, 0, 0, 1+j, 0, 0, 0, 1+j, 0, 0, 0, 1+j, 0, 0, 0, 1+j, 0, 0 }
```

Nonzero entries (12 tones, all with |S_k|² = 1):

| k | S_k | k | S_k |
|---|---|---|---|
| −24 | +sqrt(1/2)·(1+j) | +4 | −sqrt(1/2)·(1+j) |
| −20 | −sqrt(1/2)·(1+j) | +8 | −sqrt(1/2)·(1+j) |
| −16 | +sqrt(1/2)·(1+j) | +12 | +sqrt(1/2)·(1+j) |
| −12 | −sqrt(1/2)·(1+j) | +16 | +sqrt(1/2)·(1+j) |
| −8 | −sqrt(1/2)·(1+j) | +20 | +sqrt(1/2)·(1+j) |
| −4 | +sqrt(1/2)·(1+j) | +24 | +sqrt(1/2)·(1+j) |

All k are multiples of 4 → the time-domain waveform is periodic with period T_DFT/4 = **8 µs
(16 samples @ 2 MS/s)**. T_STF = 80 µs = **10 periods** (the "2 symbols" of Figure 23-1).
No cyclic prefix is inserted (the waveform is inherently periodic; Table 23-8 NOTE 2 says the STF
GI is N/A) [23.3.7, p3765].

### 2.2 Time-domain (Eq 23-14, verified from PDF p3769)

General:
```
r_STF^(iTX)(t) = 1/sqrt(N_STF_Tone * N_STS) * w_T_STF(t)
    * sum_{k=-N_SR}^{N_SR} sum_{m=1}^{N_STS}
        [Q_k]_{iTX,m} * [P_HTLTF]_{m,1} * Y_{k,BW} * S_k * exp(j*2*pi*k*dF*(t - T_CS(m)))
```
with N_SR = **26** for 2 MHz (STF only; note this differs from the Data-field N_SR = 28),
N_STF_Tone = 12.

**Specialized 2 MHz / 1 SS / 1 TX** (Q=1, P=1, Υ=1, T_CS=0):
```
r_STF(t) = (1/sqrt(12)) * w_80us(t) * sum_k S_k * exp(j*2*pi*k*31.25kHz*t),  0 <= t < 80 us
```
Discrete recipe @ 2 MS/s: place S_k into bins of a 64-point IFFT (bin k for k>0, bin 64+k for
k<0), take x[n] = sum_k S_k e^{j2πkn/64} (i.e. 64 × numpy.ifft output), scale by 1/sqrt(12), emit
n = 0…159 (x has period 16, so this is 10 repetitions of the first 16 samples).
Average power = (1/12)·Σ|S_k|² = 1.

---

## 3. LTF [23.3.8.2.2.4, pp3769-3771]

### 3.1 Frequency-domain sequence (2 MHz)

The 2 MHz LTF sequence is the 20 MHz VHT-LTF, Eq (21-36) [21.3.8.3.5, p3603], which equals
HT-LTF_{−28,28} (Eq 19-23):

```
LTF_{-28..28} = {1, 1, LTF_left, 0, LTF_right, -1, -1}
LTF_left  = {1, 1,-1,-1, 1, 1,-1, 1,-1, 1, 1, 1, 1, 1, 1,-1,-1, 1, 1,-1, 1,-1, 1, 1, 1, 1}   (26 elems, Eq 21-34)
LTF_right = {1,-1,-1, 1, 1,-1, 1,-1, 1,-1,-1,-1,-1,-1, 1, 1,-1,-1, 1,-1, 1,-1, 1, 1, 1, 1}   (26 elems, Eq 21-35)
```

Written out, all 57 entries for k = −28 … +28:

```
k:    -28 -27 -26 -25 -24 -23 -22 -21 -20 -19 -18 -17 -16 -15 -14 -13 -12 -11 -10  -9  -8  -7  -6  -5  -4  -3  -2  -1
LTF:    1   1   1   1  -1  -1   1   1  -1   1  -1   1   1   1   1   1   1  -1  -1   1   1  -1   1  -1   1   1   1   1
k:      0
LTF:    0
k:      1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28
LTF:    1  -1  -1   1   1  -1   1  -1   1  -1  -1  -1  -1  -1   1   1  -1  -1   1  -1   1  -1   1   1   1   1  -1  -1
```

56 nonzero tones (N_LTF_Tone = 56). Pilot positions carry LTF_{−21}=+1, LTF_{−7}=−1, LTF_{+7}=+1,
LTF_{+21}=+1 (part of the sequence itself).

### 3.2 A_LTF matrix (Eq 23-15, verified p3770)

```
A_LTF^k = [P_HTLTF]_{*,1} × [1 1 1 1]   if k ∈ K_Pilot_Fix     (pilot tones: first P column on every LTF symbol)
        = P_HTLTF                        otherwise
K_Pilot_Fix (2 MHz) = {±7, ±21}
```
For 1 SS: [A_LTF^k]_{1,1} = 1 for every k → A_LTF drops out entirely.

### 3.3 LTF1 time-domain (Eq 23-16, verified p3771)

General:
```
r_LTF1^(iTX)(t) = 1/sqrt(N_LTF_Tone * N_STS) * w_T_LTF1(t)
    * sum_{k=-N_SR}^{N_SR} sum_{m=1}^{N_STS}
        [Q_k]_{iTX,m} * Y_{k,BW} * [A_LTF^k]_{m,1} * LTF_k
        * exp(j*2*pi*k*dF*(t - T_GI2 - T_CS(m)))
```
N_SR = 28, N_LTF_Tone = 56, T_GI2 = 16 µs.

**Structure**: double-length GI (16 µs) followed by two periods of the 32 µs long training symbol
("two periods of the long training symbol, preceded by a double length (16 µs) cyclic prefix")
[23.3.8.2.2.4, p3771]. T_LTF1 = 16 + 32 + 32 = 80 µs.

**Specialized 2 MHz / 1 SS**:
```
r_LTF1(t) = (1/sqrt(56)) * w_80us(t) * sum_k LTF_k * exp(j*2*pi*k*dF*(t - 16us)),  0 <= t < 80us
```
Discrete @ 2 MS/s: let x[n] (n=0..63) = 64-pt IDFT of LTF_k, scaled 1/sqrt(56). Emit
`x[32..63], x[0..63], x[0..63]` — i.e. the last 32 samples as GI2, then two full 64-sample
periods. 160 samples total. Average power = 56/56 = 1.

### 3.4 LTF2…LTF_NLTF (Eq 23-17, verified p3771) — note only (absent for 1 SS)

For N_STS ≥ 2, LTFs n = 2…N_LTF are single 40 µs symbols (T_GI = 8 µs + 32 µs), with data tones
multiplied by column (n) of P_HTLTF ([A_LTF^k]_{m,(n+1)} with the equation's n running 1…N_LTF−1)
and pilot tones always using column 1:
```
r^(iTX)_{LTF2~N}(t) = 1/sqrt(56*N_STS) * sum_{n=1}^{N_LTF-1} w_T_LTF(t-(n-1)T_LTF)
    * sum_k sum_m [Q_k]_{iTX,m} Y_{k,BW} [A_LTF^k]_{m,(n+1)} LTF_k
      exp(j2πkΔF(t-(n-1)T_LTF - T_GI - T_CS(m)))
```

---

## 4. SIG field [23.3.8.2.2.5, pp3771-3775]

Two OFDM symbols (SIG-1 then SIG-2), each 40 µs (8 µs GI), each carrying 24 uncoded bits.
Total 48 uncoded bits → BCC R=1/2 → 96 coded bits → 2×48 → QBPSK.

### 4.1 Bit fields (Table 23-12, verified PDF pp3772-3774; figures 23-7/23-8 verified p3772)

SIG-1 (24 bits, B0 transmitted first):

| Bits | Width | Field | Semantics |
|---|---|---|---|
| B0 | 1 | Reserved | **Set to 1** on transmit |
| B1 | 1 | STBC | 1 = all spatial streams STBC; **0 for us** |
| B2 | 1 | Uplink Indication | 1 if PPDU addressed to an AP, else 0 (TXVECTOR UPLINK_INDICATION) |
| B3–B4 | 2 | BW | 0=CBW2, 1=CBW4, 2=CBW8, 3=CBW16 (S1G_DUP_2M: 0). **0 for us** |
| B5–B6 | 2 | Nsts | N_STS−1 (0→1 STS … 3→4 STS). **0 for us** |
| B7–B15 | 9 | ID | If Uplink Indication absent-or-1: B7–B15 = partial AID. If Uplink Indication = 0: B7–B9 = COLOR (BSS identifier), B10–B15 = partial AID |
| B16 | 1 | Short GI | 1 = short GI used in Data field. **0 for us (LGI)** |
| B17 | 1 | Coding | 0 = BCC, 1 = LDPC. **0 for us** |
| B18 | 1 | LDPC Extra | If Coding=1: 1 iff LDPC adds extra symbol(s) (21.3.10.5.4). **If Coding=0 this bit is set to 1** |
| B19–B22 | 4 | MCS | MCS index (LSB first like all fields) |
| B23 | 1 | Smoothing | 1 = channel smoothing recommended |

SIG-2 (24 bits):

| Bits | Width | Field | Semantics |
|---|---|---|---|
| B0 | 1 | Aggregation | 1 = A-MPDU (TXVECTOR AGGREGATION); required when PSDU > 511 octets |
| B1–B9 | 9 | Length | Aggregation=0: PSDU length in octets (≤511). Aggregation=1: number of symbols N_SYM (23.4.3) |
| B10–B11 | 2 | Response Indication | 0=No Response, 1=NDP Response, 2=Normal Response, 3=Long Response (TXVECTOR RESPONSE_INDICATION; Table 23-1, p3736) |
| B12 | 1 | Traveling Pilots | 1 = traveling pilots in Data field; 0 = regular pilots |
| B13 | 1 | NDP Indication | 0 = not an NDP CMAC PPDU (1 = NDP CMAC, see §6) |
| B14–B17 | 4 | CRC | Per 23.3.8.2.2.6, **c3 in B14** (c3 transmitted first) |
| B18–B23 | 6 | Tail | All 0 |

### 4.2 CRC (23.3.8.2.2.6, verified pp3775-3776)

- Protected bits: **m0…m37 = SIG-1 B0…B23 followed by SIG-2 B0…B13** (38 bits; the spec says
  "bits 0–37 of the ≥ 2 MHz SIG-A field"; N = 37).
- crc(D) = (M(D) ⊕ I(D))·D⁴ mod G(D); G(D) = D⁴ + D + 1; M(D) = m0·D^N + … + mN·D⁰;
  I(D) = Σ_{i=N−3}^{N} D^i (≡ initialize the 4-bit shift register to all 1s).
- CRC field = **1s complement** of crc(D); output c3 first (c3 → B14, c2 → B15, c1 → B16, c0 → B17).
- Reference implementation (validated against the spec's example: m0..m25 =
  1101 1001 1101 1010 0111 1011 11 → c3..c0 = 0101):

```rust
fn s2g_crc4(bits: &[u8]) -> [u8; 4] {          // bits = m0..mN in transmit order
    let (mut c3, mut c2, mut c1, mut c0) = (1u8, 1, 1, 1);
    for &b in bits {
        let fb = b ^ c3;
        c3 = c2; c2 = c1; c1 = c0 ^ fb; c0 = fb;
    }
    [1 ^ c3, 1 ^ c2, 1 ^ c1, 1 ^ c0]           // ones-complement, c3 first
}
```
(Figure 23-10, p3776: register reset to all 1s; feedback forced to 0 while shifting out; output
inverted.)

### 4.3 Encoding chain (23.3.4.3.3 pp3750-3751; 23.3.8.2.2.5 p3772)

1. **Assemble 48 bits**: SIG-1 B0…B23, SIG-2 B0…B23 (CRC + 6 zero tail bits included).
2. **BCC encode**, R = 1/2 [17.3.5.6, p3361]: K = 7, g0 = 133₈, g1 = 171₈; encoder starts in the
   all-zeros state; output A (g0) before B (g1) for each input bit. 48 → 96 coded bits.
   `A_i = b_i^b_{i-2}^b_{i-3}^b_{i-5}^b_{i-6}; B_i = b_i^b_{i-1}^b_{i-2}^b_{i-3}^b_{i-6}`.
   No puncturing. The 6 tail zeros return the encoder to state 0.
3. **Interleave** each 48-bit half separately (block size N_CBPS = 48, N_BPSC = 1) [17.3.5.7,
   p3363]: with s = max(N_BPSC/2, 1) = 1 the second permutation is the identity, so
   `j = 3*(k mod 16) + floor(k/16)` — coded bit k goes to position j
   (k = 0…47). Equivalently: write row-wise into a 3-row × 16-column array, read column-wise.
4. **BPSK map** [17.3.5.8, pp3363-3364]: bit 0 → −1, bit 1 → +1 (K_MOD = 1). Result: 96 reals;
   first 48 = d_{k,0}, second 48 = d_{k,1} (k = 0…47).
5. **Rotate 90° CCW (QBPSK)**: both SIG symbols use j·d (bit 0 → −j, bit 1 → +j; Figure 23-9,
   verified p3772). The rotation is written directly into Eq 23-18 as `j*D_{k,n,2}`. This
   distinguishes S1G_SHORT from S1G_1M and S1G_LONG.
6. **Subcarrier mapping** (Eq 23-20/23-21, verified pp3774-3775):

```
D_{k,n,2} = 0                    for k = 0, ±7, ±21
          = d_{M'2(k), n}        otherwise,  k = -26..26

M'2(k) = k+26  for -26 <= k <= -22
         k+25  for -20 <= k <= -8
         k+24  for  -6 <= k <= -1
         k+23  for   1 <= k <= 6
         k+22  for   8 <= k <= 20
         k+21  for  22 <= k <= 26
```
(M'2 is a bijection onto 0…47 and is exactly the inverse of the M(k) mapping of Eq 17-23; i.e.
logical data index 0 → k=−26, …, 47 → k=+26, skipping DC and pilots.)

7. **Pilot insertion** [17.3.5.9-17.3.5.10, pp3366-3367]: pilots at k = −21, −7, +7, +21 with
   values p_n·P_k where P_{−21} = 1, P_{−7} = 1, P_{+7} = 1, P_{+21} = **−1** (Eq 17-24) and the
   polarity p_n for SIG symbols n = 0, 1 is p_0 = p_1 = **+1** (first two elements of the
   127-element sequence p_{0..126} = {1,1,1,1,−1,−1,−1,1,…}, Eq 17-25). So both SIG symbols
   carry pilots {+1, +1, +1, −1} at {−21, −7, +7, +21}.

### 4.4 SIG time-domain (Eq 23-18/23-19, verified p3774)

General (BW ≥ 2 MHz; the SIG is duplicated per 2 MHz subchannel):
```
r_SIG^(iTX)(t) = 1/sqrt(N_SIG_Tone * N_STS) * sum_{n=0}^{1} w_T_SYML(t - n*T_SYML) *
  sum_{i_BW=0}^{N_2MHz-1} sum_{k=-26}^{26} sum_{m=1}^{N_STS}
    Y_{(k - K_Shift(i_BW)),BW} * [Q_k]_{iTX,m} * [P_HTLTF]_{m,1} * ( j*D_{k,n,2} + p_n*P_k )
    * exp( j*2*pi*(k - K_Shift(i_BW))*dF * (t - n*T_SYML - T_GI - T_CS(m)) )
K_Shift(i) = (N_2MHz - 1 - 2i)*32          (Eq 23-19)
```
For CBW2: N_2MHz = 1, K_Shift(0) = 0, Υ = 1, N_SIG_Tone = 52.

**Specialized 2 MHz / 1 SS**:
```
for n in 0..=1:
    X[k] = j*D_{k,n,2} + p_n*P_k          // data on 48 tones (±j), pilots on 4 tones (±1)
    x[s] = (1/sqrt(52)) * sum_{k=-26}^{26} X[k] * e^{j*2*pi*k*s/64},  s = 0..63
    symbol_n = [ x[48..63] , x[0..63] ]   // 16-sample GI + 64 samples = 80 samples
output = symbol_0 ++ symbol_1             // 160 samples
```
Average power = 52/52 = 1. Note the pilots are *not* rotated by j (only data tones are), and the
[P_HTLTF]_{m,1} factor multiplies both data and pilots (= 1 for 1 SS).

NOTE (p3775): the resulting QBPSK on both SIG symbols is rotated 90° CCW relative to the 3rd/4th
LTF1 repetitions of S1G_1M and relative to SIG-A2 of S1G_LONG — this is the receiver's format
discriminator.

---

## 5. Construction order summary (23.3.4.3, verified pp3749-3751)

- **STF** [23.3.4.3.1, p3749]: sequence gen → phase rotation (unity @2 MHz) → P_HTLTF column-1
  mapping → CSD → spatial mapping (Q) → IDFT → **GI of 8 µs prepend + windowing** → RF.
  (For the 80 µs periodic STF the "GI" is indistinguishable from the periodic extension; the
  time-domain field is exactly Eq 23-14 over 0…80 µs.)
- **LTF1** [23.3.4.3.2, p3750]: sequence gen → phase rotation → A_LTF column-1 (pilots handled
  per Eq 23-15) → CSD → Q → IDFT → **GI of 16 µs** + windowing → RF.
- **SIG** [23.3.4.3.3, pp3750-3751]: assemble 48 bits (reserved bits + 4-bit CRC + 6 tail) → BCC
  R=1/2 → interleave → QBPSK (48+48) → pilot insertion → P_HTLTF column-1 → CSD → Q →
  duplicate over 2 MHz subchannels + phase rotation → IDFT → GI 8 µs + windowing → RF.
- **LTF2-LTFN** [23.3.4.3.4, p3751]: absent for 1 SS.

---

## 6. NDP preamble format (23.3.11, verified pp3809-3810)

An NDP is an S1G PPDU **without the Data field**. Two uses:

### 6.1 NDP for sounding (Figure 23-17, p3809) — note only (requires N_STS ≥ 2, out of SISO scope)
`STF(2 sym) | LTF1(2 sym) | SIG(2 sym) | LTF2…LTF_NLTF(1 sym each)`; S1G_SHORT only; never
1 MHz. SIG settings: MCS = 0, Length = 0, BW = CH_BANDWIDTH of the preceding VHT NDP
Announcement, Nsts ≥ 2 (N_LTF from Table 23-11), Partial AID per 10.21, NDP Indication = 0,
Response Indication = 3 (Long Response).

### 6.2 NDP CMAC PPDU, ≥ 2 MHz (NDP_2M) (Figures 23-18, 23-21, p3810; 23.3.12.1, p3811)

PHY-level frame: **`STF(2 sym) | LTF1(2 sym) | SIG(2 sym)`** and nothing else — total 240 µs.
Exactly one LTF (single space-time stream). STF and LTF1 are identical to a normal 1-STS
S1G_SHORT preamble (§2, §3). PPDUs wider than 2 MHz are the 2 MHz NDP CMAC PPDU duplicated up
to CH_BANDWIDTH.

SIG field layout replaces the normal Table 23-12 fields (Figure 23-21, verified p3810):

| Bits | Width | Field |
|---|---|---|
| B0–B36 | 37 | NDP CMAC PPDU body (TXVECTOR NDP_CMAC_PPDU_BODY, concatenated bits, B0 first) |
| B37 | 1 | NDP Indication = **1** (same physical position as SIG-2 B13 of a normal PPDU — "the fourteenth bit of the second symbol") |
| B38–B41 | 4 | CRC (23.3.8.2.2.6, over m0…m37 = body + NDP Indication; c3 first) |
| B42–B47 | 6 | Tail = 0 |

Everything else about the SIG waveform (BCC, interleaving, QBPSK ×j, pilots, Eq 23-18, 1/sqrt(52))
is unchanged. A receiver detects NDP CMAC by decoding SIG and finding bit 37 (SIG-2 B13) = 1.
The body's first 3 bits are the NDP CMAC PPDU Type (Table 23-30, p3811: 0=CTS/CF-End, 1=PS-Poll,
2=Ack, 3=PS-Poll-Ack, 4=BlockAck, 5=BF Report Poll, 6=Paging, 7=Probe Req) — body contents are
MAC-level (23.3.12), not needed by the PHY, which just transmits the 37 bits given in TXVECTOR.

(1 MHz NDP CMAC, Figure 23-19/23-20: STF 4 sym, LTF1 4 sym, SIG 6 sym; body 25 bits B0–B24,
NDP Indication B25, CRC B26–B29, Tail B30–B35 — out of scope.)

---

## 7. S1G_LONG differences (23.3.8.2.3, pp3776-3789) — skim notes only

- Mixed-mode-like: omnidirectional portion `STF | LTF1 | SIG-A` (always 1 STS, **no Q matrix**,
  per-antenna CSD from Table 23-13) + beam-changeable portion `D-STF(1 sym) | D-LTF1…N(1 sym
  each) | SIG-B(1 sym) | Data`; supports MU and beamforming.
- SIG-A: same 2-symbol/48-bit/CRC machinery, but **SIG-A1 is QBPSK-rotated and SIG-A2 is plain
  BPSK** (Figure 23-15, verified p3779) — that asymmetry vs S1G_SHORT (both rotated) is the format
  discriminator. SU SIG-A field layout differs slightly (MU/SU bit at B0, no Aggregation-position
  match, Beam Change/Smoothing at B23, no NDP Indication/Traveling Pilots in A1, etc., Table 23-14).
- STF is identical to S1G_SHORT's STF [23.3.8.2.3.2.3, p3777]; LTF1 uses the same LTF sequence.
- D-STF/D-LTF use N_Tone = 12/56 with T_DSTF = T_DLTF = 40 µs.

---

## 8. SANITY CHECKS (verified numerically)

1. **CRC self-test**: spec example m0…m25 = {11 0110 0111 0110 1001 1110 1111} → c3…c0 = {0101}
   ✔ reproduced by the shift-register implementation in §4.2 [23.3.8.2.2.6, p3776].
2. **STF tone count**: 12 nonzero S_k = N_STF_Tone(2 MHz) = 12 (Table 23-8) ✔; all indices are
   multiples of 4 → 8 µs periodicity, 80 µs / 8 µs = 10 repetitions ✔; avg power
   (1/12)Σ|S_k|² = 1 ✔.
3. **LTF length**: 2+26+1+26+2 = 57 entries (k = −28…28), 56 nonzero = N_LTF_Tone = N_ST = 56
   (Tables 23-5/23-8) ✔; avg power 56/56 = 1 ✔.
4. **SIG tone budget**: 48 data + 4 pilots = 52 = N_SIG_Tone (Tables 23-6/23-8) ✔;
   M′₂ is a bijection {−26…26}∖{0,±7,±21} → {0…47} and is the exact inverse of Eq 17-23's M(k) ✔.
5. **SIG bit budget**: SIG-1 widths 1+1+1+2+2+9+1+1+1+4+1 = 24 ✔; SIG-2 widths 1+9+2+1+1+4+6 = 24
   ✔; protected bits 24+14 = 38 = m0…m37 (N = 37) ✔; NDP SIG: 37+1+4+6 = 48 ✔.
6. **Interleaver**: N_CBPS = 48, N_BPSC = 1 ⇒ s = 1, second permutation = identity;
   j = 3(k mod 16) + ⌊k/16⌋ is a permutation of 0…47 ✔.
7. **Coded bits**: 48 uncoded × 2 (R = 1/2) = 96 = 2 symbols × 48 = 2 × N_SD(SIG) ✔.
8. **Durations**: T_SYML = 32+8 = 40 µs; T_STF = 80 µs = 160 samples @2 MS/s; T_LTF1 =
   16+32+32 = 80 µs; T_SIG = 2×40 = 80 µs; preamble (1 SS) = 240 µs = 480 samples; N_LTF(1 STS)
   = 1 so no LTF2 block ✔ (Tables 23-5/23-11, Figure 23-5 offsets: t_LTF1=80 µs, t_SIG=160 µs,
   t_Data=240 µs for N_LTF=1).
9. **Cyclic shift / rotation degeneracy**: T_CS = 0 (Table 23-10, N_STS=1), Υ_k,2 = 1 (Eq 23-6),
   [P_HTLTF]_{1,1} = 1 (Eq 19-27), Q_k = 1 ⇒ all matrix factors vanish for SISO 2 MHz ✔.
10. **Pilot values**: P at {−21,−7,7,21} = {1,1,1,−1} (Eq 17-24); p_0 = p_1 = +1 (Eq 17-25) ⇒
    SIG pilots identical on both symbols ✔. LTF sequence values at pilot bins: {+1,−1,+1,+1} —
    these come from the LTF sequence itself, NOT from P_k.

## Gaps / cautions for implementers

- The exact discrete GI/window splice at field boundaries (rectangular vs raised-cosine, T_TR) is
  implementation-defined [17.3.2.5]; the sample recipes above use rectangular windows abutting at
  field boundaries, which is the degenerate T_TR→0 case.
- Data-field pilot polarity/traveling-pilot handling (23.3.9.10) and the Data-field waveform are
  covered by the Data-field digest, not here.
