# S1G PHY Digest — Timing Parameters & Mathematical Signal Description

**Scope:** 2 MHz bandwidth, 1 spatial stream (SISO), Long GI (8 µs), BCC, S1G_SHORT preamble, SU PPDU, no STBC / no beamforming / no MU.
Covers: 23.3.5 (MCS overview), 23.3.6 (timing parameters), 23.3.7 (mathematical description), 23.3.9.11 (OFDM modulation), with all cross-references (17.3.2.5, 17.3.5.10, 19.3.9.4.6, 21.3.10.10) resolved inline.

Page citations are PDF page numbers of IEEE Std 802.11-2024 (the printed page number is PDF page − 1). All numeric tables and equations below were verified visually against the rendered PDF pages.

---

## 1. Core timing constants — Table 23-5 [23.3.6, pp3757–3758]

Applies to **all** S1G fields except the SIG field of S1G_SHORT (see §2). Verified against PDF pp3757–3758.

2 MHz (CBW2) column, with other bandwidths for context:

| Param | CBW1 | **CBW2** | CBW4 | CBW8 | CBW16 | Meaning |
|---|---|---|---|---|---|---|
| N_SD | 24 | **52** | 108 | 234 | 468 | data subcarriers / symbol |
| N_SP | 2 | **4** | 6 | 8 | 16 | pilot subcarriers / symbol |
| N_ST | 26 | **56** | 114 | 242 | 484 | total used subcarriers (= N_SD + N_SP) |
| N_SR | 13 | **28** | 58 | 122 | 250 | highest used subcarrier index |

Bandwidth-independent (all CBW):

| Param | Value | Meaning |
|---|---|---|
| Δ_F | **31.25 kHz** | subcarrier spacing |
| T_DFT | **32 µs** = 1/Δ_F | IDFT/DFT period |
| T_GI | **8 µs** = T_DFT/4 | (long) guard interval |
| T_GI2 | **16 µs** | double guard interval |
| T_GIS | **4 µs** = T_DFT/8 | short guard interval |
| T_SYML | **40 µs** = T_DFT + T_GI = 1.25·T_DFT | symbol duration, normal (long) GI |
| T_SYMS | 36 µs = T_DFT + T_GIS = 1.125·T_DFT | symbol duration, short GI (out of scope) |
| T_SYM | T_SYML or T_SYMS per GI used → **40 µs in scope** | OFDM symbol duration |
| N_service | **8** | bits in SERVICE field |
| N_tail | **6** | tail bits per BCC encoder |

Field durations (CBW2 column; CBW1 differs, shown for context) [Table 23-5, p3758]:

| Param | CBW1 | **CBW ≥ 2** | Meaning |
|---|---|---|---|
| T_STF | 160 µs = 4·T_SYML | **80 µs = 2·T_SYML** | STF duration |
| T_LTF1 | 160 µs = 4·T_DFT + 2·T_GI + T_GI2 | **80 µs = 2·T_DFT + T_GI2** | first LTF duration |
| T_LTF | 40 µs = T_SYML | **40 µs = T_SYML** | LTF2…LTF_NLTF duration each |
| T_SIG | 240 µs = 6·T_SYML | **80 µs = 2·T_SYML** | SIG field duration |
| T_DSTF, T_DLTF, T_SIG-A, T_SIG-B | — | (S1G_LONG only, out of scope: 40/40/80/40 µs) | |

NOTE at table end: N_ST = N_SD + N_SP. [p3758]

### FFT size / sample rate (2 MHz)
- "For a 2 MHz S1G PPDU transmission, the 2 MHz is divided into **64 subcarriers**. The signal is transmitted on subcarriers **−28 to −1 and 1 to 28**, with **0 being the center (DC) subcarrier**." [23.3.7, p3761]
- Native complex sample rate = 64 × 31.25 kHz = **2 Msps** (T_s = 0.5 µs).
- Samples at native rate: T_DFT = **64 samples**; T_GI = **16 samples**; T_GI2 = **32 samples**; T_GIS = 8 samples; LGI symbol T_SYML = **80 samples**; T_STF = 160 samples; T_LTF1 = 160 samples; T_SIG = 160 samples.
- Null subcarriers (64-pt grid, indices −32…31): DC (0) and guards ±29, ±30, ±31, −32. Used tones: −28…−1, 1…28 (56 tones = 52 data + 4 pilots in LTF/Data symbols).

---

## 2. SIG-field timing constants — Table 23-6 [23.3.6, p3759]

The SIG field of S1G_SHORT (and SIG-A of S1G_LONG) uses **different subcarrier counts** — it is 11a-like (52 tones), not 56. "Short/double guard interval is not used for SIG/SIG-A field in ≥ 2 MHz PPDUs." Verified against PDF p3759.

| Param | **CBW2** value | Meaning |
|---|---|---|
| N_SD | **48** | data subcarriers per symbol (per 2 MHz subchannel) |
| N_SP | **4** | pilot subcarriers per symbol |
| N_ST | **52** | total used subcarriers |
| N_SR | **26** | highest used subcarrier index |
| Δ_F | 31.25 kHz | same spacing |
| T_DFT | 32 µs | |
| T_GI | 8 µs | SIG always uses long GI |
| T_SYM | 40 µs | |
| T_SIG | **80 µs = 2 × T_SYM** | SIG = 2 OFDM symbols |

So for the SIG field the occupied tones are −26…−1, 1…26; tones ±27, ±28 are additionally null relative to the Data field.

---

## 3. Frequently used parameters — Table 23-7 [23.3.6, pp3760–3761], specialized to SU/1SS

| Symbol | General | **In scope (SU, 1SS, 2 MHz)** |
|---|---|---|
| N_CBPS | coded bits per symbol | = N_SD·N_BPSCS (52·N_BPSCS) |
| N_CBPSS | coded bits per symbol per spatial stream | = N_CBPS (1 SS) |
| N_CBPSSI | coded bits/sym/SS per BCC interleaver block; = N_CBPSS for 1/2/4/8 MHz, N_CBPSS/2 for 16 MHz | = N_CBPS |
| N_DBPS | data bits per symbol | = N_CBPS·R |
| N_BPSCS | coded bits per subcarrier per SS | 1/2/4/6/8/10 per MCS |
| Nu | number of users; =1 for S1G_SHORT | **1** |
| N_STS, N_STS,total | space-time streams | **1** |
| N_SS | spatial streams | **1** |
| N_TX | transmit chains | **1** (SISO implementation) |
| N_ES | number of BCC encoders (Data, SU) | **1** |
| N_LTF | number of LTFs in S1G_SHORT | **1** (see Table 23-11) |
| R | coding rate | per MCS |
| M_u | stream offset for user u; M_0 = 0 | **0** |

N_LTF vs N_STS — Table 23-11 [23.3.8.2.2.4, p3770]: N_STS=1→N_LTF=1, 2→2, 3→4, 4→4.

---

## 4. MCS overview [23.3.5, p3757] and valid MCS for 2 MHz / 1 SS

The S1G-MCS determines modulation and coding of the Data field; for S1G SU PPDUs it is carried in the SIG (S1G_SHORT) or SIG-A field. Rate-dependent parameters are in Table 23-42 to Table 23-61 (23.5). Indices 0 to 9 exist generally (index 10 solely for 1 MHz, N_SS=1); equal modulation on all streams. [23.3.5, p3757]

**Table 23-46 — S1G-MCSs for 2 MHz, N_SS = 1** [23.5, pp3858–3859] (extracted text; internally consistent — see Sanity Checks):

| MCS | Mod | R | N_BPSCS | N_SD | N_SP | N_CBPS | N_DBPS | N_ES | Rate @ 8 µs GI (kb/s) | Rate @ 4 µs GI |
|---|---|---|---|---|---|---|---|---|---|---|
| 0 | BPSK | 1/2 | 1 | 52 | 4 | 52 | 26 | 1 | 650.0 | 722.2 |
| 1 | QPSK | 1/2 | 2 | 52 | 4 | 104 | 52 | 1 | 1300.0 | 1444.4 |
| 2 | QPSK | 3/4 | 2 | 52 | 4 | 104 | 78 | 1 | 1950.0 | 2166.7 |
| 3 | 16-QAM | 1/2 | 4 | 52 | 4 | 208 | 104 | 1 | 2600.0 | 2888.9 |
| 4 | 16-QAM | 3/4 | 4 | 52 | 4 | 208 | 156 | 1 | 3900.0 | 4333.3 |
| 5 | 64-QAM | 2/3 | 6 | 52 | 4 | 312 | 208 | 1 | 5200.0 | 5777.8 |
| 6 | 64-QAM | 3/4 | 6 | 52 | 4 | 312 | 234 | 1 | 5850.0 | 6500.0 |
| 7 | 64-QAM | 5/6 | 6 | 52 | 4 | 312 | 260 | 1 | 6500.0 | 7222.2 |
| 8 | 256-QAM | 3/4 | 8 | 52 | 4 | 416 | 312 | 1 | 7800.0 | 8666.7 |
| 9 | Not valid | | | | | | | | | |
| 10 | Not valid | | | | | | | | | |
| 11 | 1024-QAM | 3/4 | 10 | 52 | 4 | 520 | 390 | 1 | 9750.0 | 10833.3 |
| 12 | Not valid | | | | | | | | | |

**Valid MCS set for 2 MHz / 1 SS: {0, 1, 2, 3, 4, 5, 6, 7, 8, 11}.** (MCS 9, 10, 12 are Not valid.) Rate @ LGI = N_DBPS / 40 µs.

---

## 5. Mathematical description of signals [23.3.7, pp3761–3768]

### 5.1 Baseband → RF [Eq 23-1, p3762]

```
r_RF(t) = Re{ r(t) · exp(j·2π·fc·t) }        // fc = channel center frequency
```

### 5.2 Field concatenation for S1G_SHORT [Eq 23-2, Figure 23-5, pp3762–3763; verified on PDF p3763]

```
r_PPDU(t) = r_STF(t)
          + r_LTF1(t − t_LTF1)
          + r_SIG(t − t_SIG)
          + Σ_{i_LTF=2}^{N_LTF} r_LTF^{(i_LTF)}(t − t_LTF2 − (i_LTF − 2)·T_LTF)
          + r_Data(t − t_Data)
where
  t_LTF1 = T_STF
  t_SIG  = t_LTF1 + T_LTF1
  t_LTF2 = t_SIG + T_SIG
  t_Data = t_LTF2 + (N_LTF − 1)·T_LTF
```

**Specialized (2 MHz, N_LTF = 1):** start times
- STF: t = 0 (duration 80 µs = 160 samples)
- LTF1: t = 80 µs (duration 80 µs)
- SIG: t = 160 µs (duration 80 µs, 2 symbols)
- LTF2…: none (N_LTF = 1)
- Data: t = **240 µs** (sample 480 at 2 Msps)

Figure 23-5(a) [p3762] confirms order STF | LTF1 | SIG | LTF2…LTF_NLTF | Data symbols, with t=0 at STF start.

### 5.3 Generic subfield IDFT [Eq 23-4, p3763; verified on PDF p3763]

Each field is a summation of one or more subfields; each subfield is:

```
r_Field^(iTX)(t) = 1/sqrt(N_Tone_Field · N_Norm) · w_TField(t) ·
    Σ_{k=−N_SR}^{N_SR} Σ_{u=0}^{Nu−1} Σ_{m=1}^{N_STS,u}
        [Q_k]_{iTX,(Mu+m)} · Υ_{k,BW} · X_{k,u}^{(m)} ·
        exp( j·2π·k·Δ_F·(t − T_GI,Field − T_CS(Mu+m)) )
```

where [pp3764–3766, verified on PDF pp3765–3766]:
- **N_Norm** = N_STS,total (for all cases except omni portion of S1G_LONG where it is N_TX). **In scope: N_Norm = 1.**
- **w_TField(t)**: windowing function; example in 17.3.2.5 (see §5.6).
- **T_Subfield** (window duration): T_STF for STF, T_LTF1 for LTF1, T_SIG for SIG, T_LTF for LTF2…LTF_NLTF, T_SIG-B for SIG-B. [p3765]
- **Q_k**: spatial mapping matrix (N_TX × N_STS,total). **In scope (1×1, no beamforming): Q_k = 1** (direct mapping; see 23.3.9.11 note referencing 19.3.11.11.2 examples — direct map is Q = 1). [p3766, p3806]
- **X_{k,u}^{(m)}**: frequency-domain symbol on subcarrier k; zero for DC, guard tones, and unmodulated STF tones. [p3766]
- **T_GI,Field**: per-field GI from Table 23-8 (see §5.4). The `(t − T_GI,Field)` shift creates the cyclic prefix.
- **T_CS(l)**: cyclic shift per space-time stream, Table 23-10 (see §5.5). **In scope: 0.**
- **Υ_{k,BW}**: per-subcarrier phase rotation (see §5.5).
- Multi-bandwidth power note (S1G_LONG only) omitted — out of scope.

### 5.4 Tone-scaling factors N_Tone_Field and per-field GI — Table 23-8 [pp3764–3765; verified on PDF pp3764–3765]

2 MHz column (S1G / S1G_SHORT-relevant rows):

| Field | **N_Tone (2 MHz)** | GI duration |
|---|---|---|
| STF | **12** | N/A — STF symbols have no GI (NOTE 2) |
| LTF1 | **56** | **T_GI2** (16 µs) for ≥2 MHz |
| SIG | **52** | **T_GI** (8 µs) |
| LTF2~LTF_NLTF | 56 | T_GI |
| First Data symbol | **56** | **T_GI** (NOTE 3: first Data symbol *always* uses T_GI regardless of GI_TYPE) |
| 2nd…last Data symbols | **56** | T_GI (LONG_GI) or T_GIS (SHORT_GI) → **T_GI in scope** |
| (S1G_LONG rows: SIG-A 52, D-STF 12, D-LTF 56, SIG-B 56 — out of scope) | | |
| (S1G_DUP rows — out of scope) | | |

So the amplitude normalization per field at 2 MHz / 1 STS:
- STF: 1/√12
- LTF1, LTF2+, Data: 1/√56
- SIG: 1/√52

**GI structure per field (2 MHz, LGI), in native samples:**
- **STF:** no GI; 80 µs window over the periodic STF waveform (160 samples).
- **LTF1:** double GI then two full DFT periods: [GI2 = 32 samples][64][64] = 160 samples (T_LTF1 = 2·T_DFT + T_GI2).
- **SIG:** 2 symbols, each [GI = 16][64] = 80 samples.
- **Data:** every symbol [GI = 16][64] = 80 samples (first symbol always long GI; with LGI selected, all symbols identical).

### 5.5 Phase rotation Υ_{k,BW} and cyclic shift T_CS

**Υ_{k,BW}** selected by TXVECTOR CH_BANDWIDTH (Table 23-9, p3766). Definitions [Eqs 23-5 … 23-13, pp3766–3767; verified on PDF pp3766–3767]:

- CBW1: Υ_{k,1} = 1 (Eq 23-5)
- **CBW2 (FORMAT S1G): Υ_{k,2} = 1 for all k (Eq 23-6). → No phase rotation at 2 MHz; multiply by +1 everywhere.**
- CBW4 (S1G/S1G_DUP_2M): Υ_{k,4} = 1 for k<0, j for k≥0 (Eq 23-7)
- CBW8: 1 for k<−64, −1 for k≥−64 (Eq 23-8)
- CBW16: 1 for k<−192; −1 for −192≤k<0; 1 for 0≤k<64; −1 for 64≤k (Eq 23-9)
- S1G_DUP_1M variants (Eqs 23-10…23-13) — out of scope.

**T_CS(n)** — Table 23-10, cyclic shift for S1G_SHORT preamble PPDU, ≥2 MHz [23.3.8.2.2.2, p3768; verified on PDF]:

| N_STS,total | stream 1 | 2 | 3 | 4 |
|---|---|---|---|---|
| **1** | **0 µs** | — | — | — |
| 2 | 0 | −4 µs | — | — |
| 3 | 0 | −4 | −2 µs | — |
| 4 | 0 | −4 | −2 | −6 µs |

**In scope (1 STS): T_CS = 0 — no cyclic shift anywhere.** The same cyclic shift applies to STF, LTF, SIG, and Data of the S1G_SHORT PPDU [23.3.8.2.2.2, p3768].

### 5.6 Windowing function w_T(t) [17.3.2.5, Eq 17-4, p3349; discrete example Eq 17-5, p3350]

Cross-reference resolved from Clause 17 ("For a description of the conventions ... see 17.3.2.5" [23.3.7, p3761]):

```
            /  sin^2( (π/2)·(0.5 + t/T_TR) )          −T_TR/2 <  t < T_TR/2
w_T(t) =   {   1                                       T_TR/2 ≤ t < T − T_TR/2
            \  sin^2( (π/2)·(0.5 − (t−T)/T_TR) )      T − T_TR/2 ≤ t < T + T_TR/2
```

- T = subfield duration; T_TR = transition time, "about 100 ns". With T_TR → 0 this degenerates to a rectangular pulse of duration T. The standard *describes* waveforms with the rectangular pulse; smoothing (or frequency-domain filtering) is an implementation choice to meet spectral mask / EVM — not normative. [17.3.2.5, p3349]
- Consecutive subfields **overlap by T_TR** (overlap-add): each windowed subfield extends T_TR/2 before its nominal start and T_TR/2 past its nominal end; adjacent field contributions add in the overlap. [Figure 17-2, p3350]
- Discrete-time example [Eq 17-5, p3350]: for T = 4.0 µs, T_TR = 100 ns at 20 Msps: w[n] = 1 for 1 ≤ n ≤ 79; 0.5 at n = 0 and n = 80; 0 otherwise. I.e. the boundary sample is shared 50/50 between adjacent symbols.
  - **S1G equivalent at 2 Msps:** T_SYML = 40 µs → 80 samples; the analogous discrete window is w[n] = 1 for 1…79, 0.5 at n ∈ {0, 80}: symbol n's sample 80 coincides with symbol n+1's sample 0 and they are averaged. (At 2 Msps, T_TR = 100 ns is sub-sample; a pure rectangular window — 80 samples per symbol, no overlap — is also spec-conformant since windowing is non-normative.)
- IFFT mapping convention [17.3.2.6, p3350]: with a 64-point IFFT, subcarriers 1…26 (S1G: 1…28) map to IFFT inputs 1…26 (1…28); subcarriers −26…−1 (−28…−1) map to inputs 38…63 (36…63); remaining inputs (incl. DC) are 0. Output is cyclically extended and windowed to symbol length.

---

## 6. OFDM modulation of the Data field [23.3.9.11, Eq 23-55, pp3804–3806; verified on PDF p3804]

For S1G_SHORT, the Data field on transmit chain i_TX:

```
r_Data^(iTX)(t) =
  1/sqrt(N_Tone_Data · N_STS) ·
  Σ_{n=0}^{N_SYM−1}  w_TSYM(t − T_Accum(n)) ·
    Σ_{k=−N_SR}^{N_SR} Σ_{m=1}^{N_STS}
      [Q_k]_{iTX,m} · Υ_{k,BW} ·
      ( D̃_{k,m,n,BW} + [P_HTLTF]_{m,g(n)} · p_{n+2} · P_n^k ) ·
      exp( j·2π·k·Δ_F·( t − T_Accum(n) − T_GI,Data(n) − T_CS(m) ) )       (23-55)

g(n) = (n mod 2) + 1  if STBC bit AND Traveling Pilots bit in SIG are both 1
     = 1              otherwise                                            [p3804]
```

Supporting definitions [pp3805–3806]:

- **T_GI,Data(n)** [Eq 23-58]:
  ```
  T_GI,Data(n) = T_GI    if n = 0                                (always long GI on first Data symbol)
               = T_GI    if n > 0 and Short GI subfield of SIG = 0
               = T_SGI   if n > 0 and Short GI subfield of SIG = 1
  ```
  **In scope (LGI): T_GI,Data(n) = 8 µs for every n.**

- **T_Accum(n)** — accumulated duration of data symbols 0…n−1 [Eq 23-61]:
  ```
  T_Accum(n) = 0                          if n = 0
             = T_SYML + (n−1)·T_SYM       if n > 0
  ```
  **In scope: T_Accum(n) = n · 40 µs** (since T_SYM = T_SYML).

- **D̃_{k,m,n,BW}** — data with pilot positions zeroed [Eq 23-59]:
  ```
  D̃_{k,m,n,BW} = 0                        if k ∈ K_Pilot(n)
               = d_{M'_BW(k), m, n}       otherwise
  ```
  (Note d is also implicitly zero at DC k=0 and unused |k|>N_SR; only the 52 data tones carry d.)

- **K_Pilot(n)** [Eq 23-60]: = K_Pilot_Fix if Traveling Pilot bit = 0 (in scope), else K_Pilot_Travel(n).
  **K_Pilot_Fix for 2 MHz = {−21, −7, 7, 21}** [23.3.8.2.2.4, p3770; also 23.3.9.10].

- **M'_2(k)** — subcarrier→data-symbol-index map for 2 MHz [Eq 23-30, 23.3.8.2.3.3.5, p3786; verified on PDF]:
  ```
  M'_2(k) = k + 28   for −28 ≤ k ≤ −22
          = k + 27   for −20 ≤ k ≤ −8
          = k + 26   for  −6 ≤ k ≤ −1
          = k + 25   for   1 ≤ k ≤ 6
          = k + 24   for   8 ≤ k ≤ 20
          = k + 23   for  22 ≤ k ≤ 28
  ```
  This walks data symbols d_0 … d_51 across k = −28…28 skipping pilots (±21, ±7) and DC. (Ranges deliberately exclude k ∈ {−21,−7,0,7,21}.)

- **P_HTLTF** — HT-LTF mapping matrix [Eq 19-27, 19.3.9.4.6, p3439; verified on PDF]:
  ```
  P_HTLTF = [  1  −1   1   1
               1   1  −1   1
               1   1   1  −1
              −1   1   1   1 ]
  ```
  Eq 23-55 uses [P_HTLTF]_{m,g(n)}. **In scope (m = 1, no STBC ⇒ g(n) = 1): [P_HTLTF]_{1,1} = +1**, i.e. the pilot term reduces to p_{n+2}·P_n^k.

- **p_n** — pilot polarity sequence [17.3.5.10, Eq 17-25, p3363]: cyclic extension of the 127-element sequence (index n mod 127):
  ```
  p_0..126 = [ 1, 1, 1, 1, −1,−1,−1, 1, −1,−1,−1,−1,  1, 1,−1, 1,
              −1,−1, 1, 1, −1, 1, 1,−1,  1, 1, 1, 1,  1, 1,−1, 1,
               1, 1,−1, 1,  1,−1,−1, 1,  1, 1,−1, 1, −1,−1,−1, 1,
              −1, 1,−1,−1,  1,−1,−1, 1,  1, 1, 1, 1, −1,−1, 1, 1,
              −1,−1, 1,−1,  1,−1, 1, 1, −1,−1,−1, 1,  1,−1,−1,−1,
              −1, 1,−1,−1,  1,−1, 1, 1,  1, 1,−1, 1, −1, 1,−1, 1,
              −1,−1,−1,−1, −1, 1,−1, 1,  1,−1, 1,−1,  1, 1, 1,−1,
              −1, 1,−1,−1, −1, 1, 1, 1, −1,−1,−1,−1, −1,−1,−1 ]
  ```
  (Generated by the 11a scrambler with all-1s initial state, 1→−1 / 0→+1.) Data symbol n uses **p_{n+2}** in S1G_SHORT (p_0, p_1 are consumed by the two SIG symbols; see the SIG field definition).

- **P_n^k** — pilot mapping [where-list p3805 → 23.3.9.10, p3804ff]. For S1G_SHORT with fixed pilots, "P_n^k with same FFT sizes is identical to what is defined in 21.3.10.10" [23.3.9.10, p3804]. From 21.3.10.10 Eq 21-91 (20 MHz / 64-FFT case) [p3629–3630]:
  ```
  P_n^{−21,−7,7,21} = { Ψ_{n mod 4}, Ψ_{(n+1) mod 4}, Ψ_{(n+2) mod 4}, Ψ_{(n+3) mod 4} }
  P_n^k = 0 for k ∉ {−21,−7,7,21}
  ```
  where Ψ is the N_STS=1 row of Table 19-19: **Ψ_0 = 1, Ψ_1 = 1, Ψ_2 = 1, Ψ_3 = −1**.
  (A dedicated pilots digest should confirm Table 19-19 and the SIG-symbol pilot values; flagged as a soft spot in Gaps.)

- N_Tone_Data = 56 (2 MHz, Table 23-8); N_SR = 28 (Data-field value, Table 23-5); Q_k, Υ, Δ_F, T_CS as in §5. Q_k spatial-mapping notes [p3806]: examples per 19.3.11.11.2; direct-map/CSD forms recommended for LTF1 smoothness. For SISO simply Q_k = 1.

Equations 23-56 (S1G_LONG, uses p_{z(n)} with z(n)=n+2 for SU / n+3 for MU) and 23-57 (S1G_1M, uses p_{n+6}) exist — out of scope. [pp3804–3805]

### 6.1 Fully specialized Data-field equation (2 MHz, 1 SS, LGI, BCC, fixed pilots)

With N_STS = 1, Q_k = 1, Υ_{k,2} = 1, T_CS = 0, [P_HTLTF]_{1,1} = 1:

```
r_Data(t) = (1/√56) · Σ_{n=0}^{N_SYM−1} w_T40µs(t − n·40µs) ·
            Σ_{k=−28}^{28} ( D̃_{k,n} + p_{n+2}·P_n^k ) ·
            exp( j·2π·k·(31.25 kHz)·(t − n·40µs − 8µs) )
```

Per-symbol sample-domain recipe (native 2 Msps):
1. Build X[k], k = −28…28: put d_{M'_2(k),n} on the 52 data tones; put p_{n+2}·Ψ_{(n+j) mod 4} on pilots k ∈ {−21,−7,7,21} (j = 0,1,2,3 respectively); X[0] = 0; X[k] = 0 for |k| > 28.
2. 64-pt IFFT (tone k → bin k mod 64); scale by 1/√56. (Any IFFT normalization convention is fine as long as the resulting per-symbol power matches — the spec writes the analog sum; see Sanity Checks.)
3. Prepend the last 16 time samples as cyclic prefix → 80 samples.
4. Overlap-add consecutive symbols/fields with the w_T boundary treatment of §5.6 (or plain concatenation for the rectangular reference waveform).

---

## 7. Field-by-field TX timeline (S1G_SHORT, 2 MHz, 1 SS, LGI) — summary

| Field | Start (µs / samples@2Msps) | Duration | Structure (samples) | N_Tone (scale 1/√N) | GI |
|---|---|---|---|---|---|
| STF | 0 / 0 | 80 µs / 160 | 10 repetitions of 8 µs (16-sample) pattern; windowed as one 80 µs subfield | 12 | none |
| LTF1 | 80 / 160 | 80 µs / 160 | [32 GI2][64 LTS][64 LTS] | 56 | T_GI2 |
| SIG | 160 / 320 | 80 µs / 160 | 2 × ([16 GI][64]) | 52 | T_GI |
| Data | 240 / 480 | N_SYM × 40 µs | N_SYM × ([16 GI][64]) | 56 | T_GI (all symbols, LGI) |

(STF/LTF1/SIG content sequences are covered by the preamble digest; STF is defined via Eq 19-8 with Eq 23-14, LTF via 21.3.8.3.5 equations — cross-references noted at [23.3.8.2.2.3, p3768] and [23.3.8.2.2.4, p3770].)

---

## SANITY CHECKS (verified numerically)

1. **Subcarrier/FFT:** 64 × 31.25 kHz = 2.0 MHz exactly; T_DFT = 1/31.25 kHz = 32 µs. ✓
2. **N_ST = N_SD + N_SP:** Data: 52 + 4 = 56 ✓; SIG: 48 + 4 = 52 ✓ (matches Table 23-5/23-6 NOTE).
3. **Index ranges:** Data uses −28…28 minus {0} = 56 tones = N_ST ✓ (N_SR = 28 ✓). SIG uses −26…26 minus {0} = 52 = N_ST ✓ (N_SR = 26 ✓).
4. **M'_2(k) coverage:** the six ranges contain 7+13+6+6+13+7 = 52 subcarriers mapping bijectively onto data indices 0…51 (checked endpoints: M'(−28)=0, M'(−22)=6, M'(−20)=7, M'(−8)=19, M'(−6)=20, M'(−1)=25, M'(1)=26, M'(6)=31, M'(8)=32, M'(20)=44, M'(22)=45, M'(28)=51). ✓ Excluded k: {−21,−7,0,7,21} = pilots + DC. ✓
5. **Symbol durations:** T_SYML = 32 + 8 = 40 µs = 1.25·T_DFT ✓; T_SYMS = 32 + 4 = 36 µs ✓; T_GI = 32/4 = 8 µs ✓; T_GIS = 32/8 = 4 µs ✓; T_GI2 = 16 µs = 2·T_GI ✓.
6. **Field durations (2 MHz):** T_STF = 2·40 = 80 µs ✓; T_LTF1 = 2·32 + 16 = 80 µs ✓; T_SIG = 2·40 = 80 µs ✓. Sample counts at 2 Msps: 160/160/160; Data symbol 80. Preamble total = 240 µs = 480 samples ✓.
7. **N_CBPS = N_SD·N_BPSCS** for every valid 2 MHz/1SS MCS: 52·1=52, 52·2=104, 52·4=208, 52·6=312, 52·8=416, 52·10=520 ✓ (matches Table 23-46 column). **N_DBPS = N_CBPS·R**: e.g. 104·3/4=78, 312·5/6=260, 416·3/4=312, 520·3/4=390 ✓.
8. **Data rates:** N_DBPS/40 µs: 26→650 kb/s, 260→6500, 312→7800, 390→9750 ✓; N_DBPS/36 µs: 26→722.2, 390→10833.3 ✓ (matches both rate columns).
9. **p_n sequence length:** 127 elements, generated by 11a scrambler from all-ones state ✓ (counted from Eq 17-25 groups: 31×4 + 3 = 127).
10. **γ_{k,2} = +1 for all k** (Eq 23-6): no per-subcarrier rotation at 2 MHz ✓ (rotation first appears at CBW4).
11. **T_CS = 0 for 1 STS** (Table 23-10 row 1) ✓ — cyclic shift is a no-op in scope.
12. **P_HTLTF[1,1] = 1** (Eq 19-27) and g(n)=1 without STBC ⇒ pilot term = p_{n+2}·P_n^k ✓.
13. **Window discrete form:** T=40 µs at 2 Msps → 80 samples/symbol; boundary sample weight 0.5 mirrors Eq 17-5 (T=4 µs at 20 Msps → 80 samples, w[0]=w[80]=0.5) ✓ (same sample count, scaled 10×).

---

## Cross-reference resolution map

| Reference in Clause 23 | Resolved content | Where inlined |
|---|---|---|
| 17.3.2.5 (conventions, w_T) | Eq 17-4 window, T_TR ≈ 100 ns, overlap-add, Fig 17-2, Eq 17-5 discrete example, IFFT bin mapping (17.3.2.6) | §5.6 |
| 17.3.5.10 (p_n) | Eq 17-25 full 127-element polarity sequence | §6 |
| 19.3.9.4.6 / Eq 19-27 (P_HTLTF) | 4×4 matrix, verified PDF p3439 | §6 |
| 21.3.10.10 (P_n^k fixed pilots, 64-FFT) | pilots ±7, ±21; Ψ row rotation, Ψ = [1,1,1,−1] | §6 |
| 23.3.8.2.2.2 / Table 23-10 (T_CS) | 0 µs for 1 STS | §5.5 |
| 23.3.8.2.2.4 / Table 23-11 (N_LTF) | N_LTF = 1 for N_STS = 1 | §3 |
| 23.3.8.2.3.3.5 / Eq 23-30 (M'_2) | full piecewise map, verified PDF p3786 | §6 |
| 19.3.11.11.2 (Q_k forms) | direct-map Q = 1 for SISO | §5.3, §6 |
