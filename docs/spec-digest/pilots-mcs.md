# S1G PHY digest: Pilot subcarriers + 2 MHz MCS parameter tables

Scope: IEEE 802.11-2024, Clause 23 (S1G PHY). Baseline configuration for the worked examples: **2 MHz bandwidth, S1G_SHORT preamble, SU PPDU, NSS = NSTS = 1, no STBC, BCC, Long GI (8 us)** (the implementation also covers short GI, LDPC and S1G_LONG SU). All PDF page citations are the *printed* page number (PDF viewer page = printed page + 1; e.g. printed p3798 is PDF viewer page 3799).

---

## 1. Where pilots fit (context)

Pilot insertion is a step of the Data-field construction for every S1G format ("Pilot insertion: Insert pilots following the steps described in 23.3.9.10") [23.3.4.2, ~p3752-3755]. Pilots are added in the frequency domain *after* constellation mapping; data subcarriers skip the pilot positions (see Section 5.4 below). The SIG field of S1G_SHORT gets its pilots by a different (Clause-17 style) rule — see Section 4.

Timing/subcarrier constants needed here (Table 23-5) [23.3.6, p3756]:

| Param | CBW2 value | Meaning |
|---|---|---|
| N_SD | 52 | data subcarriers per OFDM symbol |
| N_SP | 4 | pilot subcarriers per OFDM symbol |
| N_ST | 56 | total used subcarriers (N_SD + N_SP) |
| N_SR | 28 | highest used subcarrier index (tones k = -28..+28, k=0 unused) |
| Delta_F | 31.25 kHz | subcarrier spacing |
| T_DFT | 32 us | IDFT/DFT period |
| T_GI | 8 us | long GI |
| T_GIS | 4 us | short GI |
| T_SYML | 40 us | long-GI symbol duration |
| T_SYMS | 36 us | short-GI symbol duration |

(For 1 MHz: N_SD=24, N_SP=2; 4 MHz: 108/6; 8 MHz: 234/8; 16 MHz: 468/16 — out of scope.)

---

## 2. Fixed pilots, 2 MHz Data field [23.3.9.10, p3798]

### 2.1 Positions

Four pilot tones at subcarrier indices:

```
K_Pilot_Fix = { -21, -7, +7, +21 }        // pilot index l = 0,1,2,3 in this order
```

For S1G_SHORT and S1G_LONG PPDUs with fixed pilots, the pilot mapping `P_n^k` "with same FFT sizes is identical to what is defined in 21.3.10.10" [23.3.9.10, p3798]. The S1G 2 MHz waveform uses a 64-point FFT, i.e. it is identical to the VHT/HT **20 MHz** pilot definition. (The 1 MHz two-pilot rule, Eq (23-50) with Psi from Table 21-21, applies only to S1G_1M — out of scope.)

### 2.2 Values (resolved cross-reference to 21.3.10.10 / Eq (21-91), p3629, and Table 19-19, p3452)

Equation (21-91) [21.3.10.10, p3629], specialized to NSTS = 1:

```
P_n^{{-21,-7,7,21}} = { Psi_(n mod 4), Psi_((n+1) mod 4), Psi_((n+2) mod 4), Psi_((n+3) mod 4) }
P_n^k = 0   for k not in {-21,-7,7,21}
```

where `Psi` is the NSTS = 1 row of Table 19-19 ("Pilot values for 20 MHz transmission") [19.3.11.10, p3452]:

```
Psi = [ 1, 1, 1, -1 ]        // Psi_0, Psi_1, Psi_2, Psi_3   (NSTS=1, iSTS=1 row)
```

So the pilot **pattern rotates cyclically one position per data symbol**. Written out (before the polarity multiply of Section 3):

| n mod 4 | k=-21 | k=-7 | k=+7 | k=+21 |
|---|---|---|---|---|
| 0 | +1 | +1 | +1 | -1 |
| 1 | +1 | +1 | -1 | +1 |
| 2 | +1 | -1 | +1 | +1 |
| 3 | -1 | +1 | +1 | +1 |

Code-ready:

```rust
const PSI: [i8; 4] = [1, 1, 1, -1];
const PILOT_POS: [i32; 4] = [-21, -7, 7, 21];   // pilot index l = 0..3
// value at pilot l for data symbol n (n = 0 at FIRST Data symbol):
// pilot_val(l, n) = p[n + 2] * PSI[(n + l) % 4]      (see Section 3 for p[] and the +2)
```

`n` here is the **data symbol index starting at 0 at the first Data-field symbol** (n = 0..N_SYM-1); the SIG symbols do NOT advance this rotation [Eq (23-55), 23.3.9.11, p3803: the pattern index inside `P_n^k` is the Data-field symbol counter n].

Note (sanity): at n = 0 this rotating pattern equals the Clause-17 fixed pilot values {1,1,1,-1} at the same four positions.

### 2.3 Multi-stream factor (why it disappears for SISO)

Eq (23-55) multiplies the pilot by `[P_HTLTF]_{m, g(n)}` where m is the space-time stream index and, for non-STBC/non-traveling-pilot PPDUs, `g(n) = 1` ("(n mod 2)+1 if the STBC bit and Traveling Pilots bit in SIG field are both set to 1; 1 otherwise") [23.3.9.11, p3803]. P_HTLTF is the HT LTF mapping matrix of 19.3.9.4.6, whose (1,1) entry is +1. **For NSTS = 1 this factor is exactly 1 — ignore it.**

---

## 3. Pilot polarity sequence p_n and index offsets

### 3.1 The sequence (resolved cross-reference to 17.3.5.10, Eq (17-25), p3366)

"The polarity of the pilot subcarriers is controlled by the sequence, p_n, which is a cyclic extension of the 127 elements sequence" [17.3.5.10, p3366]. It is the output of the Clause-17 scrambler (Figure 17-7, x^7+x^4+1) seeded with all 1s, with scrambler output 1 mapped to -1 and 0 mapped to +1. Full sequence, verified against the PDF:

```
p[0..127) = [
  1, 1, 1, 1,  -1,-1,-1, 1,  -1,-1,-1,-1,   1, 1,-1, 1,
 -1,-1, 1, 1,  -1, 1, 1,-1,   1, 1, 1, 1,   1, 1,-1, 1,
  1, 1,-1, 1,   1,-1,-1, 1,   1, 1,-1, 1,  -1,-1,-1, 1,
 -1, 1,-1,-1,   1,-1,-1, 1,   1, 1, 1, 1,  -1,-1, 1, 1,
 -1,-1, 1,-1,   1,-1, 1, 1,  -1,-1,-1, 1,   1,-1,-1,-1,
 -1, 1,-1,-1,   1,-1, 1, 1,   1, 1,-1, 1,  -1, 1,-1, 1,
 -1,-1,-1,-1,  -1, 1,-1, 1,   1,-1, 1,-1,   1, 1, 1,-1,
 -1, 1,-1,-1,  -1, 1, 1, 1,  -1,-1,-1,-1,  -1,-1,-1
]
// 127 elements; use cyclically: p_n = p[n mod 127]
```

### 3.2 Index conventions for S1G_SHORT (the "z offset")

- **SIG field (2 symbols, SIG-1 and SIG-2):** SIG symbol n (n = 0, 1) uses polarity **p_n** — i.e. p_0 for SIG-1, p_1 for SIG-2. The SIG waveform Eq (23-18) contains the term `(j*D_{k,n,2} + p_n * P_k)` and states "p_n and P_k are defined in 17.3.5.10" [23.3.8.2.2.5, Eq (23-18), p3773-3774]. So SIG pilots use the **Clause-17 fixed (non-rotating) pilot values** P_k (Eq (17-24), p3366):

  ```
  P_-21 = +1,  P_-7 = +1,  P_+7 = +1,  P_+21 = -1        // constant for both SIG symbols
  SIG pilot value at tone k, SIG symbol n:  p_n * P_k     // n = 0, 1
  ```

  (The `j` multiplies only the 48 BPSK *data* tones of each SIG symbol — QBPSK rotation for format detection; pilots are NOT rotated by j.) [23.3.8.2.2.5, p3771, p3774]

- **Data field:** data symbol n (n = 0..N_SYM-1) uses polarity **p_{n+2}**: Eq (23-55) pilot term is `[P_HTLTF]_{m,g(n)} * p_{n+2} * P_n^k` [23.3.9.11, p3803]. The offset **z = 2** accounts for the 2 SIG symbols having consumed p_0 and p_1, so the polarity index runs continuously across SIG + Data.

  ```
  data-field pilot at tone k, symbol n:  p[(n + 2) % 127] * P_n^k     // P_n^k from Section 2.2
  ```

- For reference (out of scope): S1G_1M Data uses p_{n+6} (its SIG is 6 symbols) [Eq (23-57), p3803]; S1G_LONG uses p_{z(n)} with a z(n) defined for its variable-length preamble [Eq (23-56), p3803].

### 3.3 Normalization

Pilot tones enter the OFDM modulation sum on the same footing as data tones; the whole symbol is scaled by `1/sqrt(N_Data^Tone * NSTS)` where `N_Data^Tone = 56` for a 2 MHz Data symbol (Table 23-8, "First Data Symbol"/"Second to last Data Symbols" row, 2 MHz column) [23.3.7, Table 23-8, p3763-3764; Eq (23-55), p3803]. There is no extra per-pilot scaling for fixed pilots — pilot amplitude is +/-1 like a BPSK data tone (traveling pilots differ: Section 5).

---

## 4. Which pilots go in which field (S1G_SHORT summary)

| Field | Pilot rule | Polarity index |
|---|---|---|
| STF / LTF1 / LTF2..LTFN | no pilots (whole-sequence training symbols) | — |
| SIG-1, SIG-2 | fixed Clause-17 pilots P_k = {1,1,1,-1} at {-21,-7,7,21} | p_0, p_1 |
| Data symbol n | rotating VHT-style P_n^k (Section 2.2), or traveling pilots if TP bit set (Section 5) | p_{n+2} |

SIG-field pilots are **always at the fixed positions** — the traveling-pilot mechanism is defined only "for data symbol n" [23.3.9.10, p3798].

---

## 5. Traveling pilots (S1G Doppler mode) [23.3.9.10, p3798-3802]

**Skippable when TRAVELING_PILOTS = 0.** The SIG-2 "Traveling Pilots" bit (B12) is "Set to 1 to indicate traveling pilots usage in PPDU. Otherwise 0 to indicate regular pilot tone locations" [Table 23-12, p3773]. With the bit 0, everything in this section is bypassed and Sections 2-3 fully describe the pilots. A minimal fixed-pilot-only implementation is spec-compliant on transmit (traveling pilots are a TXVECTOR option). Mechanism, briefly, for completeness:

### 5.1 Value rule, Eq (23-51) [p3798]

At data symbol n:

```
P_n^k = 1.5 * P_{n,fix}^{k_Pilot_Fix(l)}   if k == K_Pilot_Travel^(l)(n)   (for some l)
P_n^k = 0                                   otherwise
```

- `P_{n,fix}^k` is the fixed-pilot mapping of Section 2.2 (identical values).
- `k_Pilot_Fix(l)` for 2 MHz is `{-21,-7,7,21}` for l = 0..3 [p3798].
- I.e. the value that pilot index l *would* have carried on its fixed tone is transmitted instead on the traveling position for l, **scaled by 1.5 in amplitude**. Combined with polarity (Eq (23-55)): traveling pilot l at symbol n = `1.5 * p[(n+2)%127] * PSI[(n+l)%4]` at tone `K_Pilot_Travel^(l)(n)`.

### 5.2 Positions for NSTS = 1, 2 MHz — Table 23-23 [p3799], full transcription

Positions vary per symbol with pattern index `m(n) = n mod N_TP,BW`, Eq (23-52), with `N_TP,2MHz = 14` (1 MHz: 13; 4 MHz: 19; 8/16 MHz: 32) [p3799].

| l \ m | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 13 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0 | -28 | -24 | -20 | -16 | -26 | -22 | -18 | -27 | -23 | -19 | -15 | -25 | -21 | -17 |
| 1 | -12 | -8 | -4 | -2 | -14 | -10 | -6 | -11 | -7 | -3 | 1 | -13 | -9 | -5 |
| 2 | 4 | 8 | 12 | 16 | 2 | 6 | 10 | 5 | 9 | 13 | 17 | -1 | 3 | 7 |
| 3 | 20 | 24 | 28 | 26 | 14 | 18 | 22 | 21 | 25 | 23 | 27 | 11 | 15 | 19 |

(Defined only for: SU PPDU with NSTS = 1 [Tables 23-22..23-25], or NSTS = 2 with STBC = 1 [Tables 23-26..23-29, positions change every *other* symbol, m(n) = floor(n/2) mod N_TP with N_TP,2MHz = 7]. "For S1G MU PPDUs, or S1G SU PPDUs with more than two space-time streams, or S1G SU PPDUs with two space-time streams without STBC, traveling pilots are not defined." [p3801]. 16 MHz derives from the 8 MHz table +/-128 [Eq (23-54), p3801].)

### 5.3 Side effect on data mapping

The data-tone mapping skips the *current* pilot set: `D~_{k,m,n,BW} = 0 if k in K_Pilot(n), else d_{M'_BW(k),m,n}` with `K_Pilot(n) = K_Pilot_Fix` when the Traveling Pilot bit is 0, `= K_Pilot_Travel(n)` when it is 1 [23.3.9.11, "where" list after Eq (23-57), p3804]. So with traveling pilots, the 52 data tones occupy a per-symbol-varying set (fixed pilot tones carry data on symbols where pilots have traveled elsewhere). With TP = 0 the mapping is the constant one (data on all used tones except {-21,-7,7,21}).

### 5.4 Receiver-side note

Traveling pilots exist so a receiver can track channel variation (Doppler) across all tones. Nothing else in the Data-field pipeline changes. No midamble exists anywhere in Clause 23 (grep of the full clause: zero hits for "midamble") — S1G handles Doppler purely via traveling pilots.

---

## 6. S1G-MCS overview [23.3.5, p3756]

- The S1G-MCS is carried in the SIG field (S1G_SHORT: SIG-1 bits B19-B22, a 4-bit MCS index) and determines Data-field modulation and coding.
- Defined MCS indices are 0-9 generally, plus index 10 **solely for 1 MHz, NSS = 1** (BPSK rate-1/2 with 2x repetition — out of scope here), plus 11-12 (1024-QAM) where valid. "Equal modulation is applied to all streams for a particular user." [23.3.5, p3756]
- Support requirements [23.5, p3855-3856]:
  - 1 MHz and 2 MHz with NSS = 1: **mandatory**. NSS = 2..4 and wider bandwidths: optional.
  - MCS 8 and 9 (when valid): optional. MCS 11 and 12 (when valid): optional. 4 us GI: optional.
  - An S1G AP STA shall support 1SS MCS 0-7 for all supported channel widths; a non-AP S1G STA shall support 1SS MCS 0-2 for 1 and 2 MHz.
- There are **no other per-MCS restrictions** for 2 MHz/1SS (no MCS-dependent pilot or interleaver change; "Not valid" rows simply cannot be signaled).

---

## 7. Table 23-46 — S1G-MCSs for 2 MHz, NSS = 1 [23.5, p3857-3858] (verified against PDF)

Common to all rows: N_SD = 52, N_SP = 4, N_ES = 1 (single BCC encoder).

| MCS | Modulation | R | N_BPSCS | N_SD | N_SP | N_CBPS | N_DBPS | N_ES | Rate, 8 us GI (kb/s) | Rate, 4 us GI (kb/s) |
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
| 9 | *Not valid* | | | | | | | | | |
| 10 | *Not valid* | | | | | | | | | |
| 11 | 1024-QAM | 3/4 | 10 | 52 | 4 | 520 | 390 | 1 | 9750 | 10833.3 |
| 12 | *Not valid* | | | | | | | | | |

Key answers to scoping questions:

- **MCS 9 (256-QAM 5/6) is NOT valid for 2 MHz / 1SS** (416 x 5/6 = 346.67 is not an integer). Valid indices for 2 MHz 1SS are **0-8 and 11**; 9, 10, 12 are "Not valid" [Table 23-46, p3858].
- **MCS 10** (BPSK 1/2 with 2x repetition, N_DBPS = 6, 150 kb/s LGI) exists only in Table 23-42 (1 MHz, NSS = 1) [p3856] — out of scope.
- MCS 11 (1024-QAM 3/4) is valid but optional; a BCC 2MHz/1SS implementation covering MCS 0-8 satisfies all mandatory requirements.

Data-rate formula (for computing durations): `rate = N_DBPS / T_SYM`. With LGI, `rate_kbps = N_DBPS * 1000/40 = N_DBPS * 25`; with SGI, `rate_kbps = N_DBPS * 1000/36`.

For reference, the corresponding IDs of the neighbor tables (all same layout, not expanded): 23-42..45 = 1 MHz NSS 1..4; 23-47..49 = 2 MHz NSS 2..4; 23-50..61 = 4/8/16 MHz [23.5, p3856-3865].

---

## SANITY CHECKS (all verified numerically)

1. **N_CBPS = N_SD x N_BPSCS x NSS** for every valid 2 MHz 1SS MCS: 52x1=52, 52x2=104, 52x4=208, 52x6=312, 52x8=416, 52x10=520. Matches Table 23-46.
2. **N_DBPS = N_CBPS x R**: 52x1/2=26; 104x1/2=52; 104x3/4=78; 208x1/2=104; 208x3/4=156; 312x2/3=208; 312x3/4=234; 312x5/6=260; 416x3/4=312; 520x3/4=390. Matches. MCS9 would give 416x5/6=346.67 (non-integer) — consistent with its "Not valid" entry.
3. **LGI data rate = N_DBPS x 25 kb/s** (T_SYML = 40 us): 26 -> 650.0, 52 -> 1300.0, 78 -> 1950.0, 104 -> 2600.0, 156 -> 3900.0, 208 -> 5200.0, 234 -> 5850.0, 260 -> 6500.0, 312 -> 7800.0, 390 -> 9750. All match the "8 us GI" column exactly.
4. **SGI data rate = N_DBPS x 1000/36 kb/s** (T_SYMS = 36 us): 26 -> 722.2, 52 -> 1444.4, 78 -> 2166.7, 104 -> 2888.9, 156 -> 4333.3, 208 -> 5777.8, 234 -> 6500.0, 260 -> 7222.2, 312 -> 8666.7, 390 -> 10833.3. All match the "4 us GI" column.
5. **N_ST = N_SD + N_SP**: 52 + 4 = 56 = N_ST (Table 23-5), = N_Data^Tone (Table 23-8). Tones span -28..+28 excluding DC: 56 used of 57 slots, N_SR = 28. Consistent.
6. **Polarity sequence**: 127 elements (counted: 4 rows of 32+32+32+31 in Eq (17-25)); first 8 = {1,1,1,1,-1,-1,-1,1}, last 3 = {-1,-1,-1}; matches scrambler(seed=all-ones) output with 1->-1, 0->+1 mapping.
7. **Pilot pattern consistency**: rotating VHT pattern at n=0 = {1,1,1,-1} at {-21,-7,7,21} = Clause-17 fixed P_k values at the same positions (Eq (17-24)), so SIG-to-Data pilot handoff is value-continuous at n=0 apart from polarity p_1 -> p_2.
8. **Table 23-23 (traveling positions)**: each of the 14 columns has 4 strictly increasing entries; all 56 entries lie in [-28, 28]\{0}; each column's entries fall in disjoint bands per pilot index (l=0: [-28,-15], l=1: [-14,1], l=2: [-1,17], l=3: [11,28]). Union over all 14 columns covers 56 distinct tones = every used tone exactly once per period (each tone is a pilot exactly once per 14-symbol cycle).
9. **Traveling pilot period**: N_TP,2MHz = 14 = 56 tones / 4 pilots — consistent with check 8.
