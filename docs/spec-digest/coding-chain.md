# S1G Data-Field Bit-Level Processing Chain — BCC, 2 MHz, 1 SS, SU, Long GI

Digest of IEEE 802.11-2024 covering everything between "PSDU bits in hand" and "stream of
complex constellation points" for an S1G (802.11ah) SU PPDU, 2 MHz bandwidth, 1 spatial
stream, BCC coding, no STBC, S1G_SHORT preamble. All cross-references into Clauses 17, 19,
21 and 27 have been resolved and inlined. Page numbers refer to **PDF page numbers** of the
IEEE 802.11-2024 PDF (printed page = PDF page − 1). Every numeric table/figure below was
verified by visually rendering the PDF page cited.

Out of scope here (other digests): SIG field construction, pilot insertion, OFDM
modulation/IDFT, GI/windowing, preamble. LDPC, MU, STBC, 16 MHz segment parsing are noted
where the spec mentions them but not expanded.

---

## 1. Processing chain order [23.3.4.6.1, pp3753–3754]

For an S1G SU PPDU with BCC encoding, the Data field is constructed in this exact order:

1. **SERVICE field**: generate SERVICE (8 bits, all zero) and append the PSDU after it
   [23.3.4.6.1 step a, p3753].
2. **PHY padding**: append `N_PAD` pad bits after the PSDU (values arbitrary, 0 or 1)
   [step b, p3753; 23.3.9.4.3.2, p3795].
3. **Scrambler**: scramble SERVICE + PSDU + pad bits [step c, p3753; 23.3.9.3, p3794].
4. **Tail bits**: append `N_tail × N_ES = 6` **unscrambled zero** tail bits after the
   scrambled bits [23.3.9.4.3.2, p3795].
5. **BCC encoder**: encode with the rate-1/2 K=7 convolutional code, then puncture to the
   MCS rate. `N_ES = 1` for every valid 2 MHz / 1 SS MCS, so there is exactly one encoder
   and the round-robin encoder parsing is the identity [step d, p3753; 23.3.9.4.2, p3795;
   21.3.10.5.2, p3614].
6. **Stream parser**: rearrange encoder output into `N_SS` streams — identity for 1 SS
   [step e, p3753; 23.3.9.6, p3797; 21.3.10.6, pp3616–3617].
7. **Segment parser**: bypassed for 2 MHz (only used for contiguous 16 MHz)
   [step f, p3753].
8. **BCC interleaver**: per-OFDM-symbol block interleaver, 2 MHz uses the VHT 20 MHz
   interleaver [step g, p3753; 23.3.9.8, p3798; 21.3.10.8, pp3619–3621].
9. **Constellation mapper**: map groups of `N_BPSCS` bits to complex points
   [step h, p3754; 23.3.9.9.1, p3798].
10. (Segment deparser: bypassed for 2 MHz. STBC: not used. Pilot insertion, CSD, spatial
    mapping (Q = 1 for 1 SS), IDFT, GI — out of scope here) [steps i–q, p3754].

With long GI (`GI_TYPE = LONG_GI`) the Data-symbol GI is 8 µs for **all** symbols; the data
symbol duration is `T_SYML = 40 µs` [23.3.4.6.1 step p, p3754; Table 23-5, p3758].

---

## 2. Rate-dependent parameters — Table 23-46, S1G-MCSs for 2 MHz, N_SS = 1 [23.5, Table 23-46, pp3858–3859; verified against PDF]

| MCS | Modulation | R   | N_BPSCS | N_SD | N_SP | N_CBPS | N_DBPS | N_ES | Data rate, 8 µs GI (kb/s) | Data rate, 4 µs GI (kb/s) |
|-----|-----------|-----|---------|------|------|--------|--------|------|------------|------------|
| 0   | BPSK      | 1/2 | 1       | 52   | 4    | 52     | 26     | 1    | 650.0      | 722.2      |
| 1   | QPSK      | 1/2 | 2       | 52   | 4    | 104    | 52     | 1    | 1300.0     | 1444.4     |
| 2   | QPSK      | 3/4 | 2       | 52   | 4    | 104    | 78     | 1    | 1950.0     | 2166.7     |
| 3   | 16-QAM    | 1/2 | 4       | 52   | 4    | 208    | 104    | 1    | 2600.0     | 2888.9     |
| 4   | 16-QAM    | 3/4 | 4       | 52   | 4    | 208    | 156    | 1    | 3900.0     | 4333.3     |
| 5   | 64-QAM    | 2/3 | 6       | 52   | 4    | 312    | 208    | 1    | 5200.0     | 5777.8     |
| 6   | 64-QAM    | 3/4 | 6       | 52   | 4    | 312    | 234    | 1    | 5850.0     | 6500.0     |
| 7   | 64-QAM    | 5/6 | 6       | 52   | 4    | 312    | 260    | 1    | 6500.0     | 7222.2     |
| 8   | 256-QAM   | 3/4 | 8       | 52   | 4    | 416    | 312    | 1    | 7800.0     | 8666.7     |
| 9   | Not valid | —   | —       | —    | —    | —      | —      | —    | —          | —          |
| 10  | Not valid (1 MHz only) | | |     |      |        |        |      |            |            |
| 11  | 1024-QAM  | 3/4 | 10      | 52   | 4    | 520    | 390    | 1    | 9750       | 10833.3    |
| 12  | Not valid | —   | —       | —    | —    | —      | —      | —    | —          | —          |

Notes:
- `N_SD` = 52 data subcarriers, `N_SP` = 4 pilot subcarriers, `N_ST = N_SD + N_SP = 56`
  [Table 23-5 NOTE, p3758].
- MCS 9 is not valid at 2 MHz / 1 SS (as at VHT 20 MHz / 1 SS). MCS 10 exists only for
  1 MHz (BPSK, R=1/2, 2× repetition) [23.3.9.5, p3797 — out of scope]. MCS 11 (1024-QAM)
  is valid; MCS 12 (1024-QAM 5/6) is not valid at 2 MHz / 1 SS.
- Support for MCS 8/9 (256-QAM) is optional per 23.5 general text [23.5, p3856]; MCS 11
  likewise optional. Support for BCC **reception** is mandatory [23.3.9.4.1, p3795].
- Other bandwidths exist (1/4/8/16 MHz, Tables 23-42…23-61) — not covered here.

Fixed constants for the chain [Table 23-5, p3758]:
```
N_service = 8      // bits in the SERVICE field (S1G; NOT 16 as in 11a/HT/VHT!)
N_tail    = 6      // tail bits per BCC encoder
T_SYML    = 40 us  // long-GI data symbol duration (32 us DFT + 8 us GI)
```

---

## 3. SERVICE field [23.3.9.2 + Table 23-20, p3794; verified against PDF]

The S1G SERVICE field has **8 bits**, denoted bits 0–7; **bit 0 is transmitted first in
time** (it is the first bit into the scrambler and the encoder).

| Bits  | Field                    | Value |
|-------|--------------------------|-------|
| B0–B6 | Scrambler Initialization | Set to 0 |
| B7    | Reserved                 | Set to 0 on transmit; ignored on receive |

So the SERVICE field is simply **8 zero bits** prepended to the PSDU. There is **no CRC**
and **no explicit seed field** in SERVICE: because B0–B6 are zero before scrambling, the
first 7 *scrambled* bits on air equal the first 7 bits of the scrambling sequence, which is
how the receiver learns the scrambler initialization (XOR with zero is transparent). The
RXVECTOR parameter SCRAMBLER_OR_CRC returns exactly these 7 bits `[B0:B6]` of the SERVICE
field prior to descrambling [Table 23-1 TXVECTOR/RXVECTOR, p3740: "SCRAMBLER_OR_CRC ...
Bit sequence of 7 bits in length: [B0:B6] of the SERVICE field value prior to
descrambling"]. (This differs from Clause 17, where SERVICE is 16 bits with 9
reserved bits — do not reuse 11a code as-is.)

---

## 4. Number of symbols and PHY padding [23.3.9.4.3.2, p3795; 23.4.3 Eq (23-79)/(23-80), p3854; verified against PDF]

### 4.1 N_SYM (BCC, SU)

With `m_STBC = 1` (no STBC — m_STBC is 2 only when STBC is used) [p3854]:

- Aggregation subfield = 1 (A-MPDU): Equation (23-79):

  `N_SYM = m_STBC * ceil( (8*APEP_LENGTH + N_service + N_tail*N_ES) / (m_STBC * N_DBPS) )`

- Aggregation subfield = 0: Equation (23-80):

  `N_SYM = m_STBC * ceil( (8*PSDU_LENGTH + N_service + N_tail*N_ES) / (m_STBC * N_DBPS) )`

Specialized for 2 MHz / 1 SS / no STBC / BCC (`N_ES = 1`, `N_service = 8`, `N_tail = 6`):

```
N_SYM = ceil( (8*PSDU_LENGTH + 14) / N_DBPS )     // non-aggregated SU
```

For an NDP there is no Data field and `N_SYM = 0` [p3854]. For LDPC, N_SYM comes from
23.3.9.4.4 instead (not expanded) [p3854].

### 4.2 N_PAD [23.3.9.4.3.2, p3795]

```
N_PAD = N_SYM * N_DBPS - 8*PSDU_LENGTH - N_service - N_tail*N_ES
      = N_SYM * N_DBPS - 8*PSDU_LENGTH - 14          // 2 MHz / 1 SS specialization
```
(For SU there is a single user: `N_PAD = N_PAD,0`.)

Padding flow [p3795]:
- **Aggregation = 1**: the MAC delivers a PSDU filling the available octets (max octets ≤
  N_PAD,u budget); the PHY appends only `N_PAD mod 8` bits (< 8 bits). The SIG Length
  subfield indicates `N_SYM` data symbols.
- **Aggregation = 0**: MAC does no padding; the PHY appends all `N_PAD` bits. The SIG
  Length field indicates `PSDU_LENGTH` in octets.
- Pad bit values are arbitrary ("could be either 0 or 1") — implementers normally use 0.
- Pad bits **are scrambled** (they precede the tail): "Both the PSDU and the PHY padding
  bits are scrambled and finally the 6·N_ES zero tail bits are appended after the
  scrambled PSDU and PHY padding bits" [p3795].

Total bit budget identity (must hold exactly):
```
N_SYM * N_DBPS = N_service + 8*PSDU_LENGTH + N_PAD + N_tail*N_ES
               = 8 + 8*PSDU_LENGTH + N_PAD + 6
```

---

## 5. Scrambler [23.3.9.3, p3794 → 17.3.5.5, pp3355–3357]

The SERVICE, PSDU and PHY-pad parts (NOT the tail bits) are scrambled by the length-127
frame-synchronous scrambler of 17.3.5.5, generator polynomial:

```
S(x) = x^7 + x^4 + 1        // Equation (17-14), p3355
```

**Octet-to-bit order**: octets of the PSDU enter the serial bit stream **bit 0 (LSB)
first, bit 7 last** [17.3.5.5, p3355].

**Scrambling operation**: `scrambled[i] = data[i] XOR s[i]` where `s[i]` is the scrambling
sequence. The same device descrambles. The sequence satisfies the recurrence (from Figure
17-7, feedback = x7 XOR x4, p3355):

```rust
// seed = 7-bit nonzero value; s[0..7) = seed bits (s[0] transmitted first)
// thereafter: s[i] = s[i-4] ^ s[i-7]
fn scramble(seed7: [u1;7], data: &[u1]) -> Vec<u1> {
    let mut s = seed7.to_vec();
    (0..data.len()).map(|i| {
        if i >= 7 { s.push(s[i-4] ^ s[i-7]); }
        data[i] ^ s[i]
    }).collect()
}
```

Equivalent LFSR form (Figure 17-7, p3355): 7-bit register X7..X1; each step outputs
`x = X7 ^ X4`, shifts X6→X7 … X1→X2, X1 ← x. During the first 7 output bits the register is
arranged so the outputs equal the chosen 7 seed bits (Table 17-7 "first 7 bits of the
scrambling sequence").

**Self-check** [NOTE 1, 17.3.5.5, p3355]: with the 7-bit init value 112 (binary 1110000,
i.e., first-7-bits LSB-first = 0,0,0,0,1,1,1) the repeating 127-bit sequence starts
(leftmost used first):
```
00001110 11110010 11001001 00000010 00100110 00101110 10110110 00001100
11010100 11100111 10110100 00101010 11111010 01010001 10111000 1111111
```

**Seed selection for S1G**: Clause 23 gives no bandwidth-in-scrambler signaling; per
Table 17-7 the applicable row is "CH_BANDWIDTH_IN_NON_HT and SCRAMBLER_INITIAL_VALUE are
not present → **7-bit pseudorandom nonzero integer**" [Table 17-7, pp3356–3357]. So: pick a
random value in 1..=127 per PPDU. The receiver recovers it as the first 7 descrambler-input
bits (since SERVICE B0–B6 = 0, see §3), then runs the recurrence forward to descramble the
rest. Fields carried in the scrambling sequence are interpreted **LSB first** [p3356].

**Tail-bit zeroing**: In S1G the 6 zero tail bits are simply appended *after* scrambling
[23.3.9.4.3.2, p3795], so no "reset/overwrite scrambled tail bits to zero" step (as in
17.3.5.3) is needed — the encoder input tail is guaranteed zero, which flushes the BCC
encoder back to the zero state.

---

## 6. BCC encoder parsing [23.3.9.4.2, p3795 → 21.3.10.5.2, p3614]

Scrambled bits are distributed round-robin over `N_ES` encoders; per Equation (21-60), bit
`i` of encoder `j` is `x_i^(j) = b_(N_ES*i + j)` for
`0 ≤ i < N_SYM*N_DBPS/N_ES − N_tail`, and zero tail bits fill each encoder's last `N_tail`
positions [21.3.10.5.2, p3614].

**2 MHz / 1 SS specialization: `N_ES = 1` for all valid MCS ⇒ the parser is the
identity.** The single encoder input is exactly:

```
enc_in[0 .. N_SYM*N_DBPS) =
    scrambled( SERVICE[8] || PSDU[8*PSDU_LENGTH] || PAD[N_PAD] ) || 0,0,0,0,0,0
```
(`N_SYM*N_DBPS` input bits total; after rate-1/2 encoding and puncturing to rate R this
becomes `N_SYM*N_CBPS` coded bits — see below.)

---

## 7. BCC coding and puncturing [23.3.9.4.3.1, p3795 → 21.3.10.5.3, p3614 → 17.3.5.6, pp3361–3362; rate 5/6: 19.3.11.6, p3444; all figures verified against PDF]

### 7.1 Mother code (rate 1/2, K = 7) [17.3.5.6, Figure 17-8, p3361]

Industry-standard generator polynomials:

```
g0 = 133 octal = 1011011b   ->  A output
g1 = 171 octal = 1111001b   ->  B output
```

Constraint length 7 (6 delay elements). The encoder **state starts all-zero** at the
beginning of the Data field, and the 6 zero tail bits return it to the all-zero state at
the end (the encoder input itself is scrambled data — only the state is zero).
Output order: **A before B** for each input bit [17.3.5.6, p3361].

```rust
// state = 6 previous input bits, newest in LSB position shown explicitly:
let mut sh: u8 = 0; // bits [x-1 .. x-6]
for x in input {
    let v = (x << 6) | sh;              // 7 bits: x, x-1, ..., x-6
    let a = popcount(v & 0b1011011) & 1; // g0 = 133o (taps: x, x-2, x-3, x-5, x-6)
    let b = popcount(v & 0b1111001) & 1; // g1 = 171o (taps: x, x-1, x-2, x-3, x-6)
    emit(a); emit(b);
    sh = (v >> 1) & 0x3F;
}
```
(Tap sanity: 133₈ = 1_011_011₂ → taps at delays {0,2,3,5,6}; 171₈ = 1_111_001₂ → delays
{0,1,2,3,6}.)

Decoding by the Viterbi algorithm is recommended [p3361]. Punctured (omitted) bits are
replaced at the receiver by dummy "zero metric" insertions [p3361].

### 7.2 Puncturing patterns

Let `A_k, B_k` be the two encoder outputs for input bit `k`. Transmitted serial order is
the surviving bits, in `A_0 B_0 A_1 B_1 ...` order with stolen bits deleted.

**Rate 1/2 (MCS 0, 1, 3)** — no puncturing. Sent: `A0 B0 A1 B1 A2 B2 ...`

**Rate 2/3 (MCS 5)** [Figure 17-9, p3362] — period 2 input bits, keep 3 of 4:
```
matrix (1 = keep):   A: 1 1      period k mod 2 = 0,1
                     B: 1 0
sent: A0 B0 A1 | A2 B2 A3 | ...       (steal B1, B3, B5, ...)
```

**Rate 3/4 (MCS 2, 4, 6, 8, 11)** [Figure 17-9, p3362] — period 3 input bits, keep 4 of 6:
```
matrix:              A: 1 1 0
                     B: 1 0 1
sent: A0 B0 A1 B2 | A3 B3 A4 B5 | A6 B6 A7 B8 | ...   (steal A2, B1 per period, i.e.
      stolen bits: A2, A5, A8, ..., B1, B4, B7, ...)
```

**Rate 5/6 (MCS 7)** [21.3.10.5.3, p3614 → 19.3.11.6, Figure 19-11, p3444] — period 5
input bits, keep 6 of 10:
```
matrix:              A: 1 1 0 1 0
                     B: 1 0 1 0 1
sent: A0 B0 A1 B2 A3 B4 | A5 B5 A6 B7 A8 B9 | ...     (steal A2, A4, B1, B3 per period)
```

```rust
// Generic puncture: keep[2*k_mod] for A_k, keep[2*k_mod+1] for B_k over period P
const PUNCT: &[(usize, &[u8])] = &[
    // (period P, keep pattern interleaved [A0,B0,A1,B1,...])
    /* R=1/2 */ (1, &[1,1]),
    /* R=2/3 */ (2, &[1,1, 1,0]),
    /* R=3/4 */ (3, &[1,1, 1,0, 0,1]),
    /* R=5/6 */ (5, &[1,1, 1,0, 0,1, 1,0, 0,1]),
];
```

Bit-count check: per OFDM symbol, `N_DBPS` encoder-input bits → `2*N_DBPS` mother-code
output bits → `N_CBPS = N_DBPS / R` bits after puncturing.

### 7.3 LDPC

LDPC is the alternative coder [23.3.9.4.4, pp3796–3797; 19.3.11.7]. Not expanded here
(out of scope). The BCC interleaver and everything in §9 is **bypassed** for LDPC
[21.3.10.8, p3619]; LDPC uses a tone mapper instead [23.3.9.9.2, p3798].

---

## 8. Stream parser [23.3.9.6, p3797 → 21.3.10.6, pp3616–3617; verified against PDF]

General VHT rule: bits are taken from the encoders in blocks of `s = max(1, N_BPSCS/2)`
bits and dealt round-robin to the `N_SS` spatial streams (Equations (21-68)–(21-72),
pp3616–3617).

**1 SS / 1 encoder specialization: identity.** With `N_SS = 1`, `N_ES = 1`:
`S = s`, every block goes to the single stream, and Eq (21-71)/(21-72) reduce to
`j = 0`, `k = i`. The stream-parser output equals the encoder output, processed in groups
of `N_CBPS` bits per OFDM symbol (`N_CBPSS = N_CBPS` for 1 SS).

(For S1G the same stream parser supports up to 4 streams [23.3.9.6, p3797].)

---

## 9. BCC interleaver [23.3.9.8, p3798 → 21.3.10.8, pp3619–3621; verified against PDF]

S1G 2 MHz uses **the VHT 20 MHz interleaver** ("The BCC interleavers for S1G 2 MHz, 4 MHz,
8 MHz, and 16 MHz PPDUs are the same as those defined for 20 MHz, 40 MHz, 80 MHz, and
160 MHz PPDUs, respectively, as specified in 21.3.10.8") [23.3.9.8, p3798]. (1 MHz has its
own Table 23-21: N_COL=8, N_ROW=3×N_BPSCS, N_ROT=2 — not used here [p3798].)

Interleaving operates independently on each OFDM symbol's block of
`N_CBPSSI = N_CBPSS = N_CBPS` bits (no segment parser at 2 MHz, 1 SS)
[21.3.10.8, p3619].

Parameters [Table 21-17, p3619] specialized to 20 MHz / 2 MHz column:

```
N_COL = 13
N_ROW = 4 * N_BPSCS
N_ROT = 11        // (N_SS <= 4; irrelevant for 1 SS — rotation is skipped)
```

| MCS (2 MHz/1SS) | N_BPSCS | N_ROW | N_CBPS (=N_COL*N_ROW) | s = max(1, N_BPSCS/2) |
|-----------------|---------|-------|------------------------|------------------------|
| 0               | 1       | 4     | 52                     | 1                      |
| 1, 2            | 2       | 8     | 104                    | 1                      |
| 3, 4            | 4       | 16    | 208                    | 2                      |
| 5, 6, 7         | 6       | 24    | 312                    | 3                      |
| 8               | 8       | 32    | 416                    | 4                      |
| 11              | 10      | 40    | 520                    | 5                      |

Three permutations; **for 1 SS only the first two apply** (frequency rotation, Eq (21-78),
applies only when `2 ≤ N_SS ≤ 4`; for `N_SS = 1` the third operation is skipped — the
deinterleaver text states "When N_SS = 1, this reversal is performed by j = r")
[pp3620–3621].

**First permutation** — Equation (21-76) [p3619] (write row-wise, read column-wise):
```
i = N_ROW * (k mod N_COL) + floor(k / N_COL)          k = 0 .. N_CBPS-1
```

**Second permutation** — Equation (21-77) [p3620] (rotate bit positions within each
constellation axis group of s bits):
```
j = s * floor(i / s)
  + ( i + N_CBPS - floor(N_COL * i / N_CBPS) ) mod s   i = 0 .. N_CBPS-1
```
with `s = max(1, N_BPSCS / 2)` (Eq (21-68), p3617). Note: for N_BPSCS ∈ {1,2}, s = 1 and
the second permutation is the identity.

Combined forward interleaver (bit at input index k lands at output index j):
```rust
fn interleave(block_in: &[u1], n_bpscs: usize) -> Vec<u1> {
    let n_col = 13; let n_row = 4*n_bpscs; let n_cbps = n_col*n_row;
    let s = core::cmp::max(1, n_bpscs/2);
    let mut out = vec![0; n_cbps];
    for k in 0..n_cbps {
        let i = n_row*(k % n_col) + k/n_col;
        let j = s*(i/s) + (i + n_cbps - (n_col*i)/n_cbps) % s;
        out[j] = block_in[k];
    }
    out
}
```

Deinterleaver (receiver; r = received index) [Eqs (21-82), (21-83), p3621]:
```
i = s * floor(r / s) + ( r + floor(N_COL * r / N_CBPS) ) mod s     r = 0 .. N_CBPS-1
k = N_COL * i - (N_CBPS - 1) * floor(i / N_ROW)                    i = 0 .. N_CBPS-1
```

(For completeness, the 2 ≤ N_SS ≤ 4 frequency rotation, Eq (21-78) [p3620], is
`r = { j - [ (2*(i_SS-1)) mod 3 + 3*floor((i_SS-1)/3) ] * N_ROT * N_BPSCS } mod N_CBPSSI`
— NOT applied for 1 SS.)

---

## 10. Constellation mapping [23.3.9.9.1, p3798 → 21.3.10.9.1, pp3621–3626 → 17.3.5.8, pp3364–3366; MCS 11 → 27.3.12.9, pp4279–4281; all verified against PDF]

S1G MCS 0–9 use the VHT (21.3.10.9.1) mappings with the same MCS indices; MCS 11/12 use
the HE 1024-QAM mapping of 27.3.12.9 [23.3.9.9.1, p3798].

Interleaved bits are consumed **in order** in groups of `N_BPSCS`; within each group the
**first bit in the stream is B0** ("with the input bit, B0, being the earliest in the
stream") [17.3.5.8, p3363]. Bit-string convention for 256-QAM/1024-QAM figures follows the
same rule [21.3.10.9.1, p3621; 27.3.12.9, p4281].

Output value: `d = (I + jQ) * K_MOD` [Eq (17-20), p3365].

| Modulation | Bits/axis | I from      | Q from      | K_MOD          |
|------------|-----------|-------------|-------------|----------------|
| BPSK       | —         | B0          | (Q = 0)     | 1              |
| QPSK       | 1         | B0          | B1          | 1/sqrt(2)      |
| 16-QAM     | 2         | B0B1        | B2B3        | 1/sqrt(10)     |
| 64-QAM     | 3         | B0B1B2      | B3B4B5      | 1/sqrt(42)     |
| 256-QAM    | 4         | B0B1B2B3    | B4B5B6B7    | 1/sqrt(170)    |
| 1024-QAM   | 5         | B0B1B2B3B4  | B5B6B7B8B9  | 1/sqrt(682)    |

[Tables 17-14…17-18, pp3365–3366; K_MOD 256-QAM: p3626; K_MOD 1024-QAM: p4281.]

### 10.1 Per-axis Gray mappings (verified bit-for-bit against PDF figures)

**BPSK** [Table 17-15, p3365]: `B0=0 → I=-1; B0=1 → I=+1; Q=0` always.
(Data-field BPSK for S1G is plain BPSK on the I axis — no QBPSK rotation in the Data
field.)

**QPSK** [Table 17-16, p3365]: per axis, `0 → -1, 1 → +1`.

**16-QAM** [Table 17-17, p3366], axis bits (b_first b_second):
```
00 → -3     01 → -1     11 → +1     10 → +3
```

**64-QAM** [Table 17-18, p3366], axis bits (b_first b_mid b_last):
```
000 → -7   001 → -5   011 → -3   010 → -1
110 → +1   111 → +3   101 → +5   100 → +7
```

**256-QAM** [Figures 21-24…21-27, pp3622–3625], axis bits B0B1B2B3 (I) / B4B5B6B7 (Q):
```
0000 → -15  0001 → -13  0011 → -11  0010 → -9
0110 → -7   0111 → -5   0101 → -3   0100 → -1
1100 → +1   1101 → +3   1111 → +5   1110 → +7
1010 → +9   1011 → +11  1001 → +13  1000 → +15
```
(Verified: 1st-quadrant figure p3622 — e.g. point (1,1) = 11001100, (15,15) = 10001000;
2nd-quadrant p3623 — (-15,15) = 00001000, (-1,1) = 01001100.)

**1024-QAM** [Figures 27-38…27-41, pp4279–4281], axis bits B0..B4 (I) / B5..B9 (Q):
same reflected-Gray construction over 32 levels {-31,...,+31}. Verified from the
1st-quadrant figure (header "b0 b1 b2 b3 b4 / b5 b6 b7 b8 b9"; point (1,1) has both axis
groups = 11000; (1,9): Q group = 11110).

**Closed form covering every axis size** (M levels per axis, m = bits/axis; matches all
the tables/figures above):
```rust
// bits[0] = first (earliest) bit of the axis group; returns odd level in [-(M-1), +(M-1)]
fn axis_level(bits: &[u1]) -> i32 {
    let m = bits.len();                       // 1..=5
    let g: u32 = bits.iter().fold(0, |acc, &b| (acc << 1) | b as u32); // B_first = MSB
    // inverse Gray: n = g ^ (g>>1) ^ (g>>2) ^ ... (prefix XOR from the MSB down)
    let mut n = g;
    let mut sh = 1;
    while sh < m { n ^= n >> sh; sh <<= 1; }
    2 * (n as i32) - ((1 << m) - 1)
}
// Forward direction: gray(n) = n ^ (n >> 1) maps index n (0..M-1, MSB = B_first)
// to the axis bit pattern, with level = 2n - (M-1).
// Example m=4: n=8 -> gray(8) = 1100 -> level 2*8-15 = +1.  ✓
```

### 10.2 Output stream

For each OFDM symbol `n` (0…N_SYM−1) the mapper emits `N_SD = 52` complex numbers
`d_{k,i,n}`, `k = 0…N_SD−1`, in the order the bit groups were consumed [Eq (21-84), p3626
via 23.3.9.9.1, p3798]. `k` indexes **data** subcarriers only; the mapping of `k` to
physical subcarrier indices (±1..±28 minus pilots at ±7, ±21) happens at pilot
insertion/OFDM modulation [23.3.9.10 ff, p3799 — out of scope here].

---

## 11. End-to-end pseudocode (2 MHz / 1 SS / BCC / SU / no aggregation)

```text
inputs: psdu[PSDU_LENGTH octets], mcs, seed (1..=127 random)
params: (R, N_BPSCS, N_CBPS, N_DBPS) from Table 23-46;  N_service=8, N_tail=6, N_ES=1

1  N_SYM  = ceil((8*PSDU_LENGTH + 14) / N_DBPS)                         [Eq 23-80]
2  N_PAD  = N_SYM*N_DBPS - 8*PSDU_LENGTH - 14                           [23.3.9.4.3.2]
3  bits   = [0u1; 8]                                  // SERVICE, B0 first [23.3.9.2]
           ++ psdu octets LSB-first                                     [17.3.5.5]
           ++ [0u1; N_PAD]                            // pad (any value) [23.3.9.4.3.2]
4  sbits  = scramble(seed, bits)                                        [17.3.5.5]
5  ebits  = sbits ++ [0u1; 6]                         // zero tail       [23.3.9.4.3.2]
6  coded  = puncture(R, conv_encode_K7_133_171(ebits))                  [17.3.5.6, 19.3.11.6]
   assert coded.len() == N_SYM * N_CBPS
7  for n in 0..N_SYM:
       blk  = coded[n*N_CBPS .. (n+1)*N_CBPS]         // stream parser = identity [21.3.10.6]
       ilv  = interleave(blk, N_BPSCS)                // NCOL=13, NROW=4*N_BPSCS  [21.3.10.8]
       for k in 0..52:
           group = ilv[k*N_BPSCS .. (k+1)*N_BPSCS]    // B0 = first bit  [17.3.5.8]
           d[k][n] = K_MOD * ( axis(I bits) + j*axis(Q bits) )          [17.3.5.8 etc.]
```

---

## SANITY CHECKS (numeric identities verified)

1. `N_CBPS = N_SD * N_BPSCS` for every row of Table 23-46:
   52×1=52, 52×2=104, 52×4=208, 52×6=312, 52×8=416, 52×10=520. ✓ [pp3858–3859]
2. `N_DBPS = N_CBPS * R`: 52×1/2=26, 104×1/2=52, 104×3/4=78, 208×1/2=104, 208×3/4=156,
   312×2/3=208, 312×3/4=234, 312×5/6=260, 416×3/4=312, 520×3/4=390. ✓ [pp3858–3859]
3. `N_COL * N_ROW = 13 * 4*N_BPSCS = 52*N_BPSCS = N_CBPS` — interleaver block exactly
   covers one OFDM symbol. ✓ [Table 21-17, p3619 + Table 23-46]
4. Data rate check (long GI, T_SYML = 40 µs): rate = N_DBPS / 40 µs.
   MCS0: 26/40e-6 = 650 kb/s ✓; MCS7: 260/40e-6 = 6500 kb/s ✓; MCS8: 312/40e-6 =
   7800 kb/s ✓; MCS11: 390/40e-6 = 9750 kb/s ✓ (matches Table 23-46 column "8 µs GI").
5. Scrambler recurrence `s[i] = s[i-4] ^ s[i-7]` reproduces NOTE 1's 127-bit sequence for
   seed 112: s0..s6 = 0000111 → s7 = s3^s0 = 0 ("00001110"), s8 = s4^s1 = 1
   ("...1111..."). ✓ [p3355]
6. Generator polys: 133₈ = 1011011₂ (7 taps bits), 171₈ = 1111001₂ — both have the
   x^0 and x^6 taps set, K = 7. ✓ [p3361]
7. Puncture rates: 2/3 keeps 3 of 4 (2 in→3 out ✓); 3/4 keeps 4 of 6 (3 in→4 out ✓);
   5/6 keeps 6 of 10 (5 in→6 out ✓). Figures 17-9/19-11 stolen-bit sets
   {B1}, {A2,B1}, {A2,A4,B1,B3} per period match the keep matrices. ✓ [pp3362, 3444]
8. Gray-map closed form gray(n) = n^(n>>1), level = 2n−(M−1) reproduces:
   16-QAM n=8→1100→+1 ✓, 64-QAM n=0→000000? (m=3: n=0→000→−7 ✓, n=7→100→+7 ✓),
   256-QAM n=15→1000→+15 ✓, 1024-QAM (1,1) label 11000/11000 ✓ (n=16→11000→+1).
   [pp3364, 3622–3623, 4279]
9. SERVICE bit budget: 8 + 8·PSDU_LENGTH + N_PAD + 6 = N_SYM·N_DBPS by construction of
   N_PAD (algebraic identity of §4.2 given the ceil in §4.1; N_PAD ∈ 0..N_DBPS−1 for
   non-aggregated SU when 8·PSDU_LENGTH+14 > (N_SYM−1)·N_DBPS). ✓
10. Worked example: MCS 2 (QPSK 3/4, N_DBPS = 78, N_CBPS = 104), PSDU_LENGTH = 100 octets:
    N_SYM = ceil((800+14)/78) = ceil(10.436) = 11; N_PAD = 11·78 − 800 − 14 = 44.
    Encoder input = 8 + 800 + 44 + 6 = 858 bits = N_SYM·N_DBPS ✓.
    Mother-code output = 2·858 = 1716 bits; rate-3/4 puncturing keeps 4 of every 6 →
    1716·(2/3) = 1144 = N_SYM·N_CBPS = 11·104 ✓. Interleaver runs 11 times on 104-bit
    blocks (13 columns × 8 rows); mapper emits 11·52 = 572 QPSK points ✓.
