# iiod network protocol (legacy text mode) — reference for the Pluto backend

Pluto's firmware runs `iiod`, reachable over the USB RNDIS/ethernet interface at
`192.168.2.1`, TCP port **30431**. libiio's `ip:` backend speaks this text protocol; we
implement it directly in Rust (no native deps). Verified against libiio `libiio-v0`
`iiod-client.c` / `iiod/parser.y` / `iiod/ops.c`.

## Framing

- Client sends commands terminated with `\r\n`.
- Server replies with an ASCII decimal return code terminated `\n`. Negative values are
  negated errnos (e.g. `-22` = EINVAL). Non-negative = success / byte count.

## Commands

| Command | Reply |
|---|---|
| `VERSION\r\n` | retcode line, then version text? (retcode only on old iiod; tolerate both) |
| `PRINT\r\n` | retcode = XML byte count, then that many bytes of context XML |
| `TIMEOUT <ms>\r\n` | retcode |
| `OPEN <dev> <samples_count> <mask> [CYCLIC]\r\n` | retcode. `<mask>` = one 8-hex-digit word per 32 channels, `%08x`, highest word first (Pluto RX/TX have <32 channels → a single word, e.g. `00000003` for voltage0+voltage1 = I+Q) |
| `CLOSE <dev>\r\n` | retcode |
| `READBUF <dev> <bytes>\r\n` | loop: retcode line (= bytes in this chunk, 0 ⇒ done, negative ⇒ error), then a hex mask line (`%08x…\n`), then that many raw binary bytes |
| `WRITEBUF <dev> <bytes>\r\n` | retcode line (OK to proceed), client sends raw payload, then retcode line (bytes consumed) |
| `READ <dev> <attr>\r\n` | retcode (= value length), then value bytes + `\n` |
| `READ <dev> INPUT\|OUTPUT <chan> <attr>\r\n` | same |
| `READ <dev> DEBUG <attr>\r\n` | same |
| `WRITE <dev> [INPUT\|OUTPUT <chan>] <attr> <len>\r\n` then payload | retcode |
| `SET <dev> BUFFERS_COUNT <n>\r\n` | retcode |
| `EXIT\r\n` | connection closes |

Device/channel naming: use the ids from the PRINT XML (e.g. device ids like `iio:device0`
or names `ad9361-phy`, `cf-ad9361-lpc`, `cf-ad9361-dds-core-lpc`; channels `voltage0`,
`altvoltage0`, with `output` flag distinguishing directions). Commands accept either id or
name.

## Pluto specifics (AD9363)

- Control device `ad9361-phy`:
  - RX LO: out channel `altvoltage0`, attr `frequency` (Hz, integer string).
  - TX LO: out channel `altvoltage1`, attr `frequency`.
  - Sample rate: `voltage0` (input) `sampling_frequency` for RX ADC and `voltage0`
    (output) `sampling_frequency` for TX DAC — keep both equal (e.g. `4000000`).
  - RF bandwidth: `voltage0` in/out attr `rf_bandwidth` (e.g. `2200000`).
  - RX gain: input `voltage0` `gain_control_mode` (`manual`/`slow_attack`/`fast_attack`),
    `hardwaregain` (dB, when manual).
  - TX attenuation: output `voltage0` `hardwaregain`, **negative dB** (e.g. `-10`).
- RX stream device `cf-ad9361-lpc`: input channels `voltage0` (I), `voltage1` (Q),
  format le:S12/16 — 12-bit samples sign-extended into little-endian i16; scale by
  1/2048. Enable both channels (mask `00000003`), `OPEN` with samples_count = buffer
  size in samples.
- TX stream device `cf-ad9361-dds-core-lpc`: output `voltage0`/`voltage1` i16 I/Q
  (12-bit left-shifted? Pluto DAC uses upper 12 bits: shift left by 4); interleaved
  I,Q per sample, little endian.
- The AD9363 cannot run below ~2.083 MS/s without custom FIR coefficients → run the
  device at 4 MS/s and resample ×2 in `s1g-dsp`.
- Frequency range officially 325 MHz–3.8 GHz; 1250 MHz is fine.
- Set `TIMEOUT` generously (e.g. 3000 ms) before streaming; `SET <dev> BUFFERS_COUNT 4`.

Sources: libiio sources (`iiod-client.c`, `iiod/parser.y`, `iiod/ops.c`,
[dns_sd.h](https://github.com/analogdevicesinc/libiio/blob/main/dns_sd.h) for the port),
[iiod man page](https://www.mankier.com/1/iiod).
