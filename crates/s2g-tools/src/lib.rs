//! Shared helpers for the CLI tools: .cf32 / SigMF / ci16 file I/O (GNU
//! Radio compatible: interleaved little-endian I,Q), rate conversion of
//! recordings to the PHY's 2 MS/s, a PCAP writer, hex PSDU parsing, and the
//! impairment channel used by s2g-sim.

pub mod nic;

use anyhow::{bail, Context, Result};
use num_complex::Complex;
use std::io::{Read, Write};
use std::path::Path;

pub type Complex32 = Complex<f32>;

pub const DEFAULT_CENTER_FREQ_HZ: f64 = 1_250_000_000.0;
pub const DEFAULT_DEVICE_RATE_HZ: f64 = 4_000_000.0;

pub fn read_cf32(path: &Path) -> Result<Vec<Complex32>> {
    let mut f = std::fs::File::open(path).with_context(|| format!("open {}", path.display()))?;
    let mut raw = Vec::new();
    f.read_to_end(&mut raw)?;
    if raw.len() % 8 != 0 {
        bail!("{}: size {} is not a multiple of 8 (cf32 = f32 I,Q pairs)", path.display(), raw.len());
    }
    Ok(raw
        .chunks_exact(8)
        .map(|c| {
            Complex32::new(
                f32::from_le_bytes([c[0], c[1], c[2], c[3]]),
                f32::from_le_bytes([c[4], c[5], c[6], c[7]]),
            )
        })
        .collect())
}

pub fn write_cf32(path: &Path, samples: &[Complex32]) -> Result<()> {
    let mut f = std::fs::File::create(path).with_context(|| format!("create {}", path.display()))?;
    let mut raw = Vec::with_capacity(samples.len() * 8);
    for s in samples {
        raw.extend_from_slice(&s.re.to_le_bytes());
        raw.extend_from_slice(&s.im.to_le_bytes());
    }
    f.write_all(&raw)?;
    Ok(())
}

/// A recording plus the metadata needed to bring it to the PHY rate.
pub struct Recording {
    pub samples: Vec<Complex32>,
    pub sample_rate_hz: Option<f64>,
    pub center_freq_hz: Option<f64>,
}

/// Read a SigMF recording (`.sigmf-data` with a sibling `.sigmf-meta`, or a
/// `.sigmf` tar archive), a raw complex-int16 file (`.ci16` / `.cs16`) or a
/// `.cf32` file. Supports the `ci16_le` and `cf32_le` SigMF datatypes.
/// `skip` / `max_samples` select a window of the recording.
pub fn read_recording(path: &Path, skip: usize, max_samples: Option<usize>) -> Result<Recording> {
    let name = path.file_name().and_then(|n| n.to_str()).unwrap_or("");
    if name.ends_with(".sigmf") {
        return read_sigmf_archive(path, skip, max_samples);
    }
    if name.ends_with(".sigmf-data") {
        let meta_path = path.with_extension("sigmf-meta");
        let meta = std::fs::read_to_string(&meta_path).ok();
        let (datatype, rate, freq) = meta.as_deref().map(parse_sigmf_meta).unwrap_or((None, None, None));
        let raw = std::fs::read(path).with_context(|| format!("read {}", path.display()))?;
        let samples = decode_samples(&raw, datatype.as_deref().unwrap_or("ci16_le"), skip, max_samples)?;
        return Ok(Recording { samples, sample_rate_hz: rate, center_freq_hz: freq });
    }
    if name.ends_with(".ci16") || name.ends_with(".cs16") {
        let raw = std::fs::read(path).with_context(|| format!("read {}", path.display()))?;
        return Ok(Recording { samples: decode_samples(&raw, "ci16_le", skip, max_samples)?, sample_rate_hz: None, center_freq_hz: None });
    }
    let all = read_cf32(path)?;
    let end = max_samples.map(|n| (skip + n).min(all.len())).unwrap_or(all.len());
    Ok(Recording { samples: all[skip.min(end)..end].to_vec(), sample_rate_hz: None, center_freq_hz: None })
}

/// Minimal SigMF metadata scan: (core:datatype, core:sample_rate, the
/// first capture core:frequency).
pub fn parse_sigmf_meta(meta: &str) -> (Option<String>, Option<f64>, Option<f64>) {
    fn find_str(meta: &str, key: &str) -> Option<String> {
        let i = meta.find(key)? + key.len();
        let rest = &meta[i..];
        let q1 = rest.find('"')? + 1;
        let q2 = rest[q1..].find('"')? + q1;
        Some(rest[q1..q2].to_string())
    }
    fn find_num(meta: &str, key: &str) -> Option<f64> {
        let i = meta.find(key)? + key.len();
        let rest = &meta[i..];
        let start = rest.find(|c: char| c.is_ascii_digit() || c == '-' || c == '.')?;
        let is_num = |c: char| c.is_ascii_digit() || c == '-' || c == '.' || c == 'e' || c == 'E' || c == '+';
        let end = rest[start..].find(|c: char| !is_num(c)).map(|e| e + start).unwrap_or(rest.len());
        rest[start..end].parse().ok()
    }
    (find_str(meta, "\"core:datatype\""), find_num(meta, "\"core:sample_rate\""), find_num(meta, "\"core:frequency\""))
}

fn decode_samples(raw: &[u8], datatype: &str, skip: usize, max_samples: Option<usize>) -> Result<Vec<Complex32>> {
    let take = max_samples.unwrap_or(usize::MAX);
    let out: Vec<Complex32> = match datatype {
        "ci16_le" | "ci16" => raw
            .chunks_exact(4)
            .skip(skip)
            .take(take)
            .map(|c| {
                Complex32::new(
                    i16::from_le_bytes([c[0], c[1]]) as f32 / 32768.0,
                    i16::from_le_bytes([c[2], c[3]]) as f32 / 32768.0,
                )
            })
            .collect(),
        "cf32_le" | "cf32" => raw
            .chunks_exact(8)
            .skip(skip)
            .take(take)
            .map(|c| {
                Complex32::new(
                    f32::from_le_bytes([c[0], c[1], c[2], c[3]]),
                    f32::from_le_bytes([c[4], c[5], c[6], c[7]]),
                )
            })
            .collect(),
        other => bail!("unsupported SigMF datatype {other} (ci16_le / cf32_le only)"),
    };
    Ok(out)
}

/// A `.sigmf` archive is a POSIX tar containing the -meta and -data files.
fn read_sigmf_archive(path: &Path, skip: usize, max_samples: Option<usize>) -> Result<Recording> {
    let mut f = std::fs::File::open(path).with_context(|| format!("open {}", path.display()))?;
    let mut meta: Option<String> = None;
    let mut data: Option<Vec<u8>> = None;
    let mut hdr = [0u8; 512];
    loop {
        if f.read_exact(&mut hdr).is_err() {
            break;
        }
        if hdr.iter().all(|&b| b == 0) {
            break;
        }
        let name = String::from_utf8_lossy(&hdr[..100]).trim_end_matches('\0').to_string();
        let size_str = String::from_utf8_lossy(&hdr[124..136]).trim_end_matches(['\0', ' ']).to_string();
        let size = usize::from_str_radix(size_str.trim(), 8).unwrap_or(0);
        let padded = size.div_ceil(512) * 512;
        let typeflag = hdr[156];
        let regular = typeflag == b'0' || typeflag == 0;
        if regular && name.ends_with(".sigmf-meta") {
            let mut buf = vec![0u8; size];
            f.read_exact(&mut buf)?;
            meta = Some(String::from_utf8_lossy(&buf).to_string());
            std::io::copy(&mut (&mut f).take((padded - size) as u64), &mut std::io::sink())?;
        } else if regular && name.ends_with(".sigmf-data") {
            // Read only the window we need (8 bytes/sample is an upper bound
            // for either datatype; the decoder applies the exact skip).
            let want = match max_samples {
                Some(n) => size.min((skip + n) * 8),
                None => size,
            };
            let mut buf = vec![0u8; want];
            f.read_exact(&mut buf)?;
            data = Some(buf);
            if want < size {
                break;
            }
            std::io::copy(&mut (&mut f).take((padded - size) as u64), &mut std::io::sink())?;
        } else {
            std::io::copy(&mut (&mut f).take(padded as u64), &mut std::io::sink())?;
        }
    }
    let data = data.ok_or_else(|| anyhow::anyhow!("{}: no .sigmf-data member", path.display()))?;
    let (datatype, rate, freq) = meta.as_deref().map(parse_sigmf_meta).unwrap_or((None, None, None));
    let samples = decode_samples(&data, datatype.as_deref().unwrap_or("ci16_le"), skip, max_samples)?;
    Ok(Recording { samples, sample_rate_hz: rate, center_freq_hz: freq })
}

/// Bring a recording at `in_rate` to the PHY's 2 MS/s, optionally shifting
/// a signal centred `shift_hz` away from the capture centre to baseband
/// first. Uses the halfband decimator for exactly 4 MS/s, otherwise the
/// anti-aliased arbitrary-ratio resampler.
pub fn to_native_rate(samples: &[Complex32], in_rate: f64, shift_hz: f64) -> Vec<Complex32> {
    let shifted;
    let src: &[Complex32] = if shift_hz != 0.0 {
        shifted = s2g_dsp::frequency_shift(samples, shift_hz, in_rate);
        &shifted
    } else {
        samples
    };
    let native = s2g_phy::params::SAMPLE_RATE_HZ;
    if (in_rate - native).abs() < 1.0 {
        return src.to_vec();
    }
    if (in_rate - 2.0 * native).abs() < 1.0 {
        let mut dec = s2g_dsp::HalfbandDecim2::new();
        let mut out = Vec::with_capacity(src.len() / 2);
        dec.process(src, &mut out);
        return out;
    }
    let step = in_rate / native;
    // Pass the occupied ±0.9 MHz, stop by the output Nyquist.
    let cutoff = (0.95e6 / in_rate).min(0.5 / step * 0.98);
    s2g_dsp::resample_lowpass(src, step, cutoff, 48)
}

/// Minimal PCAP writer for 802.11 frames (link type 105, no radiotap).
pub struct PcapWriter {
    f: std::fs::File,
}

impl PcapWriter {
    pub fn create(path: &Path) -> Result<Self> {
        let mut f = std::fs::File::create(path).with_context(|| format!("create {}", path.display()))?;
        let mut hdr = Vec::with_capacity(24);
        hdr.extend_from_slice(&0xa1b2c3d4u32.to_le_bytes());
        hdr.extend_from_slice(&2u16.to_le_bytes());
        hdr.extend_from_slice(&4u16.to_le_bytes());
        hdr.extend_from_slice(&0i32.to_le_bytes());
        hdr.extend_from_slice(&0u32.to_le_bytes());
        hdr.extend_from_slice(&65535u32.to_le_bytes());
        hdr.extend_from_slice(&105u32.to_le_bytes()); // LINKTYPE_IEEE802_11
        f.write_all(&hdr)?;
        Ok(Self { f })
    }

    /// Write one frame (with FCS) at time `t_us` since the epoch.
    pub fn write(&mut self, t_us: u64, frame: &[u8]) -> Result<()> {
        let mut rec = Vec::with_capacity(16 + frame.len());
        rec.extend_from_slice(&((t_us / 1_000_000) as u32).to_le_bytes());
        rec.extend_from_slice(&((t_us % 1_000_000) as u32).to_le_bytes());
        rec.extend_from_slice(&(frame.len() as u32).to_le_bytes());
        rec.extend_from_slice(&(frame.len() as u32).to_le_bytes());
        rec.extend_from_slice(frame);
        self.f.write_all(&rec)?;
        Ok(())
    }
}

pub fn parse_hex_psdu(hex: &str) -> Result<Vec<u8>> {
    let clean: String = hex.chars().filter(|c| !c.is_whitespace() && *c != ':').collect();
    if clean.len() % 2 != 0 {
        bail!("hex PSDU has odd number of digits");
    }
    (0..clean.len() / 2)
        .map(|i| u8::from_str_radix(&clean[2 * i..2 * i + 2], 16).context("bad hex digit"))
        .collect()
}

/// Deterministic test RNG (splitmix64).
pub struct Rng(pub u64);

impl Rng {
    pub fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    pub fn uniform(&mut self) -> f32 {
        ((self.next_u64() >> 32) as f32 / (1u64 << 31) as f32) - 1.0
    }
    pub fn gauss(&mut self) -> f32 {
        let s: f32 = (0..6).map(|_| self.uniform()).sum();
        s / (2.0f32).sqrt()
    }
    pub fn bytes(&mut self, n: usize) -> Vec<u8> {
        (0..n).map(|_| (self.next_u64() >> 40) as u8).collect()
    }
}

/// Channel impairments for simulation.
#[derive(Debug, Clone, Copy)]
pub struct Impairments {
    pub snr_db: Option<f32>,
    pub cfo_hz: f32,
    /// Fractional-sample delay 0..1 (linear interpolation).
    pub frac_delay: f32,
    pub amplitude: f32,
    /// Sampling-clock offset between transmitter and receiver, ppm
    /// (positive = receiver clock fast).
    pub sfo_ppm: f64,
    /// Static echo: (delay in samples, complex gain).
    pub echo: Option<(usize, Complex32)>,
}

impl Default for Impairments {
    fn default() -> Self {
        Self { snr_db: None, cfo_hz: 0.0, frac_delay: 0.0, amplitude: 1.0, sfo_ppm: 0.0, echo: None }
    }
}

pub fn apply_channel(sig: &[Complex32], imp: &Impairments, rng: &mut Rng) -> Vec<Complex32> {
    let mut v: Vec<Complex32> = sig.iter().map(|&s| s * imp.amplitude).collect();
    if let Some((delay, gain)) = imp.echo {
        v = (0..v.len())
            .map(|i| v[i] + if i >= delay { v[i - delay] * gain } else { Complex32::new(0.0, 0.0) })
            .collect();
    }
    if imp.sfo_ppm != 0.0 {
        v = s2g_dsp::apply_sfo_ppm(&v, imp.sfo_ppm);
    }
    if imp.cfo_hz != 0.0 {
        let w = 2.0 * std::f64::consts::PI * imp.cfo_hz as f64 / 2.0e6;
        for (i, s) in v.iter_mut().enumerate() {
            *s *= Complex32::from_polar(1.0, (w * i as f64) as f32);
        }
    }
    if imp.frac_delay > 0.0 {
        let mu = imp.frac_delay;
        v = (0..v.len().saturating_sub(1)).map(|i| v[i] * (1.0 - mu) + v[i + 1] * mu).collect();
    }
    if let Some(snr) = imp.snr_db {
        let p: f32 = v.iter().map(|s| s.norm_sqr()).sum::<f32>() / v.len() as f32;
        let sigma = (p / 10f32.powf(snr / 10.0) / 2.0).sqrt();
        for s in v.iter_mut() {
            *s += Complex32::new(rng.gauss() * sigma, rng.gauss() * sigma);
        }
    }
    v
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cf32_roundtrip() {
        let dir = std::env::temp_dir().join("s2g_tools_test.cf32");
        let v: Vec<Complex32> = (0..100).map(|i| Complex32::new(i as f32, -i as f32 / 2.0)).collect();
        write_cf32(&dir, &v).unwrap();
        let r = read_cf32(&dir).unwrap();
        assert_eq!(v, r);
        let _ = std::fs::remove_file(&dir);
    }

    #[test]
    fn hex_parse() {
        assert_eq!(parse_hex_psdu("de:ad be ef").unwrap(), vec![0xde, 0xad, 0xbe, 0xef]);
        assert!(parse_hex_psdu("abc").is_err());
    }

    #[test]
    fn sigmf_meta_scan() {
        let meta = r#"{"global": {"core:datatype": "ci16_le", "core:sample_rate": 3840000.0, "core:version": "1.0.0"},
            "captures": [{"core:sample_start": 0, "core:frequency": 866000000.0}], "annotations": []}"#;
        let (dt, rate, freq) = parse_sigmf_meta(meta);
        assert_eq!(dt.as_deref(), Some("ci16_le"));
        assert_eq!(rate, Some(3.84e6));
        assert_eq!(freq, Some(866e6));
    }

    #[test]
    fn ci16_decode_window() {
        let raw: Vec<u8> = (0..8i16).flat_map(|i| [(i * 1000).to_le_bytes(), (-i * 1000).to_le_bytes()].concat()).collect();
        let v = decode_samples(&raw, "ci16_le", 2, Some(3)).unwrap();
        assert_eq!(v.len(), 3);
        assert!((v[0].re - 2000.0 / 32768.0).abs() < 1e-6);
        assert!((v[2].im + 4000.0 / 32768.0).abs() < 1e-6);
    }
}
