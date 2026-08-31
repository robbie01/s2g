//! Shared helpers for the CLI tools: .cf32 file I/O (GNU Radio compatible:
//! interleaved little-endian f32 I,Q), hex PSDU parsing, and the impairment
//! channel used by s1g-sim.

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
}

pub fn apply_channel(sig: &[Complex32], imp: &Impairments, rng: &mut Rng) -> Vec<Complex32> {
    let mut v: Vec<Complex32> = sig.iter().map(|&s| s * imp.amplitude).collect();
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
        let dir = std::env::temp_dir().join("s1g_tools_test.cf32");
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
}
