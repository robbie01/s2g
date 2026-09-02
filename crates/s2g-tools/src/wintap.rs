//! Windows L2 TAP backend for `s2g-node` on top of the OpenVPN
//! **tap-windows6** driver (`tap0901`), the only widely deployed Ethernet
//! TAP on Windows. Install it from the OpenVPN installer (select "TAP
//! Virtual Ethernet Adapter") or the standalone tap-windows package; the
//! adapter shows up as "TAP-Windows Adapter V9" in Network Connections.
//!
//! The driver is driven the way OpenVPN drives it: enumerate adapters in
//! the network-class registry key by `ComponentId == tap0901`, open
//! `\\.\Global\{NetCfgInstanceId}.tap` with overlapped I/O, tell the driver
//! the cable is plugged in (`TAP_WIN_IOCTL_SET_MEDIA_STATUS`), then read and
//! write raw Ethernet frames with `ReadFile` / `WriteFile`. Assign an
//! address to the adapter with `netsh interface ip set address "TAP-Windows
//! Adapter V9" static 10.44.0.1 255.255.255.0` (or DHCP masquerade is left
//! off on purpose). Opening the device usually needs an elevated prompt.
//!
//! Wintun (WireGuard's driver) is *not* usable here: it is a layer-3 TUN
//! and this MAC carries Ethernet frames.

use anyhow::{anyhow, bail, Context, Result};
use std::ffi::c_void;
use std::time::Duration;
use windows_sys::Win32::Foundation::{
    CloseHandle, GetLastError, ERROR_IO_PENDING, ERROR_NO_MORE_ITEMS, ERROR_SUCCESS, GENERIC_READ, GENERIC_WRITE, HANDLE,
    INVALID_HANDLE_VALUE, WAIT_OBJECT_0, WAIT_TIMEOUT,
};
use windows_sys::Win32::Storage::FileSystem::{
    CreateFileW, ReadFile, WriteFile, FILE_ATTRIBUTE_SYSTEM, FILE_FLAG_OVERLAPPED, FILE_SHARE_READ, FILE_SHARE_WRITE,
    OPEN_EXISTING,
};
use windows_sys::Win32::System::Registry::{
    RegCloseKey, RegEnumKeyExW, RegOpenKeyExW, RegQueryValueExW, HKEY, HKEY_LOCAL_MACHINE, KEY_READ,
};
use windows_sys::Win32::System::Threading::{CreateEventW, WaitForSingleObject};
use windows_sys::Win32::System::IO::{CancelIoEx, DeviceIoControl, GetOverlappedResult, OVERLAPPED};

use crate::nic::{Nic, MAX_FRAME};

const NETWORK_CLASS: &str = r"SYSTEM\CurrentControlSet\Control\Class\{4D36E972-E325-11CE-BFC1-08002BE10318}";
const CONNECTIONS: &str = r"SYSTEM\CurrentControlSet\Control\Network\{4D36E972-E325-11CE-BFC1-08002BE10318}";
const TAP_COMPONENT_ID: &str = "tap0901";

/// `CTL_CODE(FILE_DEVICE_UNKNOWN, request, METHOD_BUFFERED, FILE_ANY_ACCESS)`
/// as in tap-windows.h.
const fn tap_ioctl(request: u32) -> u32 {
    (0x22 << 16) | (request << 2)
}
const TAP_WIN_IOCTL_GET_MAC: u32 = tap_ioctl(1);
const TAP_WIN_IOCTL_GET_VERSION: u32 = tap_ioctl(2);
const TAP_WIN_IOCTL_GET_MTU: u32 = tap_ioctl(3);
const TAP_WIN_IOCTL_SET_MEDIA_STATUS: u32 = tap_ioctl(6);

/// A tap-windows6 adapter found in the registry.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TapAdapter {
    /// `NetCfgInstanceId`, e.g. `{1A2B3C4D-...}`.
    pub guid: String,
    /// Friendly connection name, e.g. "TAP-Windows Adapter V9".
    pub name: String,
}

fn wide(s: &str) -> Vec<u16> {
    s.encode_utf16().chain(std::iter::once(0)).collect()
}

fn from_wide(buf: &[u16]) -> String {
    let end = buf.iter().position(|&c| c == 0).unwrap_or(buf.len());
    String::from_utf16_lossy(&buf[..end])
}

struct Key(HKEY);

impl Key {
    fn open(root: HKEY, path: &str) -> Option<Key> {
        let mut h: HKEY = std::ptr::null_mut();
        let rc = unsafe { RegOpenKeyExW(root, wide(path).as_ptr(), 0, KEY_READ, &mut h) };
        (rc == ERROR_SUCCESS).then_some(Key(h))
    }

    fn subkeys(&self) -> Vec<String> {
        let mut out = Vec::new();
        for i in 0.. {
            let mut name = [0u16; 256];
            let mut len = name.len() as u32;
            let rc = unsafe {
                RegEnumKeyExW(self.0, i, name.as_mut_ptr(), &mut len, std::ptr::null(), std::ptr::null_mut(), std::ptr::null_mut(), std::ptr::null_mut())
            };
            if rc == ERROR_NO_MORE_ITEMS {
                break;
            }
            if rc != ERROR_SUCCESS {
                continue;
            }
            out.push(from_wide(&name[..len as usize]));
        }
        out
    }

    fn string_value(&self, name: &str) -> Option<String> {
        let mut buf = [0u16; 512];
        let mut len = (buf.len() * 2) as u32;
        let mut ty = 0u32;
        let rc = unsafe { RegQueryValueExW(self.0, wide(name).as_ptr(), std::ptr::null(), &mut ty, buf.as_mut_ptr() as *mut u8, &mut len) };
        (rc == ERROR_SUCCESS).then(|| from_wide(&buf[..(len as usize / 2).min(buf.len())]))
    }
}

impl Drop for Key {
    fn drop(&mut self) {
        unsafe { RegCloseKey(self.0) };
    }
}

/// Every tap-windows6 adapter installed on this machine.
pub fn list_tap_adapters() -> Vec<TapAdapter> {
    let mut found = Vec::new();
    let Some(class) = Key::open(HKEY_LOCAL_MACHINE, NETWORK_CLASS) else { return found };
    for sub in class.subkeys() {
        let Some(k) = Key::open(HKEY_LOCAL_MACHINE, &format!("{NETWORK_CLASS}\\{sub}")) else { continue };
        if k.string_value("ComponentId").as_deref() != Some(TAP_COMPONENT_ID) {
            continue;
        }
        let Some(guid) = k.string_value("NetCfgInstanceId") else { continue };
        let name = Key::open(HKEY_LOCAL_MACHINE, &format!("{CONNECTIONS}\\{guid}\\Connection"))
            .and_then(|c| c.string_value("Name"))
            .unwrap_or_else(|| guid.clone());
        found.push(TapAdapter { guid, name });
    }
    found
}

/// Overlapped I/O context with its own event, at a stable heap address.
struct Overlapped {
    ov: Box<OVERLAPPED>,
}

impl Overlapped {
    fn new() -> Result<Self> {
        let event = unsafe { CreateEventW(std::ptr::null(), 1, 0, std::ptr::null()) };
        if event.is_null() {
            bail!("CreateEvent failed: {}", unsafe { GetLastError() });
        }
        let mut ov: Box<OVERLAPPED> = Box::new(unsafe { std::mem::zeroed() });
        ov.hEvent = event;
        Ok(Self { ov })
    }
    fn reset(&mut self) {
        let event = self.ov.hEvent;
        *self.ov = unsafe { std::mem::zeroed() };
        self.ov.hEvent = event;
    }
}

impl Drop for Overlapped {
    fn drop(&mut self) {
        unsafe { CloseHandle(self.ov.hEvent) };
    }
}

/// An open tap-windows6 adapter.
pub struct WinTapNic {
    handle: HANDLE,
    adapter: TapAdapter,
    read: Overlapped,
    write: Overlapped,
    /// Read buffer handed to the driver; stays allocated while a read is
    /// pending (the driver writes into it asynchronously).
    rbuf: Vec<u8>,
    read_pending: bool,
    mtu: u32,
}

// The handle and buffers may move to the node's I/O thread; nothing here
// is tied to the creating thread.
unsafe impl Send for WinTapNic {}

impl WinTapNic {
    /// Open the adapter whose connection name (or GUID) is `name`, or the
    /// first tap-windows6 adapter when `None`.
    pub fn new(name: Option<&str>) -> Result<Self> {
        let adapters = list_tap_adapters();
        if adapters.is_empty() {
            bail!(
                "no tap-windows6 adapter (ComponentId tap0901) is installed; install OpenVPN's \
                 TAP-Windows driver, then retry (Wintun is a layer-3 TUN and cannot carry Ethernet frames)"
            );
        }
        let adapter = match name {
            Some(n) if !n.is_empty() => adapters
                .iter()
                .find(|a| a.name.eq_ignore_ascii_case(n) || a.guid.eq_ignore_ascii_case(n))
                .cloned()
                .ok_or_else(|| {
                    anyhow!(
                        "no TAP adapter named {n:?}; installed: {}",
                        adapters.iter().map(|a| format!("{} ({})", a.name, a.guid)).collect::<Vec<_>>().join(", ")
                    )
                })?,
            _ => adapters[0].clone(),
        };
        let path = format!(r"\\.\Global\{}.tap", adapter.guid);
        let handle = unsafe {
            CreateFileW(
                wide(&path).as_ptr(),
                GENERIC_READ | GENERIC_WRITE,
                FILE_SHARE_READ | FILE_SHARE_WRITE,
                std::ptr::null(),
                OPEN_EXISTING,
                FILE_ATTRIBUTE_SYSTEM | FILE_FLAG_OVERLAPPED,
                std::ptr::null_mut(),
            )
        };
        if handle == INVALID_HANDLE_VALUE {
            let e = unsafe { GetLastError() };
            bail!("cannot open {path} (error {e}); is another program using the adapter, and are you elevated?");
        }
        let mut nic = Self {
            handle,
            adapter,
            read: Overlapped::new()?,
            write: Overlapped::new()?,
            rbuf: vec![0u8; MAX_FRAME.max(2048)],
            read_pending: false,
            mtu: 1500,
        };
        // Cable plugged in.
        let one: u32 = 1;
        let mut echo: u32 = 0;
        nic.ioctl(TAP_WIN_IOCTL_SET_MEDIA_STATUS, &one as *const u32 as *const c_void, 4, &mut echo as *mut u32 as *mut c_void, 4)
            .context("TAP_WIN_IOCTL_SET_MEDIA_STATUS")?;
        let mut mtu: u32 = 0;
        if nic.ioctl(TAP_WIN_IOCTL_GET_MTU, std::ptr::null(), 0, &mut mtu as *mut u32 as *mut c_void, 4).is_ok() && mtu > 0 {
            nic.mtu = mtu;
        }
        Ok(nic)
    }

    /// Driver version as (major, minor), if the ioctl is supported.
    pub fn driver_version(&mut self) -> Option<(u32, u32)> {
        let mut v = [0u32; 3];
        self.ioctl(TAP_WIN_IOCTL_GET_VERSION, std::ptr::null(), 0, v.as_mut_ptr() as *mut c_void, 12).ok()?;
        Some((v[0], v[1]))
    }

    /// The adapter's own MAC address.
    pub fn mac(&mut self) -> Option<[u8; 6]> {
        let mut m = [0u8; 6];
        self.ioctl(TAP_WIN_IOCTL_GET_MAC, std::ptr::null(), 0, m.as_mut_ptr() as *mut c_void, 6).ok()?;
        Some(m)
    }

    pub fn mtu(&self) -> u32 {
        self.mtu
    }

    fn ioctl(&mut self, code: u32, inp: *const c_void, inlen: u32, out: *mut c_void, outlen: u32) -> Result<u32> {
        self.write.reset();
        let mut returned = 0u32;
        let ok = unsafe { DeviceIoControl(self.handle, code, inp, inlen, out, outlen, &mut returned, &mut *self.write.ov) };
        if ok == 0 {
            let e = unsafe { GetLastError() };
            if e != ERROR_IO_PENDING {
                bail!("DeviceIoControl 0x{code:x} failed: error {e}");
            }
            if unsafe { GetOverlappedResult(self.handle, &*self.write.ov, &mut returned, 1) } == 0 {
                bail!("DeviceIoControl 0x{code:x} did not complete: error {}", unsafe { GetLastError() });
            }
        }
        Ok(returned)
    }
}

impl Nic for WinTapNic {
    fn recv_frame(&mut self, timeout: Duration) -> Result<Option<Vec<u8>>> {
        if !self.read_pending {
            self.read.reset();
            let mut got = 0u32;
            let ok = unsafe { ReadFile(self.handle, self.rbuf.as_mut_ptr(), self.rbuf.len() as u32, &mut got, &mut *self.read.ov) };
            if ok != 0 {
                return Ok(Some(self.rbuf[..got as usize].to_vec()));
            }
            let e = unsafe { GetLastError() };
            if e != ERROR_IO_PENDING {
                bail!("TAP ReadFile failed: error {e}");
            }
            self.read_pending = true;
        }
        let ms = timeout.as_millis().min(u32::MAX as u128) as u32;
        match unsafe { WaitForSingleObject(self.read.ov.hEvent, ms) } {
            WAIT_OBJECT_0 => {
                let mut got = 0u32;
                let ok = unsafe { GetOverlappedResult(self.handle, &*self.read.ov, &mut got, 0) };
                self.read_pending = false;
                if ok == 0 {
                    bail!("TAP read did not complete: error {}", unsafe { GetLastError() });
                }
                Ok(Some(self.rbuf[..got as usize].to_vec()))
            }
            WAIT_TIMEOUT => Ok(None),
            other => bail!("TAP wait failed: {other}"),
        }
    }

    fn send_frame(&mut self, frame: &[u8]) -> Result<()> {
        self.write.reset();
        let mut written = 0u32;
        let ok = unsafe { WriteFile(self.handle, frame.as_ptr(), frame.len() as u32, &mut written, &mut *self.write.ov) };
        if ok == 0 {
            let e = unsafe { GetLastError() };
            if e != ERROR_IO_PENDING {
                bail!("TAP WriteFile failed: error {e}");
            }
            if unsafe { GetOverlappedResult(self.handle, &*self.write.ov, &mut written, 1) } == 0 {
                bail!("TAP write did not complete: error {}", unsafe { GetLastError() });
            }
        }
        Ok(())
    }

    fn describe(&self) -> String {
        format!("tap-windows6 {} ({}), mtu {}", self.adapter.name, self.adapter.guid, self.mtu)
    }
}

impl Drop for WinTapNic {
    fn drop(&mut self) {
        unsafe {
            if self.read_pending {
                CancelIoEx(self.handle, &*self.read.ov);
                let mut n = 0u32;
                GetOverlappedResult(self.handle, &*self.read.ov, &mut n, 1);
            }
            CloseHandle(self.handle);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn enumeration_and_open_error_are_sane() {
        // Registry enumeration must not panic, and opening a nonexistent
        // adapter must explain itself instead of crashing.
        let adapters = list_tap_adapters();
        for a in &adapters {
            assert!(a.guid.starts_with('{'), "{a:?}");
        }
        match WinTapNic::new(Some("definitely-not-an-adapter-name")) {
            Ok(_) => panic!("opened a nonexistent adapter"),
            Err(e) => {
                let msg = e.to_string();
                assert!(msg.contains("tap") || msg.contains("TAP"), "{msg}");
            }
        }
        assert_eq!(TAP_WIN_IOCTL_SET_MEDIA_STATUS, 0x22_0018);
        assert_eq!(TAP_WIN_IOCTL_GET_MTU, 0x22_000c);
    }
}
