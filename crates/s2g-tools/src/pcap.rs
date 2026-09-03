//! PCAP output of 802.11 MPDUs behind a radiotap header (link type 127):
//! a file, standard output, or a pipe that Wireshark reads live.

use anyhow::{bail, Context, Result};
use s2g_mac::{ampdu, fcs};
use s2g_phy::vector::{GuardInterval, PreambleType, ResponseIndication, RxVector, TxVector};
use std::io::Write;
use std::path::Path;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{sync_channel, Receiver, RecvTimeoutError, SyncSender, TrySendError};
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

/// LINKTYPE_IEEE802_11_RADIOTAP.
const LINKTYPE: u32 = 127;
/// Records queued for a pipe reader before new ones are dropped.
const PIPE_BACKLOG: usize = 1024;
/// How often an idle pipe is probed for a reader that went away.
const PIPE_PROBE_INTERVAL: Duration = Duration::from_secs(1);

const PRESENT_FLAGS: u32 = 1 << 1;
const PRESENT_CHANNEL: u32 = 1 << 3;
const PRESENT_DBM_ANTSIGNAL: u32 = 1 << 5;
const PRESENT_TX_FLAGS: u32 = 1 << 15;
const PRESENT_AMPDU_STATUS: u32 = 1 << 20;
const PRESENT_TLVS: u32 = 1 << 28;
const FLAG_FCS: u8 = 0x10;
const FLAG_BAD_FCS: u8 = 0x40;
const FLAG_SHORT_GI: u8 = 0x80;
const TX_FLAG_NOACK: u16 = 0x0008;
const AMPDU_LAST_KNOWN: u16 = 0x0004;
const AMPDU_IS_LAST: u16 = 0x0008;
const AMPDU_EOF: u16 = 0x0040;
const AMPDU_EOF_KNOWN: u16 = 0x0080;
/// S1G TLV type; `known` covers PPDU format, response indication, guard
/// interval, NSS, bandwidth, MCS, color and uplink indication.
const TLV_S1G: u16 = 32;
const S1G_KNOWN: u16 = 0x00ff;

/// Microseconds since the Unix epoch.
pub fn unix_time_us() -> u64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_micros() as u64).unwrap_or(0)
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Direction {
    /// Received; `signal_dbm` is the RCPI.
    Rx { signal_dbm: f32 },
    /// Transmitted; without `ack_expected` the TX flags say NOACK.
    Tx { ack_expected: bool },
}

/// A-MPDU status of one subframe.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AmpduStatus {
    /// Shared by the subframes of one A-MPDU.
    pub reference: u32,
    pub last: bool,
    /// The delimiter's EOF bit.
    pub eof: bool,
}

/// Radiotap contents for one MPDU.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Radiotap {
    pub direction: Direction,
    /// Channel center; `None` omits the Channel field.
    pub freq_hz: Option<f64>,
    pub preamble_type: PreambleType,
    pub response_indication: ResponseIndication,
    pub gi: GuardInterval,
    pub mcs: u8,
    pub color: u8,
    pub uplink_indication: bool,
    pub fcs_ok: bool,
    pub ampdu: Option<AmpduStatus>,
}

impl Radiotap {
    pub fn rx(v: &RxVector, freq_hz: Option<f64>) -> Self {
        Self {
            direction: Direction::Rx { signal_dbm: v.rcpi_dbm },
            freq_hz,
            preamble_type: v.preamble_type,
            response_indication: v.response_indication,
            gi: v.gi,
            mcs: v.mcs,
            color: v.color,
            uplink_indication: v.uplink_indication,
            fcs_ok: true,
            ampdu: None,
        }
    }

    pub fn tx(v: &TxVector, freq_hz: Option<f64>) -> Self {
        Self {
            direction: Direction::Tx { ack_expected: v.response_indication != ResponseIndication::None },
            freq_hz,
            preamble_type: v.preamble_type,
            response_indication: v.response_indication,
            gi: v.gi,
            mcs: v.mcs,
            color: v.color,
            uplink_indication: v.uplink_indication,
            fcs_ok: true,
            ampdu: None,
        }
    }

    /// Header bytes: Flags, Channel, dBm antenna signal (RX) or TX flags
    /// (TX), A-MPDU status, S1G TLV.
    pub fn encode(&self) -> Vec<u8> {
        let mut present = PRESENT_FLAGS | PRESENT_TLVS;
        if self.freq_hz.is_some() {
            present |= PRESENT_CHANNEL;
        }
        present |= match self.direction {
            Direction::Rx { .. } => PRESENT_DBM_ANTSIGNAL,
            Direction::Tx { .. } => PRESENT_TX_FLAGS,
        };
        if self.ampdu.is_some() {
            present |= PRESENT_AMPDU_STATUS;
        }
        let mut b = vec![0u8, 0, 0, 0];
        b.extend_from_slice(&present.to_le_bytes());
        let mut flags = FLAG_FCS;
        if !self.fcs_ok {
            flags |= FLAG_BAD_FCS;
        }
        if self.gi == GuardInterval::Short {
            flags |= FLAG_SHORT_GI;
        }
        b.push(flags);
        if let Some(f) = self.freq_hz {
            align(&mut b, 2);
            b.extend_from_slice(&((f / 1e6).round() as u16).to_le_bytes());
            b.extend_from_slice(&0u16.to_le_bytes());
        }
        match self.direction {
            Direction::Rx { signal_dbm } => b.push(dbm(signal_dbm) as u8),
            Direction::Tx { ack_expected } => {
                align(&mut b, 2);
                b.extend_from_slice(&(if ack_expected { 0 } else { TX_FLAG_NOACK }).to_le_bytes());
            }
        }
        if let Some(a) = self.ampdu {
            align(&mut b, 4);
            b.extend_from_slice(&a.reference.to_le_bytes());
            let mut f = AMPDU_LAST_KNOWN | AMPDU_EOF_KNOWN;
            if a.last {
                f |= AMPDU_IS_LAST;
            }
            if a.eof {
                f |= AMPDU_EOF;
            }
            b.extend_from_slice(&f.to_le_bytes());
            b.extend_from_slice(&[0, 0]);
        }
        align(&mut b, 4);
        b.extend_from_slice(&TLV_S1G.to_le_bytes());
        b.extend_from_slice(&6u16.to_le_bytes());
        b.extend_from_slice(&S1G_KNOWN.to_le_bytes());
        b.extend_from_slice(&self.s1g_data1().to_le_bytes());
        b.extend_from_slice(&self.s1g_data2().to_le_bytes());
        align(&mut b, 4);
        let len = b.len() as u16;
        b[2..4].copy_from_slice(&len.to_le_bytes());
        b
    }

    /// S1G data1: PPDU format (1 = S1G_SHORT, 2 = S1G_LONG), response
    /// indication, guard interval, NSS − 1, bandwidth (1 = 2 MHz), MCS.
    fn s1g_data1(&self) -> u16 {
        let format = match self.preamble_type {
            PreambleType::S1gShort => 1,
            PreambleType::S1gLong => 2,
        };
        let short_gi = (self.gi == GuardInterval::Short) as u16;
        format | (self.response_indication.to_bits() as u16) << 2 | short_gi << 5 | 1 << 8 | (self.mcs as u16 & 0xf) << 12
    }

    /// S1G data2: color, uplink indication, RSSI in dBm.
    fn s1g_data2(&self) -> u16 {
        let rssi = match self.direction {
            Direction::Rx { signal_dbm } => dbm(signal_dbm) as u8 as u16,
            Direction::Tx { .. } => 0,
        };
        (self.color as u16 & 7) | (self.uplink_indication as u16) << 3 | rssi << 8
    }
}

fn align(b: &mut Vec<u8>, n: usize) {
    while !b.len().is_multiple_of(n) {
        b.push(0);
    }
}

fn dbm(v: f32) -> i8 {
    v.round().clamp(-128.0, 127.0) as i8
}

fn file_header() -> [u8; 24] {
    let mut h = [0u8; 24];
    h[0..4].copy_from_slice(&0xa1b2_c3d4u32.to_le_bytes());
    h[4..6].copy_from_slice(&2u16.to_le_bytes());
    h[6..8].copy_from_slice(&4u16.to_le_bytes());
    h[16..20].copy_from_slice(&65535u32.to_le_bytes());
    h[20..24].copy_from_slice(&LINKTYPE.to_le_bytes());
    h
}

fn record(t_us: u64, radiotap: &[u8], mpdu: &[u8]) -> Vec<u8> {
    let len = (radiotap.len() + mpdu.len()) as u32;
    let mut rec = Vec::with_capacity(16 + len as usize);
    rec.extend_from_slice(&((t_us / 1_000_000) as u32).to_le_bytes());
    rec.extend_from_slice(&((t_us % 1_000_000) as u32).to_le_bytes());
    rec.extend_from_slice(&len.to_le_bytes());
    rec.extend_from_slice(&len.to_le_bytes());
    rec.extend_from_slice(radiotap);
    rec.extend_from_slice(mpdu);
    rec
}

enum Sink {
    Stream(Box<dyn Write + Send>),
    Pipe { records: SyncSender<Vec<u8>>, attached: Arc<AtomicBool> },
}

pub struct PcapWriter {
    sink: Sink,
    ampdu_reference: u32,
    dropped: u64,
    drop_reported: Option<Instant>,
}

impl PcapWriter {
    /// `path` is a file (created), `-` for standard output, a Windows named
    /// pipe `\\.\pipe\NAME` (created) or an existing FIFO. Pipe readers
    /// attach and detach at any time; each gets its own file header, and
    /// records written while nobody reads are discarded.
    pub fn open(path: &Path) -> Result<Self> {
        let mut sink = if path.as_os_str() == "-" {
            Sink::Stream(Box::new(std::io::stdout()))
        } else if let Some(server) = pipe::Server::open(path).with_context(|| format!("pipe {}", path.display()))? {
            let (records, backlog) = sync_channel(PIPE_BACKLOG);
            let attached = Arc::new(AtomicBool::new(false));
            let flag = attached.clone();
            std::thread::spawn(move || serve(server, backlog, flag));
            Sink::Pipe { records, attached }
        } else {
            let f = std::fs::File::create(path).with_context(|| format!("create {}", path.display()))?;
            Sink::Stream(Box::new(f))
        };
        if let Sink::Stream(s) = &mut sink {
            s.write_all(&file_header())?;
            s.flush()?;
        }
        Ok(Self { sink, ampdu_reference: 0, dropped: 0, drop_reported: None })
    }

    /// True while a pipe reader is attached (always for a file or stdout).
    pub fn attached(&self) -> bool {
        match &self.sink {
            Sink::Stream(_) => true,
            Sink::Pipe { attached, .. } => attached.load(Ordering::Relaxed),
        }
    }

    /// One MPDU (with FCS) at `t_us` since the Unix epoch.
    pub fn write(&mut self, t_us: u64, radiotap: &Radiotap, mpdu: &[u8]) -> Result<()> {
        let rec = record(t_us, &radiotap.encode(), mpdu);
        let mut full = false;
        match &mut self.sink {
            Sink::Stream(w) => {
                w.write_all(&rec)?;
                w.flush()?;
            }
            Sink::Pipe { records, attached } => {
                if attached.load(Ordering::Relaxed) {
                    match records.try_send(rec) {
                        Ok(()) => {}
                        Err(TrySendError::Full(_)) => full = true,
                        Err(TrySendError::Disconnected(_)) => bail!("pipe thread exited"),
                    }
                }
            }
        }
        if full {
            self.dropped += 1;
            if self.drop_reported.is_none_or(|t| t.elapsed() >= Duration::from_secs(30)) {
                eprintln!("pcap: {} records dropped, the pipe reader is not keeping up", self.dropped);
                self.drop_reported = Some(Instant::now());
            }
        }
        Ok(())
    }

    /// Every MPDU of a PSDU, each with its own FCS verdict; the subframes
    /// of an A-MPDU share a reference number.
    pub fn write_psdu(&mut self, t_us: u64, base: Radiotap, aggregation: bool, psdu: &[u8]) -> Result<()> {
        let parts = ampdu::split_psdu(psdu, aggregation);
        let reference = aggregation.then(|| {
            self.ampdu_reference += 1;
            self.ampdu_reference
        });
        let n = parts.len();
        for (i, (mpdu, eof)) in parts.into_iter().enumerate() {
            let mut rt = base;
            rt.fcs_ok = fcs::check_and_strip(mpdu).is_some();
            rt.ampdu = reference.map(|reference| AmpduStatus { reference, last: i + 1 == n, eof });
            self.write(t_us, &rt, mpdu)?;
        }
        Ok(())
    }
}

/// Serves one pipe reader after another: file header, then records until
/// the reader goes away.
fn serve(mut server: pipe::Server, records: Receiver<Vec<u8>>, attached: Arc<AtomicBool>) {
    loop {
        let mut reader = match server.connect() {
            Ok(r) => r,
            Err(e) => {
                eprintln!("pcap pipe: {e}");
                std::thread::sleep(Duration::from_secs(1));
                continue;
            }
        };
        while records.try_recv().is_ok() {}
        attached.store(true, Ordering::Relaxed);
        let mut alive = reader.write_all(&file_header()).and_then(|_| reader.flush()).is_ok();
        while alive {
            match records.recv_timeout(PIPE_PROBE_INTERVAL) {
                Ok(rec) => alive = reader.write_all(&rec).and_then(|_| reader.flush()).is_ok(),
                Err(RecvTimeoutError::Timeout) => alive = !reader.detached(),
                Err(RecvTimeoutError::Disconnected) => return,
            }
        }
        attached.store(false, Ordering::Relaxed);
    }
}

#[cfg(windows)]
mod pipe {
    use std::fs::File;
    use std::io::{self, Write};
    use std::os::windows::io::{AsRawHandle, FromRawHandle};
    use std::path::Path;
    use windows_sys::Win32::Foundation::{GetLastError, ERROR_PIPE_CONNECTED, INVALID_HANDLE_VALUE};
    use windows_sys::Win32::Storage::FileSystem::PIPE_ACCESS_DUPLEX;
    use windows_sys::Win32::System::Pipes::{ConnectNamedPipe, CreateNamedPipeW, DisconnectNamedPipe, PeekNamedPipe, PIPE_READMODE_BYTE, PIPE_TYPE_BYTE, PIPE_WAIT};

    /// Server end of a named pipe (one instance); the `File` owns the handle.
    pub struct Server {
        pipe: File,
    }

    impl Server {
        /// `Some` for a `\\.\pipe\NAME` path, which is created here.
        pub fn open(path: &Path) -> io::Result<Option<Self>> {
            let name = path.to_string_lossy().replace('/', "\\");
            if !name.to_ascii_lowercase().starts_with(r"\\.\pipe\") {
                return Ok(None);
            }
            let wide: Vec<u16> = name.encode_utf16().chain(std::iter::once(0)).collect();
            let h = unsafe {
                CreateNamedPipeW(wide.as_ptr(), PIPE_ACCESS_DUPLEX, PIPE_TYPE_BYTE | PIPE_READMODE_BYTE | PIPE_WAIT, 1, 1 << 20, 0, 0, std::ptr::null())
            };
            if h == INVALID_HANDLE_VALUE {
                return Err(io::Error::last_os_error());
            }
            Ok(Some(Self { pipe: unsafe { File::from_raw_handle(h as _) } }))
        }

        /// Blocks until a reader opens the pipe.
        pub fn connect(&mut self) -> io::Result<Reader<'_>> {
            let ok = unsafe { ConnectNamedPipe(self.pipe.as_raw_handle() as _, std::ptr::null_mut()) };
            if ok == 0 && unsafe { GetLastError() } != ERROR_PIPE_CONNECTED {
                let e = io::Error::last_os_error();
                unsafe { DisconnectNamedPipe(self.pipe.as_raw_handle() as _) };
                return Err(e);
            }
            Ok(Reader { server: self })
        }
    }

    /// One attached reader; detached on drop.
    pub struct Reader<'a> {
        server: &'a mut Server,
    }

    impl Reader<'_> {
        /// True once the reader has closed its end.
        pub fn detached(&mut self) -> bool {
            let mut avail = 0u32;
            unsafe { PeekNamedPipe(self.server.pipe.as_raw_handle() as _, std::ptr::null_mut(), 0, std::ptr::null_mut(), &mut avail, std::ptr::null_mut()) == 0 }
        }
    }

    impl Write for Reader<'_> {
        fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
            self.server.pipe.write(buf)
        }
        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }

    impl Drop for Reader<'_> {
        fn drop(&mut self) {
            unsafe { DisconnectNamedPipe(self.server.pipe.as_raw_handle() as _) };
        }
    }
}

#[cfg(unix)]
mod pipe {
    use std::fs::{File, OpenOptions};
    use std::io::{self, Write};
    use std::os::unix::fs::FileTypeExt;
    use std::os::unix::io::AsRawFd;
    use std::path::{Path, PathBuf};

    /// A FIFO; each reader is served by opening it for writing.
    pub struct Server {
        path: PathBuf,
    }

    impl Server {
        /// `Some` when `path` is an existing FIFO.
        pub fn open(path: &Path) -> io::Result<Option<Self>> {
            match std::fs::metadata(path) {
                Ok(m) if m.file_type().is_fifo() => Ok(Some(Self { path: path.to_path_buf() })),
                _ => Ok(None),
            }
        }

        /// Blocks until a reader opens the FIFO.
        pub fn connect(&mut self) -> io::Result<Reader> {
            Ok(Reader { fifo: OpenOptions::new().write(true).open(&self.path)? })
        }
    }

    /// One attached reader; the write end closes on drop.
    pub struct Reader {
        fifo: File,
    }

    impl Reader {
        /// True once no reader holds the FIFO (POLLERR on the write end).
        pub fn detached(&mut self) -> bool {
            let mut p = libc::pollfd { fd: self.fifo.as_raw_fd(), events: 0, revents: 0 };
            let ready = unsafe { libc::poll(&mut p, 1, 0) };
            ready > 0 && p.revents & libc::POLLERR != 0
        }
    }

    impl Write for Reader {
        fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
            self.fifo.write(buf)
        }
        fn flush(&mut self) -> io::Result<()> {
            self.fifo.flush()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Read;
    use std::path::PathBuf;

    fn rx_header() -> Radiotap {
        Radiotap::rx(&RxVector { mcs: 3, rcpi_dbm: -61.4, ..Default::default() }, Some(1_250_000_000.0))
    }

    fn data_frame(fcs_valid: bool) -> Vec<u8> {
        let mut m = vec![0x08, 0x00, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 0x10, 0x00, 0xaa, 0xbb];
        fcs::append(&mut m);
        if !fcs_valid {
            m[5] ^= 1;
        }
        m
    }

    /// (seconds, microseconds, radiotap header, frame) of each record.
    fn records_of(bytes: &[u8]) -> Vec<(u32, u32, &[u8], &[u8])> {
        let mut out = Vec::new();
        let mut pos = 24;
        while pos + 16 <= bytes.len() {
            let u = |i: usize| u32::from_le_bytes(bytes[pos + i..pos + i + 4].try_into().unwrap());
            let (s, us, len) = (u(0), u(4), u(8) as usize);
            let pkt = &bytes[pos + 16..pos + 16 + len];
            let rl = u16::from_le_bytes([pkt[2], pkt[3]]) as usize;
            out.push((s, us, &pkt[..rl], &pkt[rl..]));
            pos += 16 + len;
        }
        out
    }

    #[test]
    fn radiotap_fields_are_aligned_and_sized() {
        let h = rx_header().encode();
        assert_eq!(h.len(), 28);
        assert_eq!(u16::from_le_bytes([h[2], h[3]]), 28);
        assert_eq!(u32::from_le_bytes([h[4], h[5], h[6], h[7]]), PRESENT_FLAGS | PRESENT_CHANNEL | PRESENT_DBM_ANTSIGNAL | PRESENT_TLVS);
        assert_eq!(h[8], FLAG_FCS);
        assert_eq!(u16::from_le_bytes([h[10], h[11]]), 1250);
        assert_eq!(h[14] as i8, -61);
        assert_eq!(&h[16..22], &[32, 0, 6, 0, 0xff, 0]);
        let data1 = u16::from_le_bytes([h[22], h[23]]);
        assert_eq!(data1 & 3, 1);
        assert_eq!((data1 >> 8) & 0xf, 1);
        assert_eq!(data1 >> 12, 3);
        let data2 = u16::from_le_bytes([h[24], h[25]]);
        assert_eq!((data2 >> 8) as u8 as i8, -61);

        let mut tx = Radiotap::tx(&TxVector { mcs: 7, gi: GuardInterval::Short, ..Default::default() }, None);
        tx.ampdu = Some(AmpduStatus { reference: 9, last: true, eof: false });
        let h = tx.encode();
        assert_eq!(h.len(), 32);
        assert_eq!(u32::from_le_bytes([h[4], h[5], h[6], h[7]]), PRESENT_FLAGS | PRESENT_TX_FLAGS | PRESENT_AMPDU_STATUS | PRESENT_TLVS);
        assert_eq!(h[8], FLAG_FCS | FLAG_SHORT_GI);
        assert_eq!(u16::from_le_bytes([h[10], h[11]]), TX_FLAG_NOACK);
        assert_eq!(u32::from_le_bytes([h[12], h[13], h[14], h[15]]), 9);
        assert_eq!(u16::from_le_bytes([h[16], h[17]]), AMPDU_LAST_KNOWN | AMPDU_EOF_KNOWN | AMPDU_IS_LAST);
        assert_eq!(&h[20..24], &[32, 0, 6, 0]);
        assert_eq!(u16::from_le_bytes([h[26], h[27]]) >> 12, 7);
    }

    #[test]
    fn file_holds_one_record_per_mpdu_with_its_fcs_verdict() {
        let path = std::env::temp_dir().join(format!("s2g-pcap-{}.pcap", std::process::id()));
        let (good, bad) = (data_frame(true), data_frame(false));
        let psdu = ampdu::aggregate_many(&[&good, &bad], 200);
        let mut w = PcapWriter::open(&path).unwrap();
        w.write_psdu(1_700_000_000_123_456, rx_header(), true, &psdu).unwrap();
        w.write_psdu(1_700_000_001_000_000, rx_header(), false, &good).unwrap();
        drop(w);
        let bytes = std::fs::read(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        assert_eq!(&bytes[20..24], &LINKTYPE.to_le_bytes());
        let recs = records_of(&bytes);
        assert_eq!(recs.len(), 3);
        let (s, us, rt, frame) = recs[0];
        assert_eq!((s, us), (1_700_000_000, 123_456));
        assert_eq!(frame, &good[..]);
        assert_eq!(rt[8], FLAG_FCS);
        assert_eq!(u32::from_le_bytes(rt[16..20].try_into().unwrap()), 1);
        assert_eq!(u16::from_le_bytes([rt[20], rt[21]]), AMPDU_LAST_KNOWN | AMPDU_EOF_KNOWN);
        let (_, _, rt, frame) = recs[1];
        assert_eq!(frame, &bad[..]);
        assert_eq!(rt[8], FLAG_FCS | FLAG_BAD_FCS);
        assert_eq!(u16::from_le_bytes([rt[20], rt[21]]), AMPDU_LAST_KNOWN | AMPDU_EOF_KNOWN | AMPDU_IS_LAST);
        let (_, _, rt, frame) = recs[2];
        assert_eq!(frame, &good[..]);
        assert_eq!(rt.len(), 28);
    }

    fn pipe_path() -> PathBuf {
        #[cfg(windows)]
        {
            PathBuf::from(format!(r"\\.\pipe\s2g-pcap-test-{}", std::process::id()))
        }
        #[cfg(unix)]
        {
            let p = std::env::temp_dir().join(format!("s2g-pcap-test-{}", std::process::id()));
            assert!(std::process::Command::new("mkfifo").arg(&p).status().unwrap().success());
            p
        }
    }

    /// Opens the pipe as a reader (retrying while the server end is still
    /// busy with the previous reader) and returns its file header and first
    /// record.
    fn read_one(path: PathBuf) -> std::thread::JoinHandle<([u8; 24], Vec<u8>)> {
        std::thread::spawn(move || {
            let deadline = Instant::now() + Duration::from_secs(10);
            let mut f = loop {
                match std::fs::File::open(&path) {
                    Ok(f) => break f,
                    Err(e) if Instant::now() < deadline => {
                        let _ = e;
                        std::thread::sleep(Duration::from_millis(10));
                    }
                    Err(e) => panic!("open {}: {e}", path.display()),
                }
            };
            let mut header = [0u8; 24];
            f.read_exact(&mut header).unwrap();
            let mut rec_hdr = [0u8; 16];
            f.read_exact(&mut rec_hdr).unwrap();
            let len = u32::from_le_bytes(rec_hdr[8..12].try_into().unwrap()) as usize;
            let mut body = vec![0u8; len];
            f.read_exact(&mut body).unwrap();
            (header, body)
        })
    }

    #[test]
    fn pipe_readers_each_get_a_header_and_may_detach() {
        let path = pipe_path();
        let mut w = PcapWriter::open(&path).unwrap();
        let (rt, mpdu) = (rx_header(), data_frame(true));
        for round in 0..2u64 {
            let reader = read_one(path.clone());
            let deadline = Instant::now() + Duration::from_secs(10);
            while !reader.is_finished() {
                assert!(Instant::now() < deadline, "reader {round} starved");
                w.write(round + 1, &rt, &mpdu).unwrap();
                std::thread::sleep(Duration::from_millis(5));
            }
            let (header, body) = reader.join().unwrap();
            assert_eq!(&header[20..24], &LINKTYPE.to_le_bytes());
            assert_eq!(&body[body.len() - mpdu.len()..], &mpdu[..]);
            let deadline = Instant::now() + Duration::from_secs(10);
            while w.attached() {
                assert!(Instant::now() < deadline, "detach of reader {round} not noticed");
                std::thread::sleep(Duration::from_millis(20));
            }
        }
        #[cfg(unix)]
        let _ = std::fs::remove_file(&path);
    }
}
