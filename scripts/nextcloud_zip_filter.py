"""Stream a Nextcloud public-share folder zip and keep only the entries whose
names contain a pattern, without storing the whole archive.

Usage: python nextcloud_zip_filter.py URL PATTERN OUT_DIR [MAX_ENTRIES]
The streamed zips use zip64 data descriptors, so entry ends are located by
the descriptor signature followed by the next header."""
import os
import re
import ssl
import struct
import sys
import urllib.request

url, pattern, out_dir = sys.argv[1], sys.argv[2], sys.argv[3]
max_entries = int(sys.argv[4]) if len(sys.argv) > 4 else 10**9
os.makedirs(out_dir, exist_ok=True)
ctx = ssl.create_default_context()
ctx.check_hostname = False
ctx.verify_mode = ssl.CERT_NONE
resp = urllib.request.urlopen(url, timeout=300, context=ctx)

buf = b""
eof = False
seen = 0
kept = 0


def fill(n):
    """Ensure at least n bytes are buffered (or EOF)."""
    global buf, eof
    while len(buf) < n and not eof:
        chunk = resp.read(4 << 20)
        if not chunk:
            eof = True
            break
        buf += chunk


while True:
    fill(30)
    if len(buf) < 30 or buf[:4] != b"PK\x03\x04":
        break
    ver, flags, method, mt, md, crc, csz, usz, nlen, elen = struct.unpack("<HHHHHIIIHH", buf[4:30])
    fill(30 + nlen + elen)
    name = buf[30:30 + nlen].decode("utf-8", "replace")
    extra = buf[30 + nlen:30 + nlen + elen]
    buf = buf[30 + nlen + elen:]
    z64 = None
    i = 0
    while i + 4 <= len(extra):
        eid, esz = struct.unpack("<HH", extra[i:i + 4])
        if eid == 1 and esz >= 16:
            z64 = struct.unpack("<QQ", extra[i + 4:i + 20])
        i += 4 + esz
    known = csz if csz != 0xFFFFFFFF else (z64[1] if z64 and z64[1] else None)
    want = bool(re.search(pattern, name)) and not name.endswith("/")
    out = open(os.path.join(out_dir, os.path.basename(name)), "wb") if want else None
    written = 0
    if known is not None:
        remaining = known
        while remaining > 0:
            fill(min(remaining, 4 << 20))
            take = min(remaining, len(buf))
            if out:
                out.write(buf[:take])
            written += take
            buf = buf[take:]
            remaining -= take
        if flags & 8:
            fill(4)
            if buf[:4] == b"PK\x07\x08":
                fill(24)
                buf = buf[24:]
    else:
        # Unknown size: scan for the data descriptor followed by a header.
        while True:
            fill(1 << 20)
            end = None
            pos = 0
            while True:
                j = buf.find(b"PK\x07\x08", pos)
                if j < 0:
                    break
                fill(j + 28)
                if buf[j + 24:j + 28] in (b"PK\x03\x04", b"PK\x01\x02"):
                    end, dlen = j, 24
                    break
                if buf[j + 16:j + 20] in (b"PK\x03\x04", b"PK\x01\x02"):
                    end, dlen = j, 16
                    break
                pos = j + 1
            if end is not None:
                if out:
                    out.write(buf[:end])
                written += end
                buf = buf[end + dlen:]
                break
            if eof:
                if out:
                    out.write(buf)
                buf = b""
                break
            # Flush all but a tail that could hold a partial signature.
            safe = max(0, len(buf) - 64)
            if out:
                out.write(buf[:safe])
            written += safe
            buf = buf[safe:]
    if out:
        out.close()
        kept += 1
    seen += 1
    print(f"{'KEEP' if want else 'skip'} {name} ({written / 1e6:.1f} MB)", flush=True)
    if kept >= max_entries:
        break
print(f"done: {seen} entries seen, {kept} kept", flush=True)
