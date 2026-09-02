"""Compare the frames s2g-rx decoded (s2g.pcap, link type 105) with Daniel
Estévez's ground truth (802_11_ah.pcap, radiotap), per frame type and
byte-exact (FCS included in both).

The reference PCAP also contains frames whose FCS failed (radiotap Flags
bit 0x40); s2g only emits FCS-valid MPDUs, so those are reported separately
and excluded from the match statistics. Flags bit 0x80 marks short-GI PPDUs."""
import collections
import struct
import sys
import zlib


def read_pcap(path):
    f = open(path, "rb").read()
    linktype = struct.unpack("<I", f[20:24])[0]
    pos = 24
    out = []
    while pos + 16 <= len(f):
        ts_s, ts_us, incl, _ = struct.unpack("<IIII", f[pos:pos + 16])
        pos += 16
        pkt = f[pos:pos + incl]
        pos += incl
        flags = None
        if linktype == 127:
            rl = struct.unpack("<H", pkt[2:4])[0]
            present = struct.unpack("<I", pkt[4:8])[0]
            if present & 2 and not present & 1:
                flags = pkt[8]
            pkt = pkt[rl:]
        out.append((ts_s + ts_us / 1e6, pkt, flags))
    return out


def kind(fr):
    fc0 = fr[0]
    t, s = (fc0 >> 2) & 3, fc0 >> 4
    return {(2, 0): "Data", (1, 11): "RTS", (1, 7): "CtrlWrapper", (3, 1): "S1GBeacon", (0, 13): "Action"}.get((t, s), f"type{t}/sub{s}")


def fcs_ok(fr):
    return len(fr) > 4 and zlib.crc32(fr[:-4]) & 0xFFFFFFFF == struct.unpack("<I", fr[-4:])[0]


truth = read_pcap(sys.argv[1] if len(sys.argv) > 1 else "802_11_ah.pcap")
ours = read_pcap(sys.argv[2] if len(sys.argv) > 2 else "s2g.pcap")
bad = [(t, p) for t, p, _ in truth if not fcs_ok(p)]
valid = [(t, p) for t, p, _ in truth if fcs_ok(p)]
print(f"ground truth: {len(truth)} frames ({len(valid)} FCS-valid, {len(bad)} flagged bad FCS, "
      f"{sum(1 for _, _, fl in truth if fl is not None and fl & 0x80)} from short-GI PPDUs) | s2g: {len(ours)} frames")
tc = collections.Counter(kind(p) for _, p in valid)
oc = collections.Counter(kind(p) for _, p, _ in ours)
print(f"{'type':12s} {'valid':>6s} {'s2g':>6s} {'exact':>6s} {'missing':>8s} {'extra':>6s}")
for k in sorted(set(tc) | set(oc)):
    tb = collections.Counter(p for _, p in valid if kind(p) == k)
    ob = collections.Counter(p for _, p, _ in ours if kind(p) == k)
    missing = sum((tb - ob).values())
    extra = sum((ob - tb).values())
    print(f"{k:12s} {tc.get(k, 0):6d} {oc.get(k, 0):6d} {tc.get(k, 0) - missing:6d} {missing:8d} {extra:6d}")

truth_bytes = collections.Counter(p for _, p in valid)
our_bytes = collections.Counter(p for _, p, _ in ours)
missing = truth_bytes - our_bytes
extra = our_bytes - truth_bytes
n_truth = sum(truth_bytes.values())
print(f"\nFCS-valid frames in truth: {n_truth}; byte-exact matches: {n_truth - sum(missing.values())}; "
      f"missing: {sum(missing.values())}; extra (in s2g only): {sum(extra.values())}")
for p, n in list(missing.items())[:8]:
    print("  missing:", kind(p), len(p), p[:24].hex(" "), f"x{n}")
for p, n in list(extra.items())[:8]:
    print("  extra:  ", kind(p), len(p), p[:24].hex(" "), f"x{n}")
if bad:
    sizes = collections.Counter(len(p) for _, p in bad)
    print(f"bad-FCS frames in the reference (not recoverable by either decoder): {len(bad)}, sizes {sorted(sizes.items())}")
