"""Compare the frames s2g-rx decoded (s2g.pcap, link type 105) with Daniel
Estévez's ground truth (802_11_ah.pcap, radiotap). Data frames were sent as
S1G_LONG PPDUs, which s2g does not decode, so they are reported separately."""
import collections
import struct
import sys


def read_pcap(path):
    f = open(path, "rb").read()
    magic, _, _, _, _, _, linktype = struct.unpack("<IHHiIII", f[:24])
    pos = 24
    out = []
    while pos + 16 <= len(f):
        ts_s, ts_us, incl, _ = struct.unpack("<IIII", f[pos:pos + 16])
        pos += 16
        pkt = f[pos:pos + incl]
        pos += incl
        if linktype == 127:
            rl = struct.unpack("<H", pkt[2:4])[0]
            pkt = pkt[rl:]
        out.append((ts_s + ts_us / 1e6, pkt))
    return out


def kind(fr):
    fc0 = fr[0]
    t, s = (fc0 >> 2) & 3, fc0 >> 4
    return {(2, 0): "Data", (1, 11): "RTS", (1, 7): "CtrlWrapper", (3, 1): "S1GBeacon", (0, 13): "Action"}.get((t, s), f"type{t}/sub{s}")


truth = read_pcap(sys.argv[1] if len(sys.argv) > 1 else "802_11_ah.pcap")
ours = read_pcap(sys.argv[2] if len(sys.argv) > 2 else "s2g.pcap")
print(f"ground truth: {len(truth)} frames | s2g: {len(ours)} frames")
tc = collections.Counter(kind(p) for _, p in truth)
oc = collections.Counter(kind(p) for _, p in ours)
print(f"{'type':12s} {'truth':>6s} {'s2g':>6s}")
for k in sorted(set(tc) | set(oc)):
    print(f"{k:12s} {tc.get(k, 0):6d} {oc.get(k, 0):6d}")

# Exact-byte matching of the non-Data frames.
truth_bytes = collections.Counter(p for _, p in truth if kind(p) != "Data")
our_bytes = collections.Counter(p for _, p in ours)
missing = truth_bytes - our_bytes
extra = our_bytes - truth_bytes
n_truth = sum(truth_bytes.values())
print(f"\nnon-Data frames in truth: {n_truth}; byte-exact matches: {n_truth - sum(missing.values())}; "
      f"missing: {sum(missing.values())}; extra (in s2g only): {sum(extra.values())}")
for p, n in list(missing.items())[:5]:
    print("  missing:", kind(p), len(p), p[:24].hex(" "), f"x{n}")
for p, n in list(extra.items())[:5]:
    print("  extra:  ", kind(p), len(p), p[:24].hex(" "), f"x{n}")
