#!/usr/bin/env python3
"""Regression check of the receiver against the three real HaLow recordings.

Runs the release s2g-rx over each capture (see README "Validation on real
captures") and asserts the documented counts, so a PHY change that silently
loses frames fails loudly. The recordings live outside git; point --data at
the directory that holds destevez/, sigidwiki/ and fontaine/.

    python scripts/validate_captures.py            # everything (~10 min: the 20 MS/s WAVs are slow)
    python scripts/validate_captures.py --quick    # baby monitor + imec only (~2 min)
"""
import argparse
import os
import re
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)


def rx_binary():
    exe = os.path.join(ROOT, "target", "release", "s2g-rx.exe" if os.name == "nt" else "s2g-rx")
    print("building s2g-rx (release)...", flush=True)
    subprocess.run(["cargo", "build", "--release", "-p", "s2g-tools", "--bin", "s2g-rx"], cwd=ROOT, check=True)
    return exe


def run_rx(exe, args):
    t0 = time.time()
    p = subprocess.run([exe] + args, cwd=ROOT, capture_output=True, text=True, encoding="utf-8", errors="replace")
    if p.returncode != 0:
        print(p.stderr[-2000:])
        raise SystemExit(f"s2g-rx failed: {' '.join(args)}")
    return p.stderr, time.time() - t0


def summary_counts(stderr):
    m = re.search(r"MPDUs: (\d+) \| FCS ok: (\d+)", stderr)
    starts = re.search(r"RXSTART short: (\d+) long: (\d+)", stderr)
    return {
        "mpdus": int(m.group(1)) if m else 0,
        "fcs_ok": int(m.group(2)) if m else 0,
        "short": int(starts.group(1)) if starts else 0,
        "long": int(starts.group(2)) if starts else 0,
    }


class Report:
    def __init__(self):
        self.rows = []
        self.failed = False

    def check(self, name, value, minimum, note=""):
        ok = value >= minimum
        self.failed |= not ok
        self.rows.append((name, value, minimum, "ok" if ok else "REGRESSION", note))

    def print(self):
        w = max(len(r[0]) for r in self.rows) if self.rows else 10
        print(f"\n{'check':{w}s} {'value':>8s} {'minimum':>8s}  result")
        for name, value, minimum, result, note in self.rows:
            print(f"{name:{w}s} {value:8d} {minimum:8d}  {result} {note}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data", default=os.path.join(ROOT, "data"), help="directory with destevez/, sigidwiki/, fontaine/")
    ap.add_argument("--quick", action="store_true", help="skip the 20 MS/s WAV captures")
    args = ap.parse_args()
    exe = rx_binary()
    rep = Report()
    tmp = tempfile.mkdtemp(prefix="s2g-validate-")

    # 1. Baby monitor (Estévez): byte-exact comparison against the reference PCAP.
    bm = os.path.join(args.data, "destevez", "baby-monitor.sigmf")
    truth = os.path.join(args.data, "destevez", "802_11_ah.pcap")
    if os.path.exists(bm) and os.path.exists(truth):
        pcap = os.path.join(tmp, "s2g.pcap")
        err, dt = run_rx(exe, ["--in", bm, "--mac", "--quiet", "--pcap", pcap])
        c = summary_counts(err)
        print(f"baby monitor: {c} in {dt:.0f} s")
        cmp = subprocess.run([sys.executable, os.path.join(HERE, "compare_pcap.py"), truth, pcap], capture_output=True, text=True, encoding="utf-8")
        print(cmp.stdout)
        m = re.search(r"FCS-valid frames in truth: (\d+); byte-exact matches: (\d+)", cmp.stdout)
        data = re.search(r"^Data\s+(\d+)\s+(\d+)\s+(\d+)", cmp.stdout, re.M)
        rep.check("baby-monitor byte-exact frames", int(m.group(2)) if m else 0, 2740, "of 2745 FCS-valid")
        rep.check("baby-monitor S1G_LONG data frames exact", int(data.group(3)) if data else 0, 1270, "of 1278")
        rep.check("baby-monitor S1G_LONG PPDUs decoded", c["long"], 1072)
    else:
        print("baby monitor capture not found, skipped")

    # 2. imec Sub-GHz dataset: one 2 MHz file (RTL-SDR glitches cost ~1 %).
    imec = os.path.join(args.data, "fontaine", "mat", "80211ah_mcs0_chan2_g0.0dB_att10dB_freq864.0MHz_0.cf32")
    if os.path.exists(imec):
        err, dt = run_rx(exe, ["--in", imec, "--rate-hz", "2.048e6", "--mac", "--quiet"])
        c = summary_counts(err)
        print(f"imec: {c} in {dt:.0f} s")
        rep.check("imec MPDUs FCS-valid", c["fcs_ok"], 1560, "of 1585 PPDUs")
    else:
        print("imec capture not found, skipped")

    # 3. sigidwiki HaLow router (20 MS/s WAVs; slow).
    if not args.quick:
        router = os.path.join(args.data, "sigidwiki", "baseband_862004550Hz_09-28-46_19-07-2026.wav")
        transfer = os.path.join(args.data, "sigidwiki", "baseband_862004550Hz_09-40-38_19-07-2026.wav")
        if os.path.exists(router):
            for shift, minimum in [("2.0e6", 42), ("4.0e6", 42)]:
                err, dt = run_rx(exe, ["--in", router, "--shift-hz", shift, "--duration-sec", "6", "--mac", "--quiet"])
                c = summary_counts(err)
                print(f"router +{shift}: {c} in {dt:.0f} s")
                rep.check(f"router +{shift} MPDUs FCS-valid", c["fcs_ok"], minimum, f"of {c['mpdus']}")
                rep.check(f"router +{shift} no FCS failures", c["mpdus"] - c["fcs_ok"] == 0, 1)
        else:
            print("sigidwiki router capture not found, skipped")
        if os.path.exists(transfer):
            err, dt = run_rx(exe, ["--in", transfer, "--shift-hz", "4.0e6", "--skip-sec", "5", "--duration-sec", "8", "--mac", "--quiet"])
            c = summary_counts(err)
            print(f"transfer +4.0e6: {c} in {dt:.0f} s")
            rep.check("transfer S1G_SHORT PPDUs", c["short"], 263)
            rep.check("transfer S1G_LONG PPDUs decoded", c["long"], 109)
            rep.check("transfer MPDUs FCS-valid", c["fcs_ok"], 266)
        else:
            print("sigidwiki transfer capture not found, skipped")

    rep.print()
    if rep.failed:
        print("\nREGRESSION: at least one count fell below its documented minimum")
        sys.exit(1)
    print("\nall captures at or above the documented counts")


if __name__ == "__main__":
    main()
