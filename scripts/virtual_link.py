#!/usr/bin/env python3
"""End-to-end test of two s2g-node processes over the virtual Pluto.

Starts `s2g-virtual-pluto` with two radios (oscillators at +ppm/2 and
-ppm/2 so the alias search and the ppm report get exercised), two
`s2g-node` instances on Ethernet-over-UDP NICs, then pushes IPv6/UDP
frames between ULA addresses (what the good-neighbor filter lets through)
into node A and checks they emerge from node B (and back). Prints the
delivery count, the one-way latency of the real streaming path per
direction (measured while the frames are in flight), and the nodes' own log
lines (rate changes, peer carrier offset, identification, filter drops).

    python scripts/virtual_link.py                # 20 frames each way, 50 ms apart, 20 ppm apart
    python scripts/virtual_link.py --spacing-ms 0 # all 20 at once: exercises A-MPDU packing
    python scripts/virtual_link.py --frames 50 --ppm 60 --seconds 40
"""
import argparse
import os
import socket
import struct
import subprocess
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXE = ".exe" if os.name == "nt" else ""
A_MAC, B_MAC = "02:00:00:00:00:0a", "02:00:00:00:00:0b"
A_IP, B_IP = "fd00::a", "fd00::b"
UDP_PORT = 40000
IPV6_HDR_LEN, UDP_HDR_LEN, ETH_HDR_LEN = 40, 8, 14


def binary(name):
    return os.path.join(ROOT, "target", "release", name + EXE)


def udp6_frame(dst_mac, src_mac, dst_ip, src_ip, payload):
    """Ethernet II / IPv6 / UDP frame with a valid UDP checksum."""
    src, dst = socket.inet_pton(socket.AF_INET6, src_ip), socket.inet_pton(socket.AF_INET6, dst_ip)
    udp_len = UDP_HDR_LEN + len(payload)
    pseudo = src + dst + struct.pack("!IxxxB", udp_len, 17)
    udp = struct.pack("!HHHH", UDP_PORT, UDP_PORT, udp_len, 0) + payload
    data = pseudo + udp + (b"\0" if len(udp) % 2 else b"")
    total = sum(struct.unpack("!%dH" % (len(data) // 2), data))
    while total >> 16:
        total = (total & 0xFFFF) + (total >> 16)
    csum = (~total & 0xFFFF) or 0xFFFF
    udp = udp[:6] + struct.pack("!H", csum) + udp[8:]
    ip6 = struct.pack("!IHBB", 0x60000000, udp_len, 17, 64) + src + dst
    eth = bytes.fromhex(dst_mac.replace(":", "")) + bytes.fromhex(src_mac.replace(":", "")) + b"\x86\xdd"
    return eth + ip6 + udp


def udp_payload(frame):
    return frame[ETH_HDR_LEN + IPV6_HDR_LEN + UDP_HDR_LEN:]


def pump(proc, name, lines):
    for raw in proc.stderr:
        line = raw.decode("utf-8", "replace").rstrip()
        lines.append(f"[{name}] {line}")


def collect(sock, sent_at, store, stop):
    """Receive frames on `sock` until `stop` is set, stamping each first arrival."""
    sock.settimeout(0.05)
    while not stop.is_set():
        try:
            data, _ = sock.recvfrom(2000)
        except socket.timeout:
            continue
        payload = udp_payload(data)
        if payload in sent_at and payload not in store:
            store[payload] = time.time() - sent_at[payload]


def stats(lat):
    lat = sorted(lat)
    if not lat:
        return "none"
    pick = lambda q: lat[min(len(lat) - 1, int(q * len(lat)))]
    return f"min {lat[0]*1e3:.0f} ms, median {pick(0.5)*1e3:.0f} ms, p90 {pick(0.9)*1e3:.0f} ms, max {lat[-1]*1e3:.0f} ms"


def main():
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--frames", type=int, default=20)
    ap.add_argument("--spacing-ms", type=float, default=50.0, help="time between injected frames (0 = one burst)")
    ap.add_argument("--ppm", type=float, default=20.0, help="oscillator difference between the radios")
    ap.add_argument("--seconds", type=float, default=25.0, help="time budget for the whole exchange")
    ap.add_argument("--base-port", type=int, default=31431)
    ap.add_argument("--path-loss-db", type=float, default=30.0)
    ap.add_argument("--mcs", type=int, default=2)
    ap.add_argument("--max-median-ms", type=float, default=150.0, help="fail if the median one-way latency exceeds this")
    ap.add_argument("--node-args", default="", help="extra arguments for both s2g-nodes, e.g. \"--ampdu 1 --fixed-mcs\"")
    ap.add_argument("--no-build", action="store_true")
    ap.add_argument("--log", help="write every line the three processes printed to this file")
    args = ap.parse_args()
    if not args.no_build:
        subprocess.run(["cargo", "build", "--release", "-p", "s2g-tools", "--bin", "s2g-virtual-pluto", "--bin", "s2g-node"], cwd=ROOT, check=True)

    logs = []
    procs = []

    def spawn(name, cmd):
        p = subprocess.Popen(cmd, cwd=ROOT, stderr=subprocess.PIPE, stdout=subprocess.DEVNULL)
        threading.Thread(target=pump, args=(p, name, logs), daemon=True).start()
        procs.append(p)
        return p

    got_b, got_a = {}, {}
    try:
        spawn("air", [binary("s2g-virtual-pluto"), "--radios", "2", "--base-port", str(args.base_port), "--ppm", f"{args.ppm / 2},{-args.ppm / 2}", "--path-loss-db", str(args.path_loss_db)])
        time.sleep(0.5)
        common = ["--mcs", str(args.mcs), "--callsign", "N0CALL", "--id-info", "virtual-link", "--verbose"] + args.node_args.split()
        spawn("A", [binary("s2g-node"), "--uri", f"127.0.0.1:{args.base_port}", "--udp", "127.0.0.1:5001", "--udp-peer", "127.0.0.1:5002", "--mac", A_MAC] + common)
        spawn("B", [binary("s2g-node"), "--uri", f"127.0.0.1:{args.base_port + 1}", "--udp", "127.0.0.1:6001", "--udp-peer", "127.0.0.1:6002", "--mac", B_MAC] + common)
        time.sleep(2.5)  # radios up, noise floor measured

        out_a = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        out_a.bind(("127.0.0.1", 5002))
        out_b = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        out_b.bind(("127.0.0.1", 6002))
        inj = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        sent_at = {}
        stop = threading.Event()
        # Receive while injecting: a frame's latency is its own flight time,
        # not the time left in the injection loop.
        for sock, store in ((out_b, got_b), (out_a, got_a)):
            threading.Thread(target=collect, args=(sock, sent_at, store, stop), daemon=True).start()
        deadline = time.time() + args.seconds
        for i in range(args.frames):
            pa = b"A->B frame %03d " % i + bytes(range(40))
            pb = b"B->A frame %03d " % i + bytes(range(40))
            sent_at[pa] = sent_at[pb] = time.time()
            inj.sendto(udp6_frame(B_MAC, A_MAC, B_IP, A_IP, pa), ("127.0.0.1", 5001))
            inj.sendto(udp6_frame(A_MAC, B_MAC, A_IP, B_IP, pb), ("127.0.0.1", 6001))
            if args.spacing_ms > 0:
                time.sleep(args.spacing_ms / 1e3)
        while time.time() < deadline and (len(got_b) < args.frames or len(got_a) < args.frames):
            time.sleep(0.01)
        stop.set()
        print(f"\ndelivered A->B {len(got_b)}/{args.frames}, B->A {len(got_a)}/{args.frames}")
        print(f"latency A->B: {stats(got_b.values())}")
        print(f"latency B->A: {stats(got_a.values())}")
        time.sleep(1.0)
    finally:
        for p in procs:
            p.terminate()
        time.sleep(0.5)
    if args.log:
        with open(args.log, "w", encoding="utf-8") as f:
            f.write("\n".join(logs) + "\n")
    keys = ("rate", "noise floor", "id sent", "id heard", "oscillator", "DROPPED", "radio:", "virtual pluto", "WARNING", "error", "Error", "filter: dropped", "retries=", "rx end")
    interesting = [l for l in logs if any(k in l for k in keys)]
    print("\n".join(interesting[:80]))
    lat = sorted(list(got_b.values()) + list(got_a.values()))
    median_ok = bool(lat) and lat[len(lat) // 2] * 1e3 <= args.max_median_ms
    ok = len(got_b) == args.frames and len(got_a) == args.frames and median_ok
    if lat and not median_ok:
        print(f"\nmedian latency {lat[len(lat) // 2]*1e3:.0f} ms exceeds {args.max_median_ms:.0f} ms")
    print("\nvirtual link OK" if ok else "\nvirtual link FAILED")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
