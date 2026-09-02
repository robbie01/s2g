#!/usr/bin/env python3
"""End-to-end test of two s2g-node processes over the virtual Pluto.

Starts `s2g-virtual-pluto` with two radios (oscillators at +ppm/2 and
-ppm/2 so the alias search and the ppm report get exercised), two
`s2g-node` instances on Ethernet-over-UDP NICs, then pushes Ethernet frames
into node A and checks they emerge from node B (and back). Prints the
delivery count, the round-trip latency of the real streaming path, and the
nodes' own log lines (rate changes, peer carrier offset, identification).

    python scripts/virtual_link.py                # 20 frames each way, 20 ppm apart
    python scripts/virtual_link.py --frames 50 --ppm 60 --seconds 40
"""
import argparse
import os
import socket
import subprocess
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXE = ".exe" if os.name == "nt" else ""
A_MAC, B_MAC = "02:00:00:00:00:0a", "02:00:00:00:00:0b"


def binary(name):
    return os.path.join(ROOT, "target", "release", name + EXE)


def eth_frame(dst, src, payload, ethertype=0x0800):
    return bytes.fromhex(dst.replace(":", "")) + bytes.fromhex(src.replace(":", "")) + ethertype.to_bytes(2, "big") + payload


def pump(proc, name, lines):
    for raw in proc.stderr:
        line = raw.decode("utf-8", "replace").rstrip()
        lines.append(f"[{name}] {line}")


def main():
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--frames", type=int, default=20)
    ap.add_argument("--ppm", type=float, default=20.0, help="oscillator difference between the radios")
    ap.add_argument("--seconds", type=float, default=25.0, help="time budget for the whole exchange")
    ap.add_argument("--base-port", type=int, default=31431)
    ap.add_argument("--path-loss-db", type=float, default=30.0)
    ap.add_argument("--mcs", type=int, default=2)
    args = ap.parse_args()
    subprocess.run(["cargo", "build", "--release", "-p", "s2g-tools", "--bin", "s2g-virtual-pluto", "--bin", "s2g-node"], cwd=ROOT, check=True)

    logs = []
    procs = []

    def spawn(name, cmd):
        p = subprocess.Popen(cmd, cwd=ROOT, stderr=subprocess.PIPE, stdout=subprocess.DEVNULL)
        threading.Thread(target=pump, args=(p, name, logs), daemon=True).start()
        procs.append(p)
        return p

    try:
        spawn("air", [binary("s2g-virtual-pluto"), "--radios", "2", "--base-port", str(args.base_port), "--ppm", f"{args.ppm / 2},{-args.ppm / 2}", "--path-loss-db", str(args.path_loss_db)])
        time.sleep(0.5)
        common = ["--mcs", str(args.mcs), "--callsign", "N0CALL", "--id-info", "virtual-link", "--verbose"]
        spawn("A", [binary("s2g-node"), "--uri", f"127.0.0.1:{args.base_port}", "--udp", "127.0.0.1:5001", "--udp-peer", "127.0.0.1:5002", "--mac", A_MAC] + common)
        spawn("B", [binary("s2g-node"), "--uri", f"127.0.0.1:{args.base_port + 1}", "--udp", "127.0.0.1:6001", "--udp-peer", "127.0.0.1:6002", "--mac", B_MAC] + common)
        time.sleep(2.5)  # radios up, noise floor measured

        out_a = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        out_a.bind(("127.0.0.1", 5002))
        out_b = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        out_b.bind(("127.0.0.1", 6002))
        out_a.settimeout(0.05)
        out_b.settimeout(0.05)
        inj = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        sent_at = {}
        got_b, got_a = {}, {}
        deadline = time.time() + args.seconds
        for i in range(args.frames):
            pa = b"A->B frame %03d " % i + bytes(range(40))
            pb = b"B->A frame %03d " % i + bytes(range(40))
            inj.sendto(eth_frame(B_MAC, A_MAC, pa), ("127.0.0.1", 5001))
            inj.sendto(eth_frame(A_MAC, B_MAC, pb), ("127.0.0.1", 6001))
            sent_at[pa] = sent_at[pb] = time.time()
            time.sleep(0.05)
        while time.time() < deadline and (len(got_b) < args.frames or len(got_a) < args.frames):
            for sock, store in ((out_b, got_b), (out_a, got_a)):
                try:
                    data, _ = sock.recvfrom(2000)
                except socket.timeout:
                    continue
                payload = data[14:]
                if payload in sent_at and payload not in store:
                    store[payload] = time.time() - sent_at[payload]
        lat = sorted(list(got_b.values()) + list(got_a.values()))
        print(f"\ndelivered A->B {len(got_b)}/{args.frames}, B->A {len(got_a)}/{args.frames}")
        if lat:
            print(f"latency: min {lat[0]*1e3:.0f} ms, median {lat[len(lat)//2]*1e3:.0f} ms, max {lat[-1]*1e3:.0f} ms")
        time.sleep(1.0)
    finally:
        for p in procs:
            p.terminate()
        time.sleep(0.5)
    interesting = [l for l in logs if any(k in l for k in ("rate", "noise floor", "id sent", "id heard", "oscillator", "DROPPED", "radio:", "virtual pluto", "WARNING", "error", "Error"))]
    print("\n".join(interesting[:60]))
    ok = len(got_b) == args.frames and len(got_a) == args.frames
    print("\nvirtual link OK" if ok else "\nvirtual link FAILED")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
