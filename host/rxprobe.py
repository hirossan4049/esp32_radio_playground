#!/usr/bin/env python3
"""Host client for the esp32c3_rxprobe firmware.

  rxprobe.py cmd "INFO"                   send one text command, print the reply
  rxprobe.py cap --ctrl 2001 --n 16384 --tune 2437000 --gain 40 --out captures/x.bin
  rxprobe.py analyze captures/x.bin [--fs 40e6]

The capture file is raw little-endian uint32 words as written by the dump
engine; a JSON sidecar records the firmware's status line.
"""
import argparse, json, sys, time, struct
import serial

PORT = "/dev/cu.usbmodem2101"

def open_port(port):
    s = serial.Serial(port, 115200, timeout=2)
    s.dtr = True; s.rts = False   # leave the lines alone afterwards
    time.sleep(0.2)
    s.reset_input_buffer()
    return s

def cmd(s, text, wait=1.0):
    s.write((text + "\n").encode())
    deadline = time.time() + wait
    out = b""
    while time.time() < deadline:
        chunk = s.read(s.in_waiting or 1)
        if chunk:
            out += chunk
            if out.endswith(b"\r\n") and b"CAP " not in out:
                break
    return out.decode(errors="replace")

def capture(s, ctrl, n, command=None):
    s.reset_input_buffer()
    s.write((command or f"CAP {ctrl:x} {n}").encode() + b"\n")
    # status line; skip PHY/console log lines that may precede it
    extra = []
    for _ in range(50):
        line = s.read_until(b"\r\n", 4096).decode(errors="replace").strip()
        if line.startswith("CAP "):
            break
        if not line:
            raise RuntimeError("no CAP status line, got: %r" % extra)
        extra.append(line)
    else:
        raise RuntimeError("no CAP status line, got: %r" % extra)
    magic = s.read(4)
    if magic != b"\x31\x50\x41\x43":
        raise RuntimeError("bad magic %r" % magic)
    nwords = struct.unpack("<I", s.read(4))[0]
    data = bytearray()
    deadline = time.time() + 20
    while len(data) < nwords * 4 and time.time() < deadline:
        data += s.read(min(65536, nwords * 4 - len(data)))
    tail = s.read_until(b"END\r\n", 64)
    if len(data) != nwords * 4:
        raise RuntimeError("short payload %d/%d" % (len(data), nwords * 4))
    meta = dict(kv.split("=", 1) for kv in line[4:].split() if "=" in kv)
    meta["raw_status"] = line
    meta["log_lines"] = extra
    return bytes(data), meta

def analyze(path, fs=None, layout="s3"):
    import numpy as np
    words = np.fromfile(path, dtype="<u4")
    meta = {}
    try:
        meta = json.load(open(path + ".json"))
    except Exception:
        pass
    n = len(words)
    print(f"{path}: {n} words")
    unwritten = int(np.count_nonzero(words == 0xDEADBEEF))
    print(f"  unwritten sentinel words: {unwritten}")
    bits = [(int(np.count_nonzero(words & (1 << b)))) for b in range(32)]
    print("  bit usage (count of ones per bit 0..31):")
    print("   ", " ".join(f"{b}:{c}" for b, c in enumerate(bits)))
    uniq = np.unique(words)
    print(f"  distinct words: {len(uniq)}; first 16 words: {' '.join(f'{w:08x}' for w in words[:16])}")
    # S3 layout: I bits 0..9, Q bits 10..19, two's complement
    i = ((words & 1023) ^ 512).astype(np.int64) - 512
    q = (((words >> 10) & 1023) ^ 512).astype(np.int64) - 512
    print(f"  S3-layout I: mean {i.mean():.1f} std {i.std():.1f} min {i.min()} max {i.max()}")
    print(f"  S3-layout Q: mean {q.mean():.1f} std {q.std():.1f} min {q.min()} max {q.max()}")
    x = (i - i.mean()) + 1j * (q - q.mean())
    if fs is None and meta.get("cycles") and meta.get("n"):
        fs = int(meta["n"]) / (int(meta["cycles"]) / 160e6)
        print(f"  fs from fill time: {fs/1e6:.3f} MS/s (cycles={meta['cycles']} words={meta['n']})")
    fs = fs or 1.0
    nfft = 4096 if n >= 4096 else n
    X = np.fft.fftshift(np.fft.fft(x[:nfft] * np.hanning(nfft)))
    P = 20 * np.log10(np.abs(X) + 1e-9)
    f = np.fft.fftshift(np.fft.fftfreq(nfft, 1 / fs))
    med = np.median(P)
    idx = np.argsort(P)[::-1][:12]
    print(f"  FFT {nfft} pts, median {med:.1f} dB; top bins (freq offset, dB above median):")
    for k in sorted(idx, key=lambda k: -P[k]):
        print(f"    {f[k]/1e6:+9.4f} MHz  {P[k]-med:6.1f} dB")
    # power envelope over time (bursts = Wi-Fi packets?)
    seg = 256
    env = np.abs(x[: (n // seg) * seg].reshape(-1, seg)).mean(axis=1)
    print(f"  envelope over {len(env)} x {seg}-sample segments: min {env.min():.1f} max {env.max():.1f} median {np.median(env):.1f}")
    return words

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=PORT)
    sub = ap.add_subparsers(dest="mode", required=True)
    c = sub.add_parser("cmd"); c.add_argument("text", nargs="+")
    k = sub.add_parser("cap"); k.add_argument("--ctrl", default="2001"); k.add_argument("--n", type=int, default=16384)
    k.add_argument("--tune", type=int); k.add_argument("--gain", type=int); k.add_argument("--out", required=True); k.add_argument("--analyze", action="store_true"); k.add_argument("--adctrig", help="call librftest adctrig with these 9 args instead of CAP")
    a = sub.add_parser("analyze"); a.add_argument("path"); a.add_argument("--fs", type=float)
    args = ap.parse_args()
    if args.mode == "analyze":
        analyze(args.path, args.fs); return
    s = open_port(args.port)
    if args.mode == "cmd":
        for t in args.text:
            print(cmd(s, t), end="")
        return
    if args.tune: print(cmd(s, f"TUNE {args.tune}"), end="")
    if args.gain is not None: print(cmd(s, f"GAIN {args.gain}"), end="")
    data, meta = capture(s, int(args.ctrl, 16), args.n, command=("ADCTRIG " + args.adctrig) if args.adctrig else None)
    open(args.out, "wb").write(data)
    meta["port"] = args.port; meta["time"] = time.strftime("%Y-%m-%dT%H:%M:%S")
    json.dump(meta, open(args.out + ".json", "w"), indent=1)
    for l in meta.get("log_lines", []): print("  log:", l)
    print(meta["raw_status"])
    if args.analyze: analyze(args.out)

if __name__ == "__main__":
    main()
