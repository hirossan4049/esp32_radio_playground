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
    cmd(s, "WR 60033d90 000c2040")           # byte selector; the Wi-Fi driver leaves it 0
    data, meta = capture(s, int(args.ctrl, 16), args.n, command=("ADCTRIG " + args.adctrig) if args.adctrig else None)
    open(args.out, "wb").write(data)
    meta["port"] = args.port; meta["time"] = time.strftime("%Y-%m-%dT%H:%M:%S")
    json.dump(meta, open(args.out + ".json", "w"), indent=1)
    for l in meta.get("log_lines", []): print("  log:", l)
    print(meta["raw_status"])
    if args.analyze: analyze(args.out)

if __name__ == "__main__" and not (len(sys.argv) > 1 and sys.argv[1] in ("sweep", "lockscan")):
    main()

# ---------------------------------------------------------------- sweep
def psd_db(words, nfft=1024):
    import numpy as np
    i = ((words & 1023) ^ 512).astype(np.float64) - 512
    q = (((words >> 10) & 1023) ^ 512).astype(np.float64) - 512
    x = (i - i.mean()) + 1j * (q - q.mean())
    nseg = len(x) // nfft
    w = np.hanning(nfft)
    acc = np.zeros(nfft)
    for k in range(nseg):
        X = np.fft.fft(x[k * nfft:(k + 1) * nfft] * w)
        acc += np.abs(X) ** 2
    P = 10 * np.log10(acc / nseg + 1e-9)
    return np.fft.fftshift(P)

def peaks(P, f, thresh_db=8.0, maxn=10):
    import numpy as np
    med = np.median(P)
    idx = [k for k in np.argsort(P)[::-1] if P[k] - med > thresh_db]
    out = []
    for k in idx:
        if all(abs(f[k] - f[j]) > 3 * (f[1] - f[0]) for j, _ in out):
            out.append((k, P[k] - med))
        if len(out) >= maxn:
            break
    return out

def sweep(port, los_khz, ctrl, n, fs, tag, selector=True):
    import numpy as np
    s = open_port(port)
    table = []
    for lo in los_khz:
        r = cmd(s, f"TUNE {lo}")
        if selector:
            cmd(s, "WR 60033d90 000c2040")
        data, meta = capture(s, ctrl, n)
        path = f"captures/{tag}_lo{lo}.bin"
        open(path, "wb").write(data)
        meta["lo_khz"] = lo; meta["fs"] = fs; meta["tune_reply"] = r.strip()
        json.dump(meta, open(path + ".json", "w"), indent=1)
        words = np.frombuffer(data, dtype="<u4")
        P = psd_db(words)
        f = np.fft.fftshift(np.fft.fftfreq(len(P), 1 / fs))
        pk = peaks(P, f)
        print(f"LO {lo/1000:.3f} MHz  {meta.get('cycles')} cyc  I/Q std {words.size}w  median {np.median(P):.1f} dB  | {r.strip().splitlines()[-1] if r.strip() else ''}")
        for k, d in pk:
            off = f[k]
            print(f"    offset {off/1e6:+8.3f} MHz  {d:5.1f} dB   -> RF if LO-RF: {lo/1e3 - off/1e6:9.3f} MHz   RF if RF-LO: {lo/1e3 + off/1e6:9.3f} MHz")
        table.append((lo, [(f[k], d) for k, d in pk]))
    return table

if __name__ == "__main__" and len(sys.argv) > 1 and sys.argv[1] == "sweep":
    ap = argparse.ArgumentParser()
    ap.add_argument("sweep"); ap.add_argument("--port", default=PORT)
    ap.add_argument("--los", default="2430000,2435000,2437000,2440000,2445000,2450000,2460000")
    ap.add_argument("--ctrl", default="13fff"); ap.add_argument("--n", type=int, default=16384)
    ap.add_argument("--fs", type=float, default=16e6); ap.add_argument("--tag", default="sweep")
    a = ap.parse_args()
    sweep(a.port, [int(x) for x in a.los.split(",")], int(a.ctrl, 16), a.n, a.fs, a.tag)
    sys.exit(0)

# ---------------------------------------------------------------- lockscan
def lockscan(port, lo_list_khz, marker_off_hz, ctrl=0x13fff, n=16384, fs=16e6, tag="lockscan"):
    """Tune to each LO, capture, and report the PSD level at a fixed baseband offset where an
    internal spur (40 MHz crystal harmonic at RF = LO - marker_off) should appear if the LO is real."""
    import numpy as np
    s = open_port(port)
    rows = []
    for lo in lo_list_khz:
        r = cmd(s, f"TUNE {lo}").strip().splitlines()
        r = r[-1] if r else ""
        cmd(s, "WR 60033d90 000c2040")
        try:
            data, meta = capture(s, ctrl, n)
        except Exception as e:
            print(f"LO {lo/1000:9.3f} MHz  capture failed: {e}"); continue
        words = np.frombuffer(data, dtype="<u4")
        path = f"captures/{tag}_lo{lo}.bin"; open(path, "wb").write(data)
        meta["lo_khz"] = lo; meta["tune_reply"] = r; json.dump(meta, open(path + ".json", "w"), indent=1)
        P = psd_db(words); f = np.fft.fftshift(np.fft.fftfreq(len(P), 1 / fs)); med = np.median(P)
        k = int(np.argmin(np.abs(f - marker_off_hz)))
        local = P[max(0, k - 2):k + 3].max() - med
        i = ((words & 1023) ^ 512).astype(np.float64) - 512
        best = max(((P[j] - med, f[j]) for j in range(len(P)) if abs(f[j]) > 0.1e6), key=lambda t: t[0])
        rows.append((lo, local, best, i.std(), r))
        print(f"LO {lo/1000:9.3f} MHz  marker@{marker_off_hz/1e6:+.1f}MHz {local:5.1f} dB  strongest {best[0]:5.1f} dB @ {best[1]/1e6:+7.3f} MHz  I std {i.std():6.1f}  | {r}")
    return rows

if __name__ == "__main__" and len(sys.argv) > 1 and sys.argv[1] == "lockscan":
    ap = argparse.ArgumentParser()
    ap.add_argument("lockscan"); ap.add_argument("--port", default=PORT)
    ap.add_argument("--start", type=int, default=2203000); ap.add_argument("--stop", type=int, default=2883000); ap.add_argument("--step", type=int, default=40000)
    ap.add_argument("--marker", type=float, default=-3e6); ap.add_argument("--tag", default="lockscan")
    a = ap.parse_args()
    lockscan(a.port, list(range(a.start, a.stop + 1, a.step)), a.marker, tag=a.tag)
    sys.exit(0)
