#!/usr/bin/env python3
"""Host side for CSTREAM (buffered on-chip decimated I/Q) and USTREAM (continuous int8 I/Q over USB).

  cstream.py buf --decim 16 --shift 4 --tune 2437000 --out captures/cs.bin      # on-chip buffer, then upload
  cstream.py usb --decim 64 --shift 6 --seconds 5 --out captures/us.bin         # continuous USB stream
  cstream.py analyze captures/cs.bin --fs 1e6
"""
import argparse, json, struct, sys, time
sys.path.insert(0, "host"); import rxprobe

def read_exact(s, n, timeout=30):
    data = bytearray(); dl = time.time() + timeout
    while len(data) < n and time.time() < dl:
        data += s.read(min(65536, n - len(data)))
    return bytes(data)

def buf(args):
    s = rxprobe.open_port(args.port)
    if args.tune: print(rxprobe.cmd(s, f"TUNE {args.tune}").strip().splitlines()[-1])
    if args.gain is not None: rxprobe.cmd(s, f"GAIN {args.gain}")
    rxprobe.cmd(s, "WR 60033d90 000c2040")
    s.reset_input_buffer(); s.write(f"CSTREAM {args.decim} {args.n} {args.shift}\n".encode())
    line = ""
    for _ in range(50):
        line = s.read_until(b"\r\n", 4096).decode(errors="replace").strip()
        if line.startswith("CSTREAM "): break
    print(line)
    assert read_exact(s, 4) == b"\x33\x50\x41\x43", "bad magic"
    n = struct.unpack("<I", read_exact(s, 4))[0]
    data = read_exact(s, n * 2); s.read_until(b"END\r\n", 64)
    open(args.out, "wb").write(data)
    meta = dict(kv.split("=", 1) for kv in line.split()[1:] if "=" in kv); meta["fs"] = 16e6 / args.decim; meta["format"] = "int8 I,Q"
    json.dump(meta, open(args.out + ".json", "w"), indent=1)
    print(f"saved {len(data)} bytes = {n} samples at {16e6/args.decim/1e3:.1f} kS/s ({n*args.decim/16e6*1e3:.1f} ms)")
    if args.analyze: analyze_file(args.out, 16e6 / args.decim)

def usb(args):
    s = rxprobe.open_port(args.port)
    if args.tune: print(rxprobe.cmd(s, f"TUNE {args.tune}").strip().splitlines()[-1])
    rxprobe.cmd(s, "WR 60033d90 000c2040")
    s.reset_input_buffer(); s.write(f"USTREAM {args.decim} {args.shift}\n".encode())
    hdr = s.read_until(b"\r\n", 4096).decode(errors="replace").strip(); print(hdr)
    t0 = time.time(); data = bytearray()
    while time.time() - t0 < args.seconds:
        data += s.read(s.in_waiting or 1)
    s.write(b"x")                      # any byte stops the stream
    time.sleep(0.5); tail = s.read(s.in_waiting or 1)
    # the status line follows the binary; find it
    txt = (bytes(data) + tail)
    k = txt.rfind(b"CSTREAM decim=")
    status = txt[k:].decode(errors="replace").strip() if k >= 0 else "(no status line)"
    payload = txt[:k] if k >= 0 else bytes(txt)
    open(args.out, "wb").write(payload)
    rate = len(payload) / (time.time() - t0)
    print(status); print(f"received {len(payload)} bytes in {time.time()-t0:.1f} s = {rate/1e3:.1f} kB/s = {rate/2/1e3:.1f} kS/s (expected {16e6/args.decim/1e3:.1f} kS/s)")
    if args.analyze: analyze_file(args.out, 16e6 / args.decim)

def analyze_file(path, fs):
    import numpy as np
    raw = np.fromfile(path, dtype=np.int8)
    x = raw[0::2].astype(float) + 1j * raw[1::2].astype(float)
    x -= x.mean()
    n = len(x); print(f"{path}: {n} samples, {n/fs*1e3:.1f} ms, I std {raw[0::2].std():.1f} Q std {raw[1::2].std():.1f} |max| {np.abs(raw).max()}")
    nfft = 1024 if n >= 1024 else n
    nseg = n // nfft; acc = np.zeros(nfft); w = np.hanning(nfft)
    for k in range(nseg): acc += np.abs(np.fft.fft(x[k*nfft:(k+1)*nfft] * w)) ** 2
    P = np.fft.fftshift(10 * np.log10(acc / max(nseg, 1) + 1e-9)); f = np.fft.fftshift(np.fft.fftfreq(nfft, 1 / fs)); med = np.median(P)
    pk = rxprobe.peaks(P, f, 8.0, 8)
    print("  peaks: " + ", ".join(f"{f[k]/1e3:+.1f} kHz {d:.1f} dB" for k, d in pk))
    seg = max(1, int(fs * 1e-3))   # 1 ms envelope
    env = np.abs(x[:(n // seg) * seg].reshape(-1, seg)).mean(axis=1)
    print(f"  1 ms envelope: median {np.median(env):.1f} max {env.max():.1f} (max/median {env.max()/np.median(env):.1f}), bursts>2x: {(env > 2*np.median(env)).sum()}/{len(env)}")

if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--port", default=rxprobe.PORT)
    sub = ap.add_subparsers(dest="mode", required=True)
    b = sub.add_parser("buf"); b.add_argument("--decim", type=int, default=16); b.add_argument("--shift", type=int, default=4); b.add_argument("--n", type=int, default=0)
    b.add_argument("--tune", type=int); b.add_argument("--gain", type=int); b.add_argument("--out", required=True); b.add_argument("--analyze", action="store_true")
    u = sub.add_parser("usb"); u.add_argument("--decim", type=int, default=64); u.add_argument("--shift", type=int, default=6); u.add_argument("--seconds", type=float, default=5)
    u.add_argument("--tune", type=int); u.add_argument("--out", required=True); u.add_argument("--analyze", action="store_true")
    a = sub.add_parser("analyze"); a.add_argument("path"); a.add_argument("--fs", type=float, required=True)
    args = ap.parse_args()
    if args.mode == "buf": buf(args)
    elif args.mode == "usb": usb(args)
    else: analyze_file(args.path, args.fs)
