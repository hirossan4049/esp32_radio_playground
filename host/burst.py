#!/usr/bin/env python3
"""Host side for BURST (power time series + raw blocks above a threshold) and WIFICNT (MAC reference).

  burst.py wificnt --ms 3000 [--normal]                 count frames the chip's own MAC decodes
  burst.py series --tune 2437000 --gain 20 --ms 4000 --out captures/b_2437.json
  burst.py catch  --tune 2437000 --gain 20 --thr 60 --maxsend 6 --ms 6000 --out captures/b_2437
  burst.py period captures/b_2437.json                  look for the 102.4 ms beacon period in the series
"""
import argparse, json, struct, sys, time
sys.path.insert(0, "host"); import rxprobe

def read_line(s, timeout=10):
    dl = time.time() + timeout
    buf = b""
    while time.time() < dl:
        c = s.read(1)
        if not c: continue
        buf += c
        if buf.endswith(b"\r\n"): return buf[:-2].decode(errors="replace")
    raise RuntimeError("timeout, partial %r" % buf[:80])

def read_exact(s, n, timeout=30):
    data = bytearray(); dl = time.time() + timeout
    while len(data) < n and time.time() < dl:
        data += s.read(min(65536, n - len(data)))
    return bytes(data)

def setup(args):
    s = rxprobe.open_port(args.port)
    if getattr(args, "tune", None): print(rxprobe.cmd(s, f"TUNE {args.tune}").strip().splitlines()[-1])
    if getattr(args, "gain", None) is not None: print(rxprobe.cmd(s, f"GAIN {args.gain}").strip().splitlines()[-1])
    rxprobe.cmd(s, "WR 60033d90 000c2040")
    return s

def wificnt(args):
    s = setup(args)
    s.reset_input_buffer(); s.write(f"WIFICNT {args.ms} {1 if args.normal else 0}\n".encode())
    lines = []
    while True:
        l = read_line(s, args.ms / 1000 + 10)
        print(l); lines.append(l)
        if l.startswith("END WIFICNT"): break
    if args.out: json.dump(lines, open(args.out, "w"), indent=1)

def run_burst(s, args, thr, maxsend, outprefix):
    s.reset_input_buffer()
    s.write(f"BURST {args.ctrl} {args.n} {thr} {maxsend} {args.ms}\n".encode())
    series, blocks, hdr = [], [], None
    while True:
        l = read_line(s, args.ms / 1000 + 30)
        if l.startswith("BURST start"): hdr = l; print(l)
        elif l.startswith("CAP "):
            meta = dict(kv.split("=", 1) for kv in l[4:].split() if "=" in kv)
            assert read_exact(s, 4) == b"\x31\x50\x41\x43", "bad magic"
            n = struct.unpack("<I", read_exact(s, 4))[0]
            data = read_exact(s, n * 4); s.read_until(b"END\r\n", 64)
            fn = f"{outprefix}_blk{len(blocks):02d}.bin"
            open(fn, "wb").write(data); meta["file"] = fn; blocks.append(meta)
            print("  raw block", fn, meta["idx"], meta["t_us"], meta["mean"], meta["peak"])
        elif l.startswith("S "):
            series.append([int(x) for x in l.split()[1:]])
        elif l.startswith("BURST done"): print(l)
        elif l.startswith("END BURST"): break
    return dict(header=hdr, series=series, blocks=blocks, tune=getattr(args, "tune", None), gain=getattr(args, "gain", None))

def series_cmd(args):
    s = setup(args)
    r = run_burst(s, args, 0xFFFFFFFF, 0, args.out.replace(".json", ""))
    json.dump(r, open(args.out, "w"))
    summarize(r)

def catch_cmd(args):
    s = setup(args)
    r = run_burst(s, args, args.thr, args.maxsend, args.out)
    json.dump(r, open(args.out + ".json", "w"))
    summarize(r)

def summarize(r):
    import numpy as np
    S = np.array(r["series"], dtype=np.int64)
    if len(S) == 0: print("no series"); return
    t = (S[:, 1] - S[0, 1]) / 1e3; mean = S[:, 2]; peak = S[:, 3]
    print(f"blocks {len(S)} span {t[-1]:.1f} ms, block pitch {np.median(np.diff(t)):.3f} ms")
    print(f"mean|I|+|Q|: median {np.median(mean):.0f}  p90 {np.percentile(mean,90):.0f}  max {mean.max()}   peak: median {np.median(peak):.0f} max {peak.max()}  sat(>=500) blocks {int((peak>=500).sum())}")
    period(S)

def period(S):
    import numpy as np
    t = (S[:, 1] - S[0, 1]) / 1e6; mean = S[:, 2].astype(float)
    base = np.median(mean); thr = base * 1.5 + 5
    hot = t[mean > thr]
    print(f"bursty blocks (mean > {thr:.0f}): {len(hot)} of {len(S)}")
    if len(hot) < 4: return
    ev = [hot[0]]
    for x in hot[1:]:
        if x - ev[-1] > 0.0025: ev.append(x)
    ev = np.array(ev)
    print(f"events: {len(ev)} over {t[-1]:.2f} s = {len(ev)/t[-1]:.1f}/s; first intervals (ms): " + " ".join(f"{d*1e3:.1f}" for d in np.diff(ev)[:24]))
    for P in (0.1024, 0.1, 0.0960, 0.1100):
        # events having a partner one period later (within +-2 ms); and phase clustering mod P
        has = sum(1 for e in ev if np.any(np.abs(ev - (e + P)) < 0.002)) ; cand = int(np.sum(ev + P <= ev[-1] + 0.002))
        ph = (ev % P) / P * 2 * np.pi
        R = np.abs(np.mean(np.exp(1j * ph)))             # mean resultant length (1 = all at the same phase)
        hist, _ = np.histogram((ev % P) * 1e3, bins=int(P * 1e3 / 2))
        print(f"  period {P*1e3:6.1f} ms: partner at +P for {has}/{cand} events; phase histogram (2 ms bins) top bins {sorted(hist)[-4:]} of {len(ev)}")

def period_cmd(args):
    import numpy as np
    r = json.load(open(args.file)); S = np.array(r["series"], dtype=np.int64); summarize(r)

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--port", default=rxprobe.PORT)
    sub = ap.add_subparsers(dest="sub", required=True)
    a = sub.add_parser("wificnt"); a.add_argument("--ms", type=int, default=3000); a.add_argument("--normal", action="store_true"); a.add_argument("--tune", type=int); a.add_argument("--gain", type=int); a.add_argument("--out")
    for name in ("series", "catch"):
        b = sub.add_parser(name); b.add_argument("--tune", type=int); b.add_argument("--gain", type=int); b.add_argument("--ctrl", default="13fff"); b.add_argument("--n", type=int, default=16384)
        b.add_argument("--ms", type=int, default=4000); b.add_argument("--out", required=True)
        if name == "catch": b.add_argument("--thr", type=int, required=True); b.add_argument("--maxsend", type=int, default=4)
    c = sub.add_parser("period"); c.add_argument("file")
    args = ap.parse_args()
    {"wificnt": wificnt, "series": series_cmd, "catch": catch_cmd, "period": period_cmd}[args.sub](args)

if __name__ == "__main__": main()
