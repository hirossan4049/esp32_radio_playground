#!/usr/bin/env python3
"""RX-C3-017: LO-fundamental sweep looking for any k-th order response to the UHF ISDB-T band.

A k-th order response (RF harmonic generation / sub-harmonic mixing) to a broadcast at fc would
put a k x 5.57 MHz wide plateau at baseband when the LO fundamental F is near k*fc. Sweep F over
the whole reachable range (1842..2803 MHz, 5/6 CKGEN mode below 2203) at 40 MS/s, record the
median PSD per point, and look for broad elevations of the in-band floor relative to a running
baseline. Candidate elevations are then checked against 3fc/4fc/5fc for channels 13..52 and
against the Wi-Fi (2400-2500) and TD-LTE (2496-2690) bands. Receive only.

  uhf_scan.py sweep --start 1842 --stop 2803 --step 3 --blocks 6 --gain 60 --out logs/RX-C3-017_sweep1.json
  uhf_scan.py analyze logs/RX-C3-017_sweep1.json [more.json]
"""
import sys, json, time, argparse
import numpy as np

FS = 40e6; NFFT = 1024; CTRL = 0x0bfff
f = np.fft.fftshift(np.fft.fftfreq(NFFT, 1 / FS)) / 1e6

def chan_fc(ch): return 473 + 6 * (ch - 13) + 1 / 7

def plan(F):
    if 2203 <= F <= 2803: return int(round(F * 1000)), False
    P = F * 6 / 5
    if 2203 <= P <= 2803: return int(round(P * 1000)), True
    return None

def sweep(a):
    sys.path.insert(0, "host"); import rxprobe
    s = rxprobe.open_port(rxprobe.PORT)
    def reset():
        s.dtr = False; s.rts = True; time.sleep(0.1); s.rts = False; time.sleep(3); s.reset_input_buffer()
    def ckgen(alt):
        rep = rxprobe.cmd(s, "I2C 65 0", wait=0.4); m = [l for l in rep.splitlines() if l.startswith("I2C")]
        cur = int(m[-1].split("=")[-1], 16); new = (cur | 0x10) if alt else (cur & ~0x10)
        if new != cur: rxprobe.cmd(s, f"I2CW 65 0 {new:02x}", wait=0.3)
    def setup(pll, alt):
        rxprobe.cmd(s, f"TUNE {pll}"); rxprobe.cmd(s, f"GAIN {a.gain}"); ckgen(alt); rxprobe.cmd(s, "WR 60033d90 000c2040")
    out = dict(fs=FS, nfft=NFFT, ctrl=CTRL, f_mhz=f.tolist(), gain=a.gain, blocks=a.blocks, points=[])
    Fs = np.arange(a.start, a.stop + 1e-9, a.step)
    print(f"sweep {len(Fs)} points, {a.blocks} blocks each at 40 MS/s, gain {a.gain}")
    for F in Fs:
        p = plan(F)
        if not p: continue
        setup(*p); Ps = []; sat = 0; fails = 0
        for _ in range(a.blocks):
            try: data, meta = rxprobe.capture(s, CTRL, 16384)
            except Exception:
                fails += 1; reset(); setup(*p); continue
            w = np.frombuffer(data, dtype="<u4"); i = ((w & 1023) ^ 512).astype(np.int32) - 512; q = (((w >> 10) & 1023) ^ 512).astype(np.int32) - 512
            if np.abs(i).max() >= 500 or np.abs(q).max() >= 500: sat += 1
            x = (i + 1j * q).astype(complex); x -= x.mean()
            n = len(x) // NFFT; X = np.fft.fftshift(np.fft.fft(x[:n * NFFT].reshape(n, NFFT) * np.hanning(NFFT), axis=1), axes=1)
            Ps.append(np.median(np.abs(X) ** 2, axis=0))
        ckgen(False)
        if not Ps: out["points"].append(dict(F=float(F), failed=True)); continue
        P = 10 * np.log10(np.median(np.array(Ps), axis=0) + 1e-9)
        inband = np.median(P[(np.abs(f) > 1) & (np.abs(f) < 9)])
        out["points"].append(dict(F=float(F), pll_khz=p[0], alt=p[1], sat=sat, fails=fails, floor_db=float(inband), psd_db=P.round(2).tolist()))
        print(f"F {F:7.1f} MHz pll {p[0]/1e3:7.1f} {'5/6' if p[1] else '   '} floor {inband:6.2f} dB sat {sat} fails {fails}", flush=True)
        json.dump(out, open(a.out, "w"))
    print("done")

def analyze(a):
    runs = [json.load(open(p)) for p in a.files]
    F = np.array([p["F"] for p in runs[0]["points"] if "floor_db" in p])
    fl = np.array([[p["floor_db"] for p in r["points"] if "floor_db" in p] for r in runs])
    fm = fl.mean(0)
    # running median baseline over +-60 MHz, excluding the point itself
    base = np.array([np.median(fm[(np.abs(F - F0) <= 60) & (np.abs(F - F0) > 0.1)]) for F0 in F])
    dev = fm - base
    noise = np.std(np.diff(fm)) / np.sqrt(2)
    print(f"points {len(F)}, floor mean {fm.mean():.2f} dB, point-to-point sigma {noise:.3f} dB, runs {len(runs)}")
    if len(runs) > 1: print(f"run-to-run rms difference {np.std(fl[0]-fl[1]):.3f} dB")
    thr = max(4 * noise, 0.3)
    print(f"elevations above baseline by > {thr:.2f} dB:")
    for i in np.where(dev > thr)[0]:
        F0 = F[i]; tags = []
        for k in (3, 4, 5):
            for ch in range(13, 53):
                if abs(k * chan_fc(ch) - F0) <= k * 3.0: tags.append(f"{k}x ch{ch}")
        if 2400 <= F0 <= 2500: tags.append("Wi-Fi band")
        if 2496 <= F0 <= 2690: tags.append("TD-LTE band")
        print(f"  F {F0:7.1f}: +{dev[i]:.2f} dB  {' '.join(tags)}")
    # per-order summary: mean deviation at all F that are k x some channel vs others
    for k in (3, 4, 5):
        m = np.zeros(len(F), bool)
        for ch in range(13, 53): m |= np.abs(F - k * chan_fc(ch)) <= 2.0
        if m.any(): print(f"order {k}: {m.sum()} points at k*fc; mean deviation {dev[m].mean():+.3f} dB vs others {dev[~m].mean():+.3f} dB")
    print("floor by region (dB): " + ", ".join(f"{lo}-{hi}: {fm[(F>=lo)&(F<hi)].mean():.2f}" for lo, hi in [(1840,2000),(2000,2200),(2200,2400),(2400,2500),(2500,2700),(2700,2810)] if ((F>=lo)&(F<hi)).any()))

ap = argparse.ArgumentParser(); sub = ap.add_subparsers(dest="cmd", required=True)
sw = sub.add_parser("sweep"); sw.add_argument("--start", type=float, default=1842); sw.add_argument("--stop", type=float, default=2803); sw.add_argument("--step", type=float, default=3)
sw.add_argument("--blocks", type=int, default=6); sw.add_argument("--gain", type=int, default=60); sw.add_argument("--out", required=True)
an = sub.add_parser("analyze"); an.add_argument("files", nargs="+")
a = ap.parse_args(); {"sweep": sweep, "analyze": analyze}[a.cmd](a)
