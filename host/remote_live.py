#!/usr/bin/env python3
"""Live decoder for the 2402 MHz lighting remote: automatic gain and threshold, continuous reception,
one line per detected press. Also writes everything it keeps to SigMF so it can be re-analysed.

  remote_live.py [--freq 2402] [--rate 16000000] [--log logs/remote_live.txt] [--out captures/web/live]

How it works
  1. Measures the noise level with gain G (no threshold) for 1 s; sets the event threshold to 2.5x the noise.
  2. Streams in event mode; every energetic block is demodulated at once (1 Mbps GFSK, tolerant sync 0xADF645C7).
  3. If bursts clip (peak > 90% full scale) the gain is lowered by 4; if bursts are weak (peak < 25%) it is raised.
  4. Packets of type 6DBF within one counter step are majority-voted; a new (command, parameter, counter) prints a line.
Stop with Ctrl-C (or the --duration limit).
"""
import argparse, sys, os, time, collections
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from esp32sdr import Device
from esp32sdr.sigmf import SigMFWriter
import pkt, lightremote

CODES = lightremote.CODES
PARAM = {0xa1: "明るさ↑", 0xa2: "明るさ↑", 0x60: "明るさ↓", 0x63: "明るさ↓", 0x90: "昼光色", 0x91: "昼光色", 0x93: "昼光色"}

def unpack_frame(payload, fmt):
    if fmt: raw = np.frombuffer(payload, dtype="<i2"); x = raw[0::2].astype(np.float32) + 1j * raw[1::2].astype(np.float32)
    else: raw = np.frombuffer(payload, dtype=np.int8); x = raw[0::2].astype(np.float32) + 1j * raw[1::2].astype(np.float32)
    return x

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--freq", type=float, default=2402); ap.add_argument("--rate", type=float, default=16e6); ap.add_argument("--gain", type=int, default=30)
    ap.add_argument("--duration", type=float, default=3600); ap.add_argument("--log", default="logs/remote_live.txt"); ap.add_argument("--out", default="captures/web/remote_live"); ap.add_argument("--chunk", type=float, default=20, help="seconds per stream segment (gain/threshold are re-evaluated between segments)")
    a = ap.parse_args(); rate = int(a.rate); fullscale = 511 if rate >= 16e6 else 127
    dev = Device(); dev.tune(a.freq * 1e6); gain = a.gain; dev.set_gain(gain)
    logf = open(a.log, "a"); 
    def say(msg):
        line = time.strftime("%H:%M:%S") + "  " + msg; print(line, flush=True); logf.write(line + "\n"); logf.flush()
    say(f"start LO {a.freq} MHz rate {rate} gain {gain}")
    w = SigMFWriter(a.out, rate, a.freq * 1e6, "ci16_le" if rate >= 16e6 else "ci8", "ESP32-C3 remote_live", description="live remote decode", extra={"esp32c3:gain_index": gain})
    sys_t0 = time.time(); seen = collections.OrderedDict(); last_key = None; stats = dict(frames=0, bursts=0, pk=0, clip=0, weak=0, strong_peak=0.0)
    while time.time() - sys_t0 < a.duration:
        # --- noise measurement (threshold 0 for 1 s): mean |I|+|Q| of the raw 10-bit words is what the firmware compares
        noise = []
        def on_noise(seq, t_us, fmt, flags, payload):
            if not payload: return
            x = unpack_frame(payload, fmt)   # both formats are on the raw 10-bit scale (box-car average keeps the scale), which is what the firmware threshold compares
            noise.append(float(np.abs(x.real).mean() + np.abs(x.imag).mean()))
        dev.stream(rate, 1.0, on_noise, thr=0)
        nz = float(np.percentile(noise, 30)) if noise else 40.0; thr = int(max(20, nz * 2.5))
        say(f"noise {nz:.0f} -> threshold {thr}; gain {gain}")
        # --- event-mode segment with live decoding
        stats.update(frames=0, bursts=0, pk=0, clip=0, weak=0, strong_peak=0.0); win = []; win_t = None
        def flush_window():
            nonlocal win, win_t, last_key
            if not win: return
            for kind, pat in (("6DBF", 0xcfbd6dbf), ("4086", 0xcfbd4086)):
                P = np.array([int(c) for c in f"{pat:032b}"], dtype=np.int16); sel = [b for b in win if np.sum(b[:32] != P) <= 6]
                if kind != "6DBF" or len(sel) < 2: continue
                M = np.array(sel); v = (M.mean(axis=0) > 0.5).astype(np.uint8); by = bytes(np.packbits(v)); conf = float(np.mean(np.abs(M.mean(axis=0) - 0.5) * 2))
                key = (by[23], by[30], by[29] >> 2)
                if key != last_key:
                    last_key = key; name = CODES.get(by[23], f"code 0x{by[23]:02x}"); pname = PARAM.get(by[30], f"param 0x{by[30]:02x}") if by[23] == 0x86 else f"param {by[30]:02x}"
                    say(f"PRESS  {name:4s} {pname:10s}  counter {by[29] >> 2:3d}  session {by[28]:02x}  votes {len(sel):3d} conf {conf:.2f}  bytes22-39 {by[22:40].hex(' ')}")
            win = []; win_t = None
        def on_frame(seq, t_us, fmt, flags, payload):
            nonlocal win, win_t
            stats["frames"] += 1
            if not payload: w.add_block(payload, t_us, seq, flags); return
            w.add_block(payload, t_us, seq, flags); x = unpack_frame(payload, fmt); fs = rate
            peak = float(max(np.abs(x.real).max(), np.abs(x.imag).max())); stats["strong_peak"] = max(stats["strong_peak"], peak)
            if peak > 0.9 * fullscale: stats["clip"] += 1
            thr_b = max(15.0, float(np.percentile(np.abs(x), 20)) * 3)
            for s0, s1 in pkt.bursts_of(x, fs, thr_b, 150):
                stats["bursts"] += 1; seg = x[max(0, s0 - 64):s1 + 64]; best = None
                for ph, b, m in pkt.bitstreams(seg, fs, 1e6, True):
                    s = "".join(map(str, b)); k = lightremote.sync_pos(s, 3)
                    if k >= 0 and len(s) >= k + 320 and (best is None or m > best[0]): best = (m, s[k:k + 320])
                if best:
                    stats["pk"] += 1; bits = np.array([int(c) for c in best[1]], dtype=np.int16); tnow = t_us / 1e6
                    if win_t is not None and tnow - win_t > 1.0: flush_window()
                    if win_t is None: win_t = tnow
                    win.append(bits)
        dev.stream(rate, a.chunk, on_frame, thr=thr); flush_window()
        say(f"segment: frames {stats['frames']} bursts {stats['bursts']} packets {stats['pk']} clipped {stats['clip']} peak {stats['strong_peak']:.0f}/{fullscale}")
        # --- gain adaptation
        if stats["clip"] > 2 and gain > 10: gain -= 4; dev.set_gain(gain); say(f"gain -> {gain} (clipping)")
        elif stats["bursts"] and stats["strong_peak"] < 0.25 * fullscale and gain < 56: gain += 4; dev.set_gain(gain); say(f"gain -> {gain} (weak)")
        elif not stats["bursts"] and gain < 44: gain += 4; dev.set_gain(gain); say(f"gain -> {gain} (nothing heard)")
    w.close(); say("stop")

if __name__ == "__main__":
    try: main()
    except KeyboardInterrupt: print("\nstopped")
