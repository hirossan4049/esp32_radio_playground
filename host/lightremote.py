#!/usr/bin/env python3
"""Decoder for the 2402 MHz lighting remote seen in RX-C3-025/026 (sync 0xADF645C7, 1 Mbps GFSK, 16 ms pitch).

Known layout of the "6DBF" packet type (after the sync word):
  [0..21]  fixed (device ID / header; starts cf bd 6d bf)
  [22]     0x18
  [23]     code byte: 0x42 = 点灯, 0x46 = 消灯 (seen once), 0x86 = 明るさ↑ ; same value in two sessions
  [24..27] e5 40 00 00
  [28]     session byte (0x01 in one session, 0x02 in the next)
  [29]     counter, +4 every ~1.5 s while transmitting, continues across presses
  [30]     state byte (00/02/03 with 点灯, a1/a2 with 明るさ↑)
  [31..36] change with every counter step (derived values; generator unknown)
  [37..38] 4d c8..cb
  [39]     check byte (algorithm unknown)
The "4086" packet type alternates with it and has no fixed fields after byte 16.

  lightremote.py capture.sigmf-meta [--all]
"""
import argparse, sys, os
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from esp32sdr.sigmf import read_sigmf
import pkt

CODES = {0x42: "点灯", 0x46: "消灯", 0x86: "明るさ↑"}
SYNC = f"{0xadf645c7:032b}"

def packets(path):
    meta, blocks = read_sigmf(path); fs = meta["global"]["core:sample_rate"]; T0 = blocks[0][0]; thr = pkt.auto_thr(blocks); out = []
    for tt, x in blocks:
        for s0, s1 in pkt.bursts_of(x, fs, thr, 150):
            seg = x[max(0, s0 - 64):s1 + 64]; best = None
            for ph, b, m in pkt.bitstreams(seg, fs, 1e6, True):
                s = "".join(map(str, b)); k = s.find(SYNC)
                if k >= 0 and (best is None or m > best[0]): best = (m, s[k + 32:])
            if best and len(best[1]) >= 320: out.append(((tt - T0) / 1e6 + s0 / fs, bytes(int(best[1][i:i + 8], 2) for i in range(0, 320, 8))))
    return sorted(out)

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("files", nargs="+"); ap.add_argument("--all", action="store_true"); a = ap.parse_args()
    for f in a.files:
        pk = packets(f); b = [(t, by) for t, by in pk if by[2:4] == b"\x6d\xbf"]
        print(f"{f}: {len(pk)} packets ({len(b)} of type 6DBF)")
        last = None; presses = []
        for t, by in b:
            key = (by[23], by[29]) if a.all else by[23]
            if key != last:
                last = key; presses.append((t, by))
        for t, by in presses:
            print(f"  t={t:7.2f}s  {CODES.get(by[23], f'code 0x{by[23]:02x}'):8s}  counter {by[29] >> 2:3d}  state {by[30]:02x}  session {by[28]:02x}  id {by[4:12].hex()}  check {by[39]:02x}")

if __name__ == "__main__": main()
