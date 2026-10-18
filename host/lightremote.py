#!/usr/bin/env python3
"""Decoder for the 2402 MHz lighting remote seen in RX-C3-025/026 (sync 0xADF645C7, 1 Mbps GFSK, 16 ms pitch).

Known layout of the "6DBF" packet type (after the sync word):
  [0..21]  fixed (device ID / header; starts cf bd 6d bf)
  [22]     0x18
  [23]     command class: 0x42 = 点灯, 0x46 = 消灯, 0x86 = 調整 (明るさ↑/↓, 昼光色, 電球色, 全灯)
  [24..27] e5 40 00 00  (全灯: e5 43 ff fc)
  [28..29] 14-bit packet-group counter: byte 28 = high bits, byte 29 bits 7..2 = low 6 bits (+1 every ~1.5 s of transmission, continues across presses)
  [30]     bits 7..2 = parameter: 0x50 明るさ↑, 0xa0 明るさ↓, 0x60 昼光色, 0x90 電球色, 0x08 全灯 (with the 43 ff fc sub-format); 0x00 with 点灯/消灯. bits 1..0 = packet sequence number
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

CODES = {0x42: "点灯", 0x46: "消灯", 0x86: "調整"}
PARAM = {0x50: "明るさ↑", 0xa0: "明るさ↓", 0x60: "昼光色", 0x90: "電球色", 0x08: "全灯"}
SYNC = f"{0xadf645c7:032b}"

def sync_pos(s, maxerr=3, sync_hex=None):
    """Position after a sync word (default 0xADF645C7, or sync_hex) with up to maxerr bit errors (or -1)."""
    pat = SYNC if sync_hex is None else f"{int(sync_hex, 16):0{len(sync_hex) * 4}b}"
    n = len(pat); best = -1; bd = maxerr + 1
    for k in range(0, len(s) - n):
        d = sum(1 for i in range(n) if s[k + i] != pat[i])
        if d < bd: bd = d; best = k + n
        if bd == 0: break
    return best

def vote_packets(path, thr=0, step_s=1.0, nbytes=40):
    """Group bursts into ~1 s windows, align each on a tolerant sync, majority-vote the bits -> one packet per window."""
    meta, blocks = read_sigmf(path); fs = meta["global"]["core:sample_rate"]; T0 = blocks[0][0]; thr = thr or pkt.auto_thr(blocks)
    rows = []
    for tt, x in blocks:
        for s0, s1 in pkt.bursts_of(x, fs, thr, 150):
            seg = x[max(0, s0 - 64):s1 + 64]; best = None
            for ph, b, m in pkt.bitstreams(seg, fs, 1e6, True):
                st = "".join(map(str, b)); k = sync_pos(st)
                if k >= 0 and (best is None or m > best[0]): best = (m, st[k:k + nbytes * 8])
            if best and len(best[1]) >= nbytes * 8: rows.append(((tt - T0) / 1e6 + s0 / fs, np.array([int(c) for c in best[1]], dtype=np.int16)))
    rows.sort(key=lambda r: r[0]); out = []; grp = []
    # the two packet types alternate every 16 ms; separate them by the first 4 bytes before voting
    PAT = {k: np.array([int(c) for c in f"{v:032b}"], dtype=np.int16) for k, v in (("6DBF", 0xcfbd6dbf), ("4086", 0xcfbd4086))}
    def flush():
        for kind, pat in PAT.items():
            sel = [g[1] for g in grp if np.sum(g[1][:32] != pat) <= 6]
            if len(sel) >= 2:
                M = np.array(sel); v = (M.mean(axis=0) > 0.5).astype(np.uint8); conf = float(np.mean(np.abs(M.mean(axis=0) - 0.5) * 2))
                out.append((grp[0][0], bytes(np.packbits(v)), len(sel), conf))
    for t, b in rows:
        if grp and t - grp[-1][0] > step_s: flush(); grp = []
        grp.append((t, b))
    flush(); return out

def packets(path, thr=0):
    meta, blocks = read_sigmf(path); fs = meta["global"]["core:sample_rate"]; T0 = blocks[0][0]; thr = thr or pkt.auto_thr(blocks); out = []
    for tt, x in blocks:
        for s0, s1 in pkt.bursts_of(x, fs, thr, 150):
            seg = x[max(0, s0 - 64):s1 + 64]; best = None
            for ph, b, m in pkt.bitstreams(seg, fs, 1e6, True):
                s = "".join(map(str, b)); k = sync_pos(s, 2)            # tolerate up to 2 sync-bit errors
                if k >= 0 and (best is None or m > best[0]): best = (m, s[k:])
            if best and len(best[1]) >= 320: out.append(((tt - T0) / 1e6 + s0 / fs, bytes(int(best[1][i:i + 8], 2) for i in range(0, 320, 8))))
    return sorted(out)

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("files", nargs="+"); ap.add_argument("--all", action="store_true"); ap.add_argument("--thr", type=float, default=0); ap.add_argument("--vote", action="store_true", help="tolerant sync + majority vote per ~1 s window (weak signals)"); a = ap.parse_args()
    for f in a.files:
        if a.vote:
            vp = vote_packets(f, a.thr); print(f"{f}: {len(vp)} voted packets")
            for t, by, n, conf in vp:
                kind = "6DBF" if by[2:4] == b"\x6d\xbf" else by[2:4].hex()
                if kind == "6DBF": print(f"  t={t:7.2f}s  {CODES.get(by[23], f'code 0x{by[23]:02x}'):6s} param {by[30]:02x}  counter {by[29] >> 2:3d}  session {by[28]:02x}  [22:29]={by[22:29].hex(' ')}  votes {n:3d} conf {conf:.2f}  check {by[39]:02x}")
                else: print(f"  t={t:7.2f}s  type {kind}  bytes16..24 {by[16:25].hex(' ')}  votes {n:3d} conf {conf:.2f}")
            continue
        pk = packets(f, a.thr); b = [(t, by) for t, by in pk if by[2:4] == b"\x6d\xbf"]
        print(f"{f}: {len(pk)} packets ({len(b)} of type 6DBF)")
        last = None; presses = []
        for t, by in b:
            key = (by[23], by[29]) if a.all else by[23]
            if key != last:
                last = key; presses.append((t, by))
        for t, by in presses:
            full = by[25:28] == b"\x43\xff\xfc"; pname = ("全灯" if full else PARAM.get(by[30] & 0xFC, f"param {by[30] & 0xFC:02x}")) if by[23] == 0x86 else ""
            print(f"  t={t:7.2f}s  {CODES.get(by[23], f'code 0x{by[23]:02x}'):4s} {pname:6s} counter {(by[28] << 6) | (by[29] >> 2):5d}  seq {by[30] & 3}  id {by[4:12].hex()}  check {by[39]:02x}")

if __name__ == "__main__": main()
