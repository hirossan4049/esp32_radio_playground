#!/usr/bin/env python3
"""Split a remote-control capture into button events (time gaps) and tabulate the packet fields per event.

  remote_events.py capture.sigmf-meta --sync adf645c7 --labels 点灯,消灯,明るさ↑,明るさ↓ [--gap 3] [--cmd 22:29] [--roll 29:37]
"""
import argparse, sys, os, collections
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from esp32sdr.sigmf import read_sigmf
import pkt

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("file"); ap.add_argument("--sync", default="adf645c7"); ap.add_argument("--labels", default=""); ap.add_argument("--gap", type=float, default=3.0)
    ap.add_argument("--rate", type=float, default=1e6); ap.add_argument("--thr", type=float, default=0); ap.add_argument("--nbytes", type=int, default=40); ap.add_argument("--cmd", default="22:29"); ap.add_argument("--roll", default="29:37"); ap.add_argument("--variant", default="6dbf")
    a = ap.parse_args(); meta, blocks = read_sigmf(a.file); fs = meta["global"]["core:sample_rate"]; T0 = blocks[0][0]
    if a.thr <= 0: a.thr = pkt.auto_thr(blocks); print(f"burst threshold (auto): {a.thr:.1f}")
    segs = [((t - T0) / 1e6 + s0 / fs, x[max(0, s0 - 64):s1 + 64]) for t, x in blocks for s0, s1 in pkt.bursts_of(x, fs, a.thr, 150)]
    sync = "".join(f"{int(a.sync, 16):0{len(a.sync) * 4}b}"); pks = []
    for t, seg in segs:
        best = None
        for ph, b, m in pkt.bitstreams(seg, fs, a.rate, True):
            s = "".join(map(str, b)); k = s.find(sync)
            if k >= 0 and (best is None or m > best[0]): best = (m, s[k + len(sync):])
        if best:
            bits = best[1][:a.nbytes * 8]
            if len(bits) >= a.nbytes * 8: pks.append((t, bytes(int(bits[i:i + 8], 2) for i in range(0, a.nbytes * 8, 8))))
    pks.sort(); print(f"{a.file}: {len(blocks)} blocks, {len(segs)} bursts, {len(pks)} complete packets with sync 0x{a.sync}")
    # events by time gaps
    events = []; cur = []
    for t, by in pks:
        if cur and t - cur[-1][0] > a.gap: events.append(cur); cur = []
        cur.append((t, by))
    if cur: events.append(cur)
    labels = [l for l in a.labels.split(",") if l] if a.labels else []
    c0, c1 = [int(v) for v in a.cmd.split(":")]; r0, r1 = [int(v) for v in a.roll.split(":")]
    print(f"events (gap > {a.gap} s): {len(events)}; labels given: {len(labels)}")
    for i, ev in enumerate(events):
        name = labels[i] if i < len(labels) else f"event{i}"
        vb = [(t, by) for t, by in ev if by[2:4].hex() == a.variant]; va = [(t, by) for t, by in ev if by[2:4].hex() != a.variant]
        print(f"\n== {name}: {ev[0][0]:.1f}..{ev[-1][0]:.1f} s, {len(ev)} packets ({len(vb)} variant {a.variant}, {len(va)} other)")
        cmds = collections.Counter(by[c0:c1].hex(" ") for _, by in vb); print(f"   command bytes [{c0}:{c1}]: " + "; ".join(f"{k} x{n}" for k, n in cmds.most_common(4)))
        rolls = []; last = None
        for t, by in vb:
            r = by[r0:r1].hex(" ")
            if r != last: rolls.append((t, r, by[37:40].hex(" "))); last = r
        print(f"   rolling bytes [{r0}:{r1}] (changes, with tail bytes 37..39): ")
        for t, r, tail in rolls[:12]: print(f"      t={t:6.2f}  {r}   tail {tail}")
        if len(rolls) > 12: print(f"      ... {len(rolls)} distinct")
        # fixed-bytes check inside the event for the other variant
        if va:
            ca = collections.Counter(by.hex(" ") for _, by in va); print(f"   other variant distinct contents: {len(ca)}; top: {ca.most_common(1)[0][0][:60]}...")

if __name__ == "__main__": main()
