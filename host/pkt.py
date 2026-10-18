#!/usr/bin/env python3
"""Generic 1 Mbps GFSK packet extractor for SigMF captures: find bursts, demodulate, locate a sync word, print bytes.

  pkt.py capture.sigmf-meta --sync adf645c7 [--rate 1e6] [--nbytes 40] [--split 7] [--diff]

Use it for unknown 2.4 GHz remotes: first run with --find to list the most common 32-bit words after the
0101... preamble (the sync word / address), then run with --sync to dump every packet as hex with its time.
--split N groups packets by their first N bytes after the sync (packet "variants"); --diff marks the bytes that
differ between the groups' most common contents. Polarity: the C3 delivers LO-RF, so conj() is applied (BLE and
the remotes seen so far decode with it); --direct disables that.
"""
import argparse, sys, os, collections
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from esp32sdr.sigmf import read_sigmf

def bursts_of(x, fs, thr, minlen_us):
    if len(x) < 64: return []
    env = np.convolve(np.abs(x), np.ones(16) / 16, mode="same"); idx = np.flatnonzero(env > thr)
    if not len(idx): return []
    out, a = [], idx[0]
    for p, q in zip(idx[:-1], idx[1:]):
        if q - p > 40: out.append((a, p)); a = q
    out.append((a, idx[-1]))
    return [(a, b) for a, b in out if (b - a) / fs * 1e6 >= minlen_us]

def auto_thr(blocks):
    """3x the quiet level: 20th percentile of per-block mean |x| (works for int8 and 10-bit scales)."""
    m = np.array([np.abs(x).mean() for _, x in blocks if len(x) >= 64]); pk = np.array([np.abs(x).max() for _, x in blocks if len(x) >= 64])
    if not len(m): return 18.0
    return float(min(np.percentile(m, 20) * 3, np.median(pk) * 0.3))      # event-mode files hold only energetic blocks: cap by the burst peak

def bitstreams(seg, fs, rate, conj=True):
    """Yield (phase, bits) for every symbol phase; bits are hard decisions after CFO removal."""
    x = np.conj(seg) if conj else seg; x = x - x.mean(); d = np.angle(x[1:] * np.conj(x[:-1])); sps = fs / rate
    k = max(1, int(round(sps / 2))); d = np.convolve(d, np.ones(k) / k, mode="same")
    env = np.abs(x); start = int(np.flatnonzero(env > env.max() * 0.3)[0])
    for ph in np.arange(0, sps, max(1.0, sps / 8)):
        bi = np.round(start + ph + np.arange(int((len(d) - start - ph) / sps)) * sps).astype(int); bi = bi[bi < len(d)]
        v = d[bi]; cfo = np.median(v[:48]); yield ph, (v - cfo > 0).astype(np.int8), float(np.mean(np.abs(v - cfo)))

def preamble_end(b, minalt=6):
    alt = 0
    for i in range(1, len(b)):
        if b[i] != b[i - 1]: alt += 1
        else:
            if alt >= minalt: return i
            alt = 0
    return None

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("file"); ap.add_argument("--sync"); ap.add_argument("--find", action="store_true"); ap.add_argument("--rate", type=float, default=1e6)
    ap.add_argument("--nbytes", type=int, default=40); ap.add_argument("--split", type=int, default=2); ap.add_argument("--diff", action="store_true"); ap.add_argument("--thr", type=float, default=0, help="burst threshold on |x|; 0 = automatic"); ap.add_argument("--minlen", type=float, default=150); ap.add_argument("--direct", action="store_true")
    a = ap.parse_args(); meta, blocks = read_sigmf(a.file); fs = meta["global"]["core:sample_rate"]; T0 = blocks[0][0]
    if a.thr <= 0: a.thr = auto_thr(blocks)
    segs = [((t - T0) / 1e6 + s0 / fs, x[max(0, s0 - 64):s1 + 64]) for t, x in blocks for s0, s1 in bursts_of(x, fs, a.thr, a.minlen)]
    print(f"{a.file}: {len(blocks)} blocks, {len(segs)} bursts >= {a.minlen:.0f} us, fs {fs/1e6:g} MS/s, {a.rate/1e6:g} Mbps")
    if a.find or not a.sync:
        c = collections.Counter()
        for t, seg in segs:
            best = None
            for ph, b, m in bitstreams(seg, fs, a.rate, not a.direct):
                pe = preamble_end(b)
                if pe is not None and (best is None or m > best[0]): best = (m, b, pe)
            if best: b, pe = best[1], best[2]; c["".join(map(str, b[pe - 1:pe + 31]))] += 1; c["".join(map(str, b[pe:pe + 32]))] += 1
        print("most common 32-bit words at the end of the preamble (two alignments):")
        for w, n in c.most_common(8): print(f"  {n:4d}x  {w}  0x{int(w, 2):08x}")
        return
    sync = "".join(f"{int(a.sync, 16):0{len(a.sync) * 4}b}"); pk = []
    for t, seg in segs:
        best = None
        for ph, b, m in bitstreams(seg, fs, a.rate, not a.direct):
            s = "".join(map(str, b)); k = s.find(sync)
            if k >= 0 and (best is None or m > best[0]): best = (m, s[k + len(sync):])
        if best:
            bits = best[1][:a.nbytes * 8]
            if len(bits) >= 16: pk.append((t, bytes(int(bits[i:i + 8], 2) for i in range(0, len(bits) - 7, 8)), len(seg) / fs * 1e6))
    print(f"packets with sync 0x{a.sync}: {len(pk)}")
    groups = collections.defaultdict(list)
    for t, by, dur in pk: groups[by[:a.split].hex()].append((t, by, dur))
    for key, items in sorted(groups.items(), key=lambda kv: -len(kv[1])):
        c = collections.Counter(by for _, by, _ in items)
        print(f"\n== variant {key} : {len(items)} packets, {len(c)} distinct contents")
        for by, n in c.most_common(6): print(f"  {n:3d}x  {by.hex(' ')}")
        if a.diff and len(c) > 1:
            ref = c.most_common(1)[0][0]; var = set()
            for by in c: var |= {i for i in range(min(len(by), len(ref))) if by[i] != ref[i]}
            print("  bytes that vary within this variant: " + ", ".join(str(i) for i in sorted(var)))
        print("  times: " + " ".join(f"{t:.2f}" for t, _, _ in items[:30]) + (" ..." if len(items) > 30 else ""))

if __name__ == "__main__": main()
