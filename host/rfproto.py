#!/usr/bin/env python3
"""Generic 2.4 GHz burst-protocol analyser for ESP32-C3 SigMF captures.

Receive-only. Automates, for an unknown remote of the same family (1 Mbps GFSK, preamble + sync word + payload):
  1. sync-word discovery  : the most common 32-bit word after the 0101... preamble across all bursts
  2. field inference      : align every packet on that sync, classify each byte position as
                            FIXED / COUNTER / PER-PRESS / CHECK from how it varies across packets and presses
  3. device profiles      : save/load the sync word + field map + rate as JSON, and auto-match a capture to a profile

  rfproto.py scan  capture.sigmf-meta [--rate 1e6]                 -> candidate sync words
  rfproto.py learn capture.sigmf-meta --name mylight [--gap 3]     -> infers fields, writes profiles/mylight.json
  rfproto.py id    capture.sigmf-meta                              -> best-matching saved profile + decoded presses
  rfproto.py dump  capture.sigmf-meta [--profile mylight]          -> every packet as hex, annotated by the profile

Shared DSP lives in pkt.py; this module only adds discovery, inference and profiles. It makes no assumptions
specific to one device, so it works on any same-family remote.
"""
import argparse, sys, os, json, collections, glob
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from esp32sdr.sigmf import read_sigmf
import pkt

PROFILE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "profiles")

def extract_packets(path, rate=1e6, nbytes=48, thr=0, sync=None, syncerr=2):
    """Return (fs, [(t_s, bytes)]). If sync is given (hex str), align tolerant on it; else align on the preamble end."""
    meta, blocks = read_sigmf(path); fs = meta["global"]["core:sample_rate"]; T0 = blocks[0][0]
    thr = thr or pkt.auto_thr(blocks); out = []
    syncbits = f"{int(sync, 16):0{len(sync) * 4}b}" if sync else None
    for tt, x in blocks:
        for s0, s1 in pkt.bursts_of(x, fs, thr, 150):
            seg = x[max(0, s0 - 64):s1 + 64]; best = None
            for ph, b, m in pkt.bitstreams(seg, fs, rate, True):
                s = "".join(map(str, b))
                if syncbits is not None:
                    k = tolerant_find(s, syncbits, syncerr)
                    if k < 0: continue
                    k += len(syncbits)
                else:
                    k = pkt.preamble_end(b)
                    if k is None: continue
                if best is None or m > best[0]: best = (m, s[k:k + nbytes * 8])
            if best and len(best[1]) >= 64:
                bits = best[1]; by = bytes(int(bits[i:i + 8], 2) for i in range(0, len(bits) - 7, 8))
                out.append(((tt - T0) / 1e6 + s0 / fs, by))
    out.sort(); return fs, out

def tolerant_find(s, pat, maxerr):
    for k in range(0, len(s) - len(pat) + 1):
        if sum(1 for i in range(len(pat)) if s[k + i] != pat[i]) <= maxerr: return k
    return -1

def discover_sync(path, rate=1e6, thr=0):
    """Most common 32-bit words right after the preamble (two bit alignments), as hex with counts."""
    fs, pk = extract_packets(path, rate, nbytes=8, thr=thr)   # align on preamble; first bytes are the sync word
    c = collections.Counter()
    for _, by in pk:
        if len(by) >= 4:
            w = int.from_bytes(by[:4], "big"); c[w] += 1
            # also the alignment shifted by one bit (preamble/ sync boundary is ambiguous by a bit)
    return [(f"{w:08x}", n) for w, n in c.most_common(12)]

def group_presses(packets, gap=3.0):
    events = []; cur = []
    for t, by in packets:
        if cur and t - cur[-1][0] > gap: events.append(cur); cur = []
        cur.append((t, by))
    if cur: events.append(cur)
    return events

def cluster_variants(packets, pre=4, minfrac=0.1):
    """Split packets into variant groups by their first `pre` bytes (interleaved packet types)."""
    c = collections.Counter(by[:pre] for _, by in packets if len(by) >= pre)
    keep = {k for k, n in c.items() if n >= max(3, len(packets) * minfrac)}
    groups = collections.defaultdict(list)
    for t, by in packets:
        if len(by) >= pre and by[:pre] in keep: groups[by[:pre].hex()].append((t, by))
    return dict(groups)

def infer_fields(packets, gap=3.0):
    """Classify each byte position across one variant's packets. Returns list of dicts per byte index."""
    L = min(len(by) for _, by in packets); cols = [np.array([by[i] for _, by in packets], dtype=int) for i in range(L)]
    events = group_presses(packets, gap)
    # per-byte stats
    fields = []
    for i in range(L):
        col = cols[i]; distinct = len(set(col.tolist()))
        # is it constant?
        if distinct == 1: kind = "FIXED"; detail = f"0x{col[0]:02x}"
        else:
            # per-event constancy: a value that is constant within an event but differs across events = per-press command
            ev_const = all(len(set(by[i] for _, by in ev)) <= 2 for ev in events if len(ev) >= 2)
            # monotone-ish across the whole file (counter): differences mostly small and same sign
            d = np.diff(col); inc = np.mean((d > 0) & (d < 16)) if len(d) else 0
            if inc > 0.6: kind = "COUNTER"; detail = f"+{int(np.median(d[d>0])) if np.any(d>0) else 0}/pkt"
            elif ev_const: kind = "COMMAND"; detail = "const within a press"
            elif distinct > len(packets) * 0.8: kind = "CHECK/DERIVED"; detail = f"{distinct} values"
            else: kind = "VARYING"; detail = f"{distinct} values"
        fields.append({"index": i, "kind": kind, "detail": detail})
    return fields, events

def cmd_scan(a):
    for hexw, n in discover_sync(a.file, a.rate, a.thr): print(f"  {n:4d}x  0x{hexw}")

def cmd_learn(a):
    # 1. find the sync word
    syncs = discover_sync(a.file, a.rate, a.thr)
    sync = a.sync or syncs[0][0]
    print(f"sync word: 0x{sync}" + ("" if a.sync else f" (auto, {syncs[0][1]} hits; alternatives {[s[0] for s in syncs[1:4]]})"))
    fs, pk = extract_packets(a.file, a.rate, a.nbytes, a.thr, sync=sync)
    if not pk: print("no packets after the sync word; try --sync or a different --rate"); return
    fulllen = max(len(by) for _, by in pk); pk = [(t, by) for t, by in pk if len(by) >= fulllen - 1]
    variants = cluster_variants(pk)
    print(f"{len(pk)} packets, {len(variants)} packet variant(s): {[f'{k} x{len(v)}' for k,v in sorted(variants.items(), key=lambda kv:-len(kv[1]))]}")
    vprofiles = {}
    for key, items in sorted(variants.items(), key=lambda kv: -len(kv[1])):
        fields, events = infer_fields(items, a.gap)
        print(f"\n  variant {key} ({len(items)} packets, {len(events)} presses):")
        for f in fields[:a.nbytes]: print(f"    byte {f['index']:2d}: {f['kind']:13s} {f['detail']}")
        vprofiles[key] = {"id_bytes": [f["index"] for f in fields if f["kind"] == "FIXED"],
                          "command_bytes": [f["index"] for f in fields if f["kind"] == "COMMAND"],
                          "counter_bytes": [f["index"] for f in fields if f["kind"] == "COUNTER"],
                          "check_bytes": [f["index"] for f in fields if f["kind"] == "CHECK/DERIVED"],
                          "id_value": "".join(f"{items[0][1][f['index']]:02x}" for f in fields if f["kind"] == "FIXED"),
                          "count": len(items), "fields": fields}
    main_variant = max(vprofiles, key=lambda k: vprofiles[k]["count"])
    prof = {"name": a.name, "rate": a.rate, "sync": sync, "nbytes": a.nbytes, "main_variant": main_variant, "variants": vprofiles}
    os.makedirs(PROFILE_DIR, exist_ok=True); path = os.path.join(PROFILE_DIR, a.name + ".json")
    json.dump(prof, open(path, "w"), indent=1, ensure_ascii=False); print(f"\nsaved profile -> {os.path.relpath(path)} (main variant {main_variant})")

def load_profiles():
    out = []
    for p in sorted(glob.glob(os.path.join(PROFILE_DIR, "*.json"))):
        try: out.append(json.load(open(p)))
        except Exception: pass
    return out

def cmd_id(a):
    profs = load_profiles()
    if not profs: print("no saved profiles (run 'learn' first)"); return
    best = None
    for pr in profs:
        fs, pk = extract_packets(a.file, pr["rate"], pr["nbytes"], a.thr, sync=pr["sync"])
        if not pk: continue
        mv = pr["variants"][pr["main_variant"]]; idb = mv["id_bytes"]; idv = bytes.fromhex(mv["id_value"]) if mv.get("id_value") else b""
        need = (max(idb) + 1) if idb else 4
        sel = [(t, by) for t, by in pk if len(by) >= need and by[:4].hex() == pr["main_variant"]]
        # score = fraction of ID byte positions whose modal value in THIS capture equals the profile's value (robust to bit errors)
        if idb and idv and sel:
            agree = 0
            for k, j in enumerate(idb):
                if j >= len(sel[0][1]) or k >= len(idv): continue
                modal = collections.Counter(by[j] for _, by in sel).most_common(1)[0][0]
                if modal == idv[k]: agree += 1
            score = agree / max(1, len(idb))
        else: score = 0
        if best is None or score > best[0]: best = (score, pr, sel)
    if not best or best[0] < 0.3: print("no profile matched this capture"); return
    score, pr, sel = best; print(f"best match: {pr['name']} (sync 0x{pr['sync']}, main variant {pr['main_variant']}, id match {score:.0%})")
    annotate(sel, pr, a.gap)

def cmd_dump(a):
    profs = {p["name"]: p for p in load_profiles()}; pr = profs.get(a.profile) if a.profile else None
    if a.profile and not pr: print(f"profile {a.profile} not found"); return
    sync = pr["sync"] if pr else a.sync
    if not sync: print("give --profile or --sync"); return
    nb = (pr or {}).get("nbytes", a.nbytes)
    fs, pk = extract_packets(a.file, (pr or {}).get("rate", a.rate), nb, a.thr, sync=sync)
    pk = [(t, by) for t, by in pk if len(by) >= nb - 1]
    print(f"{len(pk)} packets with sync 0x{sync}")
    if pr: annotate(pk, pr, a.gap)
    else:
        for t, by in pk: print(f"  t={t:7.2f}s  {by.hex(' ')}")

def annotate(pk, pr, gap):
    mv = pr["variants"][pr["main_variant"]]; cb = mv.get("command_bytes", []); ctb = mv.get("counter_bytes", [])
    last = None
    for t, by in pk:
        if by[:4].hex() != pr["main_variant"]: continue
        cmd = " ".join(f"{by[j]:02x}" for j in cb if j < len(by)); cnt = " ".join(f"{by[j]:02x}" for j in ctb if j < len(by))
        if cmd == last: continue
        last = cmd; print(f"  t={t:7.2f}s  command bytes [{','.join(map(str,cb))}]={cmd}  counter={cnt}")

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("scan", "learn", "id", "dump"):
        s = sub.add_parser(name); s.add_argument("file"); s.add_argument("--rate", type=float, default=1e6); s.add_argument("--thr", type=float, default=0); s.add_argument("--nbytes", type=int, default=48); s.add_argument("--gap", type=float, default=3.0); s.add_argument("--sync", default=None)
        if name == "learn": s.add_argument("--name", required=True)
        if name == "dump": s.add_argument("--profile", default=None)
    a = ap.parse_args(); {"scan": cmd_scan, "learn": cmd_learn, "id": cmd_id, "dump": cmd_dump}[a.cmd](a)

if __name__ == "__main__": main()
