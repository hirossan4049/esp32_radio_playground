#!/usr/bin/env python3
"""RX-C3-018: what do the other lane-select values of 0x60033D90 record? esp-sdr documents the four
6-bit fields (0,1,2,3 = receiver I/Q bytes). Set all four fields to the same value v and capture one
16 MS/s block per v; report word statistics so digital-side signals (AGC state, demodulator internals)
can be recognised. Receive only."""
import sys, json, time
sys.path.insert(0, "host"); import rxprobe
import numpy as np
s = rxprobe.open_port(rxprobe.PORT)
print(rxprobe.cmd(s, "TUNE 2437000").strip().splitlines()[-1]); rxprobe.cmd(s, "GAIN 30")
out = []
def reset():
    s.dtr = False; s.rts = True; time.sleep(0.1); s.rts = False; time.sleep(3); s.reset_input_buffer(); rxprobe.cmd(s, "TUNE 2437000"); rxprobe.cmd(s, "GAIN 30")
for v in range(64):
    sel = v | (v << 6) | (v << 12) | (v << 18)
    rxprobe.cmd(s, f"WR 60033d90 {sel:08x}")
    try: data, meta = rxprobe.capture(s, 0x13fff, 16384)
    except Exception as e:
        print(f"v={v:2d} sel={sel:08x} ERR {e}"); out.append(dict(v=v, sel=sel, error=str(e))); reset(); continue
    w = np.frombuffer(data, dtype="<u4"); open(f"captures/lane_v{v:02d}.bin", "wb").write(data)
    b = w.view(np.uint8).reshape(-1, 4)
    uniq = [int(len(np.unique(b[:, k]))) for k in range(4)]
    ent = []
    for k in range(4):
        c = np.bincount(b[:, k], minlength=256) / len(b); c = c[c > 0]; ent.append(float(-(c * np.log2(c)).sum()))
    changes = int(np.count_nonzero(np.diff(w)))
    r = dict(v=v, sel=sel, uniq_bytes=uniq, entropy_bits=[round(e, 2) for e in ent], changes=changes, first=[f"{x:08x}" for x in w[:4]], uniq_words=int(len(np.unique(w))))
    out.append(r); print(f"v={v:2d} sel={sel:08x} uniq/byte {uniq} entropy {[round(e,1) for e in ent]} word-changes {changes} uniq-words {r['uniq_words']} first {r['first'][:2]}", flush=True)
rxprobe.cmd(s, "WR 60033d90 000c2040")
json.dump(out, open("logs/RX-C3-018_lane_probe.json", "w"), indent=0)
