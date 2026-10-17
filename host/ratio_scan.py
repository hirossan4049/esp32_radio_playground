#!/usr/bin/env python3
"""RX-C3-007: for a given CKGEN 0x65:0 value, sweep the PLL coordinate and collect narrow PSD
peaks; then test which conversion ratio r (effective LO = r * PLL) explains them as 40 MHz
crystal-harmonic lines (LO-RF convention: offset = LO_eff - 40k). Receive only."""
import sys, json, time
sys.path.insert(0, "host"); import rxprobe
import numpy as np
from fractions import Fraction

def rd65(s, r):
    rep = rxprobe.cmd(s, f"I2C 65 {r}", wait=0.5); m = [l for l in rep.splitlines() if l.startswith("I2C")]
    return int(m[-1].split("=")[-1], 16)

def scan(s, ck_val, plls, tag):
    base = rd65(s, 0)
    rows = []
    for pll in plls:
        rxprobe.cmd(s, f"TUNE {pll}")              # tune in normal mode (calibration), then set the selector
        rxprobe.cmd(s, f"I2CW 65 0 {ck_val:02x}", wait=0.3)
        rxprobe.cmd(s, "WR 60033d90 000c2040")
        try:
            data, meta = rxprobe.capture(s, 0x13fff, 16384)
        except Exception as e:
            print(f"PLL {pll/1e3:.0f}: capture failed {e}; resetting board"); rows.append(dict(pll=pll, error=str(e)))
            s.dtr = False; s.rts = True; time.sleep(0.1); s.rts = False; time.sleep(3); s.reset_input_buffer()
            continue
        finally:
            rxprobe.cmd(s, f"I2CW 65 0 {base:02x}", wait=0.3)
        w = np.frombuffer(data, dtype="<u4")
        P = rxprobe.psd_db(w); f = np.fft.fftshift(np.fft.fftfreq(len(P), 1 / 16e6)); med = np.median(P)
        pk = [(round(float(f[k]) / 1e6, 4), round(float(d), 1)) for k, d in rxprobe.peaks(P, f, 7.0, 8) if abs(f[k]) > 0.4e6]
        i = ((w & 1023) ^ 512).astype(float) - 512
        rows.append(dict(pll=pll, peaks=pk, istd=round(float(i.std()), 1)))
        print(f"PLL {pll/1e3:7.0f} ck={ck_val:02x} istd {i.std():6.1f} peaks " + " ".join(f"{o:+.3f}/{d:.0f}" for o, d in pk[:5]))
    return rows

def fit(rows, ratios, tol_mhz=0.06, min_db=7.0):
    out = []
    for r in ratios:
        hits = 0; cand = 0; details = []
        for row in rows:
            if "peaks" not in row: continue
            lo = float(r) * row["pll"] / 1e3
            off = lo - 40 * round(lo / 40)
            if abs(off) > 7.5 or abs(off) < 0.45: continue
            cand += 1
            got = [p for p in row["peaks"] if abs(p[0] - off) < tol_mhz and p[1] >= min_db]
            if got: hits += 1; details.append((row["pll"], round(off, 3), got[0]))
        if cand: out.append((hits / cand, hits, cand, r, details))
    out.sort(key=lambda t: (-t[0], -t[1]))
    return out

if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(); ap.add_argument("--ck", required=True); ap.add_argument("--start", type=int, default=2203000)
    ap.add_argument("--stop", type=int, default=2803000); ap.add_argument("--step", type=int, default=5000); ap.add_argument("--tag", default="ratio")
    a = ap.parse_args()
    s = rxprobe.open_port(rxprobe.PORT)
    ck = int(a.ck, 16)
    rows = scan(s, ck, list(range(a.start, a.stop + 1, a.step)), a.tag)
    json.dump(dict(ck=ck, rows=rows), open(f"logs/RX-C3-007_ratio_ck{ck:02x}.json", "w"), indent=1)
    ratios = sorted({Fraction(p, q) for q in range(1, 13) for p in range(1, 2 * q) if Fraction(p, q) <= Fraction(13, 10) and Fraction(p, q) >= Fraction(1, 8)})
    res = fit(rows, ratios)
    print(f"=== ck={ck:02x}: best ratio fits (fraction of predicted in-window lines actually observed) ===")
    for frac, hits, cand, r, det in res[:5]:
        print(f"  r={str(r):>6} ({float(r):.4f})  {hits:3d}/{cand:3d} = {frac:.2f}   e.g. {det[:3]}")
