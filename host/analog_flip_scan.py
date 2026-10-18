#!/usr/bin/env python3
"""RX-C3-011: single-bit flips of an analog block (default RFPLL 0x62) register, after tuning,
with a short PLL sweep and ratio fit per flip. Looks for LO post-divider or conversion bits.
Receive only; restores the register after each flip; resets the board on failure."""
import sys, json, time, argparse
sys.path.insert(0, "host"); import rxprobe, ratio_scan
import numpy as np
from fractions import Fraction

ap = argparse.ArgumentParser(); ap.add_argument("--block", default="62"); ap.add_argument("--regs", default="0-15")
ap.add_argument("--step", type=int, default=20000); ap.add_argument("--start", type=int, default=2203000); ap.add_argument("--stop", type=int, default=2803000)
a = ap.parse_args()
blk = int(a.block, 16); r0, r1 = [int(x) for x in a.regs.split("-")]
s = rxprobe.open_port(rxprobe.PORT)
plls = list(range(a.start, a.stop + 1, a.step))
ratios = sorted({Fraction(p, q) for q in range(1, 13) for p in range(1, 2 * q) if Fraction(1, 10) <= Fraction(p, q) <= Fraction(13, 10)})

# Blocks that hold power/clock settings are never scanned: 0x61 (ULP/brownout bias), 0x66 (BBPLL = CPU/ADC clock),
# 0x69 (digital LDO/SAR), 0x6d (I2C_DIG_REG = core/RTC voltage regulators; flipping reg 6 bit 4 crash-looped the chip, RX-C3-019).
FORBIDDEN = {0x61, 0x66, 0x69, 0x6d}
if blk in FORBIDDEN: sys.exit(f"block 0x{blk:02x} is a power/clock block; refusing to flip it")

def alive():
    rep = rxprobe.cmd(s, "INFO", wait=1.0)
    return "RXPROBE" in rep

def rdreg(r):
    for attempt in range(3):
        rep = rxprobe.cmd(s, f"I2C {blk:x} {r}", wait=0.5); m = [l for l in rep.splitlines() if l.startswith("I2C")]
        if m: return int(m[-1].split("=")[-1], 16)
        reset()
        if not alive(): sys.exit("board does not answer after reset (crash loop?); aborting scan")
    sys.exit("I2C read keeps failing; aborting scan")

def reset():
    s.dtr = False; s.rts = True; time.sleep(0.1); s.rts = False; time.sleep(3); s.reset_input_buffer()

out = {"block": blk, "flips": []}
for reg in range(r0, r1 + 1):
    rxprobe.cmd(s, "TUNE 2437000"); base = rdreg(reg)
    for bit in range(8):
        v1 = base ^ (1 << bit); rows = []; dead = 0
        for pll in plls:
            rxprobe.cmd(s, f"TUNE {pll}")
            cur = rdreg(reg)
            rxprobe.cmd(s, f"I2CW {blk:x} {reg} {cur ^ (1 << bit):02x}", wait=0.3)
            rxprobe.cmd(s, "WR 60033d90 000c2040")
            try:
                data, meta = rxprobe.capture(s, 0x13fff, 16384)
            except Exception as e:
                dead += 1; reset(); rows.append(dict(pll=pll, error=str(e))); continue
            finally:
                try: rxprobe.cmd(s, f"I2CW {blk:x} {reg} {cur:02x}", wait=0.3)
                except Exception: pass
            w = np.frombuffer(data, dtype="<u4"); P = rxprobe.psd_db(w); f = np.fft.fftshift(np.fft.fftfreq(len(P), 1 / 16e6)); med = np.median(P)
            pk = [(round(float(f[k]) / 1e6, 4), round(float(d), 1)) for k, d in rxprobe.peaks(P, f, 7.0, 8) if abs(f[k]) > 0.4e6]
            i = ((w & 1023) ^ 512).astype(float) - 512
            k3 = int(np.argmin(np.abs(f - 3e6)))
            rows.append(dict(pll=pll, peaks=pk, istd=round(float(i.std()), 1), m3=round(float(P[k3 - 1:k3 + 2].max() - med), 1)))
        fits = ratio_scan.fit(rows, ratios)
        best = fits[0] if fits else (0, 0, 0, None, [])
        m3s = [r["m3"] for r in rows if "m3" in r]; m3_keep = sum(1 for r in rows if "m3" in r and (r["pll"] - 3000) % 40000 == 0 and r["m3"] > 6)
        n40 = sum(1 for r in rows if (r["pll"] - 3000) % 40000 == 0)
        print(f"blk {blk:02x} reg{reg:2d} bit{bit} {base:02x}->{v1:02x}: marker(+3MHz at 40k+3) {m3_keep}/{n40}  best r={best[3]} {best[1]}/{best[2]}  dead={dead}  istd med {np.median([r['istd'] for r in rows if 'istd' in r]) if m3s else 0:.0f}", flush=True)
        out["flips"].append(dict(reg=reg, bit=bit, base=base, rows=rows, best=[str(best[3]), best[1], best[2]], dead=dead))
        json.dump(out, open(f"logs/RX-C3-011_flip_blk{blk:02x}.json", "w"))
print("DONE")
