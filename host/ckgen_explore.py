#!/usr/bin/env python3
"""RX-C3-006: flip single bits of the analog CKGEN block (0x65) registers 0..2 and look for
a changed receive conversion using the 40 MHz crystal-harmonic comb as the test signal.
For each flip: tune PLL coordinates where normal mode shows the comb line at +3 MHz,
capture at 16 Msps, record the +3 MHz level, the strongest off-DC peak, the dump fill time
(sample clock) and the register readback. Restore and verify after every flip. Receive only."""
import sys, json, time
sys.path.insert(0, "host"); import rxprobe
import numpy as np

PLLS = [2403000, 2443000, 2523000, 2683000]
s = rxprobe.open_port(rxprobe.PORT)

def reset_board():
    s.dtr = False; s.rts = True; time.sleep(0.1); s.dtr = False; s.rts = False; time.sleep(3)
    s.reset_input_buffer()

def rd65(r):
    rep = rxprobe.cmd(s, f"I2C 65 {r}", wait=0.5)
    m = [l for l in rep.splitlines() if l.startswith("I2C")]
    return int(m[-1].split("=")[-1], 16) if m else None

def wr65(r, v):
    rep = rxprobe.cmd(s, f"I2CW 65 {r} {v:02x}", wait=0.5)
    m = [l for l in rep.splitlines() if l.startswith("I2CW")]
    return int(m[-1].split("now")[-1], 16) if m else None

def measure(tag):
    out = []
    for pll in PLLS:
        rxprobe.cmd(s, f"TUNE {pll}"); rxprobe.cmd(s, "WR 60033d90 000c2040")
        try:
            data, meta = rxprobe.capture(s, 0x13fff, 16384)
        except Exception as e:
            out.append(dict(pll=pll, error=str(e))); print(f"   {tag} PLL {pll/1e3:.0f}: capture error {e}"); return out, False
        w = np.frombuffer(data, dtype="<u4")
        open(f"captures/ckgen_{tag}_pll{pll}.bin", "wb").write(data)
        P = rxprobe.psd_db(w); f = np.fft.fftshift(np.fft.fftfreq(len(P), 1 / 16e6)); med = np.median(P)
        k3 = int(np.argmin(np.abs(f - 3e6))); m3 = P[k3 - 1:k3 + 2].max() - med
        mask = np.abs(f) > 0.4e6
        kb = int(np.argmax(np.where(mask, P, -1e9))); best = (P[kb] - med, f[kb])
        i = ((w & 1023) ^ 512).astype(float) - 512
        cyc = int(meta["cycles"]); rate = 16384 / (cyc / 160e6)
        out.append(dict(pll=pll, m3=round(float(m3), 1), best_db=round(float(best[0]), 1), best_off=round(float(best[1]) / 1e6, 3), istd=round(float(i.std()), 1), cycles=cyc, msps=round(rate, 2)))
        print(f"   {tag} PLL {pll/1e3:.0f}: +3MHz {m3:5.1f} dB | best {best[0]:5.1f} dB @ {best[1]/1e6:+7.3f} MHz | I std {i.std():6.1f} | {rate:6.2f} MS/s")
    return out, True

results = {"baseline": {}, "flips": []}
base = [rd65(r) for r in range(3)]
print("CKGEN baseline regs0..2:", [f"{b:02x}" for b in base])
results["baseline"]["regs"] = base
results["baseline"]["meas"], _ = measure("base")
for r in range(3):
    for bit in range(8):
        if r == 0 and bit == 4:
            continue  # known 5/6 bit
        v0 = base[r]; v1 = v0 ^ (1 << bit)
        rb = wr65(r, v1)
        print(f"-- reg{r} bit{bit}: {v0:02x} -> {v1:02x} (readback {rb if rb is None else format(rb, '02x')})")
        meas, ok = measure(f"r{r}b{bit}")
        entry = dict(reg=r, bit=bit, written=v1, readback=rb, meas=meas, ok=ok)
        if not ok:
            print("   board unresponsive, resetting"); reset_board()
            entry["reset"] = True
        else:
            wr65(r, v0)
            chk = rd65(r)
            if chk != v0:
                print(f"   restore readback {chk} != {v0:02x}; resetting"); reset_board(); entry["reset"] = True
        results["flips"].append(entry)
        json.dump(results, open("logs/RX-C3-006_ckgen_explore.json", "w"), indent=1)
# final sanity: baseline marker back?
print("final check:"); measure("final")
