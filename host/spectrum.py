#!/usr/bin/env python3
"""Spectrum and waterfall from a SigMF capture made with capture.py.

  spectrum.py captures/x.sigmf-meta [--nfft 1024] [--png out.png] [--rf]

Each capture segment (block) is one row of the waterfall; rows are placed at their true device
time, so the gaps between blocks are visible rather than hidden. --rf flips the frequency axis so
it reads as RF (the C3 delivers I+jQ = LO - RF).
"""
import argparse, sys, os, json
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from esp32sdr.sigmf import read_sigmf, EXT

def block_psd(x, nfft):
    x = x - x.mean()
    n = len(x) // nfft
    if n == 0:
        X = np.fft.fftshift(np.fft.fft(x * np.hanning(len(x)), nfft)); return 10 * np.log10(np.abs(X) ** 2 + 1e-9)
    X = np.fft.fftshift(np.fft.fft(x[:n * nfft].reshape(n, nfft) * np.hanning(nfft), axis=1), axes=1)
    return 10 * np.log10(np.mean(np.abs(X) ** 2, axis=0) + 1e-9)

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("file"); ap.add_argument("--nfft", type=int, default=1024)
    ap.add_argument("--png"); ap.add_argument("--rf", action="store_true"); ap.add_argument("--max-rows", type=int, default=2000)
    ap.add_argument("--peaks", type=float, default=10.0, help="report peaks this many dB above the median")
    a = ap.parse_args()
    meta, blocks = read_sigmf(a.file); g = meta["global"]; fs = g["core:sample_rate"]; f0 = meta["captures"][0]["core:frequency"]
    nfft = min(a.nfft, len(blocks[0][1]))
    f = np.fft.fftshift(np.fft.fftfreq(nfft, 1 / fs)) / 1e6
    rows = []; times = []
    step = max(1, len(blocks) // a.max_rows)
    for t_us, x in blocks[::step]:
        rows.append(block_psd(x, nfft)); times.append(t_us)
    W = np.array(rows); t = (np.array(times, dtype=np.int64) - times[0]) / 1e6
    avg = 10 * np.log10(np.mean(10 ** (W / 10), axis=0)); med = np.median(avg)
    print(f"{a.file}: {len(blocks)} blocks, fs {fs/1e6:.4g} MS/s, LO {f0/1e6:.3f} MHz, {g.get('core:datatype')}, span {t[-1]:.3f} s, coverage {g.get(EXT+':duty_cycle', '?')}")
    print(f"  mean PSD median {med:.1f} dB; peaks > +{a.peaks} dB:")
    idx = [i for i in range(2, nfft - 2) if avg[i] > med + a.peaks and avg[i] >= avg[i - 2:i + 3].max()]
    for i in sorted(idx, key=lambda i: -avg[i])[:12]:
        fb = f[i]; frf = f0 / 1e6 - fb
        print(f"    baseband {fb:+8.4f} MHz  (RF {frf:9.4f} MHz)  {avg[i]-med:+5.1f} dB")
    if a.png:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 8), gridspec_kw={"height_ratios": [1, 2.2]})
        fx = (f0 / 1e6 - f) if a.rf else f; order = np.argsort(fx)
        ax1.plot(fx[order], avg[order], lw=0.8); ax1.set_ylabel("dB"); ax1.grid(alpha=0.3)
        ax1.set_title(f"{os.path.basename(a.file)}  LO {f0/1e6:.3f} MHz  fs {fs/1e6:.4g} MS/s  gain {g.get(EXT+':gain_index','?')}  {len(blocks)} blocks")
        vmin, vmax = np.percentile(W, 5), np.percentile(W, 99.5)
        # draw each row at its true time; use pcolormesh with row edges
        ax2.pcolormesh(fx[order], t, W[:, order], vmin=vmin, vmax=vmax, cmap="viridis", shading="nearest")
        ax2.set_xlabel("RF MHz" if a.rf else "baseband MHz (LO - RF)"); ax2.set_ylabel("time s (true, gaps included)")
        fig.tight_layout(); fig.savefig(a.png, dpi=110); print("  wrote", a.png)

if __name__ == "__main__": main()
