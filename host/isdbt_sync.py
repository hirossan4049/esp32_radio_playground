#!/usr/bin/env python3
"""Offline ISDB-T coarse synchronisation on complex I/Q (Phase 5 groundwork).

Guard-interval correlation: for each mode (FFT 2048/4096/8192 at the ISDB-T sample rate
512/63 MHz = 8.126984 MS/s) and guard ratio (1/4, 1/8, 1/16, 1/32), correlate x[n] with
x[n+Nfft] over the guard length and look for a periodic peak every symbol. Reports the best
(mode, GI) and a confidence metric. Also works on a one-segment capture (the correlation does
not need the full 13 segments, only the symbol structure, but it needs the ISDB-T sample clock).

  isdbt_sync.py file.bin --fs 1e6 --fmt int8     (I,Q interleaved)
  isdbt_sync.py --selftest                          (synthetic mode-3 GI 1/8 signal, SNR sweep)
"""
import argparse, sys
import numpy as np

FS_ISDBT = 512e6 / 63          # 8.126984 MS/s
MODES = {1: 2048, 2: 4096, 3: 8192}
GIS = {"1/4": 4, "1/8": 8, "1/16": 16, "1/32": 32}

def resample(x, fs_in, fs_out):
    n_out = int(len(x) * fs_out / fs_in)
    t_in = np.arange(len(x)) / fs_in; t_out = np.arange(n_out) / fs_out
    return np.interp(t_out, t_in, x.real) + 1j * np.interp(t_out, t_in, x.imag)

def gi_metric(x, nfft, ng):
    """Normalised guard correlation: moving sum over ng of x[n]*conj(x[n+nfft]) divided by power."""
    if len(x) < 2 * (nfft + ng): return 0.0, None
    prod = x[:-nfft] * np.conj(x[nfft:])
    pw = (np.abs(x[:-nfft]) ** 2 + np.abs(x[nfft:]) ** 2) / 2
    k = np.ones(ng)
    c = np.convolve(prod, k, "valid"); p = np.convolve(pw, k, "valid") + 1e-9
    m = np.abs(c) / p
    # fold over the symbol period and look for a stable peak
    ns = nfft + ng; nsym = len(m) // ns
    if nsym < 2: return float(m.max()), None
    fold = m[:nsym * ns].reshape(nsym, ns).mean(axis=0)
    peak = fold.max(); floor = np.median(fold)
    return float((peak - floor) / (floor + 1e-9)), int(np.argmax(fold))

def detect(x, fs):
    y = resample(x, fs, FS_ISDBT) if abs(fs - FS_ISDBT) > 1 else x
    y = y - y.mean()
    res = []
    for mode, nfft in MODES.items():
        for gname, gdiv in GIS.items():
            ng = nfft // gdiv
            score, pos = gi_metric(y, nfft, ng)
            res.append((score, mode, gname, pos))
    res.sort(reverse=True)
    return res

def synth(mode=3, gdiv=8, nsym=12, snr_db=10, cfo_hz=0.0, seed=1):
    rng = np.random.default_rng(seed); nfft = MODES[mode]; ng = nfft // gdiv
    # ISDB-T uses 13 segments * (nfft/16*... ) carriers; use the full-band carrier count approximation 5617 for mode 3
    ncar = {1: 1405, 2: 2809, 3: 5617}[mode]
    sym = []
    for _ in range(nsym):
        X = np.zeros(nfft, complex)
        q = (rng.integers(0, 4, ncar) * 2 + 1) * np.pi / 4
        X[1:ncar + 1] = np.exp(1j * q)
        X = np.roll(X, -(ncar // 2))
        t = np.fft.ifft(X) * np.sqrt(nfft)
        sym.append(np.concatenate([t[-ng:], t]))
    s = np.concatenate(sym)
    s *= np.exp(2j * np.pi * cfo_hz * np.arange(len(s)) / FS_ISDBT)
    p = np.mean(np.abs(s) ** 2); n = (rng.normal(size=len(s)) + 1j * rng.normal(size=len(s))) * np.sqrt(p / (2 * 10 ** (snr_db / 10)))
    return s + n

if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("path", nargs="?"); ap.add_argument("--fs", type=float); ap.add_argument("--fmt", default="int8")
    ap.add_argument("--selftest", action="store_true"); a = ap.parse_args()
    if a.selftest:
        for snr in (20, 5, 0, -3):
            x = synth(mode=3, gdiv=8, snr_db=snr, cfo_hz=1500)
            r = detect(x, FS_ISDBT)
            print(f"SNR {snr:3d} dB -> best {r[0][1]} GI {r[0][2]} score {r[0][0]:.2f}; runner-up mode {r[1][1]} GI {r[1][2]} score {r[1][0]:.2f}")
        x = synth(mode=2, gdiv=4, snr_db=5); r = detect(x, FS_ISDBT); print(f"mode2 GI1/4 test -> {r[0][1]} GI {r[0][2]} score {r[0][0]:.2f}")
        # resampling path from 1 MS/s capture of one segment would need the segment itself; test resample from 16 MS/s instead
        x = synth(mode=3, gdiv=8, snr_db=10); x16 = resample(x, FS_ISDBT, 16e6); r = detect(x16, 16e6); print(f"16 MS/s resample test -> {r[0][1]} GI {r[0][2]} score {r[0][0]:.2f}")
        sys.exit(0)
    raw = np.fromfile(a.path, dtype=np.int8 if a.fmt == "int8" else "<i2")
    x = raw[0::2].astype(float) + 1j * raw[1::2].astype(float)
    for score, mode, g, pos in detect(x, a.fs)[:5]:
        print(f"mode {mode} GI {g}: score {score:.2f} pos {pos}")
