#!/usr/bin/env python3
"""Frequency sweep with signal detection over the tunable range (about 1842..2803 MHz).

  sweep.py --start 1842 --stop 2803 --step 20 --rate 40000000 --gain 50 --blocks 2 --out captures/sweep
      -> captures/sweep.json (per-step PSD and floor), captures/sweep.png (spectrogram vs RF),
         and a printed list of detected signals (RF bands where the PSD exceeds the local floor).

The receiver sees about +-11 MHz around each LO; a 40 MS/s snapshot covers that analog passband
without aliasing, so 20 MHz steps tile the range. Detection is per 1 MHz RF bin: median PSD over
the blocks (robust to bursts) minus the sweep-wide noise floor; adjacent hot bins are merged.
"""
import argparse, sys, os, json, time
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from esp32sdr import Device, lo_plan

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--start", type=float, default=1842); ap.add_argument("--stop", type=float, default=2803); ap.add_argument("--step", type=float, default=20)
    ap.add_argument("--rate", type=float, default=40e6); ap.add_argument("--gain", type=int, default=50); ap.add_argument("--blocks", type=int, default=2)
    ap.add_argument("--nfft", type=int, default=1024); ap.add_argument("--out", default="captures/sweep"); ap.add_argument("--thresh", type=float, default=6.0)
    ap.add_argument("--view", type=float, default=10.0, help="use +-view MHz of each snapshot")
    a = ap.parse_args()
    dev = Device(); dev.set_gain(a.gain)
    ctrl, decim, shift, fs, mode = Device.plan_rate(int(a.rate)); assert mode == "snapshot", "sweep needs a full-rate snapshot rate"
    f = np.fft.fftshift(np.fft.fftfreq(a.nfft, 1 / fs)) / 1e6
    los = np.arange(a.start, a.stop + 1e-9, a.step); rows = []
    rf_bins = np.arange(np.floor(a.start - a.view), np.ceil(a.stop + a.view) + 1, 1.0); acc = np.full(len(rf_bins), np.nan)
    for lo in los:
        try: dev.tune(int(lo * 1e6))
        except ValueError as e: print("skip", lo, e); continue
        Ps = []
        def on_frame(seq, t_us, fmt, flags, payload):
            raw = np.frombuffer(payload, dtype="<i2"); x = raw[0::2].astype(np.float32) + 1j * raw[1::2].astype(np.float32); x -= x.mean()
            n = len(x) // a.nfft; X = np.fft.fftshift(np.fft.fft(x[:n * a.nfft].reshape(n, a.nfft) * np.hanning(a.nfft), axis=1), axes=1)
            Ps.append(np.median(np.abs(X) ** 2, axis=0))
        # duration: enough for `blocks` frames at ~15 frames/s
        dev.stream(int(a.rate), max(0.3, a.blocks / 12 + 0.2), on_frame)
        if not Ps: continue
        P = 10 * np.log10(np.median(np.array(Ps[:a.blocks]), axis=0) + 1e-9)
        rf = lo - f                                   # I+jQ = LO - RF
        sel = np.abs(f) <= a.view
        for b in rf_bins:
            m = sel & (rf >= b) & (rf < b + 1)
            if m.any():
                i = int(b - rf_bins[0]); v = float(np.mean(P[m])); acc[i] = v if np.isnan(acc[i]) else min(acc[i], v)   # overlap: keep the lower (safer) value
        rows.append(dict(lo_mhz=float(lo), psd_db=P.round(2).tolist(), floor_db=float(np.median(P[sel])), blocks=len(Ps)))
        print(f"LO {lo:7.1f} MHz  floor {rows[-1]['floor_db']:6.1f} dB  blocks {len(Ps)}", flush=True)
    valid = ~np.isnan(acc); floor = np.median(acc[valid]); dev_db = acc - floor
    hot = valid & (dev_db > a.thresh); sigs = []
    i = 0
    while i < len(rf_bins):
        if hot[i]:
            j = i
            while j + 1 < len(rf_bins) and hot[j + 1]: j += 1
            sigs.append(dict(rf_start_mhz=float(rf_bins[i]), rf_stop_mhz=float(rf_bins[j] + 1), peak_db=float(np.nanmax(dev_db[i:j + 1])), mean_db=float(np.nanmean(dev_db[i:j + 1])))); i = j + 1
        else: i += 1
    print(f"\nsweep floor {floor:.1f} dB; signals > +{a.thresh} dB:")
    for s_ in sigs: print(f"  {s_['rf_start_mhz']:7.0f}..{s_['rf_stop_mhz']:7.0f} MHz  peak {s_['peak_db']:+5.1f} dB  mean {s_['mean_db']:+5.1f} dB")
    json.dump(dict(rate=fs, gain=a.gain, nfft=a.nfft, f_mhz=f.tolist(), rows=rows, rf_bins_mhz=rf_bins.tolist(), rf_psd_db=[None if np.isnan(v) else round(float(v), 2) for v in acc], floor_db=float(floor), signals=sigs), open(a.out + ".json", "w"))
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 7), gridspec_kw={"height_ratios": [1, 1.6]})
    ax1.plot(rf_bins + 0.5, acc - floor, lw=0.9); ax1.axhline(a.thresh, color="r", lw=0.6, ls="--"); ax1.set_ylabel("dB above floor"); ax1.grid(alpha=0.3)
    ax1.set_title(f"ESP32-C3 sweep {a.start:.0f}-{a.stop:.0f} MHz, {fs/1e6:.0f} MS/s snapshots, gain {a.gain}")
    M = np.array([r["psd_db"] for r in rows]); sel = np.abs(f) <= a.view
    img = ax2.imshow(M[:, sel], aspect="auto", origin="lower", extent=[-a.view, a.view, rows[0]["lo_mhz"], rows[-1]["lo_mhz"]], cmap="viridis", vmin=np.percentile(M, 5), vmax=np.percentile(M, 99.5))
    ax2.set_xlabel("baseband MHz (LO - RF)"); ax2.set_ylabel("LO MHz"); fig.colorbar(img, ax=ax2, label="dB")
    fig.tight_layout(); fig.savefig(a.out + ".png", dpi=110); print("wrote", a.out + ".json", a.out + ".png")

if __name__ == "__main__": main()
