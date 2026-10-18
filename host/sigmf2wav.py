#!/usr/bin/env python3
"""Export a SigMF capture to a WAV file that SDR++ (File Source) and most SDR tools can open.

  sigmf2wav.py captures/x.sigmf-meta [--out dir] [--contiguous]

By default one WAV per capture block is NOT written; instead the blocks are concatenated and a
sidecar .json lists where each block starts and its device timestamp (the gaps are real; SDR++
plays the file as if continuous, which is fine for looking at spectra but not for timing).
The filename follows the SDR++ convention baseband_<centre Hz>_<time>.wav so the centre frequency
is picked up automatically. For GNU Radio use gr-sigmf on the .sigmf files directly, or a File
Source on the .sigmf-data (ci8 -> "byte" interleaved I/Q, ci16_le -> "short" interleaved I/Q).
"""
import argparse, sys, os, json, wave, datetime
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from esp32sdr.sigmf import read_sigmf, EXT

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("file"); ap.add_argument("--out", default=None); ap.add_argument("--flip-rf", action="store_true", help="conjugate so the spectrum reads as RF (default keeps LO-RF)")
    a = ap.parse_args()
    meta, blocks = read_sigmf(a.file); g = meta["global"]; fs = int(round(g["core:sample_rate"])); f0 = int(round(meta["captures"][0]["core:frequency"]))
    x = np.concatenate([b for _, b in blocks])
    if a.flip_rf: x = np.conj(x)
    scale = 32767 / max(1.0, np.abs(x.real).max(), np.abs(x.imag).max())
    pcm = np.empty(2 * len(x), dtype="<i2"); pcm[0::2] = np.round(x.real * scale); pcm[1::2] = np.round(x.imag * scale)
    stamp = datetime.datetime.now().strftime("%H-%M-%S_%d-%m-%Y")
    outdir = a.out or os.path.dirname(os.path.abspath(a.file)); os.makedirs(outdir, exist_ok=True)
    name = os.path.join(outdir, f"baseband_{f0}Hz_{stamp}.wav")
    with wave.open(name, "wb") as w:
        w.setnchannels(2); w.setsampwidth(2); w.setframerate(fs); w.writeframes(pcm.tobytes())
    side = {"source": os.path.abspath(a.file), "sample_rate": fs, "center_frequency_hz": f0, "iq_convention": "conj(LO-RF)=RF" if a.flip_rf else "LO-RF",
            "blocks": [{"sample_start": c["core:sample_start"], "t_us": c.get(EXT + ":t_us")} for c in meta["captures"]],
            "note": "blocks are not contiguous in time; see t_us"}
    json.dump(side, open(name[:-4] + ".json", "w"), indent=1)
    print(f"wrote {name} ({len(x)} samples, {fs} S/s) and sidecar json; {len(blocks)} blocks")

if __name__ == "__main__": main()
