#!/usr/bin/env python3
"""Capture I/Q from the ESP32-C3 into SigMF.

  capture.py --freq 2437 --rate 16000000 --gain 20 --duration 10 --output captures/wifi6
  capture.py --freq 2140 --rate 250000 --gain 50 --duration 5 --output captures/lte_b1

Rates: 80e6 / 40e6 / 16e6 give full-rate snapshots (16384 samples = 205/410/1024 us per block,
a few blocks per second because each 64 KiB frame has to cross USB). 1e6 / 500e3 / 250e3 / 125e3 /
62.5e3 give duty-cycled int8 streams: 1.024 ms of samples, then a ~0.5 ms gap, repeated (about 67%
coverage); above 250 kS/s USB cannot keep up and frames are dropped (reported). Every block is a
separate SigMF capture segment with the device timestamp, so nothing is presented as contiguous
that was not. 16 MS/s (and its decimated rates) alias the 8..11 MHz part of the analog passband.
"""
import argparse, sys, time, json, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from esp32sdr import Device, RATES, lo_plan, LO_MIN_HZ, LO_MAX_HZ
from esp32sdr.sigmf import SigMFWriter

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--freq", type=float, required=True, help="centre (LO) frequency in MHz, %.0f..%.0f" % (LO_MIN_HZ / 1e6, LO_MAX_HZ / 1e6))
    ap.add_argument("--rate", type=float, default=16e6, help="sample rate in S/s: " + ", ".join(str(r) for r in RATES))
    ap.add_argument("--gain", type=int, default=30, help="receiver gain index 0..60 (about 1 dB/step; -41 dBm input saturates above 40)")
    ap.add_argument("--duration", type=float, default=5.0, help="seconds")
    ap.add_argument("--output", required=True, help="basename for .sigmf-data/.sigmf-meta")
    ap.add_argument("--thr", type=int, default=0, help="event mode: only send blocks whose mean |I|+|Q| exceeds this (0 = all); skipped blocks leave a zero-length capture entry")
    ap.add_argument("--port", default=None); ap.add_argument("--quiet", action="store_true")
    a = ap.parse_args()
    freq_hz = int(round(a.freq * 1e6)); rate = int(round(a.rate))
    ctrl, decim, shift, actual, mode = Device.plan_rate(rate)
    dev = Device(a.port) if a.port else Device()
    pll_khz, alt = dev.tune(freq_hz); dev.set_gain(a.gain)
    hw = "ESP32-C3 Wi-Fi receiver, RF dump engine, firmware esp32c3_rxprobe"
    extra = {"esp32c3:lo_mode": "5/6" if alt else "1", "esp32c3:pll_khz": pll_khz, "esp32c3:gain_index": a.gain,
             "esp32c3:capture_mode": mode, "esp32c3:decimation": decim, "esp32c3:shift": shift, "esp32c3:dump_ctrl": f"0x{ctrl:05x}",
             "esp32c3:iq_convention": "I+jQ = LO - RF (spectrum mirrored relative to RF)",
             "esp32c3:measured_rate_hz": {16000000: 15995000, 40000000: 39970000, 80000000: 79850000}.get(rate if mode == "snapshot" else 16000000, None),
             "esp32c3:alias_warning": "16 MS/s path is a 1/5 decimation without anti-alias filter; analog passband +-11 MHz folds 8..11 MHz into the band" if ctrl == 0x13fff else None,
             "esp32c3:gaps": "blocks are not contiguous; see captures[] timestamps", "esp32c3:event_threshold": a.thr}
    extra = {k: v for k, v in extra.items() if v is not None}
    w = SigMFWriter(a.output, actual, freq_hz, "ci16_le" if mode == "snapshot" else "ci8", hw,
                    description=f"{a.freq} MHz, gain {a.gain}, {mode} mode, {a.duration} s", extra=extra)
    t_start = time.time(); n = [0]
    def on_frame(seq, t_us, fmt, flags, payload):
        w.add_block(payload, t_us, seq, flags); n[0] += 1
        if not a.quiet and n[0] % (5 if mode == "snapshot" else 200) == 0:
            print(f"\r  {n[0]} blocks, {w.nsamples} samples, {time.time()-t_start:.1f} s", end="", flush=True)
    trailer = dev.stream(rate, a.duration, on_frame, status=None if a.quiet else print, thr=a.thr)
    meta = w.close({"esp32c3:stream_trailer": trailer})
    print(f"\nwrote {w.nsamples} samples in {n[0]} blocks -> {meta}")
    print(f"  mode {mode}, rate {actual} S/s, LO {a.freq} MHz ({'5/6 mode, PLL %.3f MHz' % (pll_khz/1e3) if alt else 'direct'}), gain {a.gain}")
    print(f"  device: seq {trailer.get('seq')} sent {trailer.get('sent')} truncated {trailer.get('truncated')} host resyncs {trailer.get('resyncs')}")

if __name__ == "__main__": main()
