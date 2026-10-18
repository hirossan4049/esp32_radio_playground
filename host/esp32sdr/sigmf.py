"""SigMF writer/reader for gapped ESP32-C3 captures.

The data file is the concatenation of the received blocks (ci8 or ci16_le). The C3 cannot capture
continuously, so the file is NOT a continuous stream: every block becomes one entry in
`captures` with its `core:sample_start` (offset in the data file) and `core:datetime`, and the
device timestamp (`esp32c3:t_us`, microseconds since boot) gives the exact timeline. Consumers
that ignore the extension see a valid SigMF file whose segments are time-stamped; those that need
contiguous data must process one capture segment at a time (block_samples each).
"""
import json, datetime, os
import numpy as np

EXT = "esp32c3"

class SigMFWriter:
    def __init__(self, basename, sample_rate, freq_hz, datatype, hw, description="", extra=None):
        self.base = basename; self.data = open(basename + ".sigmf-data", "wb")
        self.rate = float(sample_rate); self.freq = float(freq_hz); self.datatype = datatype
        self.captures = []; self.annotations = []; self.nsamples = 0; self.t0 = None; self.wall0 = None
        self.bps = 2 if datatype == "ci8" else 4
        self.meta_global = {
            "core:datatype": datatype, "core:sample_rate": self.rate, "core:version": "1.2.0", "core:hw": hw,
            "core:author": "esp32_isdb host tools", "core:recorder": "esp32sdr capture.py", "core:description": description,
            "core:extensions": [{"name": EXT, "version": "0.1.0", "optional": True}],
        }
        self.meta_global.update(extra or {})
    def add_block(self, samples_bytes, t_us, seq, flags=0):
        n = len(samples_bytes) // self.bps
        if self.t0 is None: self.t0 = t_us; self.wall0 = datetime.datetime.now(datetime.timezone.utc)
        dt = (t_us - self.t0) & 0xFFFFFFFF
        when = self.wall0 + datetime.timedelta(microseconds=dt)
        self.captures.append({"core:sample_start": self.nsamples, "core:frequency": self.freq,
                              "core:datetime": when.strftime("%Y-%m-%dT%H:%M:%S.%fZ"),
                              f"{EXT}:t_us": int(t_us), f"{EXT}:seq": int(seq), f"{EXT}:block_samples": n, f"{EXT}:flags": int(flags)})
        self.data.write(samples_bytes); self.nsamples += n
    def annotate(self, start, count, **kw):
        a = {"core:sample_start": int(start), "core:sample_count": int(count)}; a.update(kw); self.annotations.append(a)
    def close(self, extra_global=None):
        self.data.close()
        g = dict(self.meta_global); g.update(extra_global or {})
        if self.captures:
            ts = [c[f"{EXT}:t_us"] for c in self.captures]
            gaps = [((b - a) & 0xFFFFFFFF) for a, b in zip(ts, ts[1:])]
            g[f"{EXT}:blocks"] = len(self.captures); g[f"{EXT}:total_samples"] = self.nsamples
            if gaps: g[f"{EXT}:block_pitch_us"] = {"min": min(gaps), "median": sorted(gaps)[len(gaps) // 2], "max": max(gaps)}
            blk_us = self.captures[0][f"{EXT}:block_samples"] / self.rate * 1e6
            g[f"{EXT}:block_duration_us"] = blk_us
            if gaps: g[f"{EXT}:duty_cycle"] = round(blk_us / (sorted(gaps)[len(gaps) // 2]), 4)
        json.dump({"global": g, "captures": self.captures, "annotations": self.annotations}, open(self.base + ".sigmf-meta", "w"), indent=1)
        return self.base + ".sigmf-meta"

def read_sigmf(path):
    """Return (meta, blocks) where blocks is a list of (t_us, complex64 array) per capture segment."""
    base = path[:-len(".sigmf-meta")] if path.endswith(".sigmf-meta") else path[:-len(".sigmf-data")] if path.endswith(".sigmf-data") else path
    meta = json.load(open(base + ".sigmf-meta")); dt = meta["global"]["core:datatype"]
    raw = np.fromfile(base + ".sigmf-data", dtype=np.int8 if dt == "ci8" else "<i2")
    x = raw[0::2].astype(np.float32) + 1j * raw[1::2].astype(np.float32)
    caps = meta["captures"]; blocks = []
    for i, c in enumerate(caps):
        s0 = c["core:sample_start"]; s1 = caps[i + 1]["core:sample_start"] if i + 1 < len(caps) else len(x)
        blocks.append((c.get(f"{EXT}:t_us", 0), x[s0:s1]))
    return meta, blocks
