#!/usr/bin/env python3
"""Offline BLE 1M advertising-channel demodulator for SigMF captures (ESP32-C3 I/Q).

  ble.py captures/ble_2402_2M_ev.sigmf-meta [--channel 37] [--fs 2e6]

Per block: conjugate (the C3 gives I+jQ = LO - RF, which inverts FSK polarity), frequency
discriminator, symbol timing from the preamble/AA correlation, hard decisions, search for the
advertising access address 0x8E89BED6 (LSB first, after the 0xAA preamble), de-whiten with the
channel LFSR (seed = channel | 0x40), parse the PDU header (type, length), check CRC-24
(init 0x555555, poly 0x00065B). Reports every AA hit and every CRC-OK PDU with its AdvA.
"""
import argparse, sys, os
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from esp32sdr.sigmf import read_sigmf

ADV_AA = 0x8E89BED6
CH_FREQ = {37: 2402, 38: 2426, 39: 2480}
PDU_TYPES = {0: "ADV_IND", 1: "ADV_DIRECT_IND", 2: "ADV_NONCONN_IND", 3: "SCAN_REQ", 4: "SCAN_RSP", 5: "CONNECT_IND", 6: "ADV_SCAN_IND", 7: "ADV_EXT_IND"}

def aa_bits(aa):                       # transmitted LSB first
    return np.array([(aa >> i) & 1 for i in range(32)], dtype=np.int8)

def whitening(channel, n):
    lfsr = (channel & 0x3F) | 0x40; out = np.zeros(n, dtype=np.int8)
    for i in range(n):
        out[i] = lfsr & 1
        lfsr >>= 1
        if out[i]: lfsr ^= 0x44          # x^7 + x^4 + 1 (bit6 is the feedback tap after the shift)
    return out

def crc24(bits):
    reg = 0x555555
    for b in bits:
        fb = ((reg >> 23) & 1) ^ int(b)
        reg = (reg << 1) & 0xFFFFFF
        if fb: reg ^= 0x00065B
    return reg

def demod_block(x, fs, channel):
    sps = fs / 1e6
    x = np.conj(x)                                     # LO - RF -> RF - LO
    x = x - x.mean()
    d = np.angle(x[1:] * np.conj(x[:-1]))              # instantaneous frequency (rad/sample)
    # low-pass the discriminator (moving average over one symbol)
    k = max(1, int(round(sps * 0.75)))                  # discriminator smoothing: ~3/4 symbol
    df = np.convolve(d, np.ones(k) / k, mode="same")
    hits = []
    pre = np.concatenate([np.array([0, 1] * 4, dtype=np.int8), aa_bits(ADV_AA)])   # preamble 0xAA (LSB first: 0,1,0,1...) + AA
    ref = np.where(pre == 1, 1.0, -1.0)
    # correlate with the NRZ pattern sampled at sps (allows fractional sps)
    idx = np.round(np.arange(len(pre)) * sps).astype(int)
    N = len(df) - idx[-1] - 1
    if N <= 0: return hits
    # remove local DC (CFO) with a window the length of the pattern
    win = int(idx[-1] + 1)
    cs = np.cumsum(np.concatenate([[0], df]))
    local_mean = (cs[win:] - cs[:-win]) / win
    corr = np.zeros(N)
    for j, i0 in enumerate(idx):
        corr += (df[i0:i0 + N] - local_mean[:N]) * ref[j]
    corr /= len(pre)
    thr = 0.35 * np.percentile(np.abs(df), 95) + 1e-6
    cand = np.flatnonzero(corr > thr)
    # keep local maxima separated by at least a symbol
    peaks = []
    for c in cand:
        if peaks and c - peaks[-1] < sps: 
            if corr[c] > corr[peaks[-1]]: peaks[-1] = c
        else: peaks.append(c)
    for p in peaks:
        cfo = local_mean[p]
        best = None
        # symbol timing: the correlation peak is good to about one sample; try the neighbouring offsets and keep the
        # first decoding whose header is sane and whose CRC checks (a wrong offset practically never passes the CRC)
        for toff in sorted(range(-1, int(np.ceil(sps))), key=lambda t: abs(t - 1)):
            start = p + idx[-1] + 1 + toff                 # first PDU bit sample (after AA)
            nb = 2 + 37 + 3                               # header, max payload, CRC -> bytes
            bi = np.round(start + np.arange(nb * 8) * sps).astype(int)
            bi = bi[(bi >= 0) & (bi < len(df))]
            if len(bi) < 16 + 24 + 8: continue
            bits = (df[bi] - cfo > 0).astype(np.int8)
            raw = bits ^ whitening(channel, len(bits))
            hdr = raw[:16]
            pdu_type = sum(int(hdr[i]) << i for i in range(4)); txadd = int(hdr[6]); length = sum(int(hdr[8 + i]) << i for i in range(8))
            if length > 37 or len(raw) < 16 + length * 8 + 24:
                if best is None: best = dict(pos=int(p), cfo_hz=float(cfo * fs / (2 * np.pi)), pdu_type=pdu_type, length=length, crc_ok=False, reason="length")
                continue
            body = raw[16:16 + length * 8]; crc_rx = raw[16 + length * 8:16 + length * 8 + 24]
            crc_rx_v = sum(int(crc_rx[i]) << (23 - i) for i in range(24))   # CRC transmitted MSB first
            ok = crc24(np.concatenate([hdr, body])) == crc_rx_v
            payload = np.packbits(body.reshape(-1, 8)[:, ::-1], axis=1).ravel() if length else np.array([], dtype=np.uint8)
            adva = ":".join(f"{b:02x}" for b in payload[:6][::-1]) if length >= 6 and pdu_type in (0, 1, 2, 4, 6) else ""
            cand = dict(pos=int(p), toff=toff, cfo_hz=float(cfo * fs / (2 * np.pi)), pdu_type=pdu_type, txadd=txadd, length=length, crc_ok=ok, adva=adva,
                        payload=bytes(payload).hex(), corr=float(corr[p]), amp=float(np.abs(x).mean()))
            if ok: best = cand; break
            if best is None or not best.get("crc_ok"): best = cand
        if best: hits.append(best)
    return hits

def parse_ad(payload_hex):
    b = bytes.fromhex(payload_hex)[6:]; out = []; i = 0
    while i + 1 < len(b):
        ln = b[i]; t = b[i + 1] if ln else 0
        if ln == 0: break
        v = b[i + 2:i + 1 + ln]
        if t in (8, 9): out.append(f"name='{v.decode(errors='replace')}'")
        elif t == 1: out.append(f"flags=0x{v[0]:02x}" if v else "flags")
        elif t == 0xff: out.append(f"mfr=0x{int.from_bytes(v[:2], 'little'):04x}{'...' if len(v) > 2 else ''}")
        elif t in (2, 3): out.append("uuid16=" + ",".join(f"{int.from_bytes(v[j:j+2],'little'):04x}" for j in range(0, len(v) - 1, 2)))
        else: out.append(f"ad{t:02x}({ln-1}B)")
        i += 1 + ln
    return " ".join(out)

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("files", nargs="+"); ap.add_argument("--channel", type=int, default=None); ap.add_argument("--all", action="store_true")
    a = ap.parse_args()
    for f in a.files:
        meta, blocks = read_sigmf(f); fs = meta["global"]["core:sample_rate"]; f0 = meta["captures"][0]["core:frequency"] / 1e6
        ch = a.channel or {v: k for k, v in CH_FREQ.items()}.get(int(round(f0)), 37)
        n_hits = n_ok = 0; seen = {}
        print(f"{f}: {len(blocks)} blocks, fs {fs/1e6:g} MS/s, LO {f0:.1f} MHz -> channel {ch}")
        for bi, (t_us, x) in enumerate(blocks):
            if len(x) < 400: continue
            for h in demod_block(x, fs, ch):
                n_hits += 1
                if h["crc_ok"]:
                    n_ok += 1; seen[h["adva"]] = seen.get(h["adva"], 0) + 1
                    print(f"  blk {bi:5d} t={t_us/1e6:9.3f}s  {PDU_TYPES.get(h['pdu_type'], h['pdu_type']):16s} len {h['length']:2d}  AdvA {h['adva']}  cfo {h['cfo_hz']/1e3:+6.1f} kHz  CRC OK  {parse_ad(h['payload'])}")
                elif a.all:
                    print(f"  blk {bi:5d} AA hit, {PDU_TYPES.get(h['pdu_type'], h['pdu_type'])} len {h['length']} CRC bad ({h.get('reason','')})")
        print(f"  access-address hits {n_hits}, CRC OK {n_ok}, distinct AdvA {len(seen)}")

if __name__ == "__main__": main()
