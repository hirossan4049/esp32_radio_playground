#!/usr/bin/env python3
"""nRF24L01(+)-style Enhanced ShockBurst decoder for SigMF captures (1 Mbps GFSK, 2402+ MHz).

Frame: preamble (0xAA/0x55, 8 bits) | address 3..5 bytes (MSB first) | PCF 9 bits (payload length 6, PID 2, NO_ACK 1)
       | payload 0..32 bytes (MSB first) | CRC-16 (CCITT, init 0xFFFF, poly 0x1021, over address+PCF+payload)
Legacy ShockBurst has no PCF (fixed payload length). The decoder tries address lengths 3/4/5, both FSK polarities and
a few symbol-timing offsets per burst and reports frames whose CRC verifies, plus address statistics.

  nrf24.py capture.sigmf-meta [--rate 1e6] [--all]
"""
import argparse, sys, os
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from esp32sdr.sigmf import read_sigmf

def crc16(bits):
    reg = 0xFFFF
    for b in bits:
        fb = ((reg >> 15) & 1) ^ int(b); reg = (reg << 1) & 0xFFFF
        if fb: reg ^= 0x1021
    return reg

def crc8(bits):
    reg = 0xFF
    for b in bits:
        fb = ((reg >> 7) & 1) ^ int(b); reg = (reg << 1) & 0xFF
        if fb: reg ^= 0x07
    return reg

def bursts_of(x, fs, thr=30, minlen=40):
    env = np.convolve(np.abs(x), np.ones(16) / 16, mode="same"); idx = np.flatnonzero(env > thr)
    if not len(idx): return []
    out, a = [], idx[0]
    for p, q in zip(idx[:-1], idx[1:]):
        if q - p > 40: out.append((a, p)); a = q
    out.append((a, idx[-1]))
    return [(a, b) for a, b in out if b - a >= minlen]

def symbols(seg, fs, rate, conj, phase):
    x = np.conj(seg) if conj else seg; x = x - x.mean()
    d = np.angle(x[1:] * np.conj(x[:-1])); sps = fs / rate
    k = max(1, int(round(sps * 0.5))); d = np.convolve(d, np.ones(k) / k, mode="same")
    env = np.abs(x); start = np.flatnonzero(env > env.max() * 0.35)[0]
    bi = np.round(start + phase + np.arange(int((len(d) - start - phase) / sps)) * sps).astype(int); bi = bi[(bi >= 0) & (bi < len(d))]
    v = d[bi]; return (v - np.median(v) > 0).astype(np.int8)

def try_decode(bits):
    """Return list of (addr_len, address_hex, pcf, payload_hex, crc_kind) for framings whose CRC verifies."""
    # locate the end of the alternating preamble: first index where bits[i] == bits[i+1] after >= 5 alternations
    n = len(bits); alt = 0; p_end = None
    for i in range(1, n):
        if bits[i] != bits[i - 1]: alt += 1
        else:
            if alt >= 5: p_end = i; break
            alt = 0
    if p_end is None: return []
    out = []
    for p0 in (p_end - 1, p_end, p_end + 1):     # tolerate one-bit timing slips at the preamble end
        for alen in (3, 4, 5):
            A = alen * 8
            if p0 + A + 9 + 16 > n: continue
            addr = bits[p0:p0 + A]; pcf = bits[p0 + A:p0 + A + 9]
            length = sum(int(pcf[i]) << (5 - i) for i in range(6)); pid = (pcf[6] << 1) | pcf[7]; noack = int(pcf[8])
            if length <= 32 and p0 + A + 9 + length * 8 + 16 <= n:
                body = bits[p0 + A + 9:p0 + A + 9 + length * 8]; crc = bits[p0 + A + 9 + length * 8:p0 + A + 9 + length * 8 + 16]
                if crc16(np.concatenate([addr, pcf, body])) == sum(int(crc[i]) << (15 - i) for i in range(16)):
                    out.append((alen, np.packbits(addr).tobytes().hex(), dict(length=length, pid=pid, noack=noack), np.packbits(body).tobytes().hex() if length else "", "esb/crc16"))
            # legacy ShockBurst (no PCF): fixed payload lengths 1..32 with CRC-16 or CRC-8 over address+payload
            for L in range(0, 33):
                e = p0 + A + L * 8
                if e + 16 <= n and crc16(bits[p0:e]) == sum(int(bits[e + i]) << (15 - i) for i in range(16)):
                    out.append((alen, np.packbits(bits[p0:p0 + A]).tobytes().hex(), dict(length=L, legacy=True), np.packbits(bits[p0 + A:e]).tobytes().hex(), "sb/crc16"))
                # (CRC-8 legacy framing is not searched: ~300 trials per burst make 1/256 chance matches meaningless)
    return out

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("files", nargs="+"); ap.add_argument("--rate", type=float, default=1e6); ap.add_argument("--all", action="store_true"); ap.add_argument("--thr", type=float, default=30)
    a = ap.parse_args()
    for f in a.files:
        meta, blocks = read_sigmf(f); fs = meta["global"]["core:sample_rate"]; sps = fs / a.rate
        nb = 0; ok = 0; addrs = {}; kinds = {}
        print(f"{f}: {len(blocks)} blocks, fs {fs/1e6:g} MS/s, trying {a.rate/1e6:g} Mbps")
        for bi, (t_us, x) in enumerate(blocks):
            for s0, s1 in bursts_of(x, fs, a.thr):
                seg = x[max(0, s0 - 8):s1 + 8]; nb += 1; found = None
                for conj in (True, False):
                    for ph in np.arange(0, sps, max(1, sps / 4)):
                        for r in try_decode(symbols(seg, fs, a.rate, conj, ph)):
                            found = (conj, ph, r); break
                        if found: break
                    if found: break
                if found:
                    ok += 1; conj, ph, (alen, addr, pcf, payload, kind) = found
                    addrs[addr] = addrs.get(addr, 0) + 1; kinds[kind] = kinds.get(kind, 0) + 1
                    print(f"  blk {bi:4d} t={t_us/1e6:9.3f}s dur {len(seg)/fs*1e6:4.0f}us  {kind:9s} addr({alen}B) {addr}  {pcf}  payload {payload}  pol={'conj' if conj else 'direct'}")
                elif a.all: print(f"  blk {bi:4d} t={t_us/1e6:9.3f}s dur {len(seg)/fs*1e6:4.0f}us  no CRC match")
        print(f"  bursts {nb}, CRC-verified {ok}; kinds {kinds}; addresses {sorted(addrs.items(), key=lambda kv: -kv[1])[:6]}")

if __name__ == "__main__": main()
