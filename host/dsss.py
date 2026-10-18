#!/usr/bin/env python3
"""802.11b DSSS 1 Mbps (DBPSK, Barker-11) demodulator for raw dump-engine captures at ~16 MS/s.

  dsss.py captures/x.bin [--fs 15.995e6] [--cfo auto]

Pipeline: unpack 10-bit I/Q -> remove DC -> Barker correlation (reference sampled at fs with
11 chips per 1 us symbol) -> symbol timing from the correlation magnitude folded over one symbol
-> DBPSK differential decision with CFO estimate from the SYNC field -> 802.11 descrambler
(z^-4 + z^-7) -> SFD search (0xF3A0, 0x05CF for short preamble) -> PLCP header (SIGNAL,
SERVICE, LENGTH, CRC-16) -> PSDU bytes -> MAC header fields and the SSID element of a beacon.
"""
import argparse, sys
import numpy as np

BARKER = np.array([1, -1, 1, 1, -1, 1, 1, 1, -1, -1, -1], dtype=float)   # IEEE 802.11 chip sequence (+1 -1 +1 +1 -1 +1 +1 +1 -1 -1 -1)

def load(path):
    if path.endswith(".sigmf-meta") or path.endswith(".sigmf-data"):
        import sys, os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        from esp32sdr.sigmf import read_sigmf
        meta, blocks = read_sigmf(path)
        return [b for _, b in blocks]                       # list of blocks; main() iterates
    w = np.fromfile(path, dtype="<u4")
    i = ((w & 1023) ^ 512).astype(np.int32) - 512
    q = (((w >> 10) & 1023) ^ 512).astype(np.int32) - 512
    return i.astype(float) + 1j * q.astype(float)

def barker_ref(fs, sym_us=1.0):
    n = int(round(fs * sym_us * 1e-6))                 # samples per symbol (16 at 16 MS/s)
    # chip index for each sample instant within the symbol (11 chips per symbol)
    t = (np.arange(n) + 0.5) / n
    return BARKER[np.minimum((t * 11).astype(int), 10)], n

def demod(x, fs, verbose=True, conj=False):
    if conj: x = np.conj(x)
    x = x - x.mean()
    ref, n = barker_ref(fs)
    # correlate (matched filter): y[k] = sum_m x[k+m] * ref[m]
    y = np.correlate(x, ref[::-1].conj(), mode="valid") if False else np.convolve(x, ref[::-1], mode="valid")
    mag = np.abs(y)
    # symbol timing: fold the magnitude over the symbol period (allow fs error: use local folding over windows)
    nsym = len(y) // n
    fold = mag[: nsym * n].reshape(nsym, n)
    # pick the phase maximising energy over the whole capture; refine by tracking per 64-symbol windows
    prof = fold.sum(0); ph0 = int(np.argmax(prof))
    peakiness = prof.max() / np.median(prof)
    # tracked timing: for each window of W symbols choose the best phase near the previous one
    W = 32; phases = []; ph = ph0
    for s0 in range(0, nsym, W):
        blk = fold[s0:s0 + W].sum(0)
        cand = [(ph + d) % n for d in (-2, -1, 0, 1, 2)]
        ph = max(cand, key=lambda c: blk[c]); phases.append((s0, ph))
    sym = np.zeros(nsym, dtype=complex); amp = np.zeros(nsym)
    for k, (s0, ph) in enumerate(phases):
        s1 = min(nsym, s0 + W)
        idx = np.arange(s0, s1) * n + ph
        sym[s0:s1] = y[idx]; amp[s0:s1] = mag[idx]
    # DBPSK
    d = sym[1:] * np.conj(sym[:-1])
    # CFO: angle of mean(d^2)/2 removes the pi ambiguity; estimate over high-amplitude symbols only
    strong = amp[1:] > np.percentile(amp, 60)
    cfo_ang = 0.5 * np.angle(np.mean((d[strong] / (np.abs(d[strong]) + 1e-9)) ** 2))
    d_rot = d * np.exp(-1j * cfo_ang)
    bits = (d_rot.real < 0).astype(np.uint8)   # phase change pi -> 1
    soft = np.abs(d_rot.real) / (np.abs(d_rot) + 1e-9)
    if verbose:
        print(f"  samples {len(x)}  symbols {nsym}  timing peakiness {peakiness:.1f}  cfo {cfo_ang/(2*np.pi)*1e6/1.0:+.0f} Hz (per 1 us symbol)  "
              f"median |corr| {np.median(amp):.0f}  p90 {np.percentile(amp,90):.0f}")
    return bits, soft, amp, sym

def descramble(bits):
    # 802.11 self-synchronising descrambler: out = in ^ s[-4] ^ s[-7] where s is the received (scrambled) sequence
    out = np.zeros_like(bits)
    for k in range(len(bits)):
        a = bits[k - 4] if k >= 4 else 0
        b = bits[k - 7] if k >= 7 else 0
        out[k] = bits[k] ^ a ^ b
    return out

def bits_to_bytes_lsb(bits):
    nb = len(bits) // 8
    b = bits[: nb * 8].reshape(nb, 8)
    return np.packbits(b[:, ::-1], axis=1).ravel()

def crc16_ccitt(data_bits):
    # ITU-T CRC-16 as used by the PLCP header: x^16 + x^12 + x^5 + 1, preset ones, ones complement
    reg = 0xFFFF
    for bit in data_bits:
        fb = ((reg >> 15) & 1) ^ int(bit)
        reg = ((reg << 1) & 0xFFFF)
        if fb: reg ^= 0x1021
    return (~reg) & 0xFFFF

def find_frames(bits, soft):
    """Search the descrambled bit stream for SFD and decode what follows."""
    de = descramble(bits)
    out = []
    # SFD 0xF3A0 transmitted LSB first: bit pattern of 0x05CF reversed... build both long and short variants
    def pattern(v): return np.array([(v >> i) & 1 for i in range(16)], dtype=np.uint8)
    pats = {"long": pattern(0xF3A0), "short": pattern(0x05CF)}
    for name, pat in pats.items():
        for k in range(16, len(de) - 16):
            if np.array_equal(de[k:k + 16], pat):
                # check the SYNC: preceding 16 descrambled bits are all ones (long) or zeros (short)
                sync = de[k - 16:k]
                want = 1 if name == "long" else 0
                if np.count_nonzero(sync == want) < 14: continue
                h0 = k + 16
                if h0 + 48 > len(de): continue
                hdr = de[h0:h0 + 48]
                signal = int(np.packbits(hdr[0:8][::-1])[0]); service = int(np.packbits(hdr[8:16][::-1])[0])
                length = sum(int(hdr[16 + i]) << i for i in range(16))
                crc_rx = sum(int(hdr[32 + i]) << (15 - i) for i in range(16))   # CRC field is sent MSB first
                crc_ok = crc16_ccitt(hdr[0:32]) == crc_rx
                out.append(dict(kind=name, sfd_at=k, signal=signal, service=service, length_us=length, crc_ok=crc_ok, psdu_start=h0 + 48,
                                sync_soft=float(np.mean(soft[k-16:k+16]))))
    return de, out

def parse_mac(psdu):
    if len(psdu) < 24: return "short PSDU"
    fc = psdu[0] | (psdu[1] << 8); ftype = (psdu[0] >> 2) & 3; subtype = (psdu[0] >> 4) & 15
    mac = lambda b: ":".join(f"{x:02x}" for x in b)
    s = f"FC=0x{fc:04x} type={ftype} subtype={subtype} dur={psdu[2]|(psdu[3]<<8)} A1={mac(psdu[4:10])} A2={mac(psdu[10:16])} A3={mac(psdu[16:22])} seq={((psdu[22]|(psdu[23]<<8))>>4)}"
    if ftype == 0 and subtype == 8 and len(psdu) >= 38:
        ts = int.from_bytes(bytes(psdu[24:32]), "little"); bi = psdu[32] | (psdu[33] << 8); cap = psdu[34] | (psdu[35] << 8)
        s += f"\n    BEACON timestamp={ts} interval={bi} TU ({bi*1.024:.1f} ms) cap=0x{cap:04x}"
        p = 36
        while p + 2 <= len(psdu):
            tag, ln = psdu[p], psdu[p + 1]; body = bytes(psdu[p + 2:p + 2 + ln])
            if tag == 0: s += f"\n    SSID({ln})='{body.decode(errors='replace')}'"
            elif tag == 1: s += f"\n    rates={[f'{(r&0x7f)/2:g}' for r in body]}"
            elif tag == 3: s += f"\n    DS channel={body[0] if body else '?'}"
            if ln == 0 and tag not in (0,): pass
            p += 2 + ln
            if p > 36 + 64: break
    return s

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("files", nargs="+"); ap.add_argument("--fs", type=float, default=15.995e6)
    ap.add_argument("--conj", action="store_true"); ap.add_argument("--dump", action="store_true")
    args = ap.parse_args()
    for f in args.files:
        loaded = load(f)
        blocks = loaded if isinstance(loaded, list) else [loaded]
        for bi, x in enumerate(blocks):
          if len(blocks) > 1: print(f"{f} block {bi}")
          else: print(f)
          print(f"  peak |I|,|Q| {max(np.abs(x.real).max(), np.abs(x.imag).max()):.0f}  mean |x| {np.abs(x).mean():.0f}")
          for conj in ([args.conj] if args.conj else [False, True]):
              bits, soft, amp, sym = demod(x, args.fs, conj=conj)
              de, frames = find_frames(bits, soft)
              print(f"  conj={conj}: {len(frames)} SFD hit(s); descrambled ones-fraction {de.mean():.2f}")
              for fr in frames:
                  print(f"   {fr['kind']} SFD at symbol {fr['sfd_at']}  SIGNAL=0x{fr['signal']:02x} SERVICE=0x{fr['service']:02x} LENGTH={fr['length_us']} us  CRC {'OK' if fr['crc_ok'] else 'BAD'}  sync soft {fr['sync_soft']:.2f}")
                  psdu_bits = de[fr["psdu_start"]:]
                  psdu = bits_to_bytes_lsb(psdu_bits)
                  print("    " + parse_mac(psdu))
                  if args.dump: print("    bytes:", bytes(psdu[:64]).hex())
              if frames: break

if __name__ == "__main__": main()
