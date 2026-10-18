"""Serial control of the esp32c3_rxprobe firmware for SDR capture.

Facts measured on the ESP32-C3 (docs/c3_findings.md):
  * dump rates 80 / 40 / 16 MS/s (ctrl bits 15/16); 16 MS/s is a plain 1/5 decimation (aliases 8..11 MHz)
  * one bank = 16384 samples = 1.024 ms at 16 MS/s; capture -> copy -> restart leaves ~0.5 ms gaps
  * LO: PLL 2203..2803 MHz locks; CKGEN 5/6 mode gives LO = 5/6 x PLL, i.e. 1836..2336 MHz
  * analog bandwidth about +-11 MHz, 10-bit I/Q, I+jQ = LO - RF (spectrum is mirrored vs RF)
"""
import struct, time
import serial

PORT = "/dev/cu.usbmodem2101"
BLOCK_WORDS = 16384
CYCLES_PER_SEC = 160e6
# nominal rates and the ctrl word that selects them; measured 79.85 / 39.97 / 15.995 MS/s
FULL_RATES = {80_000_000: 0x03fff, 40_000_000: 0x0bfff, 16_000_000: 0x13fff}
DECIMS = (4, 8, 16, 32, 64, 128, 256)          # 4 MS/s .. 62.5 kS/s int8; above 250 kS/s USB limits the coverage
RATES = sorted(list(FULL_RATES) + [16_000_000 // d for d in DECIMS], reverse=True)
PLL_MIN_HZ, PLL_MAX_HZ = 2_203_000_000, 2_803_000_000
LO_MIN_HZ, LO_MAX_HZ = int(PLL_MIN_HZ * 5 / 6), PLL_MAX_HZ

def lo_plan(freq_hz):
    """Return (pll_khz, alternate) for an effective LO, or raise ValueError."""
    f = int(freq_hz)
    if PLL_MIN_HZ <= f <= PLL_MAX_HZ: return f // 1000, False
    p = f * 6 // 5
    if PLL_MIN_HZ <= p <= PLL_MAX_HZ: return p // 1000, True
    raise ValueError(f"{f/1e6:.3f} MHz is outside the tunable range {LO_MIN_HZ/1e6:.0f}..{LO_MAX_HZ/1e6:.0f} MHz")

class Device:
    def __init__(self, port=PORT, timeout=2):
        self.s = serial.Serial(port, 115200, timeout=timeout)
        self.s.dtr = True; self.s.rts = False
        time.sleep(0.2); self.s.reset_input_buffer()
        self.freq_hz = None; self.alternate = False; self.gain = None

    def cmd(self, text, wait=1.0, until=None):
        self.s.reset_input_buffer(); self.s.write((text + "\n").encode())
        dl = time.time() + wait; out = b""
        while time.time() < dl:
            chunk = self.s.read(self.s.in_waiting or 1)
            if chunk:
                out += chunk
                if until and until.encode() in out: break
                if not until and out.endswith(b"\r\n"): break
        return out.decode(errors="replace")

    def info(self):
        return [l for l in self.cmd("INFO").splitlines() if l.startswith("RXPROBE")]

    def reset(self):
        self.s.dtr = False; self.s.rts = True; time.sleep(0.1); self.s.rts = False; time.sleep(3); self.s.reset_input_buffer()

    def _ckgen(self, alternate):
        rep = self.cmd("I2C 65 0", wait=0.5); m = [l for l in rep.splitlines() if l.startswith("I2C")]
        cur = int(m[-1].split("=")[-1], 16); new = (cur | 0x10) if alternate else (cur & ~0x10)
        if new != cur: self.cmd(f"I2CW 65 0 {new:02x}", wait=0.3)

    def tune(self, freq_hz):
        pll_khz, alt = lo_plan(freq_hz)
        rep = self.cmd(f"TUNE {pll_khz}", wait=1.5)
        if "OK TUNE" not in rep: raise RuntimeError("TUNE failed: " + rep.strip()[-120:])
        self._ckgen(alt)
        self.cmd("WR 60033d90 000c2040")          # I/Q byte lanes; the Wi-Fi driver leaves this 0
        self.freq_hz = int(freq_hz); self.alternate = alt
        return pll_khz, alt

    def set_gain(self, gain):
        rep = self.cmd(f"GAIN {int(gain)}"); self.gain = int(gain)
        if "OK GAIN" not in rep: raise RuntimeError("GAIN failed: " + rep.strip()[-80:])

    @staticmethod
    def plan_rate(rate):
        """-> (ctrl, decim, shift, actual_rate, mode)"""
        rate = int(rate)
        if rate in FULL_RATES: return FULL_RATES[rate], 1, 0, rate, "snapshot"
        for d in DECIMS:
            if 16_000_000 // d == rate: return 0x13fff, d, {4: 2, 8: 3, 16: 4, 32: 5, 64: 6, 128: 7, 256: 8}[d], rate, "duty"
        raise ValueError(f"unsupported rate {rate}; choose one of {RATES}")

    def stream(self, rate, duration_s, on_frame, status=None, thr=0):
        """Run STREAM2 and call on_frame(seq, t_us, fmt, flags, payload_bytes) per frame.
        Returns the trailer dict (seq, sent, truncated)."""
        ctrl, decim, shift, actual, mode = self.plan_rate(rate)
        ms = int(duration_s * 1000)
        self.s.reset_input_buffer(); self.s.timeout = 5
        self.s.write(f"STREAM2 {ctrl:x} {BLOCK_WORDS} {decim} {shift} {ms} {int(thr)}\n".encode())
        # header line
        hdr = b""
        dl = time.time() + 5
        while time.time() < dl:
            line = self.s.read_until(b"\r\n", 4096)
            if line.startswith(b"STREAM2 start"): hdr = line; break
        if not hdr: raise RuntimeError("no STREAM2 start line")
        if status: status(hdr.decode(errors="replace").strip())
        buf = bytearray(); frames = 0; resyncs = 0; trailer = None
        dl = time.time() + duration_s + 10
        while time.time() < dl:
            chunk = self.s.read(max(1, self.s.in_waiting))
            if chunk: buf += chunk
            # parse frames
            while True:
                k = buf.find(b"ESPB")
                if k < 0:
                    e = buf.find(b"STREAM2 end")
                    if e >= 0 and b"END\r\n" in buf[e:]:
                        trailer = buf[e:].split(b"\r\n")[0].decode(errors="replace"); buf.clear(); break
                    if len(buf) > 65536: del buf[:-65536]
                    break
                if k > 0:
                    if b"STREAM2 end" in buf[:k] and b"END\r\n" in buf[:k]:
                        trailer = buf[buf.find(b"STREAM2 end"):].split(b"\r\n")[0].decode(errors="replace")
                    resyncs += 1 if frames else 0; del buf[:k]
                if len(buf) < 16: break
                seq, t_us, nsamp, fmt, flags = struct.unpack_from("<IIHBB", buf, 4)
                plen = nsamp * (4 if fmt else 2)
                if len(buf) < 16 + plen: break
                # validate: the next bytes after the frame must be another magic, the trailer, or end of data so far
                nxt = buf[16 + plen:16 + plen + 4]
                if len(nxt) == 4 and nxt != b"ESPB" and not nxt.startswith(b"\r\n"):
                    resyncs += 1; del buf[:4]; continue          # truncated frame: resync on the next magic
                on_frame(seq, t_us, fmt, flags, bytes(buf[16:16 + plen])); frames += 1
                del buf[:16 + plen]
            if trailer: break
        if not trailer:
            self.s.write(b"x"); time.sleep(0.3); rest = self.s.read(self.s.in_waiting or 1)
            e = rest.find(b"STREAM2 end"); trailer = rest[e:].split(b"\r\n")[0].decode(errors="replace") if e >= 0 else "STREAM2 end (no trailer)"
        self.s.timeout = 2
        d = dict(kv.split("=", 1) for kv in trailer.split()[2:] if "=" in kv)
        d.update(frames=frames, resyncs=resyncs, mode=mode, decim=decim, shift=shift, ctrl=ctrl, actual_rate=actual, thr=int(thr))
        return d
