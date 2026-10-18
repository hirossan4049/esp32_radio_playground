#!/usr/bin/env python3
"""Recover an ESP32-C3 whose analog I2C registers were left in a bad state (e.g. core-voltage block
0x6d after a flip scan): the analog domain keeps its values across chip resets, so the chip may
crash-loop and even the esptool stub fails. The ROM downloader still works, and the analog I2C
master can be driven with esptool read_mem/write_mem (ROM-only, --no-stub).

Layout (from the C3 ROM: rom_i2c_readReg/writeReg bodies at 0x40038e58 / 0x40039170):
  ctrl register 0x6000E000 (+4 for blocks 0x62/0x63/0x64); busy = bit 25
  read : ctrl = 0x04000000 | reg<<8 | block, data in bits 16..23
  write: ctrl = 0x05000000 | data<<16 | reg<<8 | block
  ANA_CONFIG 0x6000E044: per-block disable bits (ROM writes ~mask(block)); 0 enables every block
  ANA_CONFIG2 0x6000E048: ROM sets (v & 0xfffe000f) | 0x1ff40 before access

  rom_recover_analog.py read  6d 0 8               read block 0x6d regs 0..7
  rom_recover_analog.py write 6d 6 9b              write one register
  rom_recover_analog.py restore6d                  write the known-good 0x6d regs 0..7 (f0 04 80 04 98 18 9b 18)
Run with esptool importable (source the IDF export.sh). Then reset the chip (esptool --before default_reset chip_id).
"""
import subprocess, re, sys
P = "/dev/cu.usbmodem2101"
GOOD_6D = [0xf0, 0x04, 0x80, 0x04, 0x98, 0x18, 0x9b, 0x18]

def esptool(*args, before="no_reset"):
    return subprocess.run([sys.executable, "-m", "esptool", "--chip", "esp32c3", "-p", P, "--no-stub", "--before", before, "--after", "no_reset", *args], capture_output=True, text=True).stdout
def rd(addr):
    m = re.search(r"= 0x([0-9a-f]+)", esptool("read_mem", f"0x{addr:08x}")); return int(m.group(1), 16) if m else None
def wr(addr, val): esptool("write_mem", f"0x{addr:08x}", f"0x{val:08x}")
def ctrl_for(blk): return 0x6000E004 if blk in (0x62, 0x63, 0x64) else 0x6000E000
def enable():
    c2 = rd(0x6000E048); wr(0x6000E048, (c2 & 0xfffe000f) | 0x1ff40); wr(0x6000E044, 0)
def ana_read(blk, reg):
    c = ctrl_for(blk); wr(c, 0x04000000 | (reg << 8) | blk); return (rd(c) >> 16) & 0xFF
def ana_write(blk, reg, data):
    c = ctrl_for(blk); wr(c, 0x05000000 | (data << 16) | (reg << 8) | blk)

def main():
    cmd = sys.argv[1]
    esptool("chip_id", before="default_reset")      # enter the ROM downloader
    enable()
    if cmd == "read":
        blk = int(sys.argv[2], 16); r0 = int(sys.argv[3]); n = int(sys.argv[4])
        print(f"blk {blk:02x}: " + " ".join(f"{ana_read(blk, r):02x}" for r in range(r0, r0 + n)))
    elif cmd == "write":
        blk = int(sys.argv[2], 16); reg = int(sys.argv[3]); val = int(sys.argv[4], 16)
        ana_write(blk, reg, val); print(f"blk {blk:02x} reg {reg}: now {ana_read(blk, reg):02x}")
    elif cmd == "restore6d":
        for r, v in enumerate(GOOD_6D):
            cur = ana_read(0x6d, r)
            if cur != v: ana_write(0x6d, r, v); print(f"reg{r}: {cur:02x} -> {ana_read(0x6d, r):02x}")
            else: print(f"reg{r}: {cur:02x} ok")
if __name__ == "__main__": main()
