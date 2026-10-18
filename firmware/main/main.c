/* esp32c3_rxprobe: receive-side probe for the ESP32-C3 Wi-Fi radio.
 *
 * Receive only. The firmware never enables a transmit path: it brings up
 * the PHY through esp_wifi (mode NULL), tunes the RF PLL with the PHY
 * library's own channel functions, forces the receiver on, and triggers the
 * radio's undocumented ADC sample-dump engine (the one librftest's adctrig
 * uses) into internal SRAM at 0x3FCB0000. The captured words go to the host
 * over USB Serial/JTAG for analysis.
 *
 * Text commands, one per line:
 *   INFO
 *   TUNE <kHz>                 e.g. TUNE 2437000 (accepted 2200000..2800000, PHY limits)
 *   GAIN <0..127>              force the AGC gain selector (0x6001C02C)
 *   CAP <ctrl_hex> <nwords>    program dump control (without the run bit), run, send the words
 *   RD <addr_hex>              read a 32-bit register / SRAM word
 *   WR <addr_hex> <val_hex>    write one (peripheral 0x6000xxxx..0x600Fxxxx or SRAM only)
 *   I2C <blk> <reg>            analog register read  (rom_i2c_readReg(blk, 1, reg))
 *   I2CW <blk> <reg> <val>     analog register write
 *   OWN                        print the SRAM owner register 0x600C1020
 */
#include <inttypes.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_attr.h"
#include "esp_cpu.h"
#include "esp_event.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_chip_info.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/usb_serial_jtag_ll.h"
#include "nvs_flash.h"
#include "heap_memory_layout.h"
#include "esp_heap_caps.h"
#include "soc/soc.h"

/* The dump engine writes the 128 KiB RF bank 0x3FCA0000..0x3FCBFFFF (data at
 * 0x3FCB0000). Keep heap and static data out of it. */
SOC_RESERVE_MEMORY_REGION(0x3fca0000, 0x3fcc0000, c3_rf_dump);

#define DUMP_CTRL   0x60033D5Cu
#define DUMP_WIDX   0x60033D60u
#define SRAM_OWNER  0x600C1020u
#define AGC_FORCE   0x6001C02Cu
#define DUMP_BANK   ((volatile uint32_t *)0x3fcb0000)
#define DUMP_BANK_WORDS 16384u
static uint32_t own_bit3 = 8u;   /* OWNBIT3 0 clears the bit-3 handoff in every start sequence */

extern void stop_tx_tone(unsigned);
extern void rom_pbus_workmode(void);
extern void rom_pbus_xpd_rx_on(unsigned);
extern void rom_pbus_xpd_tx_off(void);
extern void rom_set_rxclk_en(unsigned);
extern void set_chanfreq(unsigned, unsigned);
extern void phy_set_freq(unsigned, int);
extern void force_rx_gain(unsigned, unsigned, unsigned);
extern void adctrig(unsigned, unsigned, unsigned, unsigned, unsigned, unsigned, unsigned, unsigned, unsigned);
extern uint8_t rom_i2c_readReg(uint8_t block, uint8_t host_id, uint8_t reg_add);
extern void rom_i2c_writeReg(uint8_t block, uint8_t host_id, uint8_t reg_add, uint8_t data);

static inline uint32_t rd(uint32_t a) { return *(volatile uint32_t *)a; }
static inline void wr(uint32_t a, uint32_t v) { *(volatile uint32_t *)a = v; }

static void usb_write(const void *data, size_t n)
{
    const uint8_t *p = data;
    int64_t deadline = esp_timer_get_time() + 2000000;
    while (n) {
        if (usb_serial_jtag_ll_txfifo_writable()) {
            int w = usb_serial_jtag_ll_write_txfifo(p, n > 64 ? 64 : n);
            usb_serial_jtag_ll_txfifo_flush();
            p += w;
            n -= w;
            deadline = esp_timer_get_time() + 2000000;
        } else if (esp_timer_get_time() > deadline) {
            return;
        }
    }
}

static void say(const char *fmt, ...)
{
    char b[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(b, sizeof b, fmt, ap);
    va_end(ap);
    if (n > 0) usb_write(b, n < (int)sizeof b ? (size_t)n : sizeof b - 1);
}

/* ---- radio ---------------------------------------------------------------- */

static uint32_t tuned_khz;

static unsigned chan_mhz(unsigned ch) { return ch == 14 ? 2484 : 2407 + 5 * ch; }

/* libphy rfpll_set_freq arithmetic, 40 MHz crystal: 30 MHz * (32 + W/65536). */
static double pll_hz(unsigned mhz, int khz)
{
    const int32_t d = 120000;
    int32_t a = 4 * (int32_t)(1000 * mhz + khz) - 32 * d;
    int32_t o0 = a / d; a -= o0 * d; a <<= 8;
    int32_t o1 = a / d; a -= o1 * d; a <<= 8;
    int32_t o2 = a / d;
    return 30e6 * ((o0 & 255) + 32 + ((o1 & 255) * 256 + (o2 & 255)) / 65536.0);
}

static void rx_mode(unsigned gain)
{
    stop_tx_tone(1);
    rom_pbus_workmode();
    rom_pbus_xpd_tx_off();
    rom_pbus_xpd_rx_on(1);
    rom_set_rxclk_en(1);
    force_rx_gain(0, gain, 0);
}

static bool tune(uint32_t fkhz)
{
    /* Wide request window for lock-range mapping; the PHY decides what locks. Receive only. */
    if (fkhz < 1500000 || fkhz > 3500000) return false;
    int ch = ((int)fkhz - 2407000 + 2500) / 5000;
    ch = ch < 1 ? 1 : ch > 13 ? 13 : ch;
    unsigned mhz = fkhz / 1000;
    int khz = (int)(fkhz % 1000);
    set_chanfreq(chan_mhz(ch), 0);
    if (fkhz != chan_mhz(ch) * 1000u) phy_set_freq(mhz, khz);
    rx_mode(40);
    tuned_khz = fkhz;
    say("OK TUNE %" PRIu32 " kHz chan %d pll %.0f Hz\r\n", fkhz, ch, pll_hz(mhz, khz));
    return true;
}

static void set_gain(unsigned g)
{
    uint32_t v = rd(AGC_FORCE);
    v = (v & 0x00FFFFFFu) | ((g & 0x7Fu) << 24);
    wr(AGC_FORCE, v);
    wr(AGC_FORCE, rd(AGC_FORCE) | 0x00800000u);
    say("OK GAIN %u reg %08" PRIx32 "\r\n", g, rd(AGC_FORCE));
}

/* One-shot dump: program the control word, hand the bank to the radio, run,
 * wait for the done flag (bit 18) or a timeout, and report timing. */
static void capture(uint32_t ctrl, uint32_t nwords)
{
    if (nwords == 0 || nwords > DUMP_BANK_WORDS) nwords = DUMP_BANK_WORDS;
    for (uint32_t i = 0; i < DUMP_BANK_WORDS; i++) DUMP_BANK[i] = 0xDEADBEEFu;
    const uint32_t owner0 = rd(SRAM_OWNER);
    wr(DUMP_CTRL, rd(DUMP_CTRL) & ~(1u << 31));
    wr(DUMP_CTRL, ctrl & ~(1u << 31));
    wr(SRAM_OWNER, (owner0 & ~7u) | 2u);
    wr(SRAM_OWNER, rd(SRAM_OWNER) | own_bit3);
    __asm__ volatile("fence rw,rw" ::: "memory");
    const uint32_t owner_set = rd(SRAM_OWNER);
    uint32_t widx0 = rd(DUMP_WIDX);
    uint32_t t0 = esp_cpu_get_cycle_count();
    wr(DUMP_CTRL, rd(DUMP_CTRL) | (1u << 31));
    /* librftest adctrig (mode 0): after the run bit, pulse bit 19 to start the writer. */
    wr(DUMP_CTRL, rd(DUMP_CTRL) | (1u << 19));
    wr(DUMP_CTRL, rd(DUMP_CTRL) & ~(1u << 19));
    uint32_t t_done = 0, status = 0, widx_done = 0;
    bool done = false;
    for (uint32_t i = 0; i < 2000000; i++) {
        status = rd(DUMP_CTRL);
        if (status & (1u << 18)) { t_done = esp_cpu_get_cycle_count(); widx_done = rd(DUMP_WIDX); done = true; break; }
    }
    if (!done) { t_done = esp_cpu_get_cycle_count(); widx_done = rd(DUMP_WIDX); }
    /* Sample the write index a few times to see whether the ring keeps moving. */
    uint32_t w1 = rd(DUMP_WIDX); esp_rom_delay_us(100); uint32_t w2 = rd(DUMP_WIDX);
    wr(DUMP_CTRL, rd(DUMP_CTRL) & ~(1u << 31));
    __asm__ volatile("fence rw,rw" ::: "memory");
    wr(SRAM_OWNER, owner0);
    __asm__ volatile("fence rw,rw" ::: "memory");
    say("CAP ctrl=%08" PRIx32 " status=%08" PRIx32 " done=%d cycles=%" PRIu32 " widx0=%08" PRIx32 " widx_done=%08" PRIx32 " w1=%08" PRIx32 " w2=%08" PRIx32 " owner0=%08" PRIx32 " owner_set=%08" PRIx32 " n=%" PRIu32 " tuned=%" PRIu32 "\r\n",
        ctrl, status, done, t_done - t0, widx0, widx_done, w1, w2, owner0, owner_set, nwords, tuned_khz);
    /* Binary payload: magic, nwords, then the words. */
    uint32_t hdr[2] = {0x43415031u, nwords};
    usb_write(hdr, sizeof hdr);
    usb_write((const void *)DUMP_BANK, nwords * 4);
    say("\r\nEND\r\n");
}


/* ---- ring / continuous mode experiments ------------------------------------ */

/* RING: start the engine with the given control word, sample the write index over time,
 * read a few bank words at start and end, stop. Tells whether circular mode exists. */
static void ring_probe(uint32_t ctrl, uint32_t total_us)
{
    for (uint32_t i = 0; i < DUMP_BANK_WORDS; i++) DUMP_BANK[i] = 0xDEADBEEFu;
    const uint32_t owner0 = rd(SRAM_OWNER);
    wr(DUMP_CTRL, rd(DUMP_CTRL) & ~(1u << 31));
    wr(DUMP_CTRL, ctrl & ~(1u << 31));
    wr(SRAM_OWNER, (owner0 & ~7u) | 2u);
    wr(SRAM_OWNER, rd(SRAM_OWNER) | own_bit3);
    __asm__ volatile("fence rw,rw" ::: "memory");
    uint32_t t0 = esp_cpu_get_cycle_count();
    wr(DUMP_CTRL, rd(DUMP_CTRL) | (1u << 31));
    wr(DUMP_CTRL, rd(DUMP_CTRL) | (1u << 19));
    wr(DUMP_CTRL, rd(DUMP_CTRL) & ~(1u << 19));
    uint32_t samp_t[16], samp_w[16], samp_s[16], first_word[4], last_word[4];
    uint32_t step = total_us / 16 ? total_us / 16 : 1;
    for (int k = 0; k < 16; k++) {
        esp_rom_delay_us(step);
        samp_t[k] = esp_cpu_get_cycle_count() - t0;
        samp_w[k] = rd(DUMP_WIDX);
        samp_s[k] = rd(DUMP_CTRL);
        if (k == 0) for (int j = 0; j < 4; j++) first_word[j] = DUMP_BANK[j * 4000];
    }
    for (int j = 0; j < 4; j++) last_word[j] = DUMP_BANK[j * 4000];
    wr(DUMP_CTRL, rd(DUMP_CTRL) & ~(1u << 31));
    __asm__ volatile("fence rw,rw" ::: "memory");
    wr(SRAM_OWNER, owner0);
    say("RING ctrl=%08" PRIx32 " us=%" PRIu32 "\r\n", ctrl, total_us);
    for (int k = 0; k < 16; k++)
        say("  t=%9" PRIu32 " cyc widx=%08" PRIx32 " status=%08" PRIx32 "\r\n", samp_t[k], samp_w[k], samp_s[k]);
    say("  words[0,4000,8000,12000] first: %08" PRIx32 " %08" PRIx32 " %08" PRIx32 " %08" PRIx32 "  last: %08" PRIx32 " %08" PRIx32 " %08" PRIx32 " %08" PRIx32 "\r\n",
        first_word[0], first_word[1], first_word[2], first_word[3], last_word[0], last_word[1], last_word[2], last_word[3]);
    uint32_t unwritten = 0;
    for (uint32_t i = 0; i < DUMP_BANK_WORDS; i++) if (DUMP_BANK[i] == 0xDEADBEEFu) unwritten++;
    say("  unwritten=%" PRIu32 "\r\nEND\r\n", unwritten);
}

/* STREAM: run the ring, follow the write index and box-car decimate by `decim` into an
 * int16 I/Q buffer; stop when `nout` samples are collected. Reports overruns and timing. */
static void stream_decimate(uint32_t ctrl, uint32_t ring_words, uint32_t decim, uint32_t nout)
{
    if (decim < 1 || decim > 256 || (ring_words & (ring_words - 1)) || ring_words > DUMP_BANK_WORDS) { say("ERR STREAM args\r\n"); return; }
    size_t avail = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT | MALLOC_CAP_32BIT);
    if (avail < 8192) { say("ERR no memory\r\n"); return; }
    size_t maxout = (avail - 4096) / 4;
    if (nout == 0 || nout > maxout) nout = maxout;
    int16_t *out = heap_caps_malloc(nout * 4, MALLOC_CAP_8BIT | MALLOC_CAP_32BIT);
    if (!out) { say("ERR malloc\r\n"); return; }
    const uint32_t mask = ring_words - 1;
    const uint32_t owner0 = rd(SRAM_OWNER);
    wr(DUMP_CTRL, rd(DUMP_CTRL) & ~(1u << 31));
    wr(DUMP_CTRL, ctrl & ~(1u << 31));
    wr(SRAM_OWNER, (owner0 & ~7u) | 2u);
    wr(SRAM_OWNER, rd(SRAM_OWNER) | own_bit3);
    __asm__ volatile("fence rw,rw" ::: "memory");
    uint32_t overruns = 0, waits = 0, produced = 0, rp = 0, maxlag = 0;
    portDISABLE_INTERRUPTS();
    uint32_t t0 = esp_cpu_get_cycle_count();
    wr(DUMP_CTRL, rd(DUMP_CTRL) | (1u << 31));
    wr(DUMP_CTRL, rd(DUMP_CTRL) | (1u << 19));
    wr(DUMP_CTRL, rd(DUMP_CTRL) & ~(1u << 19));
    /* Let the writer get ahead by one block before reading. */
    while (((rd(DUMP_WIDX) - rp) & mask) < decim + 8) {}
    uint32_t guard = 0;
    while (produced < nout) {
        uint32_t widx = rd(DUMP_WIDX) & mask;
        uint32_t lag = (widx - rp) & mask;
        if (lag < decim + 8) { waits++; if (++guard > 50000000u) break; continue; }
        guard = 0;
        if (lag > maxlag) maxlag = lag;
        if (lag > ring_words - 2 * decim) { overruns++; rp = (widx - 4 * decim) & mask; continue; }
        /* Sum offset-binary I and Q fields of `decim` raw words. */
        uint32_t a = 0, b = 0;
        const volatile uint32_t *p = DUMP_BANK + rp;
        for (uint32_t k = 0; k < decim; k++) {
            uint32_t w = p[k] ^ 0x00080200u;
            a += w & 0x3FFu;
            b += w & 0xFFC00u;
        }
        out[2 * produced] = (int16_t)((int32_t)a - 512 * (int32_t)decim);
        out[2 * produced + 1] = (int16_t)((int32_t)(b >> 10) - 512 * (int32_t)decim);
        produced++;
        rp = (rp + decim) & mask;
    }
    uint32_t t1 = esp_cpu_get_cycle_count();
    wr(DUMP_CTRL, rd(DUMP_CTRL) & ~(1u << 31));
    portENABLE_INTERRUPTS();
    __asm__ volatile("fence rw,rw" ::: "memory");
    wr(SRAM_OWNER, owner0);
    say("STREAM ctrl=%08" PRIx32 " ring=%" PRIu32 " decim=%" PRIu32 " nout=%" PRIu32 " produced=%" PRIu32 " cycles=%" PRIu32 " overruns=%" PRIu32 " waits=%" PRIu32 " maxlag=%" PRIu32 " tuned=%" PRIu32 "\r\n",
        ctrl, ring_words, decim, nout, produced, t1 - t0, overruns, waits, maxlag, tuned_khz);
    uint32_t hdr[2] = {0x43415032u, produced};
    usb_write(hdr, sizeof hdr);
    usb_write(out, produced * 4);
    say("\r\nEND\r\n");
    free(out);
}

#define DUMP_BANK_A ((volatile uint32_t *)0x3fca0000)   /* reserved region, candidate second bank */

/* BANK <sel>: one-shot capture with owner low bits = sel; report which half of the 128 KiB region was written. */
static void bank_probe(uint32_t sel, uint32_t ctrl)
{
    for (uint32_t i = 0; i < DUMP_BANK_WORDS; i++) { DUMP_BANK[i] = 0xDEADBEEFu; DUMP_BANK_A[i] = 0xDEADBEEFu; }
    const uint32_t owner0 = rd(SRAM_OWNER);
    wr(DUMP_CTRL, rd(DUMP_CTRL) & ~(1u << 31));
    wr(DUMP_CTRL, ctrl & ~(1u << 31));
    wr(SRAM_OWNER, (owner0 & ~7u) | (sel & 7u));
    wr(SRAM_OWNER, rd(SRAM_OWNER) | own_bit3);
    __asm__ volatile("fence rw,rw" ::: "memory");
    uint32_t own = rd(SRAM_OWNER);
    uint32_t t0 = esp_cpu_get_cycle_count();
    wr(DUMP_CTRL, rd(DUMP_CTRL) | (1u << 31));
    wr(DUMP_CTRL, rd(DUMP_CTRL) | (1u << 19));
    wr(DUMP_CTRL, rd(DUMP_CTRL) & ~(1u << 19));
    uint32_t st = 0; bool done = false;
    for (uint32_t i = 0; i < 2000000; i++) { st = rd(DUMP_CTRL); if (st & (1u << 18)) { done = true; break; } }
    uint32_t t1 = esp_cpu_get_cycle_count();
    wr(DUMP_CTRL, rd(DUMP_CTRL) & ~(1u << 31));
    __asm__ volatile("fence rw,rw" ::: "memory");
    wr(SRAM_OWNER, owner0);
    uint32_t ua = 0, ub = 0;
    for (uint32_t i = 0; i < DUMP_BANK_WORDS; i++) { if (DUMP_BANK_A[i] == 0xDEADBEEFu) ua++; if (DUMP_BANK[i] == 0xDEADBEEFu) ub++; }
    say("BANK sel=%" PRIu32 " owner_set=%08" PRIx32 " done=%d status=%08" PRIx32 " cycles=%" PRIu32 " widx=%08" PRIx32 " unwritten_A(3fca0000)=%" PRIu32 " unwritten_B(3fcb0000)=%" PRIu32 " A[0..3]=%08" PRIx32 " %08" PRIx32 " B[0..3]=%08" PRIx32 " %08" PRIx32 "\r\n",
        sel, own, done, st, t1 - t0, rd(DUMP_WIDX), ua, ub, DUMP_BANK_A[0], DUMP_BANK_A[1], DUMP_BANK[0], DUMP_BANK[1]);
}

/* DECBENCH <decim>: time the box-car decimation of the whole bank (already captured). */
static void dec_bench(uint32_t decim)
{
    static int16_t outbuf[2 * 2048];
    if (decim < 8 || decim > 256) decim = 16;
    uint32_t n = DUMP_BANK_WORDS / decim; if (n > 2048) n = 2048;
    portDISABLE_INTERRUPTS();
    uint32_t t0 = esp_cpu_get_cycle_count();
    const volatile uint32_t *p = DUMP_BANK;
    for (uint32_t o = 0; o < n; o++) {
        uint32_t a = 0, b = 0;
        for (uint32_t k = 0; k < decim; k += 4) {
            uint32_t w0 = p[k] ^ 0x00080200u, w1 = p[k + 1] ^ 0x00080200u, w2 = p[k + 2] ^ 0x00080200u, w3 = p[k + 3] ^ 0x00080200u;
            a += (w0 & 0x3FFu) + (w1 & 0x3FFu) + (w2 & 0x3FFu) + (w3 & 0x3FFu);
            b += (w0 & 0xFFC00u) + (w1 & 0xFFC00u) + (w2 & 0xFFC00u) + (w3 & 0xFFC00u);
        }
        p += decim;
        outbuf[2 * o] = (int16_t)((int32_t)a - 512 * (int32_t)decim);
        outbuf[2 * o + 1] = (int16_t)((int32_t)(b >> 10) - 512 * (int32_t)decim);
    }
    uint32_t t1 = esp_cpu_get_cycle_count();
    portENABLE_INTERRUPTS();
    say("DECBENCH decim=%" PRIu32 " words=%" PRIu32 " cycles=%" PRIu32 " -> %.2f cycles/word (budget 10.0 at 16 MS/s, 160 MHz) out0=%d,%d\r\n",
        decim, n * decim, t1 - t0, (double)(t1 - t0) / (n * decim), outbuf[0], outbuf[1]);
}

/* ---- continuous acquisition: one-shot captures restarted back to back ----------------
 * The C3 has one usable dump bank and no circular mode (RX-C3-010), so the reader follows the
 * writer's index inside a one-shot run and restarts the engine the moment it finishes. The
 * restart costs a few microseconds of samples per 16384-word block; the count is reported so
 * the gaps are known. Box-car decimation by `decim`, output int8 I/Q (sum >> shift) or int16. */
static inline void dump_start(uint32_t ctrl)
{
    wr(DUMP_CTRL, ctrl & ~(1u << 31));
    wr(DUMP_CTRL, ctrl | (1u << 31));
    wr(DUMP_CTRL, rd(DUMP_CTRL) | (1u << 19));
    wr(DUMP_CTRL, rd(DUMP_CTRL) & ~(1u << 19));
}

static void cstream(uint32_t decim, uint32_t nout, uint32_t shift, bool to_usb)
{
    const uint32_t ctrl = 0x13fffu;                 /* 16 Msps, 16384 words one-shot */
    if (decim < 8 || decim > 256 || (DUMP_BANK_WORDS % decim)) { say("ERR CSTREAM decim 8..256, divides 16384\r\n"); return; }
    int8_t *out = NULL;
    if (!to_usb) {
        size_t avail = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
        size_t maxout = avail > 8192 ? (avail - 4096) / 2 : 0;
        if (nout == 0 || nout > maxout) nout = maxout;
        out = heap_caps_malloc(nout * 2, MALLOC_CAP_8BIT);
        if (!out) { say("ERR malloc\r\n"); return; }
    }
    static uint8_t usbbuf[64]; uint32_t usbn = 0, usb_dropped = 0;
    const uint32_t owner0 = rd(SRAM_OWNER);
    wr(DUMP_CTRL, rd(DUMP_CTRL) & ~(1u << 31));
    wr(SRAM_OWNER, (owner0 & ~7u) | 2u);
    wr(SRAM_OWNER, rd(SRAM_OWNER) | own_bit3);
    __asm__ volatile("fence rw,rw" ::: "memory");
    uint32_t produced = 0, blocks = 0, late = 0, maxlag = 0, restart_cycles_sum = 0, restart_cycles_max = 0, waits = 0;
    bool stop = false;
    if (to_usb) { say("CSTREAM start decim=%" PRIu32 " shift=%" PRIu32 " fmt=int8 usb\r\n", decim, shift); }
    portDISABLE_INTERRUPTS();
    uint32_t t0 = esp_cpu_get_cycle_count();
    dump_start(ctrl);
    while (!stop) {
        uint32_t rp = 0;
        while (rp < DUMP_BANK_WORDS) {
            uint32_t widx = rd(DUMP_WIDX) & 0x3FFFu, st = rd(DUMP_CTRL);
            bool done = (st >> 18) & 1u;
            uint32_t avail_w = done ? DUMP_BANK_WORDS - rp : (widx - rp);
            if (avail_w < decim + 4 && !done) { waits++; continue; }
            if (!done && widx - rp > maxlag) maxlag = widx - rp;
            uint32_t a = 0, b = 0;
            const volatile uint32_t *p = DUMP_BANK + rp;
            for (uint32_t k = 0; k < decim; k += 4) {
                uint32_t w0 = p[k] ^ 0x00080200u, w1 = p[k + 1] ^ 0x00080200u, w2 = p[k + 2] ^ 0x00080200u, w3 = p[k + 3] ^ 0x00080200u;
                a += (w0 & 0x3FFu) + (w1 & 0x3FFu) + (w2 & 0x3FFu) + (w3 & 0x3FFu);
                b += (w0 & 0xFFC00u) + (w1 & 0xFFC00u) + (w2 & 0xFFC00u) + (w3 & 0xFFC00u);
            }
            rp += decim;
            int32_t iv = ((int32_t)a - 512 * (int32_t)decim) >> shift, qv = ((int32_t)(b >> 10) - 512 * (int32_t)decim) >> shift;
            if (iv > 127) iv = 127;
            if (iv < -128) iv = -128;
            if (qv > 127) qv = 127;
            if (qv < -128) qv = -128;
            if (to_usb) {
                usbbuf[usbn++] = (uint8_t)iv; usbbuf[usbn++] = (uint8_t)qv;
                if (usbn == 64) {
                    if (usb_serial_jtag_ll_txfifo_writable()) { usb_serial_jtag_ll_write_txfifo(usbbuf, 64); usb_serial_jtag_ll_txfifo_flush(); }
                    else usb_dropped++;
                    usbn = 0;
                    uint8_t c; if (usb_serial_jtag_ll_read_rxfifo(&c, 1)) stop = true;   /* any byte from the host stops */
                }
            } else {
                out[2 * produced] = (int8_t)iv; out[2 * produced + 1] = (int8_t)qv;
            }
            produced++;
            if (!to_usb && produced >= nout) { stop = true; break; }
        }
        blocks++;
        if (stop) break;
        /* Block finished: the writer must have finished too (we read up to the end only when done). */
        uint32_t r0 = esp_cpu_get_cycle_count();
        if (!((rd(DUMP_CTRL) >> 18) & 1u)) late++;       /* reader finished before the writer: should not happen */
        dump_start(ctrl);
        uint32_t rc = esp_cpu_get_cycle_count() - r0;
        restart_cycles_sum += rc; if (rc > restart_cycles_max) restart_cycles_max = rc;
        if (blocks > 2000000u) break;
    }
    uint32_t t1 = esp_cpu_get_cycle_count();
    wr(DUMP_CTRL, rd(DUMP_CTRL) & ~(1u << 31));
    portENABLE_INTERRUPTS();
    __asm__ volatile("fence rw,rw" ::: "memory");
    wr(SRAM_OWNER, owner0);
    say("CSTREAM decim=%" PRIu32 " shift=%" PRIu32 " produced=%" PRIu32 " blocks=%" PRIu32 " cycles=%" PRIu32 " late=%" PRIu32 " waits=%" PRIu32 " maxlag=%" PRIu32 " restart_avg=%" PRIu32 " restart_max=%" PRIu32 " usb_dropped=%" PRIu32 " tuned=%" PRIu32 "\r\n",
        decim, shift, produced, blocks, t1 - t0, late, waits, maxlag, blocks > 1 ? restart_cycles_sum / (blocks - 1) : 0, restart_cycles_max, usb_dropped, tuned_khz);
    if (!to_usb) {
        uint32_t hdr[2] = {0x43415033u, produced};
        usb_write(hdr, sizeof hdr);
        usb_write(out, produced * 2);
        say("\r\nEND\r\n");
        free(out);
    } else {
        say("END\r\n");
    }
}

/* RING2 <owner_low4>: start a one-shot into the bank selected by the owner bits (2 -> A at
 * 0x3FCA0000, 0xA -> B at 0x3FCB0000) and, while it runs, read words from BOTH banks twice to
 * see which bank the CPU can read live (varying values) and which returns a constant. */
static void ring2(uint32_t ownbits)
{
    for (uint32_t i = 0; i < DUMP_BANK_WORDS; i++) { DUMP_BANK[i] = 0xDEADBEEFu; DUMP_BANK_A[i] = 0xDEADBEEFu; }
    const uint32_t owner0 = rd(SRAM_OWNER);
    wr(DUMP_CTRL, rd(DUMP_CTRL) & ~(1u << 31));
    wr(DUMP_CTRL, 0x13fffu);
    wr(SRAM_OWNER, (owner0 & ~0xFu) | (ownbits & 0xFu));
    __asm__ volatile("fence rw,rw" ::: "memory");
    uint32_t a1[4], b1[4], a2[4], b2[4], w1, w2;
    wr(DUMP_CTRL, rd(DUMP_CTRL) | (1u << 31));
    wr(DUMP_CTRL, rd(DUMP_CTRL) | (1u << 19));
    wr(DUMP_CTRL, rd(DUMP_CTRL) & ~(1u << 19));
    esp_rom_delay_us(300);
    w1 = rd(DUMP_WIDX);
    for (int j = 0; j < 4; j++) { a1[j] = DUMP_BANK_A[j * 1000]; b1[j] = DUMP_BANK[j * 1000]; }
    esp_rom_delay_us(300);
    w2 = rd(DUMP_WIDX);
    for (int j = 0; j < 4; j++) { a2[j] = DUMP_BANK_A[j * 1000]; b2[j] = DUMP_BANK[j * 1000]; }
    esp_rom_delay_us(1500);
    uint32_t st = rd(DUMP_CTRL);
    wr(DUMP_CTRL, rd(DUMP_CTRL) & ~(1u << 31));
    __asm__ volatile("fence rw,rw" ::: "memory");
    wr(SRAM_OWNER, owner0);
    uint32_t ua = 0, ub = 0;
    for (uint32_t i = 0; i < DUMP_BANK_WORDS; i++) { if (DUMP_BANK_A[i] == 0xDEADBEEFu) ua++; if (DUMP_BANK[i] == 0xDEADBEEFu) ub++; }
    say("RING2 own=%" PRIx32 " status=%08" PRIx32 " widx@300us=%08" PRIx32 " @600us=%08" PRIx32 " unwrittenA=%" PRIu32 " unwrittenB=%" PRIu32 "\r\n", ownbits, st, w1, w2, ua, ub);
    say("  live A@300: %08" PRIx32 " %08" PRIx32 " %08" PRIx32 " %08" PRIx32 "  A@600: %08" PRIx32 " %08" PRIx32 " %08" PRIx32 " %08" PRIx32 "\r\n", a1[0], a1[1], a1[2], a1[3], a2[0], a2[1], a2[2], a2[3]);
    say("  live B@300: %08" PRIx32 " %08" PRIx32 " %08" PRIx32 " %08" PRIx32 "  B@600: %08" PRIx32 " %08" PRIx32 " %08" PRIx32 " %08" PRIx32 "\r\n", b1[0], b1[1], b1[2], b1[3], b2[0], b2[1], b2[2], b2[3]);
    say("  final A[0,1000,2000,3000]: %08" PRIx32 " %08" PRIx32 " %08" PRIx32 " %08" PRIx32 "  B: %08" PRIx32 " %08" PRIx32 " %08" PRIx32 " %08" PRIx32 "\r\nEND\r\n",
        DUMP_BANK_A[0], DUMP_BANK_A[1000], DUMP_BANK_A[2000], DUMP_BANK_A[3000], DUMP_BANK[0], DUMP_BANK[1000], DUMP_BANK[2000], DUMP_BANK[3000]);
}

/* RING3: capture into A (owner 2), finish; then switch to B (owner 0xA), start, and while B is
 * being written read A: does the CPU see A's fresh data while the engine owns the pair? */
static void ring3(void)
{
    for (uint32_t i = 0; i < DUMP_BANK_WORDS; i++) { DUMP_BANK[i] = 0xDEADBEEFu; DUMP_BANK_A[i] = 0xDEADBEEFu; }
    const uint32_t owner0 = rd(SRAM_OWNER);
    wr(DUMP_CTRL, rd(DUMP_CTRL) & ~(1u << 31)); wr(DUMP_CTRL, 0x13fffu);
    wr(SRAM_OWNER, (owner0 & ~0xFu) | 2u);
    __asm__ volatile("fence rw,rw" ::: "memory");
    wr(DUMP_CTRL, rd(DUMP_CTRL) | (1u << 31)); wr(DUMP_CTRL, rd(DUMP_CTRL) | (1u << 19)); wr(DUMP_CTRL, rd(DUMP_CTRL) & ~(1u << 19));
    uint32_t st = 0; for (uint32_t i = 0; i < 2000000; i++) { st = rd(DUMP_CTRL); if (st & (1u << 18)) break; }
    wr(DUMP_CTRL, rd(DUMP_CTRL) & ~(1u << 31));
    uint32_t a_owned[4]; for (int j = 0; j < 4; j++) a_owned[j] = DUMP_BANK_A[j * 1000];      /* A read while still owned (idle engine) */
    /* now B */
    wr(DUMP_CTRL, 0x13fffu);
    wr(SRAM_OWNER, (rd(SRAM_OWNER) & ~0xFu) | 0xAu);
    __asm__ volatile("fence rw,rw" ::: "memory");
    uint32_t t0 = esp_cpu_get_cycle_count();
    wr(DUMP_CTRL, rd(DUMP_CTRL) | (1u << 31)); wr(DUMP_CTRL, rd(DUMP_CTRL) | (1u << 19)); wr(DUMP_CTRL, rd(DUMP_CTRL) & ~(1u << 19));
    esp_rom_delay_us(300);
    uint32_t a_live[4], b_live[4]; for (int j = 0; j < 4; j++) { a_live[j] = DUMP_BANK_A[j * 1000]; b_live[j] = DUMP_BANK[j * 1000]; }
    uint32_t w = rd(DUMP_WIDX);
    /* time a full read of A while B is being written */
    uint32_t sum = 0; uint32_t tr0 = esp_cpu_get_cycle_count();
    for (uint32_t i = 0; i < DUMP_BANK_WORDS; i++) sum += DUMP_BANK_A[i];
    uint32_t tr1 = esp_cpu_get_cycle_count();
    for (uint32_t i = 0; i < 2000000; i++) { st = rd(DUMP_CTRL); if (st & (1u << 18)) break; }
    wr(DUMP_CTRL, rd(DUMP_CTRL) & ~(1u << 31));
    __asm__ volatile("fence rw,rw" ::: "memory");
    wr(SRAM_OWNER, owner0);
    uint32_t ua = 0, ub = 0; for (uint32_t i = 0; i < DUMP_BANK_WORDS; i++) { if (DUMP_BANK_A[i] == 0xDEADBEEFu) ua++; if (DUMP_BANK[i] == 0xDEADBEEFu) ub++; }
    say("RING3 A_owned_idle: %08" PRIx32 " %08" PRIx32 " %08" PRIx32 " %08" PRIx32 "\r\n", a_owned[0], a_owned[1], a_owned[2], a_owned[3]);
    say("  during B write (widx=%08" PRIx32 "): A: %08" PRIx32 " %08" PRIx32 " %08" PRIx32 " %08" PRIx32 "  B: %08" PRIx32 " %08" PRIx32 " %08" PRIx32 " %08" PRIx32 "\r\n", w, a_live[0], a_live[1], a_live[2], a_live[3], b_live[0], b_live[1], b_live[2], b_live[3]);
    say("  A full read during B write: %" PRIu32 " cycles (%.2f/word) sum=%08" PRIx32 "\r\n", tr1 - tr0, (double)(tr1 - tr0) / DUMP_BANK_WORDS, sum);
    say("  after restore: A: %08" PRIx32 " %08" PRIx32 " %08" PRIx32 " %08" PRIx32 "  B: %08" PRIx32 " %08" PRIx32 " %08" PRIx32 " %08" PRIx32 "  unwrittenA=%" PRIu32 " unwrittenB=%" PRIu32 " cyclesB=%" PRIu32 "\r\nEND\r\n",
        DUMP_BANK_A[0], DUMP_BANK_A[1000], DUMP_BANK_A[2000], DUMP_BANK_A[3000], DUMP_BANK[0], DUMP_BANK[1000], DUMP_BANK[2000], DUMP_BANK[3000], ua, ub, esp_cpu_get_cycle_count() - t0);
}

/* CSTREAM2 <nwords_hex> <decim> <nout> <shift> <usb>: duty-cycled continuous capture.
 * capture nwords (one-shot, 16 Msps) -> stop -> fast copy to a side buffer -> restart at once
 * -> decimate the copy while the next block is being captured. Gaps = stop+copy+restart time,
 * measured in cycles and reported, so the host knows exactly how many samples are missing. */
static void cstream2(uint32_t nwords, uint32_t decim, uint32_t nout, uint32_t shift, bool to_usb)
{
    if (nwords < 1024 || nwords > 32768 || decim < 8 || decim > 256 || (nwords % decim)) { say("ERR CSTREAM2 args\r\n"); return; }
    const uint32_t ctrl = 0x10000u | (nwords - 1);          /* 16 Msps one-shot of nwords */
    uint32_t *copy = heap_caps_malloc(nwords * 4, MALLOC_CAP_8BIT | MALLOC_CAP_32BIT);
    if (!copy) { say("ERR no memory for copy (%u words)\r\n", (unsigned)nwords); return; }
    int8_t *out = NULL;
    if (!to_usb) {
        size_t avail = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
        size_t maxout = avail > 8192 ? (avail - 4096) / 2 : 0;
        if (nout == 0 || nout > maxout) nout = maxout;
        out = heap_caps_malloc(nout * 2, MALLOC_CAP_8BIT);
        if (!out) { say("ERR malloc out\r\n"); free(copy); return; }
    }
    static uint8_t usbbuf[64]; uint32_t usbn = 0, usb_dropped = 0;
    const uint32_t owner0 = rd(SRAM_OWNER);
    const uint32_t own_run = (owner0 & ~0xFu) | 2u;         /* bank A (0x3FCA0000); 32768 words spill into B if supported */
    const volatile uint32_t *bank = DUMP_BANK_A;
    uint32_t produced = 0, blocks = 0, gap_sum = 0, gap_max = 0, gap_min = 0xFFFFFFFFu, late = 0, cap_cycles = 0;
    bool stop = false;
    if (to_usb) say("CSTREAM2 start nwords=%" PRIu32 " decim=%" PRIu32 " shift=%" PRIu32 " usb\r\n", nwords, decim, shift);
    portDISABLE_INTERRUPTS();
    uint32_t t_all0 = esp_cpu_get_cycle_count();
    wr(DUMP_CTRL, rd(DUMP_CTRL) & ~(1u << 31));
    wr(SRAM_OWNER, own_run);
    __asm__ volatile("fence rw,rw" ::: "memory");
    dump_start(ctrl);
    uint32_t tcap0 = esp_cpu_get_cycle_count();
    while (!stop) {
        /* wait for the one-shot to finish */
        uint32_t st = 0, guard = 0;
        while (!((st = rd(DUMP_CTRL)) & (1u << 18))) { if (++guard > 20000000u) break; }
        uint32_t tdone = esp_cpu_get_cycle_count();
        if (blocks == 0) cap_cycles = tdone - tcap0;
        /* stop, give the pair back, copy, restart */
        wr(DUMP_CTRL, rd(DUMP_CTRL) & ~(1u << 31));
        __asm__ volatile("fence rw,rw" ::: "memory");
        wr(SRAM_OWNER, owner0);
        __asm__ volatile("fence rw,rw" ::: "memory");
        {
            const volatile uint32_t *src = bank; uint32_t *dst = copy;
            for (uint32_t i = 0; i < nwords; i += 8) {
                uint32_t a0 = src[i], a1 = src[i + 1], a2 = src[i + 2], a3 = src[i + 3], a4 = src[i + 4], a5 = src[i + 5], a6 = src[i + 6], a7 = src[i + 7];
                dst[i] = a0; dst[i + 1] = a1; dst[i + 2] = a2; dst[i + 3] = a3; dst[i + 4] = a4; dst[i + 5] = a5; dst[i + 6] = a6; dst[i + 7] = a7;
            }
        }
        wr(SRAM_OWNER, own_run);
        __asm__ volatile("fence rw,rw" ::: "memory");
        dump_start(ctrl);
        uint32_t trestart = esp_cpu_get_cycle_count();
        uint32_t gap = trestart - tdone; gap_sum += gap; if (gap > gap_max) gap_max = gap; if (gap < gap_min) gap_min = gap;
        blocks++;
        /* decimate the copy while the next block fills */
        const uint32_t *p = copy;
        for (uint32_t o = 0; o < nwords / decim && !stop; o++, p += decim) {
            uint32_t a = 0, b = 0;
            for (uint32_t k = 0; k < decim; k += 4) {
                uint32_t w0 = p[k] ^ 0x00080200u, w1 = p[k + 1] ^ 0x00080200u, w2 = p[k + 2] ^ 0x00080200u, w3 = p[k + 3] ^ 0x00080200u;
                a += (w0 & 0x3FFu) + (w1 & 0x3FFu) + (w2 & 0x3FFu) + (w3 & 0x3FFu);
                b += (w0 & 0xFFC00u) + (w1 & 0xFFC00u) + (w2 & 0xFFC00u) + (w3 & 0xFFC00u);
            }
            int32_t iv = ((int32_t)a - 512 * (int32_t)decim) >> shift, qv = ((int32_t)(b >> 10) - 512 * (int32_t)decim) >> shift;
            if (iv > 127) iv = 127;
            if (iv < -128) iv = -128;
            if (qv > 127) qv = 127;
            if (qv < -128) qv = -128;
            if (to_usb) {
                usbbuf[usbn++] = (uint8_t)iv; usbbuf[usbn++] = (uint8_t)qv;
                if (usbn == 64) {
                    if (usb_serial_jtag_ll_txfifo_writable()) { usb_serial_jtag_ll_write_txfifo(usbbuf, 64); usb_serial_jtag_ll_txfifo_flush(); } else usb_dropped++;
                    usbn = 0;
                    uint8_t c; if (usb_serial_jtag_ll_read_rxfifo(&c, 1)) stop = true;
                }
            } else {
                out[2 * produced] = (int8_t)iv; out[2 * produced + 1] = (int8_t)qv;
                if (produced + 1 >= nout) stop = true;
            }
            produced++;
        }
        if ((rd(DUMP_CTRL) >> 18) & 1u) late++;     /* decimation took longer than the capture: a block boundary was missed */
        if (blocks > 4000000u) break;
    }
    uint32_t t_all1 = esp_cpu_get_cycle_count();
    wr(DUMP_CTRL, rd(DUMP_CTRL) & ~(1u << 31));
    portENABLE_INTERRUPTS();
    __asm__ volatile("fence rw,rw" ::: "memory");
    wr(SRAM_OWNER, owner0);
    say("CSTREAM2 nwords=%" PRIu32 " decim=%" PRIu32 " shift=%" PRIu32 " produced=%" PRIu32 " blocks=%" PRIu32 " total_cycles=%" PRIu32 " capture_cycles=%" PRIu32 " gap_avg=%" PRIu32 " gap_min=%" PRIu32 " gap_max=%" PRIu32 " late=%" PRIu32 " usb_dropped=%" PRIu32 " tuned=%" PRIu32 "\r\n",
        nwords, decim, shift, produced, blocks, t_all1 - t_all0, cap_cycles, blocks ? gap_sum / blocks : 0, gap_min, gap_max, late, usb_dropped, tuned_khz);
    if (!to_usb) {
        uint32_t hdr[2] = {0x43415033u, produced};
        usb_write(hdr, sizeof hdr); usb_write(out, produced * 2); say("\r\nEND\r\n"); free(out);
    } else say("END\r\n");
    free(copy);
}

/* ---- burst capture and Wi-Fi MAC reference ------------------------------------ */

/* BURST: repeat one-shot captures of nwords (16 MS/s unless ctrl says otherwise), compute the
 * block's mean |I|+|Q| and peak |I|,|Q| from the idle bank, keep a time series of every block,
 * and send the raw words of blocks whose mean exceeds thr (up to maxsend). The series lets the
 * host look for the 102.4 ms beacon period; the raw blocks let it demodulate the DSSS PLCP. */
typedef struct { uint32_t t_us; uint16_t mean; uint16_t peak; uint8_t sent; uint8_t pad[3]; uint32_t c_wait, c_pow; } burst_rec_t;

static void burst(uint32_t ctrl, uint32_t nwords, uint32_t thr, uint32_t maxsend, uint32_t total_ms)
{
    if (nwords == 0 || nwords > DUMP_BANK_WORDS) nwords = DUMP_BANK_WORDS;
    uint32_t maxrec = 3500;
    burst_rec_t *rec = heap_caps_malloc(maxrec * sizeof *rec, MALLOC_CAP_8BIT);
    if (!rec) { say("ERR BURST malloc\r\n"); return; }
    const uint32_t owner0 = rd(SRAM_OWNER);
    const uint32_t own_run = (owner0 & ~7u) | 2u | own_bit3;
    uint32_t nrec = 0, nsent = 0, nfail = 0;
    int64_t t_end = esp_timer_get_time() + (int64_t)total_ms * 1000;
    say("BURST start ctrl=%08" PRIx32 " nwords=%" PRIu32 " thr=%" PRIu32 " maxsend=%" PRIu32 " ms=%" PRIu32 " tuned=%" PRIu32 "\r\n",
        ctrl, nwords, thr, maxsend, total_ms, tuned_khz);
    while (esp_timer_get_time() < t_end && nrec < maxrec) {
        wr(DUMP_CTRL, rd(DUMP_CTRL) & ~(1u << 31));
        wr(SRAM_OWNER, own_run);
        __asm__ volatile("fence rw,rw" ::: "memory");
        uint32_t t_us = (uint32_t)esp_timer_get_time();
        uint32_t c0 = esp_cpu_get_cycle_count();
        dump_start((ctrl & ~0x3FFFu) | (nwords - 1));
        bool done = false;
        for (uint32_t i = 0; i < 2000000; i++) { if (rd(DUMP_CTRL) & (1u << 18)) { done = true; break; } }
        uint32_t c1 = esp_cpu_get_cycle_count();
        wr(DUMP_CTRL, rd(DUMP_CTRL) & ~(1u << 31));
        __asm__ volatile("fence rw,rw" ::: "memory");
        wr(SRAM_OWNER, owner0);
        __asm__ volatile("fence rw,rw" ::: "memory");
        if (!done) { nfail++; continue; }
        uint32_t acc = 0, peak = 0;
        portDISABLE_INTERRUPTS();
        const uint32_t *bp = (const uint32_t *)DUMP_BANK;
        /* every 4th word is enough for the detector (the compute, ~18 instructions per word, is the bottleneck) */
        for (uint32_t i = 0; i < nwords; i += 16) {
            uint32_t w0 = bp[i] ^ 0x00080200u, w1 = bp[i + 4] ^ 0x00080200u, w2 = bp[i + 8] ^ 0x00080200u, w3 = bp[i + 12] ^ 0x00080200u;
#define ABSACC(w) do { int32_t iv_ = (int32_t)((w) & 0x3FFu) - 512, qv_ = (int32_t)(((w) >> 10) & 0x3FFu) - 512; \
            uint32_t ai_ = (uint32_t)(iv_ ^ (iv_ >> 31)) - (uint32_t)(iv_ >> 31), aq_ = (uint32_t)(qv_ ^ (qv_ >> 31)) - (uint32_t)(qv_ >> 31); \
            acc += ai_ + aq_; peak |= ai_ | aq_; } while (0)
            ABSACC(w0); ABSACC(w1); ABSACC(w2); ABSACC(w3);
#undef ABSACC
        }
        portENABLE_INTERRUPTS();
        /* peak here is the OR of magnitudes: >= 512 means at least one sample hit full scale (bit 9), 256..511 means >= half scale */
        uint32_t mean = acc / (nwords / 4);
        burst_rec_t *r = &rec[nrec++];
        r->t_us = t_us; r->mean = (uint16_t)mean; r->peak = (uint16_t)peak; r->sent = 0;
        r->c_wait = c1 - c0; r->c_pow = esp_cpu_get_cycle_count() - c1;
        if (mean > thr && nsent < maxsend) {
            r->sent = 1; nsent++;
            say("CAP burst=1 idx=%" PRIu32 " t_us=%" PRIu32 " mean=%" PRIu32 " peak=%" PRIu32 " ctrl=%08" PRIx32 " n=%" PRIu32 " tuned=%" PRIu32 "\r\n",
                nrec - 1, t_us, mean, peak, ctrl, nwords, tuned_khz);
            uint32_t hdr[2] = {0x43415031u, nwords};
            usb_write(hdr, sizeof hdr);
            usb_write((const void *)DUMP_BANK, nwords * 4);
            say("\r\nEND\r\n");
        }
    }
    say("BURST done nrec=%" PRIu32 " nsent=%" PRIu32 " nfail=%" PRIu32 "\r\n", nrec, nsent, nfail);
    for (uint32_t i = 0; i < nrec; i++)
        say("S %" PRIu32 " %" PRIu32 " %u %u %u %" PRIu32 " %" PRIu32 "\r\n", i, rec[i].t_us, rec[i].mean, rec[i].peak, rec[i].sent, rec[i].c_wait, rec[i].c_pow);
    say("END BURST\r\n");
    free(rec);
}

/* WIFICNT: let the chip's own Wi-Fi MAC (promiscuous mode) count frames for a while and report
 * the beacons it decoded: BSSID, SSID, RSSI, count. Independent demodulation path used as the
 * reference for the host-side DSSS decode of the raw I/Q. */
typedef struct { uint8_t bssid[6]; char ssid[33]; int rssi; uint32_t count; uint32_t last_us; uint32_t prev_us; } ap_rec_t;
static ap_rec_t aps[16];
static volatile uint32_t cnt_mgmt, cnt_ctrl, cnt_data, cnt_misc, cnt_beacon, cnt_probe_req;
static volatile uint32_t last_probe_us; static uint8_t last_probe_sa[6];

static void promisc_cb(void *buf, wifi_promiscuous_pkt_type_t type)
{
    if (type == WIFI_PKT_CTRL) { cnt_ctrl++; return; }
    if (type == WIFI_PKT_DATA) { cnt_data++; return; }
    if (type == WIFI_PKT_MISC) { cnt_misc++; return; }
    cnt_mgmt++;
    const wifi_promiscuous_pkt_t *p = buf;
    const uint8_t *f = p->payload;
    if (p->rx_ctrl.sig_len < 36) return;
    uint8_t subtype = (f[0] >> 4) & 0xF;
    uint32_t now = (uint32_t)esp_timer_get_time();
    if (subtype == 4) { cnt_probe_req++; last_probe_us = now; memcpy(last_probe_sa, f + 10, 6); return; }
    if (subtype != 8) return;
    cnt_beacon++;
    const uint8_t *bssid = f + 16;
    int slot = -1;
    for (int i = 0; i < 16; i++) {
        if (aps[i].count && !memcmp(aps[i].bssid, bssid, 6)) { slot = i; break; }
        if (!aps[i].count && slot < 0) slot = i;
    }
    if (slot < 0) return;
    ap_rec_t *a = &aps[slot];
    if (!a->count) {
        memcpy(a->bssid, bssid, 6);
        /* IEs start at 24 (header) + 12 (timestamp, interval, capability) */
        const uint8_t *ie = f + 36; int rem = p->rx_ctrl.sig_len - 36 - 4;
        a->ssid[0] = 0;
        if (rem >= 2 && ie[0] == 0) { int l = ie[1] > 32 ? 32 : ie[1]; if (l <= rem - 2) { memcpy(a->ssid, ie + 2, l); a->ssid[l] = 0; } }
    }
    a->count++; a->rssi = p->rx_ctrl.rssi; a->prev_us = a->last_us; a->last_us = now;
}

static void wificnt(uint32_t ms, bool normal_rx)
{
    memset(aps, 0, sizeof aps);
    cnt_mgmt = cnt_ctrl = cnt_data = cnt_misc = cnt_beacon = cnt_probe_req = 0;
    if (normal_rx) {
        /* hand the RF back to the driver's AGC/channel setup for the reference measurement */
        uint8_t ch = (uint8_t)((tuned_khz - 2407000u + 2500u) / 5000u); if (ch < 1 || ch > 13) ch = 6;
        esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
    }
    esp_wifi_set_promiscuous_rx_cb(promisc_cb);
    vTaskDelay(pdMS_TO_TICKS(ms));
    esp_wifi_set_promiscuous_rx_cb(NULL);
    say("WIFICNT ms=%" PRIu32 " normal=%d mgmt=%" PRIu32 " ctrl=%" PRIu32 " data=%" PRIu32 " misc=%" PRIu32 " beacon=%" PRIu32 " probe_req=%" PRIu32 " tuned=%" PRIu32 " agc=%08" PRIx32 "\r\n",
        ms, normal_rx, cnt_mgmt, cnt_ctrl, cnt_data, cnt_misc, cnt_beacon, cnt_probe_req, tuned_khz, rd(AGC_FORCE));
    for (int i = 0; i < 16; i++) if (aps[i].count)
        say("AP %02x:%02x:%02x:%02x:%02x:%02x rssi=%d count=%" PRIu32 " last_us=%" PRIu32 " dt_us=%" PRIu32 " ssid=\"%s\"\r\n",
            aps[i].bssid[0], aps[i].bssid[1], aps[i].bssid[2], aps[i].bssid[3], aps[i].bssid[4], aps[i].bssid[5],
            aps[i].rssi, aps[i].count, aps[i].last_us, aps[i].last_us - aps[i].prev_us, aps[i].ssid);
    if (cnt_probe_req) say("PROBE last_us=%" PRIu32 " sa=%02x:%02x:%02x:%02x:%02x:%02x\r\n", last_probe_us,
        last_probe_sa[0], last_probe_sa[1], last_probe_sa[2], last_probe_sa[3], last_probe_sa[4], last_probe_sa[5]);
    say("END WIFICNT\r\n");
}

/* ---- SDR frame stream ------------------------------------------------------------ */
/* STREAM2 ctrl_hex nwords decim shift ms
 * General-purpose capture for the host SDR tools. Duty-cycled like CSTREAM2 (capture -> stop ->
 * copy -> restart -> process), but every block is emitted as a self-describing frame so the host
 * can rebuild the exact timeline and detect dropped frames:
 *   magic "ESPB" | seq u32 | t_us u32 (esp_timer at capture start) | nsamp u16 | fmt u8 | flags u8
 *   fmt 0: int8 I,Q pairs (box-car decimated by decim, >> shift)   fmt 1: int16 I,Q pairs (decim 1, raw 10-bit)
 *   flags bit0: the previous frame was truncated (host resyncs on the magic)
 * USB writes are non-blocking with a short deadline; when the host cannot keep up the rest of the
 * frame is dropped and the next header says so. Any byte from the host stops the stream. */
static bool stream2_write(const uint8_t *p, uint32_t n, uint32_t deadline_us)
{
    while (n) {
        int64_t t0 = esp_timer_get_time();
        while (!usb_serial_jtag_ll_txfifo_writable()) { if (esp_timer_get_time() - t0 > (int64_t)deadline_us) return false; }
        int w = usb_serial_jtag_ll_write_txfifo(p, n > 64 ? 64 : n);
        usb_serial_jtag_ll_txfifo_flush();
        p += w; n -= w;
    }
    return true;
}

/* web format (STREAM2 ... web=1): Chrome's WebSerial enables termios PARMRK and treats FF 00 00 in the data as a break
 * (dropping it) and FF FF as one FF. So frames for the browser never contain 0xFF: magic "ESPW", header fields in 7-bit
 * groups (seq 5 B, t_us 5 B, nsamp 3 B, fmt 1 B, flags 1 B = 19 B), samples as unsigned offset-binary 0..254
 * (int8 + 128, or raw 10-bit >> 2 + 128 for the full-rate path), saturated at 254. */
static void put7(uint8_t *p, uint32_t v, int n) { for (int i = 0; i < n; i++) { p[i] = (uint8_t)(v & 0x7F); v >>= 7; } }

static void stream2(uint32_t ctrl_in, uint32_t nwords, uint32_t decim, uint32_t shift, uint32_t total_ms, uint32_t thr, bool web)
{
    if (nwords < 1024 || nwords > DUMP_BANK_WORDS || decim == 0 || decim > 1024 || (decim > 1 && (decim % 4 || nwords % decim))) { say("ERR STREAM2 args\r\n"); return; }
    const uint32_t ctrl = (ctrl_in & ~0x3FFFu) | (nwords - 1);
    const uint32_t nsamp = nwords / decim;
    const uint32_t fmt = decim == 1 ? 1u : 0u;
    /* decimated mode: copy the bank, restart the capture at once, then decimate and send while the next block fills
     * (CSTREAM2 pattern, ~67% coverage). Full-rate mode: convert from the idle bank and send (USB bound). */
    uint32_t *copy = fmt ? NULL : heap_caps_malloc(nwords * 4, MALLOC_CAP_8BIT | MALLOC_CAP_32BIT);
    if (!fmt && !copy) { say("ERR STREAM2 malloc copy\r\n"); return; }
    const uint32_t HDR = web ? 19u : 16u;
    uint8_t *frame = heap_caps_malloc(fmt ? HDR + nwords * 4 : HDR + 64, MALLOC_CAP_8BIT);
    if (!frame) { say("ERR STREAM2 malloc frame\r\n"); free(copy); return; }
    const uint32_t owner0 = rd(SRAM_OWNER);
    const uint32_t own_run = fmt ? ((owner0 & ~7u) | 2u | own_bit3) : ((owner0 & ~0xFu) | 2u);
    const volatile uint32_t *bank = fmt ? DUMP_BANK : DUMP_BANK_A;
    uint32_t seq = 0, sent = 0, truncated = 0, prev_trunc = 0, skipped = 0;
    int64_t t_end = esp_timer_get_time() + (int64_t)total_ms * 1000;
    say("STREAM2 start ctrl=%08" PRIx32 " nwords=%" PRIu32 " decim=%" PRIu32 " shift=%" PRIu32 " nsamp=%" PRIu32 " fmt=%" PRIu32 " ms=%" PRIu32 " thr=%" PRIu32 " tuned=%" PRIu32 " agc=%08" PRIx32 "\r\n",
        ctrl, nwords, decim, shift, nsamp, fmt, total_ms, thr, tuned_khz, rd(AGC_FORCE));
    bool stop = false;
    /* first capture */
    wr(DUMP_CTRL, rd(DUMP_CTRL) & ~(1u << 31));
    wr(SRAM_OWNER, own_run);
    __asm__ volatile("fence rw,rw" ::: "memory");
    uint32_t t_us = (uint32_t)esp_timer_get_time();
    dump_start(ctrl);
    while (!stop && esp_timer_get_time() < t_end) {
        bool done = false;
        for (uint32_t i = 0; i < 4000000; i++) { if (rd(DUMP_CTRL) & (1u << 18)) { done = true; break; } }
        wr(DUMP_CTRL, rd(DUMP_CTRL) & ~(1u << 31));
        __asm__ volatile("fence rw,rw" ::: "memory");
        wr(SRAM_OWNER, owner0);
        __asm__ volatile("fence rw,rw" ::: "memory");
        uint32_t t_blk = t_us;
        if (!done) {            /* restart and skip */
            wr(SRAM_OWNER, own_run); __asm__ volatile("fence rw,rw" ::: "memory"); t_us = (uint32_t)esp_timer_get_time(); dump_start(ctrl); continue;
        }
        /* event mode: mean |I|+|Q| over every 16th word; below thr the block is not sent, only a 16-byte
         * marker frame (nsamp = 0, flags bit1) so the host timeline stays complete */
        bool skip = false;
        if (thr) {
            uint32_t acc = 0; const uint32_t *bp = (const uint32_t *)bank;
            for (uint32_t i = 0; i < nwords; i += 16) {
                uint32_t w = bp[i] ^ 0x00080200u; int32_t iv = (int32_t)(w & 0x3FFu) - 512, qv = (int32_t)((w >> 10) & 0x3FFu) - 512;
                acc += (uint32_t)(iv < 0 ? -iv : iv) + (uint32_t)(qv < 0 ? -qv : qv);
            }
            skip = acc / (nwords / 16) < thr;
        }
        const uint32_t ns = skip ? 0 : nsamp, fl = prev_trunc | (skip ? 2u : 0u);
        if (web) { frame[0] = 'E'; frame[1] = 'S'; frame[2] = 'P'; frame[3] = 'W'; put7(frame + 4, seq, 5); put7(frame + 9, t_blk, 5); put7(frame + 14, ns, 3); frame[17] = (uint8_t)fmt; frame[18] = (uint8_t)fl; }
        else { frame[0] = 'E'; frame[1] = 'S'; frame[2] = 'P'; frame[3] = 'B'; memcpy(frame + 4, &seq, 4); memcpy(frame + 8, &t_blk, 4); frame[12] = (uint8_t)ns; frame[13] = (uint8_t)(ns >> 8); frame[14] = (uint8_t)fmt; frame[15] = (uint8_t)fl; }
        bool ok;
        if (skip) {
            wr(SRAM_OWNER, own_run); __asm__ volatile("fence rw,rw" ::: "memory"); t_us = (uint32_t)esp_timer_get_time(); dump_start(ctrl);
            ok = stream2_write(frame, HDR, 2000u); skipped++;
        } else if (fmt) {
            if (web) {
                uint8_t *o = frame + HDR;
                for (uint32_t i = 0; i < nwords; i++) {
                    uint32_t w = bank[i] ^ 0x00080200u;
                    uint32_t a = (w & 0x3FFu) >> 2, b = ((w >> 10) & 0x3FFu) >> 2;     /* 0..255 offset binary (10-bit >> 2) */
                    o[2 * i] = (uint8_t)(a > 254 ? 254 : a); o[2 * i + 1] = (uint8_t)(b > 254 ? 254 : b);
                }
            } else {
                int16_t *o = (int16_t *)(frame + HDR);
                for (uint32_t i = 0; i < nwords; i++) {
                    uint32_t w = bank[i] ^ 0x00080200u;
                    o[2 * i] = (int16_t)((int32_t)(w & 0x3FFu) - 512); o[2 * i + 1] = (int16_t)((int32_t)((w >> 10) & 0x3FFu) - 512);
                }
            }
            /* restart before the long USB transfer so the timestamps stay honest (the block is idle anyway) */
            wr(SRAM_OWNER, own_run); __asm__ volatile("fence rw,rw" ::: "memory"); t_us = (uint32_t)esp_timer_get_time(); dump_start(ctrl);
            ok = stream2_write(frame, HDR + nwords * (web ? 2 : 4), 500000u);
        } else {
            portDISABLE_INTERRUPTS();
            { const volatile uint32_t *src = bank; uint32_t *dst = copy;
              for (uint32_t i = 0; i < nwords; i += 8) {
                  uint32_t a0 = src[i], a1 = src[i + 1], a2 = src[i + 2], a3 = src[i + 3], a4 = src[i + 4], a5 = src[i + 5], a6 = src[i + 6], a7 = src[i + 7];
                  dst[i] = a0; dst[i + 1] = a1; dst[i + 2] = a2; dst[i + 3] = a3; dst[i + 4] = a4; dst[i + 5] = a5; dst[i + 6] = a6; dst[i + 7] = a7; } }
            wr(SRAM_OWNER, own_run); __asm__ volatile("fence rw,rw" ::: "memory"); t_us = (uint32_t)esp_timer_get_time(); dump_start(ctrl);
            portENABLE_INTERRUPTS();
            /* header, then decimate into 64-byte chunks that go out while the next block fills */
            ok = stream2_write(frame, HDR, 2000u);
            uint8_t *o = frame + HDR; uint32_t on = 0; const uint32_t *p = copy;
            for (uint32_t s_ = 0; s_ < nsamp && ok; s_++, p += decim) {
                uint32_t a = 0, b = 0;
                for (uint32_t k = 0; k < decim; k += 4) {
                    uint32_t w0 = p[k] ^ 0x00080200u, w1 = p[k + 1] ^ 0x00080200u, w2 = p[k + 2] ^ 0x00080200u, w3 = p[k + 3] ^ 0x00080200u;
                    a += (w0 & 0x3FFu) + (w1 & 0x3FFu) + (w2 & 0x3FFu) + (w3 & 0x3FFu);
                    b += (w0 & 0xFFC00u) + (w1 & 0xFFC00u) + (w2 & 0xFFC00u) + (w3 & 0xFFC00u);
                }
                int32_t iv = ((int32_t)a - 512 * (int32_t)decim) >> shift, qv = ((int32_t)(b >> 10) - 512 * (int32_t)decim) >> shift;
                if (iv > 127) iv = 127;
                if (iv < -128) iv = -128;
                if (qv > 127) qv = 127;
                if (qv < -128) qv = -128;
                if (web) { iv += 128; qv += 128; if (iv > 254) iv = 254; if (qv > 254) qv = 254; }
                o[on++] = (uint8_t)iv; o[on++] = (uint8_t)qv;
                if (on == 64) { ok = stream2_write(o, 64, 1500u); on = 0; }
            }
            if (ok && on) ok = stream2_write(o, on, 1500u);
        }
        if (ok) sent++; else truncated++;
        prev_trunc = ok ? 0 : 1;
        seq++;
        uint8_t c; if (usb_serial_jtag_ll_read_rxfifo(&c, 1)) stop = true;
    }
    wr(DUMP_CTRL, rd(DUMP_CTRL) & ~(1u << 31));
    __asm__ volatile("fence rw,rw" ::: "memory");
    wr(SRAM_OWNER, owner0);
    free(frame); free(copy);
    esp_rom_delay_us(20000);
    say("\r\nSTREAM2 end seq=%" PRIu32 " sent=%" PRIu32 " truncated=%" PRIu32 " skipped=%" PRIu32 " tuned=%" PRIu32 "\r\nEND\r\n", seq, sent, truncated, skipped, tuned_khz);
}

static bool addr_ok(uint32_t a)
{
    return (a >= 0x60000000u && a < 0x60100000u) || (a >= 0x3FC80000u && a < 0x3FCE0000u);
}

static void handle(char *line)
{
    char *s = line;
    char *cmd = strsep(&s, " ");
    if (!cmd) return;
    if (!strcmp(cmd, "INFO")) {
        esp_chip_info_t ci; esp_chip_info(&ci);
        say("RXPROBE 1 model %d rev %d cpu %u MHz tuned %" PRIu32 " kHz dumpctrl %08" PRIx32 " widx %08" PRIx32 " owner %08" PRIx32 " agc %08" PRIx32 "\r\n",
            ci.model, ci.revision, CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ, tuned_khz, rd(DUMP_CTRL), rd(DUMP_WIDX), rd(SRAM_OWNER), rd(AGC_FORCE));
    } else if (!strcmp(cmd, "TUNE")) {
        uint32_t k = s ? strtoul(s, NULL, 10) : 0;
        if (!tune(k)) say("ERR TUNE range 1500000..3500000 kHz\r\n");
    } else if (!strcmp(cmd, "GAIN")) {
        set_gain(s ? strtoul(s, NULL, 10) : 40);
    } else if (!strcmp(cmd, "RXMODE")) {
        rx_mode(s ? strtoul(s, NULL, 10) : 40); say("OK RXMODE\r\n");
    } else if (!strcmp(cmd, "CAP")) {
        char *a = strsep(&s, " "); char *b = s;
        uint32_t ctrl = a ? strtoul(a, NULL, 16) : 0x2001u;
        uint32_t n = b ? strtoul(b, NULL, 10) : DUMP_BANK_WORDS;
        capture(ctrl, n);
    } else if (!strcmp(cmd, "RD")) {
        uint32_t a = s ? strtoul(s, NULL, 16) : 0;
        if (addr_ok(a)) say("RD %08" PRIx32 " = %08" PRIx32 "\r\n", a, rd(a)); else say("ERR addr\r\n");
    } else if (!strcmp(cmd, "WR")) {
        char *a = strsep(&s, " "); char *v = s;
        uint32_t ad = a ? strtoul(a, NULL, 16) : 0, val = v ? strtoul(v, NULL, 16) : 0;
        if (addr_ok(ad)) { wr(ad, val); say("WR %08" PRIx32 " = %08" PRIx32 " now %08" PRIx32 "\r\n", ad, val, rd(ad)); } else say("ERR addr\r\n");
    } else if (!strcmp(cmd, "I2C")) {
        char *b = strsep(&s, " "); char *r = s;
        unsigned blk = b ? strtoul(b, NULL, 16) : 0, reg = r ? strtoul(r, NULL, 10) : 0;
        say("I2C %02x:%u = %02x\r\n", blk, reg, rom_i2c_readReg(blk, 1, reg));
    } else if (!strcmp(cmd, "I2CW")) {
        char *b = strsep(&s, " "); char *r = strsep(&s, " "); char *v = s;
        unsigned blk = b ? strtoul(b, NULL, 16) : 0, reg = r ? strtoul(r, NULL, 10) : 0, val = v ? strtoul(v, NULL, 16) : 0;
        rom_i2c_writeReg(blk, 1, reg, val);
        say("I2CW %02x:%u = %02x now %02x\r\n", blk, reg, val, rom_i2c_readReg(blk, 1, reg));
    } else if (!strcmp(cmd, "ADCTRIG")) {
        /* Call librftest's own adctrig(a0..a7, stack) with 9 decimal arguments; defaults are the
         * values get_rx_buffer1 uses: 8192 0 0 1 0 0 0 0 0. Then send the bank. */
        unsigned a[9] = {8192, 0, 0, 1, 0, 0, 0, 0, 0};
        for (int i = 0; i < 9 && s; i++) { char *t = strsep(&s, " "); if (t && *t) a[i] = strtoul(t, NULL, 10); }
        for (uint32_t i = 0; i < DUMP_BANK_WORDS; i++) DUMP_BANK[i] = 0xDEADBEEFu;
        uint32_t t0 = esp_cpu_get_cycle_count();
        adctrig(a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], a[8]);
        uint32_t t1 = esp_cpu_get_cycle_count();
        say("CAP ctrl=%08" PRIx32 " status=%08" PRIx32 " done=%d cycles=%" PRIu32 " widx0=%08" PRIx32 " widx_done=%08" PRIx32 " w1=0 w2=0 owner0=%08" PRIx32 " owner_set=0 n=%u tuned=%" PRIu32 " via=adctrig\r\n",
            (uint32_t)0, rd(DUMP_CTRL), 1, t1 - t0, (uint32_t)0, rd(DUMP_WIDX), rd(SRAM_OWNER), DUMP_BANK_WORDS, tuned_khz);
        uint32_t hdr[2] = {0x43415031u, DUMP_BANK_WORDS};
        usb_write(hdr, sizeof hdr);
        usb_write((const void *)DUMP_BANK, DUMP_BANK_WORDS * 4);
        say("\r\nEND\r\n");
    } else if (!strcmp(cmd, "RING")) {
        char *a = strsep(&s, " "); char *b = s;
        ring_probe(a ? strtoul(a, NULL, 16) : 0x34000u, b ? strtoul(b, NULL, 10) : 2000);
    } else if (!strcmp(cmd, "STREAM")) {
        char *a = strsep(&s, " "); char *b = strsep(&s, " "); char *c = strsep(&s, " "); char *d = s;
        stream_decimate(a ? strtoul(a, NULL, 16) : 0x34000u, b ? strtoul(b, NULL, 10) : 16384, c ? strtoul(c, NULL, 10) : 16, d ? strtoul(d, NULL, 10) : 0);
    } else if (!strcmp(cmd, "HEAP")) {
        say("HEAP free %u largest %u\r\n", (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT), (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    } else if (!strcmp(cmd, "BANK")) {
        char *a = strsep(&s, " "); char *b = s;
        bank_probe(a ? strtoul(a, NULL, 10) : 2, b ? strtoul(b, NULL, 16) : 0x13fffu);
    } else if (!strcmp(cmd, "DECBENCH")) {
        dec_bench(s ? strtoul(s, NULL, 10) : 16);
    } else if (!strcmp(cmd, "CSTREAM")) {
        char *a = strsep(&s, " "); char *b = strsep(&s, " "); char *c = s;
        cstream(a ? strtoul(a, NULL, 10) : 16, b ? strtoul(b, NULL, 10) : 0, c ? strtoul(c, NULL, 10) : 4, false);
    } else if (!strcmp(cmd, "USTREAM")) {
        char *a = strsep(&s, " "); char *c = s;
        cstream(a ? strtoul(a, NULL, 10) : 64, 0, c ? strtoul(c, NULL, 10) : 6, true);
    } else if (!strcmp(cmd, "OWNBIT3")) {
        own_bit3 = (s && *s == '0') ? 0u : 8u; say("OK OWNBIT3 %u\r\n", (unsigned)(own_bit3 ? 1 : 0));
    } else if (!strcmp(cmd, "RING2")) {
        ring2(s ? strtoul(s, NULL, 16) : 2u);
    } else if (!strcmp(cmd, "RING3")) {
        ring3();
    } else if (!strcmp(cmd, "CSTREAM2")) {
        char *a = strsep(&s, " "); char *b = strsep(&s, " "); char *c = strsep(&s, " "); char *d = strsep(&s, " "); char *e = s;
        cstream2(a ? strtoul(a, NULL, 16) : 16384u, b ? strtoul(b, NULL, 10) : 16, c ? strtoul(c, NULL, 10) : 0, d ? strtoul(d, NULL, 10) : 4, e && *e == '1');
    } else if (!strcmp(cmd, "BURST")) {
        char *a = strsep(&s, " "); char *b = strsep(&s, " "); char *c = strsep(&s, " "); char *d = strsep(&s, " "); char *e = s;
        burst(a ? strtoul(a, NULL, 16) : 0x13fffu, b ? strtoul(b, NULL, 10) : 16384u, c ? strtoul(c, NULL, 10) : 0xFFFFFFFFu,
              d ? strtoul(d, NULL, 10) : 0, e ? strtoul(e, NULL, 10) : 2000);
    } else if (!strcmp(cmd, "WIFICNT")) {
        char *a = strsep(&s, " "); char *b = s;
        wificnt(a ? strtoul(a, NULL, 10) : 2000, b && *b == '1');
    } else if (!strcmp(cmd, "STREAM2")) {
        char *a = strsep(&s, " "); char *b = strsep(&s, " "); char *c = strsep(&s, " "); char *d = strsep(&s, " "); char *e = strsep(&s, " "); char *f = strsep(&s, " "); char *g = s;
        stream2(a ? strtoul(a, NULL, 16) : 0x13fffu, b ? strtoul(b, NULL, 10) : 16384u, c ? strtoul(c, NULL, 10) : 64u, d ? strtoul(d, NULL, 10) : 6u, e ? strtoul(e, NULL, 10) : 2000u, f ? strtoul(f, NULL, 10) : 0u, g && *g == '1');
    } else if (!strcmp(cmd, "OWN")) {
        say("OWN %08" PRIx32 "\r\n", rd(SRAM_OWNER));
    } else {
        say("ERR ? INFO TUNE GAIN RXMODE CAP ADCTRIG RING STREAM HEAP RD WR I2C I2CW OWN\r\n");
    }
}

void app_main(void)
{
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        e = nvs_flash_init();
    }
    ESP_ERROR_CHECK(e);
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    ESP_ERROR_CHECK(esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE));
    tune(2437000);

    char line[128];
    size_t used = 0;
    for (;;) {
        uint8_t c;
        if (!usb_serial_jtag_ll_read_rxfifo(&c, 1)) { vTaskDelay(1); continue; }
        if (c == '\r') continue;
        if (c != '\n') { if (used < sizeof line - 1) line[used++] = (char)c; continue; }
        line[used] = 0; used = 0;
        if (line[0]) handle(line);
    }
}
