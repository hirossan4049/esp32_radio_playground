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
    wr(SRAM_OWNER, rd(SRAM_OWNER) | 8u);
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
    wr(SRAM_OWNER, rd(SRAM_OWNER) | 8u);
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
    wr(SRAM_OWNER, rd(SRAM_OWNER) | 8u);
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
    wr(SRAM_OWNER, rd(SRAM_OWNER) | 8u);
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
