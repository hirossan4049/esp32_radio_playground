// dsp.js — signal processing and band knowledge for the ESP32-C3 SDR viewer. Runs in the browser and in node (tests).
"use strict";
const DSP = (() => {
  // ---------- known bands (Japan-centric; frequencies in MHz) ----------
  const BANDS = [];
  for (let ch = 1; ch <= 13; ch++) BANDS.push({ name: `Wi-Fi ch${ch}`, kind: "wifi", f0: 2412 + 5 * (ch - 1), bw: 20, group: "Wi-Fi 2.4 GHz" });
  BANDS.push({ name: "Wi-Fi ch14 (11b)", kind: "wifi", f0: 2484, bw: 22, group: "Wi-Fi 2.4 GHz" });
  for (const [ch, f0] of [[37, 2402], [38, 2426], [39, 2480]]) BANDS.push({ name: `BLE adv ch${ch}`, kind: "ble_adv", f0, bw: 2, group: "BLE", bleChannel: ch });
  for (let k = 0; k <= 36; k++) { const f0 = k < 11 ? 2404 + 2 * k : 2428 + 2 * (k - 11); BANDS.push({ name: `BLE data ch${k}`, kind: "ble", f0, bw: 2, group: "BLE" }); }
  for (let ch = 11; ch <= 26; ch++) BANDS.push({ name: `Zigbee/Thread ch${ch}`, kind: "zigbee", f0: 2405 + 5 * (ch - 11), bw: 2, group: "802.15.4" });
  const R = (name, kind, lo, hi, group, note) => BANDS.push({ name, kind, f0: (lo + hi) / 2, bw: hi - lo, group, note });
  R("LTE Band 3 下り (1.7 GHz)", "lte", 1805, 1880, "携帯電話 下り", "docomo 1845-1880 など。連続波");
  R("J-DECT / sPHS", "dect", 1893.5, 1906.1, "コードレス電話", "1.728 MHz 幅のバースト");
  R("LTE Band 1 上り (2 GHz)", "lte_ul", 1920, 1980, "携帯電話 上り", "端末からの間欠送信");
  R("LTE Band 1 下り (2 GHz)", "lte", 2110, 2170, "携帯電話 下り", "基地局。連続波");
  R("ISM 2.4 GHz", "ism", 2400, 2483.5, "免許不要帯", "Wi-Fi / Bluetooth / Zigbee / 電子レンジ");
  R("TD-LTE Band 41 (2.5 GHz BWA)", "lte_td", 2545, 2645, "携帯電話 TDD", "UQ / SoftBank(WCP)。時分割");
  R("LTE Band 7 (2.6 GHz)", "lte", 2620, 2690, "携帯電話 下り", "日本では未使用");
  const KIND_COLOR = { wifi: "rgba(79,195,247,0.22)", ble_adv: "rgba(255,112,67,0.45)", ble: "rgba(255,112,67,0.12)", zigbee: "rgba(129,199,132,0.25)", lte: "rgba(186,104,200,0.25)", lte_ul: "rgba(186,104,200,0.12)", lte_td: "rgba(171,71,188,0.3)", dect: "rgba(255,213,79,0.3)", ism: "rgba(255,255,255,0.05)" };
  // internal spurs of the C3 (baseband offsets in MHz, LO-relative; docs/c3_findings.md RX-C3-016)
  const INTERNAL_BB = [{ f: 0, w: 0.15, name: "DC / LO 漏れ" }, { f: -2.0, w: 0.1, name: "内部スプリアス (LO−2 MHz)" }, { f: -7.0, w: 0.1, name: "内部スプリアス (LO−7 MHz)" }, { f: -11.0, w: 0.1, name: "2448 MHz チップスプリアス" }, { f: +5.0, w: 0.1, name: "2448 MHz の折り返し (16 MS/s 系)" }];
  function bandsAt(rfMHz) { return BANDS.filter(b => rfMHz >= b.f0 - b.bw / 2 && rfMHz <= b.f0 + b.bw / 2); }
  function internalAt(bbMHz, loMHz) {
    for (const s of INTERNAL_BB) if (Math.abs(bbMHz - s.f) <= s.w) return s.name;
    // 40 MHz crystal harmonics: RF = 40k -> baseband = LO - 40k
    const k = Math.round(loMHz / 40); const bb = loMHz - 40 * k; if (Math.abs(bbMHz - bb) <= 0.05) return `水晶 ${40 * k} MHz 高調波`;
    return null;
  }
  // ---------- FFT / PSD ----------
  function fft(re, im) {
    const n = re.length; for (let i = 1, j = 0; i < n; i++) { let bit = n >> 1; for (; j & bit; bit >>= 1) j ^= bit; j ^= bit; if (i < j) { [re[i], re[j]] = [re[j], re[i]]; [im[i], im[j]] = [im[j], im[i]]; } }
    for (let len = 2; len <= n; len <<= 1) { const ang = -2 * Math.PI / len, wr = Math.cos(ang), wi = Math.sin(ang);
      for (let i = 0; i < n; i += len) { let cr = 1, ci = 0; for (let j = 0; j < len / 2; j++) { const a = i + j, b = a + len / 2; const tr = re[b] * cr - im[b] * ci, ti = re[b] * ci + im[b] * cr; re[b] = re[a] - tr; im[b] = im[a] - ti; re[a] += tr; im[a] += ti; const nr = cr * wr - ci * wi; ci = cr * wi + ci * wr; cr = nr; } } }
  }
  // payload (Uint8Array) -> {I, Q} Float32Arrays (DC removed)
  function unpack(payload, fmt) {
    const dv = new DataView(payload.buffer, payload.byteOffset, payload.byteLength); const n = fmt ? payload.length / 4 : payload.length / 2;
    const I = new Float32Array(n), Q = new Float32Array(n); let mi = 0, mq = 0;
    for (let i = 0; i < n; i++) { I[i] = fmt ? dv.getInt16(4 * i, true) : dv.getInt8(2 * i); Q[i] = fmt ? dv.getInt16(4 * i + 2, true) : dv.getInt8(2 * i + 1); mi += I[i]; mq += Q[i]; }
    mi /= n; mq /= n; for (let i = 0; i < n; i++) { I[i] -= mi; Q[i] -= mq; }
    return { I, Q };
  }
  // averaged periodogram, dB, fftshifted (index 0 = -fs/2). Returns Float32Array(nfft)
  function psd(I, Q, nfft, win, re, im) {
    const n = I.length, out = new Float32Array(nfft); let segs = 0;
    const seg = (s0, m) => { for (let i = 0; i < nfft; i++) { const ok = i < m; re[i] = ok ? I[s0 + i] * win[i] : 0; im[i] = ok ? Q[s0 + i] * win[i] : 0; } fft(re, im); for (let i = 0; i < nfft; i++) out[i] += re[i] * re[i] + im[i] * im[i]; segs++; };
    for (let s0 = 0; s0 + nfft <= n; s0 += nfft) seg(s0, nfft);
    if (!segs) seg(0, Math.min(n, nfft));
    const db = new Float32Array(nfft); for (let i = 0; i < nfft; i++) db[(i + nfft / 2) % nfft] = 10 * Math.log10(out[i] / segs + 1e-6);
    return db;
  }
  function median(a) { const b = Float32Array.from(a).sort(); return b[b.length >> 1]; }
  // ---------- noise-floor tracking and signal detection ----------
  // Two floor estimates, combined per bin with min(): a slow-rising min-hold over time (finds the noise under bursty
  // signals such as Wi-Fi that fill the whole span) and the row's 20th percentile (finds it under continuous signals
  // such as LTE that occupy part of the span). The true noise is below both.
  function makeFloor(nfft) { return { hold: null, update(db) { const s = Float32Array.from(db).sort(); const p20 = s[Math.floor(0.2 * s.length)];
      if (!this.hold) this.hold = Float32Array.from(db); const out = new Float32Array(db.length);
      for (let i = 0; i < db.length; i++) { this.hold[i] = Math.min(db[i], this.hold[i] + 0.25); out[i] = Math.min(this.hold[i], p20); } return out; } }; }
  // returns [{bbLo, bbHi (MHz), peakDb, meanDb, width}] of contiguous (smoothed) bins above floor+thr; gaps < 3 bins are merged
  function detect(db, fs, thr, floor) {
    const n = db.length, binW = fs / n / 1e6, sm = new Float32Array(n), k = n >= 512 ? 2 : 1;
    for (let i = 0; i < n; i++) { let a = 0, c = 0; for (let j = i - k; j <= i + k; j++) if (j >= 0 && j < n) { a += db[j]; c++; } sm[i] = a / c; }
    const fl = floor || new Float32Array(n).fill(median(db));
    const hot = new Uint8Array(n); for (let i = 0; i < n; i++) hot[i] = sm[i] - fl[i] > thr ? 1 : 0;
    const out = []; let i = 0;
    while (i < n) {
      if (hot[i]) { let j = i; for (;;) { while (j + 1 < n && hot[j + 1]) j++; let g = j + 1; while (g < n && !hot[g] && g - j <= 3) g++; if (g < n && hot[g] && g - j <= 3) { j = g; continue; } break; }
        let pk = -1e9, sum = 0; for (let q = i; q <= j; q++) { const v = db[q] - fl[q]; if (v > pk) pk = v; sum += v; }
        out.push({ bbLo: (i / n - 0.5) * fs / 1e6, bbHi: ((j + 1) / n - 0.5) * fs / 1e6, peakDb: pk, meanDb: sum / (j - i + 1), width: (j - i + 1) * binW }); i = j + 1; }
      else i++;
    }
    return out;
  }
  // classify a detection by its width (MHz), persistence (fraction of recent blocks it was present), and the band table
  function classify(det, loMHz, fs, persist) {
    const bbC = (det.bbLo + det.bbHi) / 2, rfC = loMHz - bbC, rfLo = loMHz - det.bbHi, rfHi = loMHz - det.bbLo;
    const internal = det.width < 0.3 ? internalAt(bbC, loMHz) : null;
    if (internal) return { label: internal, kind: "internal", rfC, rfLo, rfHi, conf: "ESP32 内部由来" };
    const bands = bandsAt(rfC).filter(b => b.kind !== "ism"), span = fs / 1e6, w = det.width;
    let kind, guess;
    if (span < 8 && w >= 0.7 * span) { kind = "fill"; guess = `視野 (${span.toFixed(1)} MHz) いっぱいの信号。レートを 16 MS/s 以上にすると幅で判別できます`; }
    else if (w >= 12 || (span <= 16.5 && w >= 0.7 * span)) { kind = "wide"; guess = "広帯域 (≥ 12 MHz)"; }
    else if (w >= 3.5) { kind = "mid"; guess = `約 ${w.toFixed(0)} MHz 幅`; }
    else if (w >= 0.6) { kind = "ble"; guess = "1〜2 MHz 幅のバースト (BLE / Zigbee 的)"; }
    else { kind = "tone"; guess = "トーン"; }
    const pick = kinds => bands.filter(b => kinds.includes(b.kind)).sort((a, b) => Math.abs(a.f0 - rfC) - Math.abs(b.f0 - rfC)).map(b => b.name);   // nearest centre first
    let label, conf;
    if (kind === "fill") { const nm = pick(["wifi", "lte", "lte_td", "lte_ul", "dect", "ble_adv", "ble", "zigbee"]); label = (nm.length ? nm[0] + " 付近 — " : "") + guess; conf = "視野が狭く未判定"; kind = "unknown"; }
    else if (kind === "wide") { const wifi = pick(["wifi"]), lte = pick(["lte", "lte_td", "lte_ul"]); label = (wifi.length ? wifi[0] + (wifi.length > 1 ? "（±1 ch 重なり）" : "") : lte.length ? lte[0] : "未知の広帯域") + " — " + guess; conf = wifi.length || lte.length ? "バンド表と一致" : "幅からの推定"; kind = wifi.length && !lte.length ? "wifi" : "lte"; }
    else if (kind === "ble") { const nm = pick(["ble_adv", "ble", "zigbee"]); label = (nm.length ? nm.slice(0, 2).join(" または ") : "未知") + " — " + guess; conf = nm.length ? "バンド表と一致" : "幅からの推定"; }
    else if (kind === "mid") { const nm = pick(["lte", "lte_td", "lte_ul", "dect"]); label = (nm.length ? nm[0] + " — " : "") + guess + (nm.length ? "" : " (未知)"); conf = nm.length ? "バンド表と一致" : "幅からの推定"; kind = nm.length ? "lte" : "unknown"; }
    else { const inb = bands.map(b => b.name)[0]; label = `トーン ${rfC.toFixed(3)} MHz` + (inb ? `（${inb} の範囲内）` : ""); conf = "幅からの推定"; }
    return { label, kind, rfC, rfLo, rfHi, conf };
  }
  // ---------- BLE 1M advertising decode (port of host/ble.py) ----------
  const ADV_AA = 0x8E89BED6, PDU_TYPES = ["ADV_IND", "ADV_DIRECT_IND", "ADV_NONCONN_IND", "SCAN_REQ", "SCAN_RSP", "CONNECT_IND", "ADV_SCAN_IND", "ADV_EXT_IND"];
  function whitening(channel, n) { let reg = [1, 0, 0, 0, 0, 0, 0]; for (let i = 0; i < 6; i++) reg[1 + i] = (channel >> (5 - i)) & 1; const out = new Int8Array(n);
    for (let i = 0; i < n; i++) { const fb = reg[6]; out[i] = fb; const nr = [fb, reg[0], reg[1], reg[2], reg[3] ^ fb, reg[4], reg[5]]; reg = nr; } return out; }
  function crc24(bits, n) { let reg = 0x555555; for (let i = 0; i < n; i++) { const fb = ((reg >>> 23) & 1) ^ bits[i]; reg = (reg << 1) & 0xFFFFFF; if (fb) reg ^= 0x00065B; } return reg; }
  function adParse(payload) { const out = []; let i = 6; while (i + 1 < payload.length) { const ln = payload[i]; if (!ln) break; const t = payload[i + 1]; const v = payload.subarray(i + 2, i + 1 + ln);
      if (t === 8 || t === 9) out.push("name='" + String.fromCharCode(...v) + "'"); else if (t === 1) out.push("flags=0x" + v[0].toString(16)); else if (t === 0xff) out.push("mfr=0x" + ((v[1] << 8) | v[0]).toString(16).padStart(4, "0") + (v[0] === 0x4c && v[1] === 0 ? "(Apple)" : "")); else if (t === 2 || t === 3) out.push("uuid16"); else if (t === 0x16) out.push("svcdata"); else out.push("ad" + t.toString(16)); i += 1 + ln; } return out.join(" "); }
  function bleDecode(I, Q, fs, channel) {
    const sps = fs / 1e6, n = I.length, hits = [];
    // conj (LO-RF -> RF-LO): Q -> -Q ; discriminator
    const d = new Float32Array(n - 1); for (let i = 0; i < n - 1; i++) { const a = I[i], b = -Q[i], c = I[i + 1], e = -Q[i + 1]; d[i] = Math.atan2(e * a - c * b, c * a + e * b); }
    const k = Math.max(1, Math.round(sps * 0.75)); const df = new Float32Array(d.length); for (let i = 0; i < d.length; i++) { let s = 0, c = 0; for (let j = i - (k >> 1); j <= i + (k >> 1); j++) if (j >= 0 && j < d.length) { s += d[j]; c++; } df[i] = s / c; }
    const pre = []; for (let i = 0; i < 8; i++) pre.push(i & 1); for (let i = 0; i < 32; i++) pre.push((ADV_AA >>> i) & 1);
    const ref = pre.map(b => b ? 1 : -1), idx = pre.map((_, j) => Math.round(j * sps)), span = idx[idx.length - 1] + 1, N = df.length - span;
    if (N <= 0) return hits;
    const cs = new Float64Array(df.length + 1); for (let i = 0; i < df.length; i++) cs[i + 1] = cs[i] + df[i];
    const lm = new Float32Array(N); for (let i = 0; i < N; i++) lm[i] = (cs[i + span] - cs[i]) / span;
    const corr = new Float32Array(N); for (let j = 0; j < pre.length; j++) { const i0 = idx[j], r = ref[j]; for (let i = 0; i < N; i++) corr[i] += (df[i0 + i] - lm[i]) * r; }
    const absSorted = Float32Array.from(df, Math.abs).sort(); const thr = 0.35 * absSorted[Math.floor(0.95 * absSorted.length)] * pre.length + 1e-6;
    const peaks = []; for (let i = 0; i < N; i++) if (corr[i] > thr) { if (peaks.length && i - peaks[peaks.length - 1] < sps) { if (corr[i] > corr[peaks[peaks.length - 1]]) peaks[peaks.length - 1] = i; } else peaks.push(i); }
    for (const p of peaks) {
      const cfo = lm[p]; let best = null; const offs = []; for (let t = -1; t < Math.ceil(sps); t++) offs.push(t); offs.sort((a, b) => Math.abs(a - 1) - Math.abs(b - 1));
      for (const toff of offs) {
        const start = p + span + toff, nb = 42 * 8, bits = new Int8Array(nb); let m = 0;
        for (let i = 0; i < nb; i++) { const bi = Math.round(start + i * sps); if (bi < 0 || bi >= df.length) break; bits[m++] = df[bi] - cfo > 0 ? 1 : 0; }
        if (m < 48) continue;
        const wh = whitening(channel, m); const raw = new Int8Array(m); for (let i = 0; i < m; i++) raw[i] = bits[i] ^ wh[i];
        let pduType = 0; for (let i = 0; i < 4; i++) pduType |= raw[i] << i; let length = 0; for (let i = 0; i < 8; i++) length |= raw[8 + i] << i;
        if (length > 37 || m < 16 + length * 8 + 24) continue;
        const nbits = 16 + length * 8; let rx = 0; for (let i = 0; i < 24; i++) rx = (rx << 1) | raw[nbits + i]; rx >>>= 0;
        const ok = crc24(raw, nbits) === rx;
        const payload = new Uint8Array(length); for (let b = 0; b < length; b++) { let v = 0; for (let i = 0; i < 8; i++) v |= raw[16 + 8 * b + i] << i; payload[b] = v; }
        const adva = length >= 6 ? [5, 4, 3, 2, 1, 0].map(i => payload[i].toString(16).padStart(2, "0")).join(":") : "";
        const cand = { pduType: PDU_TYPES[pduType] || pduType, length, crcOk: ok, adva, ad: length > 6 ? adParse(payload) : "", cfoHz: cfo * fs / (2 * Math.PI), pos: p };
        if (ok) { best = cand; break; } if (!best) best = cand;
      }
      if (best) hits.push(best);
    }
    return hits;
  }
  return { BANDS, KIND_COLOR, INTERNAL_BB, bandsAt, internalAt, fft, unpack, psd, median, makeFloor, detect, classify, bleDecode };
})();
if (typeof module !== "undefined") module.exports = DSP;
