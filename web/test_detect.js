// node web/test_detect.js <sigmf-meta> : run the viewer's PSD -> detect -> classify pipeline over a capture and print the aggregated "what is this" table
const fs = require("fs"), DSP = require("./dsp.js");
const metaPath = process.argv[2]; const meta = JSON.parse(fs.readFileSync(metaPath)); const data = fs.readFileSync(metaPath.replace(".sigmf-meta", ".sigmf-data"));
const rate = meta.global["core:sample_rate"], ci8 = meta.global["core:datatype"] === "ci8", lo = meta.captures[0]["core:frequency"] / 1e6, caps = meta.captures;
const nfft = rate >= 16e6 ? 1024 : rate >= 2e6 ? 512 : 256, re = new Float32Array(nfft), im = new Float32Array(nfft), win = Float32Array.from({ length: nfft }, (_, i) => 0.5 - 0.5 * Math.cos(2 * Math.PI * i / (nfft - 1)));
const agg = new Map(); let nb = 0; const floor = DSP.makeFloor(nfft);
for (let i = 0; i < caps.length; i++) {
  const s0 = caps[i]["core:sample_start"], s1 = i + 1 < caps.length ? caps[i + 1]["core:sample_start"] : data.length / (ci8 ? 2 : 4); if (s1 - s0 < 64) { nb++; continue; }
  const bps = ci8 ? 2 : 4; const { I, Q } = DSP.unpack(new Uint8Array(data.buffer, data.byteOffset + s0 * bps, (s1 - s0) * bps), ci8 ? 0 : 1);
  const db = DSP.psd(I, Q, nfft, win, re, im); nb++; const fl = floor.update(db); const keys = new Set();
  for (const d of DSP.detect(db, rate, 8, fl)) { const cl = DSP.classify(d, lo, rate, 1); const key = cl.kind + ":" + (Math.round(cl.rfC * 2) / 2).toFixed(1); if (keys.has(key)) continue; keys.add(key); const a = agg.get(key) || { n: 0, peak: 0, label: cl.label, kind: cl.kind, lo: cl.rfLo, hi: cl.rfHi, conf: cl.conf }; a.n++; a.peak = Math.max(a.peak, d.peakDb); a.lo = Math.min(a.lo, cl.rfLo); a.hi = Math.max(a.hi, cl.rfHi); agg.set(key, a); }
}
console.log(`${metaPath}: ${nb} blocks, LO ${lo} MHz, ${rate} S/s`);
for (const a of [...agg.values()].sort((x, y) => y.peak - x.peak).slice(0, 8)) { const persist = a.n / nb; const label = a.label + (a.kind === "wifi" || a.kind === "lte" || a.kind === "unknown" ? (persist > 0.6 ? "・連続" : persist < 0.3 ? "・間欠バースト" : "") : "");
  console.log(`  ${label}  | ${a.lo.toFixed(2)}-${a.hi.toFixed(2)} MHz | +${a.peak.toFixed(0)} dB | ${(100 * persist).toFixed(0)}% | ${a.conf}`); }
