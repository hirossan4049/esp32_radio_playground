// node web/test_dsp.js captures/ble_2426_4M_ev.sigmf-meta  -> runs the JS BLE decoder on a SigMF capture (compare with host/ble.py)
const fs = require("fs"), DSP = require("./dsp.js");
const metaPath = process.argv[2]; const meta = JSON.parse(fs.readFileSync(metaPath)); const data = fs.readFileSync(metaPath.replace(".sigmf-meta", ".sigmf-data"));
const rate = meta.global["core:sample_rate"], ci8 = meta.global["core:datatype"] === "ci8"; const f0 = meta.captures[0]["core:frequency"] / 1e6;
const ch = { 2402: 37, 2426: 38, 2480: 39 }[Math.round(f0)] || 37; let ok = 0, hits = 0; const seen = {};
const caps = meta.captures;
for (let i = 0; i < caps.length; i++) {
  const s0 = caps[i]["core:sample_start"], s1 = i + 1 < caps.length ? caps[i + 1]["core:sample_start"] : data.length / (ci8 ? 2 : 4);
  if (s1 - s0 < 400) continue;
  const bps = ci8 ? 2 : 4; const payload = new Uint8Array(data.buffer, data.byteOffset + s0 * bps, (s1 - s0) * bps);
  const { I, Q } = DSP.unpack(payload, ci8 ? 0 : 1);
  for (const h of DSP.bleDecode(I, Q, rate, ch)) { hits++; if (h.crcOk) { ok++; seen[h.adva] = (seen[h.adva] || 0) + 1; console.log(`blk ${i} ${h.pduType} len ${h.length} AdvA ${h.adva} cfo ${(h.cfoHz / 1e3).toFixed(1)} kHz ${h.ad}`); } }
}
console.log(`AA hits ${hits}, CRC OK ${ok}, distinct AdvA ${Object.keys(seen).length}`);
