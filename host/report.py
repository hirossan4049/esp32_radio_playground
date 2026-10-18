#!/usr/bin/env python3
"""Reception report generator for an ESP32-C3 SigMF capture. Receive-only.

Produces one self-contained HTML file (embedded plots, no external assets) summarising a capture:
 - reception conditions (LO, rate, gain, 5/6 mode, duration, coverage/duty, clipping)
 - averaged spectrum + waterfall (true time axis, gaps shown)
 - detected signals with RF frequency, bandwidth, persistence, band-table identification, confidence
 - decoded packets: BLE advertising (ble.py) and, if a device profile matches, remote presses (rfproto/lightremote)
 - quality metrics: noise floor, peak dBFS, SNR estimate, frame-loss, I/Q imbalance, DC offset

  report.py capture.sigmf-meta [--out report.html] [--rate 1e6] [--open]
"""
import argparse, sys, os, io, base64, json, datetime, collections
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from esp32sdr.sigmf import read_sigmf, EXT
import bands, pkt

def psd_db(x, nfft):
    x = x - x.mean(); n = len(x) // nfft
    if n == 0:
        X = np.fft.fftshift(np.fft.fft(x * np.hanning(len(x)), nfft)); return 10 * np.log10(np.abs(X) ** 2 + 1e-9)
    X = np.fft.fftshift(np.fft.fft(x[:n * nfft].reshape(n, nfft) * np.hanning(nfft), axis=1), axes=1)
    return 10 * np.log10(np.mean(np.abs(X) ** 2, axis=0) + 1e-9)

def detect(db, fs, floor, thr=8):
    n = len(db); binW = fs / n / 1e6; sm = np.convolve(db, np.ones(5) / 5, mode="same"); hot = sm - floor > thr
    out = []; i = 0
    while i < n:
        if hot[i]:
            j = i
            while j + 1 < n and hot[j + 1]: j += 1
            pk = float((db[i:j + 1] - floor[i:j + 1]).max()); out.append(((i / n - 0.5) * fs / 1e6, ((j + 1) / n - 0.5) * fs / 1e6, pk, (j - i + 1) * binW)); i = j + 1
        else: i += 1
    return out

def classify(bbLo, bbHi, pk, width, lo_mhz, fs):
    bbC = (bbLo + bbHi) / 2; rfC = lo_mhz - bbC; rfLo, rfHi = lo_mhz - bbHi, lo_mhz - bbLo
    if width < 0.3:
        intl = bands.internal_at(bbC, lo_mhz)
        if intl: return dict(label=intl, kind="internal", rfC=rfC, rfLo=rfLo, rfHi=rfHi, conf="ESP32内部由来")
    bb = [b for b in bands.bands_at(rfC) if b["kind"] != "ism"]; span = fs / 1e6
    if span < 8 and width >= 0.7 * span: kind, g = "unknown", f"視野({span:.1f}MHz)いっぱい・要広帯域"
    elif width >= 12 or (span <= 16.5 and width >= 0.7 * span): kind, g = "wide", "広帯域(≥12MHz)"
    elif width >= 3.5: kind, g = "mid", f"約{width:.0f}MHz幅"
    elif width >= 0.6: kind, g = "ble", "1〜2MHz幅バースト(BLE/Zigbee的)"
    else: kind, g = "tone", "トーン"
    near = sorted(bb, key=lambda b: abs(b["f0"] - rfC))
    label = (near[0]["name"] + " — " if near else "") + g; conf = "バンド表と一致" if near else "幅からの推定"
    return dict(label=label, kind=kind if near else "unknown", rfC=rfC, rfLo=rfLo, rfHi=rfHi, conf=conf)

def fig_to_b64(fig):
    buf = io.BytesIO(); fig.savefig(buf, format="png", dpi=100, bbox_inches="tight"); buf.seek(0)
    import matplotlib.pyplot as plt; plt.close(fig)
    return "data:image/png;base64," + base64.b64encode(buf.read()).decode()

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("file"); ap.add_argument("--out", default=None); ap.add_argument("--rate", type=float, default=1e6); ap.add_argument("--open", action="store_true")
    a = ap.parse_args()
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    meta, blocks = read_sigmf(a.file); g = meta["global"]; fs = g["core:sample_rate"]; f0 = meta["captures"][0]["core:frequency"]
    full = [(t, x) for t, x in blocks if len(x) >= 64]
    nfft = min(1024, len(full[0][1]) if full else 256)
    f = np.fft.fftshift(np.fft.fftfreq(nfft, 1 / fs)) / 1e6
    # spectra and waterfall
    rows = []; tms = []; step = max(1, len(full) // 1500)
    for t, x in full[::step]: rows.append(psd_db(x, nfft)); tms.append(t)
    W = np.array(rows); t = (np.array(tms, dtype=np.int64) - tms[0]) / 1e6
    avg = 10 * np.log10(np.mean(10 ** (W / 10), axis=0)); floor = np.minimum(np.percentile(W, 20, axis=0), np.median(W, axis=0))
    med = float(np.median(avg))
    # quality metrics
    amp = np.array([np.abs(x).mean() for _, x in full]); peak = np.array([np.abs(x).max() for _, x in full])
    fullscale = 511 if g["core:datatype"] == "ci16_le" else 127
    allx = np.concatenate([x for _, x in full[:200]]) if full else np.array([0j])
    irms, qrms = float(np.std(allx.real)), float(np.std(allx.imag)); dc = complex(np.mean(allx))
    snr = float(avg.max() - med)
    # detections aggregated over blocks
    agg = {}; fl = np.minimum(np.percentile(W, 20, axis=0), np.median(W, axis=0))
    for t_, x in full:
        db = psd_db(x, nfft); seen_keys = set()
        for bbLo, bbHi, pk, width in detect(db, fs, fl):
            cl = classify(bbLo, bbHi, pk, width, f0 / 1e6, fs); key = cl["kind"] + ":" + f"{round(cl['rfC']*2)/2:.1f}"
            d = agg.setdefault(key, dict(n=0, pk=0, lo=cl["rfLo"], hi=cl["rfHi"], label=cl["label"], conf=cl["conf"], kind=cl["kind"]))
            if key not in seen_keys: d["n"] += 1; seen_keys.add(key)       # count each key at most once per block
            d["pk"] = max(d["pk"], pk); d["lo"] = min(d["lo"], cl["rfLo"]); d["hi"] = max(d["hi"], cl["rfHi"])
    nb = len(full) or 1
    dets = sorted(agg.values(), key=lambda d: -d["pk"])[:15]
    # BLE decode
    ble_rows = []
    try:
        import ble
        ch = {2402: 37, 2426: 38, 2480: 39}.get(round(f0 / 1e6))
        if ch and fs >= 2e6:
            seen = {}
            for bi, (t_, x) in enumerate(blocks):
                if len(x) < 400: continue
                for h in ble.demod_block(x, fs, ch):
                    if h.get("crc_ok"): seen[h["adva"]] = seen.get(h["adva"], {"n": 0, "types": set(), "ad": ""}); seen[h["adva"]]["n"] += 1; seen[h["adva"]]["types"].add(h["pdu_type"]); seen[h["adva"]]["ad"] = h.get("payload", "")[:20]
            ble_rows = [(k, v) for k, v in sorted(seen.items(), key=lambda kv: -kv[1]["n"])]
    except Exception as e: ble_rows = []
    # remote profile match
    remote_rows = []; remote_name = None
    try:
        import rfproto, lightremote
        profs = rfproto.load_profiles()
        for pr in profs:
            fs2, pk2 = rfproto.extract_packets(a.file, pr["rate"], pr["nbytes"], 0, sync=pr["sync"])
            mv = pr["variants"][pr["main_variant"]]; idb = mv["id_bytes"]; idv = bytes.fromhex(mv["id_value"]) if mv.get("id_value") else b""
            sel = [(tt, by) for tt, by in pk2 if len(by) >= (max(idb) + 1 if idb else 4) and by[:4].hex() == pr["main_variant"]]
            if not sel: continue
            agree = sum(1 for k2, j in enumerate(idb) if j < len(sel[0][1]) and k2 < len(idv) and collections.Counter(by[j] for _, by in sel).most_common(1)[0][0] == idv[k2])
            if idb and agree / len(idb) >= 0.7:
                remote_name = pr["name"]; cb = mv.get("command_bytes", []); need = max(cb) + 1 if cb else 0
                labels = {tuple(bytes.fromhex(kk)): vv for kk, vv in pr.get("labels", {}).items()}; last = None
                for tt, by in sel:
                    if len(by) < need: continue                            # only packets long enough to hold every command byte
                    raw = [by[j] for j in cb]
                    # apply the profile's byte masks (e.g. the lighting remote masks byte 30 with 0xFC)
                    if pr.get("cmd_mask"): raw = [v & m for v, m in zip(raw, pr["cmd_mask"])]
                    cmd = bytes(raw)
                    if cmd == last: continue
                    last = cmd; remote_rows.append((tt, labels.get(tuple(cmd), ""), cmd.hex(" ")))
                break
    except Exception: pass
    # --- figures ---
    fig1, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6.5), gridspec_kw={"height_ratios": [1, 1.8]})
    rf = f0 / 1e6 - f; order = np.argsort(rf)
    ax1.plot(rf[order], avg[order], lw=0.8, color="#1f77b4"); ax1.plot(rf[order], floor[order], lw=0.6, color="#bbb")
    ax1.set_ylabel("dB"); ax1.set_title(f"平均スペクトル / ウォーターフォール  LO {f0/1e6:.3f} MHz  {fs/1e6:g} MS/s"); ax1.grid(alpha=0.3)
    vmin, vmax = np.percentile(W, 5), np.percentile(W, 99.5)
    ax2.pcolormesh(rf[order], t, W[:, order], vmin=vmin, vmax=vmax, cmap="viridis", shading="nearest")
    ax2.set_xlabel("RF MHz"); ax2.set_ylabel("時刻 s（取得時のみ・隙間含む）")
    img = fig_to_b64(fig1)
    # activity timeline
    fig2, ax = plt.subplots(figsize=(10, 2.2)); tt = np.array([ (tm - tms[0]) / 1e6 for tm,_ in [(m,0) for m in tms]])
    tall = (np.array([tm for tm, _ in full], dtype=np.int64) - full[0][0]) / 1e6
    ax.plot(tall, amp, lw=0.6); ax.set_xlabel("時刻 s"); ax.set_ylabel("ブロック平均|x|"); ax.set_title("受信強度の時間推移"); ax.grid(alpha=0.3)
    img2 = fig_to_b64(fig2)
    # --- plain-language summary ---
    summary = []
    namedsig = [d for d in dets if d["kind"] not in ("internal", "unknown") and d["conf"] == "バンド表と一致"]
    kinds_present = collections.Counter(d["kind"] for d in namedsig)
    if remote_rows:
        buttons = collections.Counter(lab for _, lab, _ in remote_rows if lab)
        if buttons: summary.append(f"🎛 リモコン「{remote_name}」の操作を {sum(buttons.values())} 回受信: " + "、".join(f"{k}×{v}" for k, v in buttons.most_common()))
        else: summary.append(f"🎛 リモコン「{remote_name}」のパケットを受信（ボタン未確定 {len(remote_rows)} 件）")
    if ble_rows: summary.append(f"🔵 BLE 機器 {len(ble_rows)} 台の広告を復号（CRC OK）")
    wifi = [d for d in namedsig if d["kind"] == "wifi"]
    if wifi: summary.append(f"📶 Wi-Fi 信号: " + "、".join(sorted({d['label'].split(' —')[0] for d in wifi}))[:80])
    lte = [d for d in namedsig if d["kind"] in ("lte",)]
    if lte: summary.append(f"📡 携帯基地局(LTE)らしき連続波: " + "、".join(sorted({d['label'].split(' —')[0] for d in lte}))[:80])
    if not summary: summary.append("既知の信号は検出されませんでした（雑音のみ、または未知の信号）。")
    peakdbfs = 20 * np.log10(max(1, peak.max()) / fullscale)
    cond = "適正" if -12 < peakdbfs < -2 else ("飽和気味（ゲインを下げてください）" if peakdbfs >= -2 else "弱め（ゲインを上げると改善）")
    summary.append(f"📈 受信強度は{cond}（ピーク {peakdbfs:.0f} dBFS）。")
    summ_html = "".join(f"<li>{s}</li>" for s in summary)
    # --- html ---
    alt = g.get(EXT + ":lo_mode", "1"); mode = g.get(EXT + ":capture_mode", "?")
    def tr(cells): return "<tr>" + "".join(f"<td>{c}</td>" for c in cells) + "</tr>"
    detrows = "".join(tr([f"<span class='k {d['kind']}'></span>{d['label']}", f"{d['lo']:.2f}〜{d['hi']:.2f}", f"+{d['pk']:.0f} dB", f"{100*d['n']/nb:.0f}%", d["conf"]]) for d in dets)
    blerows = "".join(tr([k, ",".join(map(str, v["types"])), v["n"]]) for k, v in ble_rows[:15]) or tr(["—", "", ""])
    remrows = "".join(tr([f"{tt_:.2f} s", lab, hexc]) for tt_, lab, hexc in remote_rows[:40]) or tr(["—", "", ""])
    html = f"""<!doctype html><html lang=ja><head><meta charset=utf-8><title>受信レポート {os.path.basename(a.file)}</title>
<style>body{{font:14px/1.5 -apple-system,"Hiragino Sans",sans-serif;max-width:960px;margin:20px auto;padding:0 16px;color:#222}}
h1{{font-size:20px}}h2{{font-size:16px;border-bottom:2px solid #1f77b4;padding-bottom:3px;margin-top:28px}}
table{{border-collapse:collapse;width:100%;font-size:13px;margin:6px 0}}td,th{{border:1px solid #ddd;padding:4px 7px;text-align:left}}th{{background:#f4f6f8}}
.kv{{display:grid;grid-template-columns:auto 1fr;gap:2px 14px;font-size:13px}}.kv b{{color:#555;font-weight:600}}
img{{width:100%;border:1px solid #ddd;border-radius:4px}}.k{{display:inline-block;width:10px;height:10px;border-radius:2px;margin-right:5px;vertical-align:middle;border:1px solid #999}}
.wifi{{background:#4fc3f7}}.ble{{background:#ff7043}}.ble_adv{{background:#ff7043}}.lte{{background:#ba68c8}}.tone{{background:#ffd54f}}.internal{{background:#888}}.unknown{{background:#aaa}}
.note{{color:#777;font-size:12px}}</style></head><body>
<h1>受信レポート</h1>
<p class=note>ファイル {os.path.basename(a.file)} ／ 生成 {datetime.datetime.now().strftime('%Y-%m-%d %H:%M')} ／ ESP32-C3 受信（送信なし）</p>
<div class=summary><h2 style=border:none;margin-top:0>受信したもの</h2><ul>{summ_html}</ul></div>
<h2>1. 受信条件</h2><div class=kv>
<b>中心周波数(LO)</b><span>{f0/1e6:.3f} MHz（{'5/6 変換モード' if alt!='1' else '直接'}）</span>
<b>サンプルレート</b><span>{fs/1e6:g} MS/s（{mode}）</span>
<b>ゲイン指標</b><span>{g.get(EXT+':gain_index','?')}（約1 dB/step, フルスケール ±{fullscale}）</span>
<b>取得時間</b><span>{t[-1]:.1f} s / データブロック {len(full)}（空ブロック除く {len(blocks)-len(full)}）</span>
<b>時間カバー率</b><span>{g.get(EXT+':duty_cycle','?')}（ブロック間隔 {g.get(EXT+':block_pitch_us',{}).get('median','?') if isinstance(g.get(EXT+':block_pitch_us'),dict) else '?'} µs）</span>
<b>データ形式</b><span>{g.get('core:datatype')}（I+jQ = LO−RF, スペクトルはRFに対し鏡像）</span></div>
<h2>2. 品質指標</h2><div class=kv>
<b>雑音床</b><span>{med:.1f} dB（相対）</span>
<b>最大ピーク</b><span>+{snr:.0f} dB（床上）, 20·log10(peak/FS) = {20*np.log10(max(1,peak.max())/fullscale):.1f} dBFS</span>
<b>飽和ブロック</b><span>{int((peak>=fullscale*0.98).sum())} / {len(full)}（ピーク≥98%FS）</span>
<b>I/Q 雑音 rms</b><span>I {irms:.2f} / Q {qrms:.2f} LSB</span>
<b>DC オフセット</b><span>({dc.real:+.1f}, {dc.imag:+.1f}) LSB</span></div>
<h2>3. スペクトル</h2><img src="{img}"><img src="{img2}">
<h2>4. 検出された信号（床+8 dB 超, 出現=検出ブロック割合）</h2>
<table><tr><th>種類</th><th>RF (MHz)</th><th>強さ</th><th>出現</th><th>判定</th></tr>{detrows}</table>
<h2>5. 復号: BLE 広告（CRC OK）</h2>
<table><tr><th>AdvA</th><th>PDU種別</th><th>回数</th></tr>{blerows}</table>
<h2>6. 復号: リモコン{(' — プロファイル '+remote_name) if remote_name else '（一致プロファイルなし）'}</h2>
<table><tr><th>時刻</th><th>ボタン</th><th>コマンドバイト</th></tr>{remrows}</table>
<p class=note>生成: host/report.py（受信専用）。詳細な機器仕様は docs/remote_spec.md を参照。</p>
</body></html>"""
    out = a.out or (os.path.splitext(a.file)[0] + "_report.html")
    open(out, "w").write(html); print(f"wrote {out}  ({len(dets)} signals, {len(ble_rows)} BLE AdvA, {len(remote_rows)} remote presses)")
    if a.open: os.system(f"open '{out}'")

if __name__ == "__main__": main()
