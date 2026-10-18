# ESP32-C3 を汎用 RF キャプチャ装置として使う（host/ ツール群）

目標の言い方は「どんな電波でも」ではなく **「同調可能な帯域で生 I/Q を取得できる」**。実測に基づく制約を先に書く。

## できること・できないこと（実測, docs/c3_findings.md）
| 項目 | 値 |
|---|---|
| 同調範囲（実効 LO） | 約 **1836〜2803 MHz**（PLL 2203〜2803 直接、1836〜2336 は CKGEN 5/6 モード。host が自動選択） |
| 瞬時帯域 | アナログ ±10〜11 MHz（20 MHz チャネル用フィルタ） |
| サンプルレート | 80 / 40 / 16 MS/s のスナップショット（16384 サンプル = 205 / 410 / 1024 µs）、4 M / 2 M / 1 M / 500 k / 250 k / 125 k / 62.5 kS/s の int8 デューティ運転 |
| 連続性 | **連続取得不可**。1.024 ms 取得 + 約 0.5 ms 欠落の繰り返し（デューティ運転で約 67%、USB 律速時はさらに低下）。スナップショットは毎秒十数ブロック（USB 64 KiB/ブロック） |
| 分解能 | 10 bit I/Q（スナップショットは int16 に展開、デューティ運転は箱型間引き後 int8） |
| 周波数規約 | I+jQ = LO − RF（スペクトルは RF に対して鏡像。`--rf` で表示を反転） |
| エイリアス | 16 MS/s 系は反エイリアス無しの 1/5 間引き。8〜11 MHz の成分が ±8 MHz 帯に折り返す。40/80 MS/s はアナログ帯域内で折り返しなし |
| 感度 | ゲイン指標 0〜60（約 1 dB/step、25→30 で約 10 dB の段）。−41 dBm のビーコンはゲイン 30 で −5 dBFS、40 以上で飽和 |
| 既知スプリアス | LO−3.000 MHz（40 MHz 水晶高調波）、LO−7 MHz、LO−11 MHz（2448 MHz）、DC 近傍、DC オフセット約 10 LSB |

## ツール
```
python3 host/capture.py --freq 2437 --rate 16000000 --gain 30 --duration 10 --output captures/wifi6
python3 host/capture.py --freq 2140 --rate 250000  --gain 50 --duration 5  --output captures/lte_b1   # 5/6 モード自動
python3 host/spectrum.py captures/wifi6.sigmf-meta --png captures/wifi6.png --rf
python3 host/sweep.py --start 1842 --stop 2803 --step 20 --gain 50 --out captures/sweep
python3 host/sigmf2wav.py captures/wifi6.sigmf-meta        # SDR++ File Source 用 WAV（baseband_<Hz>_… 命名）
```
システム python3（pyenv 3.11, numpy/scipy/matplotlib/pyserial 入り）で実行する。IDF の export.sh を読んだシェルでは python が IDF 環境に切り替わるので注意。

## SigMF の表現（欠落を隠さない）
- `.sigmf-data` は受信ブロックを連結したもの（`ci16_le` または `ci8`）。**連続ストリームではない**。
- `captures[]` に 1 ブロック = 1 エントリ。`core:sample_start`（データ内オフセット）、`core:datetime`、拡張 `esp32c3:t_us`（デバイス µs タイムスタンプ）、`esp32c3:seq`、`esp32c3:block_samples`。
  連続する `t_us` の差がブロック間隔で、`global` に `esp32c3:block_pitch_us`（min/median/max）と `esp32c3:duty_cycle` を記録。
- `global` には LO モード（1 または 5/6）、PLL 値、ゲイン、ダンプ制御語、エイリアス警告、ストリーム末尾の統計（送信/切り捨てフレーム数）を入れる。
- 読み手は `esp32sdr.sigmf.read_sigmf()` でブロック単位（t_us, complex 配列）に戻せる。GNU Radio は gr-sigmf、または File Source（ci8 = byte, ci16_le = short の I/Q インタリーブ）で `.sigmf-data` を直接読める。SDR++ は `sigmf2wav.py` の WAV。

## ファームウェア側のプロトコル（`STREAM2`）
`STREAM2 ctrl_hex nwords decim shift ms`。各ブロックをフレーム `"ESPB" | seq u32 | t_us u32 | nsamp u16 | fmt u8 | flags u8 | payload` で送る。
fmt 0 = int8 I/Q（間引き）、fmt 1 = int16 I/Q（全レート）。USB が追いつかない時はフレームの残りを捨て、次のヘッダの flags bit0 で知らせる。ホストはマジックで再同期する。
間引きモードは CSTREAM2 と同じ「取得 → 停止 → コピー → 再始動 → 間引き＋送信」で、コピー（約 0.5 ms）だけが欠落になる。

## ロードマップ上の位置
- Phase 1 Capture / Phase 2 SigMF: 本ツール群で完了（RX-C3-021 で実測）。
- Phase 3 Spectrum: `spectrum.py`（平均 PSD＋真の時刻軸ウォーターフォール）。
- Phase 4 Sweep: `sweep.py`（1 MHz ビンの床超過で信号帯を列挙）。
- Phase 5 連携: SigMF（GNU Radio）、WAV（SDR++）。ライブストリーミング（SoapySDR 等）は未実装。
- Phase 6 復調器: 実績として `host/dsss.py`（802.11b 1 Mbps ビーコン）。BLE/Zigbee は帯域（1〜2 MHz）とデューティ運転の相性次第で、1 ms 窓にパケット全体（BLE 広告 ≤ 376 µs）が収まるため見込みはある。
- 将来 S3 等で連続取得が得られても、`esp32sdr.device` の差し替えだけで解析層は再利用できる。


## イベントモード（取得率改善）
`capture.py --thr N`: ブロック平均 |I|+|Q|（10 bit 生値）が N 未満のブロックはデータを送らず 16 B のマーカーだけ送る。SigMF には `block_samples: 0` の capture エントリとして残り、
「取得したが無信号」と「取得していない時間」を区別できる。ゲイン 40 で雑音床は約 27、閾値 45 が目安。2402 MHz 10 s で時間軸の 86% を監視しつつデータは 53 ブロックだけだった（RX-C3-023）。
BLE 広告のようなバースト信号はこのモード＋4 MS/s で捕まえる（`host/ble.py`、RX-C3-022）。

## 時刻について
`esp32c3:t_us` はデバイス起動からの相対 µs（32 bit, 約 71 分で一周、ホストは差分で扱う）。`core:datetime` はホストが最初のフレーム受信時刻を基準に t_us で補間した UTC で、
USB 遅延分（数十 ms）の絶対誤差がある。別セッション間の絶対時刻比較には外部同期（PPS 等）が必要で、本装置には無い。

## WebSerial ビューア
`web/esp32sdr.html` を Chrome/Edge で開く（file:// で動作しない場合は `python3 -m http.server` で http://localhost から）。「接続」で ESP32-C3（USB VID 0x303a）を選び、LO・レート・ゲイン・閾値を指定して開始。
平均スペクトルとウォーターフォールを描画し、ブロック間の未取得時間は暗い行で表示、捕捉率・フレーム落ち・切り捨て数を表示する。5/6 モードの選択と I/Q レーン設定はページ側で行う（ファーム変更不要）。

## SoapySDR 連携の設計（未実装）
- `readStream()` は 1 ブロック = 1 バースト: `SOAPY_SDR_HAS_TIME | SOAPY_SDR_END_BURST` を立て、`timeNs` に t_us 由来の時刻を入れる。ブロック間はサンプルを返さない（ゼロ埋めしない）。
- 欠落をゼロ埋めして連続に見せるオプションは、復調器が連続データと誤認するため提供しない。必要なら受信側で `END_BURST` を見て明示的に行う。
- 周波数・ゲイン・レートの setter は `esp32sdr.device` の `tune/set_gain/plan_rate` と同じ制約（1836〜2803 MHz、離散レート）。
