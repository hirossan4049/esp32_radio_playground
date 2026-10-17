# ESP32-C3 受信経路の調査メモ

更新日: 2026-10-17
対象ボード: ESP32-C3 (QFN32, revision v0.4, 内蔵 4MB フラッシュ, 40 MHz 水晶), Espressif 純正 USB Serial/JTAG (303a:1001),
USB シリアル番号 = MAC E8:F6:0A:16:BD:84。開発ボード型式は不明（基板上の PCB アンテナ使用）。
ツール: ESP-IDF v5.2.1 (`~/Documents/fun/pg/fun/esp-idf`), riscv32-esp-elf gcc 13.2.0, esptool 5.4.0, macOS。
ファームウェア: `firmware/`（本リポジトリ）。ホスト: `host/rxprobe.py`。

## 区分の凡例
- [実測] このボードで測定・確認した事実
- [静的] ライブラリ逆アセンブル等から読み取った事実（ハード未検証）
- [文献] 他者 README / 文書の主張
- [仮説] 未検証

## 計画との差分
- 計画書の第一候補は ESP32-S3 だが、接続されている実機は **ESP32-C3**。既知の I/Q 受信実績（eSpDR / esp32-sdr-trx）は S3 のみ [文献]。
  C3 で同等の経路が使えるかは本調査で確認する（S3 の結果を横滑りさせない）。

## 既知事例の要点（Phase 0 読解）
- esp32-sdr-trx / eSpDR (S3) [文献]:
  - 受信 LO の法則: `LO = 960 MHz + 30 MHz × W/65536`（W: 24bit SDM 語、40 MHz 水晶）。PLL が実際にロックするのは約 2210–2790 MHz。
  - 5/6 変換モード（アナログ CKGEN ブロック 0x65 reg0 bit4）で実効 LO = PLL×5/6 → 1.84–2.33 GHz。**これ以外の分周比は未発見**。
    470–710 MHz には PLL 座標の約 1/4〜1/3 が必要で、既知の仕組みでは届かない。
  - ダンプエンジン: 0x60033D5C（制御: bit31 run, bit17 circular, bit16 16Msps, bits0..15 リング長）, 0x60033D60（書込インデックス）,
    0x60033D90（バイトセレクタ, 0x000C2040 で 32bit 語全体）, 0x600C101C（バンク選択）。語形式 I=bits0..9, Q=bits10..19（10bit 2 の補数）。
  - 16 Msps ダンプ → CIC/16 → FIR で 250/333 ksps。USB Full-Speed (0.87 MB/s) が転送上限。16 Msps のフル取得は FPGA 必須。
- ESP32-DATV (C3) [文献]: libphy の `set_chanfreq`/`phy_set_freq` で 2200–2800 MHz に同調（S3 と同じ 30 MHz×(32+W/65536) 法則）。
  `dactrig` 用 DAC 再生エンジン 0x60033D64、SRAM バンク 0x3FCB0000、所有権レジスタ 0x600C1020。送信のみ。

## C3 librftest / libphy の静的解析 [静的]
- `adctrig` は C3 の librftest にも存在し、S3 と同じ 0x60033D5C / 0x60033D60 を使う。SRAM 所有権は 0x600C1020（`(v & ~7) | 2` のあと `| 8`）。
- `adctrig(a0..a7, stack)` の制御語: `((a4<<17)&0x20000) | (a1<<28) | ((!(a3&1))<<16) | (a2<<20) | (a0+1)`。
  run (bit31) をセット後、**bit19 をセット→クリア（パルス）** して書込みを開始。bit18 が完了フラグ。
- rftest の `get_rx_buffer1` は `adctrig(0x2000, 0, 0, 1, 0, 0, 0, 0, 0)` を呼ぶ（モード 0）。
- `phy_chan_dump_cfg` は 0x6001CD0C を触る（S3 と同一アドレス）。C3 ライブラリに 0x60033D90 への参照は無い。
- AGC 強制: 0x6001C02C bits24..30 = ゲイン選択, bit23 = force（S3 と同一）。

## 実測 [実測]
### RX-C3-001: ダンプエンジンの起動（2026-10-17）
- 条件: LO 2437 MHz（set_chanfreq(2437)）, PCB アンテナ, RX 強制 (`force_rx_gain(0,40,0)`), Wi-Fi ドライバ起動（mode NULL, promiscuous）。
- 自前シーケンス（bit19 パルス無し）: run ビットは立つが 1 語も書かれず、インデックス 0 のまま。0x60033084 bit30/29 を立てても変化無し。
- `adctrig(8192,0,0,1,0,0,0,0,0)`（ライブラリ呼出）: 8193 語が 0x3FCB0000 に書かれた。
  - 語は `0x2d2d2d2d` のように **同一バイトの 4 回繰返し**（170 種）。S3 で「バイトセレクタ未設定」のときに観測された症状と一致 → C3 でも
    セレクタ相当の設定が必要（アドレス未特定）。現状は 8bit 分の情報しか得られていない。
  - 書込完了まで 238,997 CPU サイクル（160 MHz, phy_printf を含む）→ 下限 5.5 MS/s。正確な値は bit19 パルス版で測定する。
- 解釈: unverified（RF 入力 ON/OFF・周波数変更・LO 変更の対照実験は未実施）。
