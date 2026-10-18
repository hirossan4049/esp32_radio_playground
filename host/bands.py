#!/usr/bin/env python3
"""Known 2.4 GHz / cellular band table (Japan) and ESP32-C3 internal-spur table, shared by the host tools.
Mirrors web/dsp.js so the Python report and the browser viewer label signals the same way. Receive-only reference."""
BANDS = []
for ch in range(1, 14): BANDS.append(dict(name=f"Wi-Fi ch{ch}", kind="wifi", f0=2412 + 5 * (ch - 1), bw=20))
BANDS.append(dict(name="Wi-Fi ch14", kind="wifi", f0=2484, bw=22))
for ch, f0 in ((37, 2402), (38, 2426), (39, 2480)): BANDS.append(dict(name=f"BLE adv ch{ch}", kind="ble_adv", f0=f0, bw=2))
for k in range(37): f0 = 2404 + 2 * k if k < 11 else 2428 + 2 * (k - 11); BANDS.append(dict(name=f"BLE data ch{k}", kind="ble", f0=f0, bw=2))
for ch in range(11, 27): BANDS.append(dict(name=f"Zigbee/Thread ch{ch}", kind="zigbee", f0=2405 + 5 * (ch - 11), bw=2))
def _r(name, kind, lo, hi, note=""): BANDS.append(dict(name=name, kind=kind, f0=(lo + hi) / 2, bw=hi - lo, note=note))
_r("LTE Band 3 下り", "lte", 1805, 1880, "携帯基地局 下り"); _r("J-DECT", "dect", 1893.5, 1906.1, "コードレス電話")
_r("LTE Band 1 上り", "lte_ul", 1920, 1980, "端末 上り"); _r("LTE Band 1 下り", "lte", 2110, 2170, "携帯基地局 下り")
_r("ISM 2.4 GHz", "ism", 2400, 2483.5, "免許不要帯"); _r("TD-LTE Band 41", "lte_td", 2545, 2645, "携帯 TDD")
_r("LTE Band 7", "lte", 2620, 2690, "携帯基地局 下り")
INTERNAL_BB = [(0.0, 0.15, "DC / LO 漏れ"), (-2.0, 0.1, "内部スプリアス LO−2 MHz"), (-7.0, 0.1, "内部スプリアス LO−7 MHz"),
               (-11.0, 0.1, "2448 MHz チップスプリアス"), (5.0, 0.1, "2448 MHz 折り返し(16 MS/s系)")]
def bands_at(rf_mhz):
    return [b for b in BANDS if b["f0"] - b["bw"] / 2 <= rf_mhz <= b["f0"] + b["bw"] / 2]
def internal_at(bb_mhz, lo_mhz):
    for f, w, name in INTERNAL_BB:
        if abs(bb_mhz - f) <= w: return name
    k = round(lo_mhz / 40); bb = lo_mhz - 40 * k
    if abs(bb_mhz - bb) <= 0.05: return f"水晶 {40 * k:.0f} MHz 高調波"
    return None
