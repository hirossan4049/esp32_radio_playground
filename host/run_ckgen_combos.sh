#!/bin/sh
# RX-C3-009: CKGEN reg0 combinations with the 5/6 bit (bit4) set, 10 MHz PLL step.
cd "$(dirname "$0")/.." || exit 1
for ck in 70 71 72 73 74 75 76 77 78 79 7a 7b 7c 7d 7e 7f 53 33 f3 d3 b3; do
  echo "################ CKGEN reg0 = 0x$ck  $(date +%H:%M:%S)"
  python3 host/ratio_scan.py --ck $ck --step 10000 --tag combo 2>&1 | grep -v esp_image | grep -E "^===|^  r=|failed"
done
echo "DONE $(date +%H:%M:%S)"
