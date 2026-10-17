#!/bin/sh
cd "$(dirname "$0")/.." || exit 1
echo "START $(date +%H:%M:%S)"
python3 host/analog_flip_scan.py --block 62 --regs 0-15 --step 20000 2>&1 | grep -v esp_image
python3 host/analog_flip_scan.py --block 63 --regs 0-2 --step 20000 2>&1 | grep -v esp_image
python3 host/analog_flip_scan.py --block 63 --regs 6-15 --step 20000 2>&1 | grep -v esp_image
echo "ALLDONE $(date +%H:%M:%S)"
