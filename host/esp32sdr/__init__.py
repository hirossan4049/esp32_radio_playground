"""esp32sdr: host library for the ESP32-C3 RF capture firmware (firmware/).

RF acquisition layer (device.py) and analysis layer (sigmf.py, spectrum tools) are separate so the
host tools keep working when a chip with continuous capture (e.g. ESP32-S3) replaces the C3.
"""
from .device import Device, RATES, lo_plan, LO_MIN_HZ, LO_MAX_HZ
from .sigmf import SigMFWriter, read_sigmf
