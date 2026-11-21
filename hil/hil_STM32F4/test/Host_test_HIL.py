#!/usr/bin/env python3
# Copyright (c) 2026 Diptopal Basu (embeddedfreedom)
# Licensed under the MIT License
"""
Hardware-in-the-Loop (HIL) DMA Loopback Test at 921,600 Baud
Target   : STM32F411 bare-metal USART1 (PA9/PA10 via DMA2)
Protocol : 12-byte symmetrical packet (IEEE 754 Little-Endian)
"""

import os
import sys
import time
import struct
import termios

# Locate framework/common (single source of truth for MDCS paths)
_HERE = os.path.dirname(os.path.abspath(__file__))
for _ in range(8):
    _CAND = os.path.join(_HERE, "framework", "common")
    if os.path.isdir(_CAND):
        sys.path.insert(0, _CAND)
        break
    _HERE = os.path.dirname(_HERE)
from mdcs_config import MDCS_HIL_TEST_PORT

# ===============================================================================
# CONFIGURATION
# ===============================================================================
DEFAULT_PORT = MDCS_HIL_TEST_PORT
BAUD_RATE    = 921600
PACKET_SIZE  = 12

SYNC_HOST_TO_MCU = bytes([0xAA, 0x55])
SYNC_MCU_TO_HOST = bytes([0x55, 0xAA])

# ===============================================================================
# SERIAL PORT WRAPPER (Standard Linux termios, zero pip dependencies)
# ===============================================================================
class RawSerial:
    def __init__(self, port, baudrate=921600):
        self.port = port
        print(f"[*] Opening {port} at {baudrate} baud (8-N-1)...")
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        
        # Configure raw 8-N-1 mode
        attrs = termios.tcgetattr(self.fd)
        attrs[0] = 0                           # iflag: raw input, no translation
        attrs[1] = 0                           # oflag: raw output
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL  # 8N1, enable rx
        attrs[3] = 0                           # lflag: non-canonical, no echo
        
        baud_const = getattr(termios, f"B{baudrate}")
        attrs[4] = baud_const                  # ispeed
        attrs[5] = baud_const                  # ospeed
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        self.flush()

    def flush(self):
        termios.tcflush(self.fd, termios.TCIOFLUSH)

    def write(self, data):
        return os.write(self.fd, data)

    def read_exact(self, num_bytes, timeout_sec=0.2):
        buf = bytearray()
        start = time.perf_counter()
        while len(buf) < num_bytes and (time.perf_counter() - start) < timeout_sec:
            try:
                chunk = os.read(self.fd, num_bytes - len(buf))
                if chunk:
                    buf.extend(chunk)
                else:
                    time.sleep(0.0001)
            except BlockingIOError:
                time.sleep(0.0001)
        return bytes(buf)

    def close(self):
        os.close(self.fd)

# ===============================================================================
# PACKET HELPERS
# ===============================================================================
def compute_checksum(data_bytes):
    cs = 0
    for b in data_bytes:
        cs ^= b
    return cs

def build_packet(pos1=1.0, pos2=2.0, cmd=0x01):
    payload = struct.pack("<BBffB", 0xAA, 0x55, pos1, pos2, cmd)
    checksum = compute_checksum(payload)
    return payload + bytes([checksum])

# ===============================================================================
# TEST EXECUTION
# ===============================================================================
def main():
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    
    try:
        ser = RawSerial(port, BAUD_RATE)
    except Exception as e:
        print(f"[ERROR] Failed to open port {port}: {e}")
        print("Tip: Check device with 'ls -l /dev/ttyUSB*' and verify dialout permissions.")
        sys.exit(1)

    time.sleep(0.05)  # Let adapter stabilize
    ser.flush()

    print("\n" + "=" * 65)
    print(" 1. SINGLE PACKET TEST (Round-Trip Latency Verification)")
    print("=" * 65)

    test_pkt = build_packet(pos1=1.0, pos2=2.0, cmd=0x01)
    print(f"TX ({len(test_pkt)} bytes) : {' '.join(f'{b:02X}' for b in test_pkt)}")
    print(f"             [Sync: AA 55 | pos1: 1.00 | pos2: 2.00 | cmd: 01 | CS: {test_pkt[-1]:02X}]")

    t_start = time.perf_counter()
    ser.write(test_pkt)
    rx_data = ser.read_exact(PACKET_SIZE, timeout_sec=5.0)
    t_roundtrip_us = (time.perf_counter() - t_start) * 1e6

    if not rx_data:
        print("\n[FAIL] No response received within 100 ms timeout!")
        print("Things to check:")
        print("  1. Press RESET button on STM32 board to re-arm DMA2 Stream 5.")
        print("  2. Verify PA9 (TX) connects to adapter RX, PA10 (RX) to adapter TX.")
        print("  3. Verify GND wire is connected.")
        ser.close()
        return

    print(f"RX ({len(rx_data)} bytes) : {' '.join(f'{b:02X}' for b in rx_data)}")
    print(f"Latency      : {t_roundtrip_us:.1f} microseconds (Wire transfer @ 921600)")

    # Validate Framing
    if rx_data[:2] == SYNC_MCU_TO_HOST or rx_data[:2] == SYNC_HOST_TO_MCU:
        print("\n[SUCCESS] Sync bytes valid! DMA RX -> Processing -> DMA TX completed!")
        out1, out2, status, cs = struct.unpack("<ffBB", rx_data[2:])
        print(f"Decoded Data : Output1={out1:.2f}, Output2={out2:.2f}, Status=0x{status:02X}, CS=0x{cs:02X}")
    else:
        print(f"\n[WARNING] Received bytes, but sync magic mismatch: {' '.join(f'{b:02X}' for b in rx_data[:2])}")

    # ===========================================================================
    # 2. CONTINUOUS STREAMING TEST
    # ===========================================================================
    print("\n" + "=" * 65)
    print(" 2. CONTINUOUS 1 kHz STREAMING TEST (Press Ctrl+C to stop)")
    print("=" * 65)
    
    count = 0
    errors = 0
    t_accum = 0.0

    try:
        while True:
            pkt = build_packet(pos1=count * 0.01, pos2=-count * 0.01, cmd=0x01)
            t0 = time.perf_counter()
            ser.write(pkt)
            resp = ser.read_exact(PACKET_SIZE, timeout_sec=0.01)
            dt_us = (time.perf_counter() - t0) * 1e6

            if len(resp) == PACKET_SIZE:
                count += 1
                t_accum += dt_us
                if count % 100 == 0:
                    avg_us = t_accum / 100
                    print(f"Packets: {count:05d} | Avg Latency: {avg_us:.1f} us | Status: OK")
                    t_accum = 0.0
            else:
                errors += 1
                print(f"[MISMATCH] Incomplete packet (got {len(resp)} bytes)")

            time.sleep(0.001)  # 1 ms period (1 kHz rate)

    except KeyboardInterrupt:
        print(f"\n[*] Stopped. Total Packets: {count}, Errors: {errors}")

    ser.close()

if __name__ == "__main__":
    main()