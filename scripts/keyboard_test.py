#!/usr/bin/env python3
"""
keyboard_test.py  —  send single-char commands to Arduino over USB Serial.

Keys (no Enter needed):
  e    ARM               q    DISARM / emergency stop
  w    forward + throttle up   s    reverse + throttle up
  a    steer left         d    steer right
  SPC  stop               x    centre steer
  ESC  quit

Usage:
  pip install pyserial readchar
  python keyboard_test.py             # auto-detect Arduino
  python keyboard_test.py COM12
  python keyboard_test.py /dev/ttyUSB0
"""

import sys
import threading
import time

import serial
import serial.tools.list_ports

try:
    import readchar
except ImportError:
    sys.exit("Run:  pip install pyserial readchar")


def find_port():
    for p in serial.tools.list_ports.comports():
        if any(
            k in (p.description or "").lower()
            for k in ("ch340", "ch341", "arduino", "uart")
        ):
            return p.device
    ports = serial.tools.list_ports.comports()
    return ports[0].device if ports else None


port = sys.argv[1] if len(sys.argv) > 1 else find_port()
if not port:
    sys.exit("No port found. Usage: python keyboard_test.py COM12")

print(f"Opening {port} @ 115200 …")
ser = serial.Serial(port, 115200, timeout=0.05)
time.sleep(2)  # Arduino resets on DTR; wait for boot

stop_flag = threading.Event()


def reader():
    while not stop_flag.is_set():
        line = ser.readline()
        if line:
            print(f"  << {line.decode(errors='replace').rstrip()}")


threading.Thread(target=reader, daemon=True).start()

print("""
  e = ARM     q = DISARM
  w = fwd     s = rev
  a = left    d = right
  SPC = stop  x = centre steer
  ESC = quit
""")

while True:
    k = readchar.readkey()
    if k == readchar.key.ESC:
        ser.write(b"q")  # disarm before exit
        break
    if k in "eEqQwWsSaAdD xX \x20":
        ser.write(k.encode())
        labels = {
            "e": "ARM",
            "E": "ARM",
            "q": "DISARM",
            "Q": "DISARM",
            "w": "FWD+thr↑",
            "W": "FWD+thr↑",
            "s": "REV+thr↑",
            "S": "REV+thr↑",
            "a": "steer←",
            "A": "steer←",
            "d": "steer→",
            "D": "steer→",
            " ": "STOP",
            "\x20": "STOP",
            "x": "steer ctr",
            "X": "steer ctr",
        }
        print(f"  >> {labels.get(k, k)}")

stop_flag.set()
ser.close()
print("Bye.")
