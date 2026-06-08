#!/usr/bin/env python3
"""
keyboard_drive.py — Drive the golf cart from a keyboard via USB Serial.
Place: AGC---Edge-computing/agent/keyboard_drive.py

Controls
--------
  E           ARM (must do this first!)
  Q           DISARM / emergency stop
  W / S       Throttle up / down
  D / A       Direction: Forward / Reverse
  Space       Neutral (coast, zeroes throttle)
  ← / →       Steer left / right
  X           Zero throttle + neutral
  Ctrl-C      Quit (auto-disarms)

Usage
-----
  pip install pyserial readchar
  python agent/keyboard_drive.py                 # auto-detect port
  python agent/keyboard_drive.py /dev/ttyUSB0
  python agent/keyboard_drive.py COM12
"""

import sys
import threading
import time

import serial
import serial.tools.list_ports

try:
    import readchar
except ImportError:
    print("Install deps first:  pip install pyserial readchar")
    sys.exit(1)

# ── tunables ──────────────────────────────────────────────────────────────────
BAUD = 115200
SEND_HZ = 20  # CMD sends per second
THR_STEP = 40  # µs per W/S press
STEER_STEP = 60  # µs per ←/→ press
THR_CENTER = 1500
STEER_CENTER = 1500
DIR_FWD = 1600
DIR_REV = 1400
DIR_NEUTRAL = 1500
ARM_ON = 2000
ARM_OFF = 1000
# ─────────────────────────────────────────────────────────────────────────────


def find_arduino():
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        if any(k in desc for k in ("arduino", "ch340", "ch341", "uart", "usb serial")):
            return p.device
    ports = [p.device for p in serial.tools.list_ports.comports()]
    return ports[0] if ports else None


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# shared state
state = {
    "thr": THR_CENTER,
    "dir": DIR_NEUTRAL,
    "steer": STEER_CENTER,
    "arm": ARM_OFF,
    "running": True,
}
lock = threading.Lock()


def sender_thread(ser):
    interval = 1.0 / SEND_HZ
    while True:
        with lock:
            if not state["running"]:
                break
            t, d, s, a = state["thr"], state["dir"], state["steer"], state["arm"]
        try:
            ser.write(f"CMD:{t},{d},{s},{a}\n".encode())
        except serial.SerialException:
            break
        time.sleep(interval)


def reader_thread(ser):
    while True:
        with lock:
            if not state["running"]:
                break
        try:
            raw = ser.readline()
            if raw:
                print(f"\r  [ARD] {raw.decode(errors='replace').rstrip():<60}")
                print_status(state)
        except serial.SerialException:
            break


def print_status(s):
    armed_str = "ARMED   " if s["arm"] > 1800 else "DISARMED"
    dir_str = "FWD" if s["dir"] > 1550 else ("REV" if s["dir"] < 1450 else "NEU")
    thr_pct = abs(s["thr"] - THR_CENTER) * 100 // 400
    deg = (s["steer"] - STEER_CENTER) * 1.35
    print(
        f"\r  [{armed_str}] thr={thr_pct:3d}%  dir={dir_str}  steer={deg:+.0f}°    ",
        end="",
        flush=True,
    )


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_arduino()
    if not port:
        print("No serial port found. Usage: python keyboard_drive.py /dev/ttyUSB0")
        sys.exit(1)

    print(f"Connecting to {port} @ {BAUD} …")
    try:
        ser = serial.Serial(port, BAUD, timeout=0.1)
    except serial.SerialException as e:
        print(f"Cannot open port: {e}")
        sys.exit(1)
    time.sleep(2)  # wait for Arduino reset after DTR toggle

    print("""
╔══════════════════════════════════════════╗
║      GOLF CART KEYBOARD CONTROL          ║
╠══════════════════════════════════════════╣
║  E         ARM (do this first)           ║
║  Q         DISARM / emergency stop       ║
║  W / S     Throttle up / down            ║
║  D / A     Forward / Reverse             ║
║  Space     Neutral (coast)               ║
║  ← →       Steer left / right            ║
║  X         Zero throttle + neutral       ║
║  Ctrl-C    Quit                          ║
╚══════════════════════════════════════════╝
""")

    threading.Thread(target=sender_thread, args=(ser,), daemon=True).start()
    threading.Thread(target=reader_thread, args=(ser,), daemon=True).start()

    try:
        while True:
            key = readchar.readkey()
            with lock:
                s = dict(state)

            if key in (readchar.key.CTRL_C, readchar.key.CTRL_D):
                break
            elif key in ("e", "E"):
                s["arm"] = ARM_ON
            elif key in ("q", "Q"):
                s["arm"] = ARM_OFF
                s["thr"] = THR_CENTER
                s["dir"] = DIR_NEUTRAL
                s["steer"] = STEER_CENTER
            elif key in ("w", "W"):
                s["thr"] = clamp(s["thr"] + THR_STEP, 1100, 1900)
            elif key in ("s", "S"):
                s["thr"] = clamp(s["thr"] - THR_STEP, 1100, 1900)
            elif key in ("d", "D"):
                s["dir"] = DIR_FWD
            elif key in ("a", "A"):
                s["dir"] = DIR_REV
            elif key == " ":
                s["thr"] = THR_CENTER
                s["dir"] = DIR_NEUTRAL
            elif key in ("x", "X"):
                s["thr"] = THR_CENTER
                s["dir"] = DIR_NEUTRAL
            elif key == readchar.key.LEFT:
                s["steer"] = clamp(s["steer"] - STEER_STEP, 1100, 1900)
            elif key == readchar.key.RIGHT:
                s["steer"] = clamp(s["steer"] + STEER_STEP, 1100, 1900)

            with lock:
                state.update(s)
            print_status(s)

    except KeyboardInterrupt:
        pass
    finally:
        print("\nDisarming and closing …")
        with lock:
            state.update(
                {
                    "arm": ARM_OFF,
                    "thr": THR_CENTER,
                    "dir": DIR_NEUTRAL,
                    "steer": STEER_CENTER,
                    "running": False,
                }
            )
        time.sleep(0.4)
        ser.close()
        print("Done.")


if __name__ == "__main__":
    main()
