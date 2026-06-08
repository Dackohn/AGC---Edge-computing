#!/usr/bin/env python3
"""MQTT ↔ Serial bridge for AGC golf cart (mini computer side).

Topics are namespaced per vehicle as agc/{VEHICLE_ID}/... to match the
multi-vehicle dashboard backend.

Subscribes to:
  agc/{VEHICLE_ID}/command/manual          { direction, speed }  — from web dashboard
  agc/{VEHICLE_ID}/command/emergency_stop  {}                    — from web dashboard

Translates direction+speed → µs values → CMD:<thr>,<dir>,<steer>,<arm>\\n to Arduino.

Reads Arduino serial output, parses debug lines, publishes to:
  agc/{VEHICLE_ID}/telemetry   { thr, arm, steer, relay, deg, armed, ts }
  agc/{VEHICLE_ID}/status      { message }
"""

import json
import logging
import os
import re
import threading
import time
from pathlib import Path

import paho.mqtt.client as mqtt
import serial
from dotenv import load_dotenv

# Load .env from the same directory as this script
load_dotenv(Path(__file__).parent / ".env")

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
log = logging.getLogger("agc-agent")

MQTT_BROKER = os.environ.get("MQTT_BROKER", "localhost")
MQTT_PORT = int(os.environ.get("MQTT_PORT", "1883"))
MQTT_USERNAME = os.environ.get("MQTT_USERNAME", "")
MQTT_PASSWORD = os.environ.get("MQTT_PASSWORD", "")
SERIAL_PORT = os.environ.get("SERIAL_PORT", "/dev/ttyUSB0")
SERIAL_BAUD = int(os.environ.get("SERIAL_BAUD", "115200"))
VEHICLE_ID = os.environ.get("VEHICLE_ID", "vehicle")

TOPIC_MANUAL = f"agc/{VEHICLE_ID}/command/manual"
TOPIC_EMERGENCY_STOP = f"agc/{VEHICLE_ID}/command/emergency_stop"
TOPIC_TELEMETRY = f"agc/{VEHICLE_ID}/telemetry"
TOPIC_STATUS = f"agc/{VEHICLE_ID}/status"

# Steering µs values → (steer - 1500) * 1.35 = motor degrees
STEER_LEFT = 1200  # -405° motor rotation
STEER_RIGHT = 1800  # +405° motor rotation
STEER_CTR = 1500  #    0° (straight)
ARM_ON = 2000  # >1800 → ARMED
ARM_OFF = 1000  # <1200 → DISARMED

_ser: serial.Serial | None = None
_mqtt_client: mqtt.Client | None = None


def direction_to_us(direction: str, speed: float) -> tuple[int, int, int, int]:
    """Map UI direction string + speed (0.0–1.0) to (thr, dir, steer, arm) µs."""
    spd = int(1500 + max(0.0, min(1.0, speed)) * 400)  # 1500–1900
    match direction:
        case "forward":
            return spd, 1600, STEER_CTR, ARM_ON
        case "backward":
            return spd, 1400, STEER_CTR, ARM_ON
        case "left":
            return spd, 1600, STEER_LEFT, ARM_ON
        case "right":
            return spd, 1600, STEER_RIGHT, ARM_ON
        case "stop":
            return 1500, 1500, STEER_CTR, ARM_ON
        case _:
            return 1500, 1500, STEER_CTR, ARM_ON


def send_serial_cmd(thr: int, dir_us: int, steer: int, arm: int) -> None:
    if _ser is None or not _ser.is_open:
        log.debug("no serial — would send CMD:%d,%d,%d,%d", thr, dir_us, steer, arm)
        return
    line = f"CMD:{thr},{dir_us},{steer},{arm}\n"
    try:
        _ser.write(line.encode())
    except serial.SerialException as e:
        log.warning("serial write error: %s", e)


def on_mqtt_message(client, userdata, msg: mqtt.MQTTMessage) -> None:
    try:
        payload = json.loads(msg.payload.decode())
    except Exception:
        return

    if msg.topic == TOPIC_MANUAL:
        direction = payload.get("direction", "stop")
        speed = float(payload.get("speed", 0.5))
        thr, dir_us, steer, arm = direction_to_us(direction, speed)
        send_serial_cmd(thr, dir_us, steer, arm)
        log.info(
            "manual %-8s spd=%.2f → CMD:%d,%d,%d,%d",
            direction,
            speed,
            thr,
            dir_us,
            steer,
            arm,
        )

    elif msg.topic == TOPIC_EMERGENCY_STOP:
        send_serial_cmd(1500, 1500, STEER_CTR, ARM_OFF)
        log.warning("EMERGENCY STOP sent to Arduino")


# Parses: "RC thr=1548 arm=2008 steer=1504 relay=984 -> deg=0.00"
#      or "SER thr=1700 arm=2000 steer=1500 relay=1600 -> deg=12.50"
_DEBUG_RE = re.compile(
    r"(?:RC|SER)\s+thr=(\d+)\s+arm=(\d+)\s+steer=(\d+)\s+relay=(\d+)\s+->\s+deg=([-\d.]+)"
)


def serial_reader_loop() -> None:
    while True:
        if _ser is None or not _ser.is_open:
            time.sleep(1)
            continue
        try:
            raw = _ser.readline()
            if not raw:
                continue
            line = raw.decode(errors="replace").strip()
            log.debug("arduino: %s", line)

            m = _DEBUG_RE.match(line)
            if m and _mqtt_client:
                arm_us = int(m.group(2))
                telemetry = {
                    "thr": int(m.group(1)),
                    "arm": arm_us,
                    "steer": int(m.group(3)),
                    "relay": int(m.group(4)),
                    "deg": float(m.group(5)),
                    "armed": arm_us > 1800,
                    "ts": int(time.time() * 1000),
                }
                _mqtt_client.publish(TOPIC_TELEMETRY, json.dumps(telemetry))

            # Forward status/warning lines to the dashboard
            if line.startswith(("ARMED", "DISARM", "WARN", "MCP", "CAN", "KEYA")):
                if _mqtt_client:
                    _mqtt_client.publish(TOPIC_STATUS, json.dumps({"message": line}))

        except serial.SerialException as e:
            log.warning("serial read error: %s", e)
            time.sleep(1)
        except Exception as e:
            log.warning("serial_reader_loop: %s", e)


def main() -> None:
    global _ser, _mqtt_client

    try:
        _ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
        log.info("serial open: %s @ %d baud", SERIAL_PORT, SERIAL_BAUD)
    except serial.SerialException as e:
        log.error(
            "cannot open serial %s: %s — running without hardware", SERIAL_PORT, e
        )

    threading.Thread(target=serial_reader_loop, daemon=True).start()

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.on_message = on_mqtt_message
    if MQTT_USERNAME:
        client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    if MQTT_PORT == 8883:
        client.tls_set()
    client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
    client.subscribe(
        [
            (TOPIC_MANUAL, 0),
            (TOPIC_EMERGENCY_STOP, 0),
        ]
    )
    _mqtt_client = client
    log.info(
        "MQTT connected %s:%d — vehicle_id=%s — waiting for commands",
        MQTT_BROKER,
        MQTT_PORT,
        VEHICLE_ID,
    )
    client.loop_forever()


if __name__ == "__main__":
    main()
