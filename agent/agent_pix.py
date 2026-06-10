#!/usr/bin/env python3
"""DroneKit-based Pixhawk agent with MQTT bridge for the AGC autonomous yard vehicle.

Connects to the Pixhawk via DroneKit, streams telemetry over MQTT, and accepts
drive / mission commands that are forwarded directly to the flight controller.
Designed for ArduRover (ground vehicle) but also works with ArduCopter.

MQTT topics subscribed:
  agc/{VEHICLE_ID}/command/manual          { direction, speed }        RC override
  agc/{VEHICLE_ID}/command/emergency_stop  {}                          disarm + HOLD
  agc/{VEHICLE_ID}/command/mission         { waypoints:[{lat,lon},...]} GUIDED multi-wp
  agc/{VEHICLE_ID}/command/home            {}                          RTL mode
  agc/{VEHICLE_ID}/command/mode            { mode: "GUIDED"|"MANUAL"|"RTL"|... }

MQTT topics published:
  agc/{VEHICLE_ID}/telemetry   lat, lon, alt, heading, groundspeed, airspeed,
                               battery_voltage, battery_level, armed, mode, ts
  agc/{VEHICLE_ID}/status      { message }
"""

import json
import logging
import math
import os
import threading
import time
from pathlib import Path

import collections
import collections.abc
for _attr in ("Callable", "Mapping", "MutableMapping", "MutableSequence",
              "Sequence", "Iterable", "Iterator", "MutableSet"):
    if not hasattr(collections, _attr):
        setattr(collections, _attr, getattr(collections.abc, _attr))

import paho.mqtt.client as mqtt
from dotenv import load_dotenv

try:
    from dronekit import connect as dk_connect, VehicleMode, LocationGlobalRelative
    _DRONEKIT_OK = True
except ImportError as _dk_err:
    _DRONEKIT_OK = False
    _dk_err_msg  = str(_dk_err)

load_dotenv(Path(__file__).parent / ".env")

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
log = logging.getLogger("agc-pix")

# ---------------------------------------------------------------------------
# Config (all overridable via environment / .env)
# ---------------------------------------------------------------------------

MQTT_BROKER      = os.environ.get("MQTT_BROKER",      "localhost")
MQTT_PORT        = int(os.environ.get("MQTT_PORT",     "1883"))
MQTT_USERNAME    = os.environ.get("MQTT_USERNAME",     "")
MQTT_PASSWORD    = os.environ.get("MQTT_PASSWORD",     "")
VEHICLE_ID       = os.environ.get("VEHICLE_ID",        "vehicle")
PIXHAWK_PORT     = os.environ.get("PIXHAWK_PORT",      "/dev/ttyUSB0")
PIXHAWK_BAUD     = int(os.environ.get("PIXHAWK_BAUD",  "115200"))
TELEMETRY_HZ     = float(os.environ.get("TELEMETRY_HZ", "0.5"))
CRUISE_ALT_M     = float(os.environ.get("CRUISE_ALT_M", "0"))
ARRIVAL_RADIUS_M = float(os.environ.get("ARRIVAL_RADIUS_M", "3.0"))
ARM_TIMEOUT_S    = float(os.environ.get("ARM_TIMEOUT_S", "15.0"))

# RC channel mapping (ArduRover: ch1=steer, ch3=throttle)
RC_STEER_CH    = int(os.environ.get("RC_STEER_CH",    "1"))
RC_THROTTLE_CH = int(os.environ.get("RC_THROTTLE_CH", "3"))

# PWM limits
PWM_CENTER  = 1500
PWM_MIN     = 1000
PWM_MAX     = 2000
STEER_LEFT  = 1367
STEER_RIGHT = 1633
STEER_CTR   = 1500

# ---------------------------------------------------------------------------
# MQTT topics
# ---------------------------------------------------------------------------

TOPIC_MANUAL         = f"agc/{VEHICLE_ID}/command/manual"
TOPIC_EMERGENCY_STOP = f"agc/{VEHICLE_ID}/command/emergency_stop"
TOPIC_MISSION        = f"agc/{VEHICLE_ID}/command/mission"
TOPIC_HOME           = f"agc/{VEHICLE_ID}/command/home"
TOPIC_MODE           = f"agc/{VEHICLE_ID}/command/mode"
TOPIC_TELEMETRY      = f"agc/{VEHICLE_ID}/telemetry"
TOPIC_STATUS         = f"agc/{VEHICLE_ID}/status"

# ---------------------------------------------------------------------------
# Shared state
# ---------------------------------------------------------------------------

_vehicle     = None          # dronekit Vehicle object
_mqtt_client = None
_vehicle_lock = threading.Lock()

_nav: dict = {"active": False, "waypoints": [], "wp_index": 0, "status": "idle"}
_nav_lock  = threading.Lock()
_nav_event = threading.Event()
_emergency = threading.Event()


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _publish_status(message: str) -> None:
    if _mqtt_client:
        _mqtt_client.publish(TOPIC_STATUS, json.dumps({"message": message}))
    log.info("STATUS: %s", message)


def _haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    R = 6_371_000
    p = math.pi / 180
    a = (math.sin((lat2 - lat1) * p / 2) ** 2
         + math.cos(lat1 * p) * math.cos(lat2 * p) * math.sin((lon2 - lon1) * p / 2) ** 2)
    return 2 * R * math.asin(math.sqrt(a))


def _set_rc_override(throttle_pwm: int, steer_pwm: int) -> None:
    with _vehicle_lock:
        v = _vehicle
    if v is None:
        return
    try:
        v.channels.overrides = {
            str(RC_THROTTLE_CH): throttle_pwm,
            str(RC_STEER_CH):    steer_pwm,
        }
    except Exception as e:
        log.warning("RC override error: %s", e)


def _clear_rc_override() -> None:
    with _vehicle_lock:
        v = _vehicle
    if v is None:
        return
    try:
        v.channels.overrides = {}
    except Exception as e:
        log.warning("clear RC override error: %s", e)


def _direction_to_rc(direction: str, speed: float) -> None:
    spd_fwd = int(PWM_CENTER + speed * (PWM_MAX - PWM_CENTER))
    spd_rev = int(PWM_CENTER - speed * (PWM_CENTER - PWM_MIN))
    match direction:
        case "forward":  _set_rc_override(spd_fwd, STEER_CTR)
        case "backward": _set_rc_override(spd_rev, STEER_CTR)
        case "left":     _set_rc_override(spd_fwd, STEER_LEFT)
        case "right":    _set_rc_override(spd_fwd, STEER_RIGHT)
        case _:          _clear_rc_override()


def _stop_vehicle(disarm: bool = False) -> None:
    _clear_rc_override()
    if disarm:
        _disarm_vehicle()


def _set_mode(mode_name: str) -> bool:
    with _vehicle_lock:
        v = _vehicle
    if v is None:
        return False
    try:
        v.mode = VehicleMode(mode_name)
        deadline = time.time() + 5
        while v.mode.name != mode_name and time.time() < deadline:
            time.sleep(0.1)
        return v.mode.name == mode_name
    except Exception as e:
        log.warning("set_mode(%s) error: %s", mode_name, e)
        return False


def _arm_vehicle() -> bool:
    with _vehicle_lock:
        v = _vehicle
    if v is None:
        return False
    try:
        if not v.is_armable:
            log.warning("vehicle not armable (GPS fix?)")
        v.armed = True
        deadline = time.time() + ARM_TIMEOUT_S
        while not v.armed and time.time() < deadline:
            time.sleep(0.5)
        return v.armed
    except Exception as e:
        log.warning("arm error: %s", e)
        return False


def _disarm_vehicle() -> None:
    with _vehicle_lock:
        v = _vehicle
    if v is None:
        return
    try:
        v.armed = False
    except Exception as e:
        log.warning("disarm error: %s", e)


# ---------------------------------------------------------------------------
# Telemetry publisher thread
# ---------------------------------------------------------------------------

def telemetry_loop() -> None:
    interval = 1.0 / TELEMETRY_HZ
    while True:
        time.sleep(interval)
        with _vehicle_lock:
            v = _vehicle
        if v is None or _mqtt_client is None:
            continue
        try:
            loc  = v.location.global_relative_frame
            batt = v.battery
            telemetry = {
                "lat":             loc.lat,
                "lon":             loc.lon,
                "alt":             loc.alt,
                "heading":         v.heading,
                "groundspeed":     v.groundspeed,
                "airspeed":        v.airspeed,
                "battery_voltage": batt.voltage if batt else None,
                "battery_level":   batt.level   if batt else None,
                "armed":           v.armed,
                "mode":            v.mode.name,
                "ts":              int(time.time() * 1000),
            }
            _mqtt_client.publish(TOPIC_TELEMETRY, json.dumps(telemetry))
        except Exception as e:
            log.debug("telemetry error: %s", e)


# ---------------------------------------------------------------------------
# Navigation loop thread — GUIDED mode multi-waypoint missions
# ---------------------------------------------------------------------------

def navigation_loop() -> None:
    while True:
        _nav_event.wait()
        _nav_event.clear()

        with _nav_lock:
            if not _nav["active"]:
                continue
            waypoints = list(_nav["waypoints"])
            _nav["wp_index"] = 0

        log.info("NAV starting — %d waypoints", len(waypoints))
        _publish_status(f"Navigation started — {len(waypoints)} waypoints")

        if not _set_mode("GUIDED"):
            _publish_status("NAV failed: could not enter GUIDED mode")
            with _nav_lock:
                _nav["active"] = False
                _nav["status"] = "stopped"
            continue

        if not _arm_vehicle():
            _publish_status("NAV failed: vehicle did not arm")
            with _nav_lock:
                _nav["active"] = False
                _nav["status"] = "stopped"
            continue

        for wp_i, wp in enumerate(waypoints):
            if _emergency.is_set():
                break
            with _nav_lock:
                if not _nav["active"]:
                    break
                _nav["wp_index"] = wp_i

            target_lat = wp["lat"]
            target_lon = wp["lon"]
            target_alt = wp.get("alt", CRUISE_ALT_M)
            target_loc = LocationGlobalRelative(target_lat, target_lon, target_alt)

            log.info("NAV waypoint %d/%d → (%.7f, %.7f)", wp_i + 1, len(waypoints), target_lat, target_lon)
            _publish_status(f"Heading to waypoint {wp_i + 1}/{len(waypoints)}")

            with _vehicle_lock:
                v = _vehicle
            if v is None:
                break
            v.simple_goto(target_loc)

            while True:
                if _emergency.is_set():
                    break
                with _nav_lock:
                    if not _nav["active"]:
                        break

                with _vehicle_lock:
                    v = _vehicle
                if v is None:
                    break

                try:
                    cur = v.location.global_relative_frame
                    if cur.lat is None or cur.lon is None:
                        time.sleep(0.5)
                        continue
                    dist = _haversine(cur.lat, cur.lon, target_lat, target_lon)
                except Exception:
                    time.sleep(0.5)
                    continue

                if dist <= ARRIVAL_RADIUS_M:
                    log.info("NAV waypoint %d reached (dist=%.1fm)", wp_i + 1, dist)
                    _publish_status(f"Waypoint {wp_i + 1} reached")
                    break

                log.debug("NAV dist=%.1fm to wp %d", dist, wp_i + 1)
                time.sleep(0.5)

            with _nav_lock:
                if not _nav["active"]:
                    break

        # Mission complete / cancelled
        with _nav_lock:
            was_active = _nav["active"]
            _nav["active"] = False
            _nav["status"] = "arrived" if was_active else "stopped"

        if was_active and not _emergency.is_set():
            _set_mode("HOLD")
            _publish_status("Mission complete")
            log.info("NAV mission complete")
        else:
            _publish_status("Navigation cancelled")
            log.info("NAV cancelled")


# ---------------------------------------------------------------------------
# MQTT callbacks
# ---------------------------------------------------------------------------

def on_mqtt_message(client, userdata, msg: mqtt.MQTTMessage) -> None:
    try:
        payload = json.loads(msg.payload.decode())
    except Exception:
        return

    if msg.topic == TOPIC_MANUAL:
        with _nav_lock:
            _nav["active"] = False

        direction = payload.get("direction", "stop")
        speed     = max(0.0, min(1.0, float(payload.get("speed", 0.5))))

        _direction_to_rc(direction, speed)
        log.info("manual %-8s spd=%.2f", direction, speed)

    elif msg.topic == TOPIC_MISSION:
        waypoints = payload.get("waypoints", [])
        if not waypoints:
            log.warning("mission received with no waypoints")
            return
        _emergency.clear()
        _clear_rc_override()
        with _nav_lock:
            _nav["active"]    = True
            _nav["waypoints"] = waypoints
            _nav["status"]    = "navigating"
        _nav_event.set()
        log.info("mission received — %d waypoints", len(waypoints))

    elif msg.topic == TOPIC_EMERGENCY_STOP:
        _emergency.set()
        with _nav_lock:
            _nav["active"] = False
        _stop_vehicle(disarm=True)
        log.warning("EMERGENCY STOP")
        _publish_status("EMERGENCY STOP")

    elif msg.topic == TOPIC_HOME:
        _emergency.set()
        with _nav_lock:
            _nav["active"] = False
        _stop_vehicle()
        _set_mode("RTL")
        log.info("HOME — RTL mode activated")
        _publish_status("Returning to launch (RTL)")

    elif msg.topic == TOPIC_MODE:
        mode_name = payload.get("mode", "")
        if mode_name:
            ok = _set_mode(mode_name.upper())
            status = f"Mode set to {mode_name}" if ok else f"Failed to set mode {mode_name}"
            _publish_status(status)


# ---------------------------------------------------------------------------
# DroneKit connection thread — auto-reconnects on drop
# ---------------------------------------------------------------------------

def vehicle_connection_loop() -> None:
    global _vehicle
    if not _DRONEKIT_OK:
        log.error("dronekit not available (%s) — run: pip install future dronekit", _dk_err_msg)
        return

    while True:
        try:
            log.info("Connecting to Pixhawk on %s (baud=%d) ...", PIXHAWK_PORT, PIXHAWK_BAUD)
            v = dk_connect(PIXHAWK_PORT, baud=PIXHAWK_BAUD, wait_ready=True,
                           heartbeat_timeout=30)
            log.info("Pixhawk connected — firmware: %s  armed: %s  mode: %s",
                     v.version, v.armed, v.mode.name)
            _publish_status(f"Pixhawk connected — mode: {v.mode.name}")

            with _vehicle_lock:
                _vehicle = v

            # Wait until connection drops
            while True:
                try:
                    _ = v.armed  # lightweight heartbeat check
                    time.sleep(1)
                except Exception:
                    break

            log.warning("Pixhawk connection lost — reconnecting in 5s")
            with _vehicle_lock:
                _vehicle = None
            try:
                v.close()
            except Exception:
                pass

        except Exception as e:
            log.warning("Pixhawk connect error: %s — retrying in 5s", e)

        time.sleep(5)





# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    global _mqtt_client

    threading.Thread(target=vehicle_connection_loop, daemon=True).start()
    threading.Thread(target=navigation_loop,         daemon=True).start()
    threading.Thread(target=telemetry_loop,          daemon=True).start()

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.on_message = on_mqtt_message
    if MQTT_USERNAME:
        client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    if MQTT_PORT == 8883:
        client.tls_set()

    client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
    client.subscribe([
        (TOPIC_MANUAL,         0),
        (TOPIC_EMERGENCY_STOP, 0),
        (TOPIC_MISSION,        0),
        (TOPIC_HOME,           0),
        (TOPIC_MODE,           0),
    ])
    _mqtt_client = client
    log.info("MQTT connected %s:%d — vehicle_id=%s", MQTT_BROKER, MQTT_PORT, VEHICLE_ID)
    client.loop_forever()


if __name__ == "__main__":
    main()
