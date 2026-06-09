#!/usr/bin/env python3
"""MQTT ↔ Serial bridge + autonomous GPS navigation for AGC golf cart.

Subscribes to:
  agc/{VEHICLE_ID}/command/manual          { direction, speed }
  agc/{VEHICLE_ID}/command/emergency_stop  {}
  agc/{VEHICLE_ID}/command/mission         { waypoints: [{lat, lon}, ...] }
  agc/{VEHICLE_ID}/command/home            {}

Publishes to:
  agc/{VEHICLE_ID}/telemetry   { thr, arm, steer, relay, deg, armed, ts }
  agc/{VEHICLE_ID}/status      { message }
"""

import json
import logging
import math
import os
import re
import threading
import time
from pathlib import Path

import paho.mqtt.client as mqtt
import serial
from dotenv import load_dotenv

load_dotenv(Path(__file__).parent / ".env")

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
log = logging.getLogger("agc-agent")

MQTT_BROKER   = os.environ.get("MQTT_BROKER", "localhost")
MQTT_PORT     = int(os.environ.get("MQTT_PORT", "1883"))
MQTT_USERNAME = os.environ.get("MQTT_USERNAME", "")
MQTT_PASSWORD = os.environ.get("MQTT_PASSWORD", "")
SERIAL_PORT   = os.environ.get("SERIAL_PORT", "/dev/ttyUSB0")
SERIAL_BAUD   = int(os.environ.get("SERIAL_BAUD", "115200"))
VEHICLE_ID    = os.environ.get("VEHICLE_ID", "vehicle")
PIXHAWK_PORT  = os.environ.get("PIXHAWK_PORT", "")
PIXHAWK_BAUD  = int(os.environ.get("PIXHAWK_BAUD", "115200"))

TOPIC_MANUAL         = f"agc/{VEHICLE_ID}/command/manual"
TOPIC_EMERGENCY_STOP = f"agc/{VEHICLE_ID}/command/emergency_stop"
TOPIC_MISSION        = f"agc/{VEHICLE_ID}/command/mission"
TOPIC_HOME           = f"agc/{VEHICLE_ID}/command/home"
TOPIC_TELEMETRY      = f"agc/{VEHICLE_ID}/telemetry"
TOPIC_STATUS         = f"agc/{VEHICLE_ID}/status"

# Navigation tuning
ARRIVAL_RADIUS_M  = 3.0   # metres — consider waypoint reached within this distance
HEADING_DEADBAND  = 10.0  # degrees — steer only outside this band
MAX_STEER_ANGLE   = 180.0  # degrees — disabled (always drive forward while steering)
NAV_LOOP_HZ       = 2     # navigation update rate
NAV_SPEED         = 0.1   # 0.0–1.0 throttle during autonomous navigation
MANUAL_SPEED      = 0.05  # 0.0–1.0 default manual speed (overridden by payload speed)

# Steering µs — 0.5 rotations = 180° → steer = 1500 ± (180/1.35) = 1500 ± 133
STEER_LEFT  = 1367   # -180° / -0.5 rotations
STEER_RIGHT = 1633   # +180° / +0.5 rotations
STEER_CTR   = 1500
ARM_ON      = 2000
ARM_OFF     = 1000

_ser: serial.Serial | None = None
_mqtt_client: mqtt.Client | None = None
_last_cmd_time: float = 0.0
_manual_cmd: tuple[str, str] | None = None  # (throttle_char, steer_char) of last manual command

# Steer position tracking — each 'a'/'d' = ±60µs from center 1500
# 0.5 rotations = 180° → 133µs → max 2 presses; use 3 for margin
STEER_MAX_PRESSES = 3
_steer_pos: int = 0  # current steer offset in presses from center (-N to +N)

# GPS state (populated by Pixhawk reader thread)
_gps: dict = {"lat": None, "lon": None, "heading_deg": None, "speed": None, "satellites": None, "fix_type": None}
_gps_lock = threading.Lock()

# Computed heading from GPS position delta (reliable when moving, compass-independent)
_computed_heading: float | None = None
_prev_gps_pos: tuple | None = None  # (lat, lon)

# Navigation state
_nav: dict = {"active": False, "waypoints": [], "wp_index": 0, "status": "idle"}
_nav_lock = threading.Lock()
_nav_event = threading.Event()
_emergency = threading.Event()  # set to immediately halt nav loop


# ---------------------------------------------------------------------------
# Serial helpers
# ---------------------------------------------------------------------------

def send_char(cmd: str) -> None:
    if _ser is None or not _ser.is_open:
        log.debug("no serial — would send: %r", cmd)
        return
    try:
        _ser.write(cmd.encode())
    except serial.SerialException as e:
        log.warning("serial write error: %s", e)


def drive(direction: str, steer: str = "x") -> None:
    """Send arm + steer + throttle chars to Arduino."""
    send_char("e")     # ARM
    send_char(steer)   # steer: 'a'=left, 'd'=right, 'x'=centre
    send_char(direction)  # 'w'=fwd, 's'=rev, ' '=stop


def stop_vehicle(disarm: bool = False) -> None:
    send_char(" ")     # stop throttle
    send_char("x")     # centre steer
    if disarm:
        send_char("q")  # DISARM


def direction_to_chars(direction: str) -> tuple[str, str]:
    """Return (throttle_char, steer_char) for a UI direction string."""
    match direction:
        case "forward":  return "w", "x"
        case "backward": return "s", "x"
        case "left":     return "w", "a"
        case "right":    return "w", "d"
        case "stop":     return " ", "x"
        case _:          return " ", "x"


def send_serial_cmd(thr: int, dir_us: int, steer: int, arm: int) -> None:
    global _steer_pos
    if arm < 1200:
        send_char(" "); send_char("q")
        _steer_pos = 0
        return
    send_char("e")
    if steer < STEER_CTR and _steer_pos > -STEER_MAX_PRESSES:
        send_char("a"); _steer_pos -= 1
    elif steer > STEER_CTR and _steer_pos < STEER_MAX_PRESSES:
        send_char("d"); _steer_pos += 1
    else:
        send_char("x"); _steer_pos = 0
    send_char("s" if dir_us > 1550 else ("w" if dir_us < 1450 else " "))


# ---------------------------------------------------------------------------
# GPS helpers
# ---------------------------------------------------------------------------

def _haversine(lat1, lon1, lat2, lon2) -> float:
    R = 6_371_000
    p = math.pi / 180
    a = (math.sin((lat2 - lat1) * p / 2) ** 2
         + math.cos(lat1 * p) * math.cos(lat2 * p) * math.sin((lon2 - lon1) * p / 2) ** 2)
    return 2 * R * math.asin(math.sqrt(a))


def _bearing(lat1, lon1, lat2, lon2) -> float:
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dlam = math.radians(lon2 - lon1)
    x = math.sin(dlam) * math.cos(phi2)
    y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlam)
    return (math.degrees(math.atan2(x, y)) + 360) % 360


def _norm_angle(a: float) -> float:
    return (a + 180) % 360 - 180


# ---------------------------------------------------------------------------
# Pixhawk GPS reader thread
# ---------------------------------------------------------------------------

def pixhawk_reader_loop() -> None:
    if not PIXHAWK_PORT:
        log.info("PIXHAWK_PORT not set — GPS reader disabled")
        return
    try:
        from pymavlink import mavutil
    except ImportError:
        log.warning("pymavlink not installed — GPS reader disabled")
        return

    global _computed_heading, _prev_gps_pos
    FIX_TYPES = {0:"No GPS",1:"No Fix",2:"2D Fix",3:"3D Fix",4:"DGPS",5:"RTK Float",6:"RTK Fixed"}
    last_log = 0.0
    LOG_INTERVAL = 0.5  # print GPS stats every 5 seconds

    while True:
        try:
            log.info("Connecting to Pixhawk on %s ...", PIXHAWK_PORT)
            conn = mavutil.mavlink_connection(PIXHAWK_PORT, baud=PIXHAWK_BAUD)
            conn.wait_heartbeat()
            log.info("Pixhawk connected")
            conn.mav.request_data_stream_send(
                conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_POSITION, 2, 1,
            )
            while True:
                msg = conn.recv_match(type=["GLOBAL_POSITION_INT", "GPS_RAW_INT"], blocking=True, timeout=5)
                if msg is None:
                    continue
                with _gps_lock:
                    if msg.get_type() == "GLOBAL_POSITION_INT":
                        new_lat = msg.lat / 1e7
                        new_lon = msg.lon / 1e7
                        _gps["lat"]         = new_lat
                        _gps["lon"]         = new_lon
                        _gps["heading_deg"] = msg.hdg / 100 if msg.hdg != 65535 else None
                        _gps["speed"]       = msg.vz / 100
                    elif msg.get_type() == "GPS_RAW_INT":
                        _gps["satellites"]  = msg.satellites_visible
                        _gps["fix_type"]    = FIX_TYPES.get(msg.fix_type, "Unknown")

                # compute heading from position delta when moving
                with _gps_lock:
                    spd = _gps["speed"]
                    lat = _gps["lat"]
                    lon = _gps["lon"]
                if lat and lon and spd is not None and abs(spd) > 0.05:
                    if _prev_gps_pos is not None:
                        prev_lat, prev_lon = _prev_gps_pos
                        dist = _haversine(prev_lat, prev_lon, lat, lon)
                        if dist > 0.5:  # at least 0.5m moved to compute reliably
                            _computed_heading = _bearing(prev_lat, prev_lon, lat, lon)
                    _prev_gps_pos = (lat, lon)
                else:
                    _prev_gps_pos = None  # reset when stopped so stale delta isn't used

                now = time.time()
                if now - last_log >= LOG_INTERVAL:
                    last_log = now
                    with _gps_lock:
                        lat      = _gps["lat"]
                        lon      = _gps["lon"]
                        hdg      = _gps["heading_deg"]
                        spd      = _gps["speed"]
                        sats     = _gps["satellites"]
                        fix      = _gps["fix_type"]
                    if lat is not None and lat != 0.0:
                        # navigation direction hint
                        with _nav_lock:
                            nav_active = _nav["active"]
                            waypoints  = _nav["waypoints"]
                            wp_index   = _nav["wp_index"]

                        nav_hint = ""
                        if nav_active and waypoints and wp_index < len(waypoints):
                            wp = waypoints[wp_index]
                            dist    = _haversine(lat, lon, wp["lat"], wp["lon"])
                            bearing = _bearing(lat, lon, wp["lat"], wp["lon"])
                            if hdg is not None:
                                err = _norm_angle(bearing - hdg)
                                turn = ("straight" if abs(err) < 10
                                        else ("turn RIGHT ▶" if err > 0 else "◀ turn LEFT"))
                                nav_hint = f"  →  {turn} ({err:+.1f}°)  dist={dist:.1f}m"
                            else:
                                nav_hint = f"  →  bearing={bearing:.1f}°  dist={dist:.1f}m"

                        log.info(
                            "GPS  lat=%.7f  lon=%.7f  compass=%s°  gps_hdg=%s°  spd=%sm/s  sats=%s  fix=%s%s",
                            lat, lon,
                            f"{hdg:.1f}" if hdg is not None else "?",
                            f"{_computed_heading:.1f}" if _computed_heading is not None else "?",
                            f"{spd:.1f}" if spd is not None else "?",
                            sats, fix, nav_hint,
                        )
                    else:
                        log.info("GPS  waiting for fix...  sats=%s  fix=%s", sats, fix)

        except Exception as e:
            log.warning("Pixhawk reader error: %s — retrying in 5s", e)
            time.sleep(5)


# ---------------------------------------------------------------------------
# Navigation loop thread
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
        send_serial_cmd(1500, 1500, STEER_CTR, ARM_ON)  # arm, hold
        time.sleep(0.5)

        interval = 1.0 / NAV_LOOP_HZ

        for wp_i, wp in enumerate(waypoints):
            target_lat = wp["lat"]
            target_lon = wp["lon"]

            with _nav_lock:
                _nav["wp_index"] = wp_i

            log.info("NAV waypoint %d/%d → (%.7f, %.7f)", wp_i + 1, len(waypoints), target_lat, target_lon)
            _publish_status(f"Heading to waypoint {wp_i + 1}/{len(waypoints)}")

            was_turning = False

            while True:
                if _emergency.is_set():
                    break
                with _nav_lock:
                    if not _nav["active"]:
                        break

                with _gps_lock:
                    cur_lat     = _gps["lat"]
                    cur_lon     = _gps["lon"]
                    cur_heading = _gps["heading_deg"]

                if cur_lat is None or cur_lon is None:
                    log.warning("NAV waiting for GPS fix...")
                    time.sleep(interval)
                    continue

                dist    = _haversine(cur_lat, cur_lon, target_lat, target_lon)
                bearing = _bearing(cur_lat, cur_lon, target_lat, target_lon)

                if dist <= ARRIVAL_RADIUS_M:
                    log.info("NAV waypoint %d reached (dist=%.1fm)", wp_i + 1, dist)
                    _publish_status(f"Waypoint {wp_i + 1} reached")
                    send_serial_cmd(1500, 1500, STEER_CTR, ARM_ON)  # stop, stay armed
                    time.sleep(0.5)
                    break

                spd_us = int(1500 + NAV_SPEED * 400)

                # Use GPS-computed heading (from position delta) — reliable when moving
                # Fall back to compass only if computed heading not yet available
                effective_heading = _computed_heading if _computed_heading is not None else cur_heading

                if effective_heading is not None:
                    heading_error = _norm_angle(bearing - effective_heading)
                    if heading_error > HEADING_DEADBAND:
                        steer = STEER_RIGHT
                    elif heading_error < -HEADING_DEADBAND:
                        steer = STEER_LEFT
                    else:
                        steer = STEER_CTR
                else:
                    heading_error = 0
                    steer = STEER_CTR  # go straight until we have a heading

                # Cut throttle while turning sharply
                if abs(heading_error) > MAX_STEER_ANGLE:
                    if not was_turning:
                        log.info("NAV sharp turn — holding throttle  err=%.1f°", heading_error)
                        was_turning = True
                    send_serial_cmd(1500, 1500, steer, ARM_ON)  # stop, keep steering
                else:
                    was_turning = False
                    send_serial_cmd(int(1500 + NAV_SPEED * 400), 1400, steer, ARM_ON)  # forward
                    log.info("NAV dist=%.1fm bearing=%.1f° heading=%s° err=%.1f°",
                             dist, bearing, cur_heading, heading_error)

                time.sleep(interval)

            with _nav_lock:
                if not _nav["active"]:
                    break

        # Mission complete or cancelled — only disarm if not immediately replaced by new mission
        with _nav_lock:
            active = _nav["active"]
            _nav["active"] = False
            _nav["status"] = "arrived" if active else "stopped"
        if not _nav_event.is_set():  # no new mission queued
            send_serial_cmd(1500, 1500, STEER_CTR, ARM_OFF)  # stop + disarm

        msg = "Mission complete" if active else "Navigation cancelled"
        log.info("NAV %s", msg)
        _publish_status(msg)


def _publish_status(message: str) -> None:
    if _mqtt_client:
        _mqtt_client.publish(TOPIC_STATUS, json.dumps({"message": message}))


# ---------------------------------------------------------------------------
# MQTT callbacks
# ---------------------------------------------------------------------------

def on_mqtt_message(client, userdata, msg: mqtt.MQTTMessage) -> None:
    try:
        payload = json.loads(msg.payload.decode())
    except Exception:
        return

    if msg.topic == TOPIC_MANUAL:
        # Cancel any active navigation first
        with _nav_lock:
            _nav["active"] = False

        global _manual_cmd
        direction = payload.get("direction", "stop")
        speed     = float(payload.get("speed", MANUAL_SPEED))
        spd_us    = int(1500 + max(0.0, min(1.0, speed)) * 400)

        match direction:
            case "forward":  thr, dir_us, steer = spd_us, 1400, STEER_CTR
            case "backward": thr, dir_us, steer = spd_us, 1600, STEER_CTR
            case "left":     thr, dir_us, steer = spd_us, 1400, STEER_LEFT
            case "right":    thr, dir_us, steer = spd_us, 1400, STEER_RIGHT
            case _:          thr, dir_us, steer = 1500,   1500, STEER_CTR

        if direction == "stop":
            _manual_cmd = None
            send_serial_cmd(1500, 1500, STEER_CTR, ARM_ON)
        else:
            _manual_cmd = (thr, dir_us, steer, ARM_ON)
            send_serial_cmd(thr, dir_us, steer, ARM_ON)
        log.info("manual %-8s spd=%.2f → CMD:%d,%d,%d,%d", direction, speed, thr, dir_us, steer, ARM_ON)

    elif msg.topic == TOPIC_MISSION:
        waypoints = payload.get("waypoints", [])
        if not waypoints:
            log.warning("mission received with no waypoints")
            return
        _emergency.clear()
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
        stop_vehicle(disarm=True)
        log.warning("EMERGENCY STOP")
        _publish_status("EMERGENCY STOP")

    elif msg.topic == TOPIC_HOME:
        _emergency.set()
        with _nav_lock:
            _nav["active"] = False
        stop_vehicle(disarm=True)
        log.info("HOME command — navigation cleared")
        _publish_status("Returning home")


# ---------------------------------------------------------------------------
# Arduino serial reader
# ---------------------------------------------------------------------------

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
            if line:
                log.info("arduino ← %s", line)

            m = _DEBUG_RE.match(line)
            if m and _mqtt_client:
                arm_us = int(m.group(2))
                telemetry = {
                    "thr":   int(m.group(1)),
                    "arm":   arm_us,
                    "steer": int(m.group(3)),
                    "relay": int(m.group(4)),
                    "deg":   float(m.group(5)),
                    "armed": arm_us > 1800,
                    "ts":    int(time.time() * 1000),
                }
                _mqtt_client.publish(TOPIC_TELEMETRY, json.dumps(telemetry))

            if line.startswith(("ARMED", "DISARM", "WARN", "MCP", "CAN", "KEYA")):
                if _mqtt_client:
                    _mqtt_client.publish(TOPIC_STATUS, json.dumps({"message": line}))

        except serial.SerialException as e:
            log.warning("serial read error: %s", e)
            time.sleep(1)
        except Exception as e:
            log.warning("serial_reader_loop: %s", e)


# ---------------------------------------------------------------------------
# Heartbeat loop — keeps Arduino from disarming during idle/nav pauses
# ---------------------------------------------------------------------------

def heartbeat_loop() -> None:
    """Repeat last manual command every 200ms so Arduino doesn't hit 500ms failsafe."""
    global _last_cmd_time
    while True:
        time.sleep(0.08)
        if _emergency.is_set():
            continue
        with _nav_lock:
            nav_active = _nav["active"]
        if not nav_active and (time.time() - _last_cmd_time) > 0.2:
            if _manual_cmd is not None:
                send_serial_cmd(*_manual_cmd)  # repeat last manual command
            else:
                send_serial_cmd(1500, 1500, STEER_CTR, ARM_ON)  # hold, stay armed
            _last_cmd_time = time.time()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    global _ser, _mqtt_client

    try:
        _ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
        log.info("serial open: %s @ %d baud", SERIAL_PORT, SERIAL_BAUD)
    except serial.SerialException as e:
        log.error("cannot open serial %s: %s — running without hardware", SERIAL_PORT, e)

    threading.Thread(target=serial_reader_loop,  daemon=True).start()
    threading.Thread(target=navigation_loop,     daemon=True).start()
    threading.Thread(target=pixhawk_reader_loop, daemon=True).start()
    threading.Thread(target=heartbeat_loop,      daemon=True).start()

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
    ])
    _mqtt_client = client
    log.info("MQTT connected %s:%d — vehicle_id=%s — waiting for commands",
             MQTT_BROKER, MQTT_PORT, VEHICLE_ID)
    client.loop_forever()


if __name__ == "__main__":
    main()
