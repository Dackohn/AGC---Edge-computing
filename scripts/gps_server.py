import math
import os
import threading
import time
from typing import List

import serial
import serial.tools.list_ports
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from pymavlink import mavutil
import uvicorn

# --- config (override via environment variables) ---

#run "Get-WmiObject -Query "SELECT * FROM Win32_PnPEntity WHERE Name LIKE '%(COM%'" | Select-Object Name" and set the ports for pixhawk and arduino

SERIAL_PORT   = os.getenv("PIXHAWK_PORT", "COM13")
BAUD_RATE     = int(os.getenv("PIXHAWK_BAUD", "115200"))



#start the see http://localhost:8000/docs usage



ARDUINO_PORT  = os.getenv("ARDUINO_PORT", "COM7")
ARDUINO_BAUD  = int(os.getenv("ARDUINO_BAUD", "115200"))

ARRIVAL_RADIUS_M   = 3.0   # metres — stop when this close
HEADING_DEADBAND   = 5.0   # degrees — steer only outside this band
NAV_LOOP_HZ        = 4     # navigation update rate (Hz)

# --- steering geometry (EG2028KSF, confirmed by measurement) ---
WHEELBASE_M        = 1.70  # metres (front axle → rear axle)
MAX_WHEEL_DEG      = 30.0  # degrees (1.5 steering-wheel rotations = 30° wheel angle)
STEERING_RATIO     = 18.0  # 1.5 turns × 360° / 30° = 18 : 1
# min turning radius derived from geometry: R = L / tan(wheel_angle)
import math as _math
MIN_TURN_RADIUS_M  = round(WHEELBASE_M / _math.tan(_math.radians(MAX_WHEEL_DEG)), 3)
# = 2.944 m

# --- navigation tuning ---
# PHASE 1: heading error > MAX_STEER_ANGLE → stop completely, turn in place
# PHASE 2: heading error <= MAX_STEER_ANGLE → drive (throttle scales with alignment)
MAX_STEER_ANGLE = 8    # degrees — must be aligned before moving (mirrors simulation)

# Steering pulses scale with heading error (proportional, like simulation)
# To calibrate: count how many 'a'/'d' Arduino presses = full lock (30° wheel)
# then set MAX_STEER_PULSES so full error sends roughly half that count per tick
HIGH_ERR_DEG     = 45   # degrees — sends MAX_STEER_PULSES at this error or above
MAX_STEER_PULSES = 3    # pulses per tick at full error
MIN_STEER_PULSES = 1    # pulses per tick near MAX_STEER_ANGLE

# Throttle scales with alignment
MAX_THROTTLE_PULSES = 2   # pulses at perfect alignment
MIN_THROTTLE_PULSES = 1   # pulses just past MAX_STEER_ANGLE

# --- shared GPS state ---
gps_data = {
    "lat":             None,
    "lon":             None,
    "alt_m":           None,
    "ground_speed_ms": None,
    "heading_deg":     None,
    "satellites":      None,
    "fix_type":        None,
    "timestamp":       None,
}
gps_lock = threading.Lock()

FIX_TYPES = {
    0: "No GPS", 1: "No Fix", 2: "2D Fix", 3: "3D Fix",
    4: "DGPS",   5: "RTK Float", 6: "RTK Fixed",
}

# --- navigation state ---
nav_state = {
    "active":         False,
    "waypoints":      [],     # list of {"lat": float, "lon": float}
    "wp_index":       0,      # current waypoint being driven toward
    "wp_total":       0,
    "target_lat":     None,
    "target_lon":     None,
    "distance_m":     None,
    "heading_error":  None,
    "status":         "idle", # idle | navigating | arrived | stopped
}
nav_lock  = threading.Lock()
nav_event = threading.Event()   # set to wake/restart the nav loop

# --- Arduino serial (lazy-opened when navigation starts) ---
arduino: serial.Serial | None = None
arduino_lock = threading.Lock()


def find_arduino_port():
    for p in serial.tools.list_ports.comports():
        if any(k in (p.description or "").lower()
               for k in ("ch340", "ch341", "arduino", "uart")):
            return p.device
    ports = serial.tools.list_ports.comports()
    return ports[0].device if ports else None


def arduino_send(cmd: str):
    global arduino
    with arduino_lock:
        try:
            if arduino is None or not arduino.is_open:
                port = ARDUINO_PORT if ARDUINO_PORT else find_arduino_port()
                if not port:
                    print("[NAV] No Arduino port found — command dropped")
                    return
                print(f"[NAV] Trying to open Arduino on {port}...")
                arduino = serial.Serial(port, ARDUINO_BAUD, timeout=0.05)
                time.sleep(2)
                print(f"[NAV] Opened Arduino on {port}")
            arduino.write(cmd.encode())
        except Exception as e:
            print(f"[NAV] Arduino error: {e}")
            arduino = None


# --- GPS reader thread ---
def gps_reader():
    print(f"Connecting to Pixhawk on {SERIAL_PORT}...")
    conn = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD_RATE)
    conn.wait_heartbeat()
    print("Connected. Waiting for GPS messages...")

    conn.mav.request_data_stream_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION, 2, 1
    )

    while True:
        msg = conn.recv_match(
            type=["GLOBAL_POSITION_INT", "GPS_RAW_INT"],
            blocking=True, timeout=5
        )
        if msg is None:
            print("No GPS message received (timeout)")
            continue

        with gps_lock:
            if msg.get_type() == "GLOBAL_POSITION_INT":
                gps_data["lat"]             = msg.lat / 1e7
                gps_data["lon"]             = msg.lon / 1e7
                gps_data["alt_m"]           = msg.alt / 1000
                gps_data["ground_speed_ms"] = msg.vz / 100
                gps_data["heading_deg"]     = msg.hdg / 100 if msg.hdg != 65535 else None
                gps_data["timestamp"]       = time.strftime("%H:%M:%S")
            elif msg.get_type() == "GPS_RAW_INT":
                gps_data["satellites"] = msg.satellites_visible
                gps_data["fix_type"]   = FIX_TYPES.get(msg.fix_type, "Unknown")

        with gps_lock:
            lat, lon, alt = gps_data["lat"], gps_data["lon"], gps_data["alt_m"]
        if lat is not None:
            print(
                f"[{gps_data['timestamp']}] "
                f"Lat: {lat:.7f}  Lon: {lon:.7f}  Alt: {alt:.1f}m  "
                f"Sats: {gps_data['satellites']}  Fix: {gps_data['fix_type']}"
            )
        else:
            print(
                f"[{gps_data['timestamp']}] Waiting for fix...  "
                f"Sats: {gps_data['satellites']}  Fix: {gps_data['fix_type']}"
            )


# --- haversine helpers ---
def haversine_distance(lat1, lon1, lat2, lon2) -> float:
    """Return distance in metres between two GPS coordinates."""
    R = 6_371_000
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi  = math.radians(lat2 - lat1)
    dlam  = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def bearing_to(lat1, lon1, lat2, lon2) -> float:
    """Return compass bearing (0–360°) from point 1 → point 2."""
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dlam = math.radians(lon2 - lon1)
    x = math.sin(dlam) * math.cos(phi2)
    y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlam)
    return (math.degrees(math.atan2(x, y)) + 360) % 360


def normalise_angle(a: float) -> float:
    """Wrap angle to (−180, +180]."""
    return (a + 180) % 360 - 180


# --- navigation loop thread ---
def navigation_loop():
    while True:
        nav_event.wait()        # sleep until a /route is issued
        nav_event.clear()

        with nav_lock:
            if not nav_state["active"]:
                continue
            waypoints = list(nav_state["waypoints"])

        if not waypoints:
            continue

        print(f"[NAV] Starting route with {len(waypoints)} waypoints")
        arduino_send('e')       # ARM
        time.sleep(0.5)

        interval    = 1.0 / NAV_LOOP_HZ
        was_turning = False
        wp_index    = 0

        while wp_index < len(waypoints):
            # check if navigation was cancelled
            with nav_lock:
                if not nav_state["active"]:
                    break

            target_lat = waypoints[wp_index]["lat"]
            target_lon = waypoints[wp_index]["lon"]

            with nav_lock:
                nav_state["wp_index"]  = wp_index
                nav_state["target_lat"] = target_lat
                nav_state["target_lon"] = target_lon

            with gps_lock:
                cur_lat     = gps_data["lat"]
                cur_lon     = gps_data["lon"]
                cur_heading = gps_data["heading_deg"]

            if cur_lat is None or cur_lon is None:
                print("[NAV] Waiting for GPS fix...")
                time.sleep(interval)
                continue

            dist    = haversine_distance(cur_lat, cur_lon, target_lat, target_lon)
            bearing = bearing_to(cur_lat, cur_lon, target_lat, target_lon)

            with nav_lock:
                nav_state["distance_m"] = round(dist, 2)

            if dist <= ARRIVAL_RADIUS_M:
                print(f"[NAV] Waypoint {wp_index + 1}/{len(waypoints)} reached — dist={dist:.1f}m")
                wp_index += 1
                if wp_index >= len(waypoints):
                    arduino_send(' ')   # STOP
                    time.sleep(0.2)
                    arduino_send('q')   # DISARM
                    with nav_lock:
                        nav_state["active"] = False
                        nav_state["status"] = "arrived"
                    print("[NAV] Route complete.")
                else:
                    print(f"[NAV] Advancing to waypoint {wp_index + 1}/{len(waypoints)}")
                continue

            if cur_heading is None:
                # no heading from Pixhawk yet — drive blind, centre wheel
                arduino_send('x')
                for _ in range(MIN_THROTTLE_PULSES):
                    arduino_send('w')
                time.sleep(interval)
                continue

            heading_error = normalise_angle(bearing - cur_heading)
            with nav_lock:
                nav_state["heading_error"] = round(heading_error, 1)

            abs_err = abs(heading_error)

            # ── PHASE 1: large error → stop and turn in place ─────────────
            if abs_err > MAX_STEER_ANGLE:
                if not was_turning:
                    arduino_send(' ')   # STOP throttle relay
                    was_turning = True

                # proportional steering pulses: more pulses for bigger error
                ratio  = min(abs_err / HIGH_ERR_DEG, 1.0)
                pulses = round(MIN_STEER_PULSES + ratio * (MAX_STEER_PULSES - MIN_STEER_PULSES))
                cmd    = 'd' if heading_error > 0 else 'a'
                for _ in range(pulses):
                    arduino_send(cmd)

                print(
                    f"[NAV] TURNING  wp={wp_index + 1}/{len(waypoints)}  "
                    f"dist={dist:.1f}m  err={heading_error:+.1f}°  pulses={pulses}{cmd}"
                )

            # ── PHASE 2: aligned → steer gently and drive ─────────────────
            else:
                was_turning = False

                # fine steering inside deadband → centre; outside → 1 pulse
                if heading_error > HEADING_DEADBAND:
                    arduino_send('d')
                elif heading_error < -HEADING_DEADBAND:
                    arduino_send('a')
                else:
                    arduino_send('x')   # centre

                # throttle scales with alignment: perfect → max pulses
                align_ratio    = 1.0 - (abs_err / MAX_STEER_ANGLE) ** 2
                throttle_pulses = max(MIN_THROTTLE_PULSES,
                                      round(align_ratio * MAX_THROTTLE_PULSES))
                for _ in range(throttle_pulses):
                    arduino_send('w')

                print(
                    f"[NAV] DRIVING  wp={wp_index + 1}/{len(waypoints)}  "
                    f"dist={dist:.1f}m  err={heading_error:+.1f}°  throttle={throttle_pulses}"
                )
            time.sleep(interval)

        # ensure vehicle is stopped after loop exits
        arduino_send(' ')
        arduino_send('q')
        print("[NAV] Navigation ended.")


# --- FastAPI app ---
app = FastAPI(
    title="AGC Autonomous Golf Cart",
    description=(
        "GPS navigation server for the autonomous yard vehicle.\n\n"
        "- Reads live GPS from Pixhawk via MAVLink\n"
        "- Sends steering/throttle commands to Arduino Nano over serial\n"
        "- POST `/route` with a waypoint list to start autonomous navigation\n"
        "- POST `/stop_nav` for emergency stop"
    ),
    version="2.0.0",
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


class Waypoint(BaseModel):
    lat: float
    lon: float


class RouteRequest(BaseModel):
    waypoints: List[Waypoint]

    model_config = {
        "json_schema_extra": {
            "example": {
                "waypoints": [
                    {"lat": 46.87436020327258, "lon": 29.23794936310003},
                    {"lat": 46.89256329928182, "lon": 29.201891342736843},
                ]
            }
        }
    }


@app.get("/gps", summary="Live GPS data", tags=["Telemetry"])
def get_gps():
    """Returns the latest GPS position, heading, speed, satellites and fix type from the Pixhawk."""
    with gps_lock:
        return dict(gps_data)


@app.post("/route", summary="Navigate a multi-waypoint route", tags=["Navigation"])
def route(req: RouteRequest):
    """
    Start autonomous navigation through a list of GPS waypoints.
    The vehicle will ARM, drive through each waypoint in order and stop at the last one.
    Sending a new /route while navigating replaces the current route immediately.
    """
    if len(req.waypoints) < 1:
        return {"error": "at least one waypoint required"}

    wps = [{"lat": wp.lat, "lon": wp.lon} for wp in req.waypoints]

    with nav_lock:
        nav_state["active"]        = True
        nav_state["waypoints"]     = wps
        nav_state["wp_index"]      = 0
        nav_state["wp_total"]      = len(wps)
        nav_state["target_lat"]    = wps[0]["lat"]
        nav_state["target_lon"]    = wps[0]["lon"]
        nav_state["status"]        = "navigating"
        nav_state["distance_m"]    = None
        nav_state["heading_error"] = None
    nav_event.set()
    return {
        "status":    "route started",
        "waypoints": len(wps),
        "first":     wps[0],
        "last":      wps[-1],
    }


@app.post("/stop_nav", summary="Emergency stop", tags=["Navigation"])
def stop_nav():
    """Immediately stops the vehicle and disarms the motor. Cancels any active navigation."""
    with nav_lock:
        nav_state["active"] = False
        nav_state["status"] = "stopped"
    arduino_send(' ')
    arduino_send('q')
    return {"status": "navigation stopped"}


@app.get("/nav_status", summary="Navigation status", tags=["Navigation"])
def nav_status():
    """Returns current navigation state including waypoint progress."""
    with nav_lock:
        return {
            "active":        nav_state["active"],
            "status":        nav_state["status"],
            "wp_index":      nav_state["wp_index"],
            "wp_total":      nav_state["wp_total"],
            "target_lat":    nav_state["target_lat"],
            "target_lon":    nav_state["target_lon"],
            "distance_m":    nav_state["distance_m"],
            "heading_error": nav_state["heading_error"],
        }


@app.get("/", include_in_schema=False)
def root():
    return {
        "status":    "running",
        "docs":      "/docs",
        "endpoints": ["/gps", "/route", "/stop_nav", "/nav_status"],
    }


# --- entry point ---
if __name__ == "__main__":
    threading.Thread(target=gps_reader,      daemon=True).start()
    threading.Thread(target=navigation_loop, daemon=True).start()

    print("Server starting at http://localhost:8000")
    uvicorn.run(app, host="0.0.0.0", port=8000)
