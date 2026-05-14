import math
import os
import threading
import time

import serial
import serial.tools.list_ports
from fastapi import FastAPI
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
HEADING_DEADBAND   = 10.0  # degrees — steer only outside this band
NAV_LOOP_HZ        = 2     # navigation update rate

# --- tuning ---
# Each Arduino 'a'/'d' press = one steering increment.
# Raise STEER_PULSES (e.g. 3) to turn the wheel more per tick.
STEER_PULSES    = 1   # 1–5 recommended

# Each Arduino 'w' press = one throttle increment.
# Raise THROTTLE_PULSES (e.g. 3) to go faster.
THROTTLE_PULSES = 1   # 1–5 recommended

# If heading error exceeds this angle the wheel is at max rotation —
# throttle relay is cut (STOP) until the car is aligned again.
MAX_STEER_ANGLE = 70   # degrees — cut throttle beyond this (e.g. 60–120)

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
    "active":       False,
    "target_lat":   None,
    "target_lon":   None,
    "distance_m":   None,
    "heading_error": None,
    "status":       "idle",   # idle | navigating | arrived | stopped
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
        nav_event.wait()        # sleep until a /goto is issued
        nav_event.clear()

        with nav_lock:
            if not nav_state["active"]:
                continue
            target_lat = nav_state["target_lat"]
            target_lon = nav_state["target_lon"]

        print(f"[NAV] Starting navigation to {target_lat}, {target_lon}")
        arduino_send('e')       # ARM
        time.sleep(0.5)

        interval = 1.0 / NAV_LOOP_HZ
        was_turning = False     # track state to send STOP only once

        while True:
            # check if navigation was cancelled
            with nav_lock:
                if not nav_state["active"]:
                    break

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
                print(f"[NAV] Arrived! Distance: {dist:.1f}m")
                arduino_send(' ')   # STOP
                time.sleep(0.2)
                arduino_send('q')   # DISARM
                with nav_lock:
                    nav_state["active"] = False
                    nav_state["status"] = "arrived"
                break

            # steer toward target
            if cur_heading is not None:
                heading_error = normalise_angle(bearing - cur_heading)
                with nav_lock:
                    nav_state["heading_error"] = round(heading_error, 1)

                if heading_error > HEADING_DEADBAND:
                    for _ in range(STEER_PULSES):
                        arduino_send('d')   # steer right
                elif heading_error < -HEADING_DEADBAND:
                    for _ in range(STEER_PULSES):
                        arduino_send('a')   # steer left
                else:
                    arduino_send('x')   # centre steer

            # cut throttle while wheel is at max rotation
            if cur_heading is not None and abs(nav_state.get("heading_error") or 0) > MAX_STEER_ANGLE:
                if not was_turning:
                    arduino_send(' ')   # STOP relay once on entry — turning in place
                    was_turning = True
                print(
                    f"[NAV] TURNING  dist={dist:.1f}m  bearing={bearing:.1f}°  "
                    f"heading={cur_heading}°  err={nav_state['heading_error']}°"
                )
            else:
                was_turning = False
                for _ in range(THROTTLE_PULSES):
                    arduino_send('w')   # forward
                print(
                    f"[NAV] dist={dist:.1f}m  bearing={bearing:.1f}°  "
                    f"heading={cur_heading}°  err={nav_state['heading_error']}°"
                )
            time.sleep(interval)

        # ensure vehicle is stopped after loop exits (arrival or cancel)
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
        "- Send `/goto` with a target coordinate to start autonomous navigation\n"
        "- Send `/stop_nav` for emergency stop"
    ),
    version="1.0.0",
)


class GotoRequest(BaseModel):
    lat: float
    lon: float

    model_config = {
        "json_schema_extra": {
            "example": {"lat": 47.0621841, "lon": 28.8676635}
        }
    }


@app.get("/gps", summary="Live GPS data", tags=["Telemetry"])
def get_gps():
    """Returns the latest GPS position, heading, speed, satellites and fix type from the Pixhawk."""
    with gps_lock:
        return dict(gps_data)


@app.post("/goto", summary="Navigate to GPS coordinate", tags=["Navigation"])
def goto(req: GotoRequest):
    """
    Start autonomous navigation to the given GPS coordinate.
    The vehicle will ARM, steer toward the target and stop automatically within 3 m.
    Sending a new `/goto` while navigating replaces the current target.
    """
    with nav_lock:
        nav_state["active"]        = True
        nav_state["target_lat"]    = req.lat
        nav_state["target_lon"]    = req.lon
        nav_state["status"]        = "navigating"
        nav_state["distance_m"]    = None
        nav_state["heading_error"] = None
    nav_event.set()
    return {"status": "navigation started", "target": {"lat": req.lat, "lon": req.lon}}


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
    """Returns current navigation state: active flag, target, distance remaining and heading error."""
    with nav_lock:
        return dict(nav_state)


@app.get("/", include_in_schema=False)
def root():
    return {
        "status": "running",
        "docs": "/docs",
        "endpoints": ["/gps", "/goto", "/stop_nav", "/nav_status"],
    }


# --- entry point ---
if __name__ == "__main__":
    threading.Thread(target=gps_reader,      daemon=True).start()
    threading.Thread(target=navigation_loop, daemon=True).start()

    print("Server starting at http://localhost:8000")
    uvicorn.run(app, host="0.0.0.0", port=8000)
