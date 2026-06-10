#!/usr/bin/env python3
"""Parse a QGC .waypoints file and send it to the vehicle via MQTT mission command.

Usage:
    python send_mission.py MISSION_1.waypoints
    python send_mission.py MISSION_1.waypoints --direct   # upload straight to Pixhawk via DroneKit
"""

import argparse
import json
import os
import sys
import time
from pathlib import Path

from dotenv import load_dotenv

load_dotenv(Path(__file__).parent / ".env")

MQTT_BROKER   = os.environ.get("MQTT_BROKER",   "localhost")
MQTT_PORT     = int(os.environ.get("MQTT_PORT",  "1883"))
MQTT_USERNAME = os.environ.get("MQTT_USERNAME",  "")
MQTT_PASSWORD = os.environ.get("MQTT_PASSWORD",  "")
VEHICLE_ID    = os.environ.get("VEHICLE_ID",     "vehicle")
PIXHAWK_PORT  = os.environ.get("PIXHAWK_PORT",   "/dev/ttyUSB0")
PIXHAWK_BAUD  = int(os.environ.get("PIXHAWK_BAUD", "115200"))


def parse_waypoints(filepath: str) -> list[dict]:
    """Parse QGC WPL 110 format; skip the home waypoint (index 0, current=1)."""
    waypoints = []
    with open(filepath) as f:
        header = f.readline().strip()
        if not header.startswith("QGC WPL"):
            raise ValueError(f"Not a QGC waypoints file: {header!r}")
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split("\t")
            if len(parts) < 12:
                continue
            # columns: index current frame command p1 p2 p3 p4 lat lon alt autocontinue
            current = int(parts[1])
            lat     = float(parts[8])
            lon     = float(parts[9])
            alt     = float(parts[10])
            if current == 1:
                continue  # skip home waypoint
            waypoints.append({"lat": lat, "lon": lon, "alt": alt})
    return waypoints


def send_via_mqtt(waypoints: list[dict]) -> None:
    import paho.mqtt.client as mqtt

    connected = [False]

    def on_connect(client, userdata, flags, rc, properties=None):
        connected[0] = True

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.on_connect = on_connect
    if MQTT_USERNAME:
        client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    if MQTT_PORT == 8883:
        client.tls_set()

    client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
    client.loop_start()

    deadline = time.time() + 10
    while not connected[0] and time.time() < deadline:
        time.sleep(0.1)
    if not connected[0]:
        print("ERROR: could not connect to MQTT broker")
        sys.exit(1)

    topic   = f"agc/{VEHICLE_ID}/command/mission"
    payload = json.dumps({"waypoints": waypoints})
    client.publish(topic, payload, qos=1)
    time.sleep(0.5)  # give broker time to deliver
    client.loop_stop()
    client.disconnect()
    print(f"Sent {len(waypoints)} waypoints to {topic}")


def send_via_dronekit(waypoints: list[dict]) -> None:
    import collections, collections.abc
    for attr in ("Callable", "Mapping", "MutableMapping", "MutableSequence",
                 "Sequence", "Iterable", "Iterator", "MutableSet"):
        if not hasattr(collections, attr):
            setattr(collections, attr, getattr(collections.abc, attr))

    from dronekit import connect, Command, VehicleMode
    from pymavlink import mavutil

    print(f"Connecting to Pixhawk on {PIXHAWK_PORT} ...")
    vehicle = connect(PIXHAWK_PORT, baud=PIXHAWK_BAUD, wait_ready=True)
    print(f"Connected — mode: {vehicle.mode.name}  armed: {vehicle.armed}")

    cmds = vehicle.commands
    cmds.clear()

    for wp in waypoints:
        cmd = Command(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 1,           # current=0, autocontinue=1
            0, 0, 0, 0,     # params 1-4
            wp["lat"], wp["lon"], wp["alt"],
        )
        cmds.add(cmd)

    cmds.upload()
    print(f"Uploaded {len(waypoints)} waypoints to Pixhawk")

    vehicle.mode = VehicleMode("AUTO")
    print("Mode set to AUTO — vehicle will execute the mission")
    vehicle.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="Send a .waypoints mission to the vehicle")
    parser.add_argument("file", help="Path to .waypoints file")
    parser.add_argument("--direct", action="store_true",
                        help="Upload directly to Pixhawk via DroneKit (bypasses MQTT agent)")
    args = parser.parse_args()

    waypoints = parse_waypoints(args.file)
    if not waypoints:
        print("No waypoints found in file")
        sys.exit(1)

    print(f"Parsed {len(waypoints)} waypoints from {args.file}")
    for i, wp in enumerate(waypoints, 1):
        print(f"  WP{i:02d}  lat={wp['lat']:.7f}  lon={wp['lon']:.7f}  alt={wp['alt']:.1f}m")

    if args.direct:
        send_via_dronekit(waypoints)
    else:
        send_via_mqtt(waypoints)


if __name__ == "__main__":
    main()
