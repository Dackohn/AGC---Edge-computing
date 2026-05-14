"""
nav_test.py  —  walk-to-target GPS guide.
Reads live GPS from gps_server and shows distance + direction arrow.

Usage:
    python nav_test.py 47.0621841 28.8676635
    python nav_test.py            (prompts for coordinates)
"""

import math
import sys
import time
import urllib.request
import json
import os

GPS_SERVER = "http://localhost:8000/gps"
ARRIVAL_M  = 3.0   # metres — "arrived" threshold


def get_gps():
    try:
        with urllib.request.urlopen(GPS_SERVER, timeout=3) as r:
            return json.loads(r.read())
    except Exception as e:
        return None


def haversine(lat1, lon1, lat2, lon2):
    R = 6_371_000
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a  = math.sin(dp/2)**2 + math.cos(p1)*math.cos(p2)*math.sin(dl/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))


def bearing(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    x  = math.sin(dl) * math.cos(p2)
    y  = math.cos(p1)*math.sin(p2) - math.sin(p1)*math.cos(p2)*math.cos(dl)
    return (math.degrees(math.atan2(x, y)) + 360) % 360


ARROWS = ["↑", "↗", "→", "↘", "↓", "↙", "←", "↖"]

def arrow(brng):
    idx = round(brng / 45) % 8
    return ARROWS[idx]

COMPASS = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]

def compass_dir(brng):
    idx = round(brng / 45) % 8
    return COMPASS[idx]


def clear():
    os.system("cls" if os.name == "nt" else "clear")


def main():
    if len(sys.argv) == 3:
        target_lat = float(sys.argv[1])
        target_lon = float(sys.argv[2])
    else:
        print("Enter target coordinates:")
        target_lat = float(input("  Latitude  : "))
        target_lon = float(input("  Longitude : "))

    print(f"\nNavigating to  {target_lat:.7f}, {target_lon:.7f}")
    print("Press Ctrl+C to stop.\n")
    time.sleep(1)

    while True:
        gps = get_gps()

        clear()
        print("=" * 40)
        print("   GPS NAVIGATION GUIDE")
        print("=" * 40)

        if gps is None:
            print("\n  [!] Cannot reach gps_server — is it running?")
            print(f"      {GPS_SERVER}")
            time.sleep(1)
            continue

        lat = gps.get("lat")
        lon = gps.get("lon")

        if lat is None or lon is None:
            print(f"\n  [!] Waiting for GPS fix...")
            print(f"      Sats: {gps.get('satellites')}  Fix: {gps.get('fix_type')}")
            time.sleep(1)
            continue

        dist    = haversine(lat, lon, target_lat, target_lon)
        brng    = bearing(lat, lon, target_lat, target_lon)
        heading = gps.get("heading_deg")

        print(f"\n  Current : {lat:.7f}, {lon:.7f}")
        print(f"  Target  : {target_lat:.7f}, {target_lon:.7f}")
        print(f"  Sats    : {gps.get('satellites')}   Fix: {gps.get('fix_type')}")
        print()

        if dist <= ARRIVAL_M:
            print(f"  *** ARRIVED!  You are {dist:.1f}m from the target ***")
        else:
            print(f"  Distance : {dist:.1f} m")
            print(f"  Direction: {arrow(brng)}  {compass_dir(brng)}  ({brng:.1f}°)")
            if heading is not None:
                err = ((brng - heading) + 180) % 360 - 180
                turn = "straight" if abs(err) < 10 else ("turn RIGHT" if err > 0 else "turn LEFT")
                print(f"  Heading  : {heading:.1f}°   →  {turn}  ({err:+.1f}°)")
            print()
            # simple distance bar
            bar_len  = 20
            progress = max(0, min(1, 1 - dist / max(dist, 50)))
            filled   = int(progress * bar_len)
            print(f"  [{'#'*filled}{'.'*(bar_len-filled)}]  {dist:.1f}m left")

        print()
        print("=" * 40)
        time.sleep(1)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")
