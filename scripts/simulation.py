"""
AGC Map Simulation — Real OSM map, click to set waypoints, vehicle follows trace.

pip install pygame requests

Controls:
  Left-click      Add waypoint to path
  Right-click     Remove last waypoint
  Middle-drag     Pan the map
  Scroll wheel    Zoom in / out
  SPACE           Start / pause simulation
  R               Reset vehicle to first waypoint
  C               Clear all waypoints
  ESC             Quit
"""

import io
import math
import os
import sys
import threading
import pygame
import requests

# ═══════════════════════════════════════════════════════════════════════
#  ADJUSTABLE PARAMETERS  ← edit these
# ═══════════════════════════════════════════════════════════════════════

# ── Real EG2028KSF golf cart dimensions ──────────────────────────────
CAR_LENGTH_M     = 2.67   # metres (total body length)
CAR_WIDTH_M      = 1.18   # metres (total body width)
WHEELBASE_M      = 1.70   # metres (front axle to rear axle)
TRACK_WIDTH_M    = 0.855  # metres (centre-to-centre of wheels on same axle)
MAX_WHEEL_DEG     = 30.0  # degrees (confirmed: 1.5 steering-wheel rotations = 30° wheel)
STEERING_RATIO    = 18.0  # steering wheel degrees per wheel degree (540° / 30°)
MIN_TURN_RADIUS_M = round(WHEELBASE_M / __import__('math').tan(__import__('math').radians(MAX_WHEEL_DEG)), 3)
# = 1.70 / tan(30°) = 2.944 m  (tighter than the 3.5 m spec sheet value)

MAX_SPEED_MS     = 2.0    # top speed (m/s) — adjust to your measured value

# Derived: max heading change rate while moving (from turning radius physics)
# At speed v, tightest arc = v / R_min  (rad/s) → deg/s
MAX_TURN_DEG_S   = round(__import__('math').degrees(MAX_SPEED_MS / MIN_TURN_RADIUS_M), 1)
# ≈ 32.7 deg/s at 2 m/s — scales automatically with speed in the sim

HEADING_DEADBAND = 5.0    # degrees — no correction inside this band
ARRIVAL_RADIUS_M = 2.0    # metres — waypoint considered reached
MAX_STEER_ANGLE  = 8.0    # degrees heading error → stop & turn in place
                           # tight: car must be nearly aligned before it moves

# Stanley path-following controller
STANLEY_K        = 0.8    # cross-track gain (higher = stronger lane correction)
KP_STEER         = 3.0    # proportional steering gain (higher = snappier turns)

# Map start location
MAP_CENTER_LAT   = 47.0621841
MAP_CENTER_LON   = 28.8676635
INITIAL_ZOOM     = 18     # 16–19 (higher = more detail)

PANEL_W          = 265
WINDOW_W         = 1280
WINDOW_H         = 800
SIM_HZ           = 60

TILE_SIZE        = 256
CACHE_DIR        = "tiles_cache"
TILE_URL         = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"
OSM_UA           = {"User-Agent": "AGC-Vehicle-Simulation/1.0"}

# ═══════════════════════════════════════════════════════════════════════

WHITE  = (255, 255, 255)
BLACK  = (0,   0,   0)
GREEN  = (40,  200, 80)
RED    = (220, 50,  50)
BLUE   = (50,  130, 220)
YELLOW = (255, 220, 0)
GRAY   = (150, 150, 150)
DARK   = (22,  22,  32)
PANEL  = (14,  14,  26)
ORANGE = (255, 150, 30)
CYAN   = (0,   210, 210)
PURPLE = (160, 80,  220)


# ── Tile helpers ──────────────────────────────────────────────────────

def deg2tile(lat, lon, zoom):
    """GPS → fractional tile coordinates."""
    n   = 1 << zoom
    x   = (lon + 180) / 360 * n
    lr  = math.radians(lat)
    y   = (1 - math.log(math.tan(lr) + 1 / math.cos(lr)) / math.pi) / 2 * n
    return x, y


def tile2deg(tx, ty, zoom):
    """Fractional tile coordinates → GPS (top-left corner convention)."""
    n   = 1 << zoom
    lon = tx / n * 360 - 180
    lat = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * ty / n))))
    return lat, lon


os.makedirs(CACHE_DIR, exist_ok=True)
_tcache: dict  = {}
_tlock         = threading.Lock()
_pending: set  = set()


def _fetch_tile(z, x, y):
    path = os.path.join(CACHE_DIR, f"{z}_{x}_{y}.png")
    try:
        if not os.path.exists(path):
            r = requests.get(TILE_URL.format(z=z, x=x, y=y),
                             headers=OSM_UA, timeout=10)
            if r.status_code != 200:
                return
            with open(path, "wb") as f:
                f.write(r.content)
        data = open(path, "rb").read()
        surf = pygame.image.load(io.BytesIO(data)).convert()
        with _tlock:
            _tcache[(z, x, y)] = surf
            _pending.discard((z, x, y))
    except Exception:
        with _tlock:
            _pending.discard((z, x, y))


def get_tile(z, x, y):
    key = (z, x, y)
    with _tlock:
        if key in _tcache:
            return _tcache[key]
        if key not in _pending:
            _pending.add(key)
            threading.Thread(target=_fetch_tile, args=(z, x, y),
                             daemon=True).start()
    return None


# ── Coordinate conversion ─────────────────────────────────────────────

def gps_to_screen(lat, lon, otx, oty):
    tx, ty = deg2tile(lat, lon, _zoom())
    return (tx - otx) * TILE_SIZE, (ty - oty) * TILE_SIZE


def screen_to_gps(sx, sy, otx, oty):
    tx = otx + sx / TILE_SIZE
    ty = oty + sy / TILE_SIZE
    return tile2deg(tx, ty, _zoom())


# zoom held in a mutable container so helpers can access it without globals
_state = {"zoom": INITIAL_ZOOM}

def _zoom():
    return _state["zoom"]


# ── Navigation helpers ────────────────────────────────────────────────

def haversine(lat1, lon1, lat2, lon2):
    R  = 6_371_000
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a  = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def compass_bearing(lat1, lon1, lat2, lon2):
    """Compass bearing 0–360° (0 = North, 90 = East)."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    x  = math.sin(dl) * math.cos(p2)
    y  = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return (math.degrees(math.atan2(x, y)) + 360) % 360


def norm_angle(a):
    return (a + 180) % 360 - 180


def signed_cross_track_error(car_lat, car_lon, p1_lat, p1_lon, p2_lat, p2_lon):
    """
    Signed perpendicular distance from car to the path segment p1→p2.
    Positive = car is to the RIGHT of the path direction.
    """
    path_brg  = compass_bearing(p1_lat, p1_lon, p2_lat, p2_lon)
    car_brg   = compass_bearing(p1_lat, p1_lon, car_lat, car_lon)
    car_dist  = haversine(p1_lat, p1_lon, car_lat, car_lon)
    angle_off = norm_angle(car_brg - path_brg)
    return car_dist * math.sin(math.radians(angle_off))


# ── Drawing ───────────────────────────────────────────────────────────

def draw_car(surf, sx, sy, hdg, color):
    lp, wp = 28, 14
    # car corners: (forward, right) in car-local space
    corners = [(lp/2, wp/2), (lp/2, -wp/2), (-lp/2, -wp/2), (-lp/2, wp/2)]
    h  = math.radians(hdg)
    sh, ch = math.sin(h), math.cos(h)
    # transform to screen: North=up, clockwise heading
    pts = [(sx + cx * sh + cy * ch,
            sy - cx * ch + cy * sh)
           for cx, cy in corners]
    pygame.draw.polygon(surf, color, pts)
    pygame.draw.polygon(surf, WHITE, pts, 1)
    # yellow front stripe
    pygame.draw.line(surf, YELLOW,
                     (int(pts[0][0]), int(pts[0][1])),
                     (int(pts[1][0]), int(pts[1][1])), 2)


def label(surf, font, msg, x, y, c=GRAY):
    surf.blit(font.render(msg, True, c), (x, y))


# ── Main ──────────────────────────────────────────────────────────────

def main():
    pygame.init()
    screen  = pygame.display.set_mode((WINDOW_W, WINDOW_H))
    pygame.display.set_caption("AGC Map Simulation")
    clock   = pygame.time.Clock()
    font_sm = pygame.font.SysFont("Consolas", 13)
    font_md = pygame.font.SysFont("Consolas", 15, bold=True)
    font_lg = pygame.font.SysFont("Consolas", 18, bold=True)

    MAP_W = WINDOW_W - PANEL_W

    # placeholder tile while real one loads
    ph = pygame.Surface((TILE_SIZE, TILE_SIZE))
    ph.fill((200, 200, 200))
    ph.blit(pygame.font.SysFont("Arial", 11).render("loading…", True, (100, 100, 100)),
            (90, 120))

    # ── view state ────────────────────────────────────────────────────
    zoom        = INITIAL_ZOOM
    _state["zoom"] = zoom
    cx, cy      = deg2tile(MAP_CENTER_LAT, MAP_CENTER_LON, zoom)
    otx         = cx - MAP_W / 2 / TILE_SIZE
    oty         = cy - WINDOW_H / 2 / TILE_SIZE

    # ── waypoints & vehicle ───────────────────────────────────────────
    waypoints: list[tuple[float, float]] = []   # [(lat, lon), …]

    def make_car(lat, lon):
        return {"lat": lat, "lon": lon, "heading": 0.0,
                "speed": 0.0, "wp_idx": 1, "arrived": False, "trail": []}

    car    = make_car(MAP_CENTER_LAT, MAP_CENTER_LON)
    paused = True
    dt     = 1.0 / SIM_HZ

    dragging    = False
    drag_start  = (0, 0)
    drag_origin = (0.0, 0.0)

    while True:
        # ── events ────────────────────────────────────────────────────
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit(); sys.exit()

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    pygame.quit(); sys.exit()

                elif event.key == pygame.K_SPACE:
                    if len(waypoints) >= 2:
                        paused = not paused

                elif event.key == pygame.K_r:
                    if waypoints:
                        car = make_car(waypoints[0][0], waypoints[0][1])
                        paused = True

                elif event.key == pygame.K_c:
                    waypoints.clear()
                    car = make_car(MAP_CENTER_LAT, MAP_CENTER_LON)
                    paused = True

            elif event.type == pygame.MOUSEBUTTONDOWN:
                mx, my = event.pos
                if mx >= MAP_W:
                    continue

                if event.button == 1:           # left → add waypoint
                    lat, lon = screen_to_gps(mx, my, otx, oty)
                    waypoints.append((lat, lon))
                    if len(waypoints) == 1:
                        car = make_car(lat, lon)

                elif event.button == 3:         # right → remove last
                    if waypoints:
                        waypoints.pop()
                        if not waypoints:
                            car = make_car(MAP_CENTER_LAT, MAP_CENTER_LON)
                            paused = True
                        elif car["wp_idx"] >= len(waypoints):
                            car["wp_idx"] = len(waypoints) - 1

                elif event.button == 2:         # middle → start pan
                    dragging    = True
                    drag_start  = event.pos
                    drag_origin = (otx, oty)

            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 2:
                    dragging = False

            elif event.type == pygame.MOUSEMOTION:
                if dragging:
                    dx  = event.pos[0] - drag_start[0]
                    dy  = event.pos[1] - drag_start[1]
                    otx = drag_origin[0] - dx / TILE_SIZE
                    oty = drag_origin[1] - dy / TILE_SIZE

            elif event.type == pygame.MOUSEWHEEL:
                mx, my = pygame.mouse.get_pos()
                if mx < MAP_W:
                    lat_m, lon_m = screen_to_gps(mx, my, otx, oty)
                    new_zoom = max(10, min(19, zoom + event.y))
                    if new_zoom != zoom:
                        zoom = new_zoom
                        _state["zoom"] = zoom
                        tx_m, ty_m = deg2tile(lat_m, lon_m, zoom)
                        otx = tx_m - mx / TILE_SIZE
                        oty = ty_m - my / TILE_SIZE

        # ── simulation step ───────────────────────────────────────────────
        if not paused and not car["arrived"] and len(waypoints) >= 2:
            idx = car["wp_idx"]
            if idx < len(waypoints):
                tgt_lat, tgt_lon = waypoints[idx]
                dist = haversine(car["lat"], car["lon"], tgt_lat, tgt_lon)

                if dist <= ARRIVAL_RADIUS_M:
                    if idx + 1 < len(waypoints):
                        car["wp_idx"] += 1
                    else:
                        car["arrived"] = True
                        car["speed"]   = 0.0
                else:
                    # direct bearing from current position to target waypoint
                    direct_brg = compass_bearing(car["lat"], car["lon"], tgt_lat, tgt_lon)
                    herr       = norm_angle(direct_brg - car["heading"])

                    # PHASE 1 — large error: stop completely and rotate in place
                    if abs(herr) > MAX_STEER_ANGLE:
                        car["speed"]   = 0.0
                        phys_turn_rate = MAX_TURN_DEG_S   # full rate when stopped
                    else:
                        # PHASE 2 — aligned: smoothly ramp speed from 0 → full
                        # sin²(herr/MAX) gives a smooth 0→1 curve
                        align_ratio    = 1.0 - (abs(herr) / MAX_STEER_ANGLE) ** 2
                        car["speed"]   = MAX_SPEED_MS * align_ratio
                        # physical turn limit from min turning radius
                        if car["speed"] > 0.05:
                            phys_turn_rate = math.degrees(car["speed"] / MIN_TURN_RADIUS_M)
                        else:
                            phys_turn_rate = MAX_TURN_DEG_S

                    # apply proportional steering capped by physical limit
                    if abs(herr) > HEADING_DEADBAND:
                        desired_rate   = min(KP_STEER * abs(herr), phys_turn_rate)
                        turn           = math.copysign(min(desired_rate * dt, abs(herr)), herr)
                        car["heading"] = (car["heading"] + turn) % 360

                    # advance position (speed=0 when turning → no drift)
                    h    = math.radians(car["heading"])
                    dlat = car["speed"] * dt * math.cos(h) / 111_320
                    dlon = (car["speed"] * dt * math.sin(h)
                            / (111_320 * math.cos(math.radians(car["lat"]))))
                    car["lat"] += dlat
                    car["lon"] += dlon
                    if car["speed"] > 0.01:
                        car["trail"].append((car["lat"], car["lon"]))

        # ── draw tiles ────────────────────────────────────────────────
        screen.fill(DARK)

        tx0 = int(math.floor(otx))
        ty0 = int(math.floor(oty))
        tx1 = int(math.ceil(otx + MAP_W / TILE_SIZE)) + 1
        ty1 = int(math.ceil(oty + WINDOW_H / TILE_SIZE)) + 1

        for ty in range(ty0, ty1):
            for tx in range(tx0, tx1):
                sx = int((tx - otx) * TILE_SIZE)
                sy = int((ty - oty) * TILE_SIZE)
                tile = get_tile(zoom, tx, ty)
                screen.blit(tile if tile else ph, (sx, sy))

        # ── draw planned path ─────────────────────────────────────────
        if len(waypoints) > 1:
            pts = []
            for lat, lon in waypoints:
                sx, sy = gps_to_screen(lat, lon, otx, oty)
                pts.append((int(sx), int(sy)))
            pygame.draw.lines(screen, ORANGE, False, pts, 3)

        # ── draw waypoint markers ─────────────────────────────────────
        for i, (lat, lon) in enumerate(waypoints):
            sx, sy = gps_to_screen(lat, lon, otx, oty)
            sx, sy = int(sx), int(sy)
            if i == 0:
                color, letter = GREEN,  "S"
            elif i == len(waypoints) - 1:
                color, letter = RED,    "E"
            else:
                color, letter = ORANGE, str(i)
            pygame.draw.circle(screen, color, (sx, sy), 8)
            pygame.draw.circle(screen, WHITE, (sx, sy), 8, 1)
            lbl = font_sm.render(letter, True, WHITE)
            screen.blit(lbl, (sx - lbl.get_width() // 2, sy - lbl.get_height() // 2))

        # ── draw driven trail ─────────────────────────────────────────
        if len(car["trail"]) > 1:
            pts = []
            for lat, lon in car["trail"]:
                sx, sy = gps_to_screen(lat, lon, otx, oty)
                pts.append((int(sx), int(sy)))
            pygame.draw.lines(screen, CYAN, False, pts, 2)

        # ── draw car ──────────────────────────────────────────────────
        csx, csy  = gps_to_screen(car["lat"], car["lon"], otx, oty)
        car_color = GREEN if car["arrived"] else BLUE
        draw_car(screen, int(csx), int(csy), car["heading"], car_color)

        # map area border
        pygame.draw.line(screen, GRAY, (MAP_W, 0), (MAP_W, WINDOW_H))

        # zoom / copyright label on map
        label(screen, font_sm, f"Zoom {zoom}  © OpenStreetMap", 6, WINDOW_H - 18, GRAY)

        # ── panel ─────────────────────────────────────────────────────
        pygame.draw.rect(screen, PANEL, (MAP_W, 0, PANEL_W, WINDOW_H))

        # compute live telemetry
        idx     = car["wp_idx"]
        dist    = 0.0
        brg_val = 0.0
        herr    = 0.0
        if waypoints and idx < len(waypoints):
            tgt_lat, tgt_lon = waypoints[idx]
            dist    = haversine(car["lat"], car["lon"], tgt_lat, tgt_lon)
            brg_val = compass_bearing(car["lat"], car["lon"], tgt_lat, tgt_lon)
            herr    = norm_angle(brg_val - car["heading"])

        px = MAP_W + 8
        y  = 8

        label(screen, font_lg, "AGC SIMULATION",      px, y, WHITE);              y += 28
        pygame.draw.line(screen, GRAY, (MAP_W, y), (WINDOW_W, y));                y += 6

        label(screen, font_sm, "HEADING",              px, y);                    y += 15
        label(screen, font_md, f"{car['heading']:.1f}°", px, y, YELLOW);          y += 24

        label(screen, font_sm, "SPEED",                px, y);                    y += 15
        label(screen, font_md, f"{car['speed']:.2f} m/s", px, y, YELLOW);        y += 24

        label(screen, font_sm, "WAYPOINT",             px, y);                    y += 15
        wp_lbl = f"{idx}/{len(waypoints)-1}" if len(waypoints) > 1 else "—"
        label(screen, font_md, wp_lbl,                 px, y, ORANGE);            y += 24

        label(screen, font_sm, "DIST TO NEXT WP",      px, y);                    y += 15
        dc = GREEN if dist <= ARRIVAL_RADIUS_M else (RED if dist > 30 else YELLOW)
        label(screen, font_md, f"{dist:.1f} m",        px, y, dc);                y += 24

        label(screen, font_sm, "BEARING TO WP",         px, y);                    y += 15
        label(screen, font_md, f"{brg_val:.1f}°",       px, y, YELLOW);            y += 24

        label(screen, font_sm, "HEADING ERROR",         px, y);                    y += 15
        ec = (GREEN if abs(herr) <= HEADING_DEADBAND
              else RED if abs(herr) > MAX_STEER_ANGLE else YELLOW)
        label(screen, font_md, f"{herr:+.1f}°",         px, y, ec);                y += 24

        phase = "TURNING" if abs(herr) > MAX_STEER_ANGLE else "DRIVING"
        label(screen, font_sm, "PHASE",                 px, y);                    y += 15
        label(screen, font_md, phase, px, y, RED if phase == "TURNING" else CYAN); y += 24

        label(screen, font_sm, "STATUS",                px, y);                    y += 15
        if not waypoints:
            st, sc = "ADD WAYPOINTS",  GRAY
        elif len(waypoints) < 2:
            st, sc = "NEED 2+ POINTS", GRAY
        elif car["arrived"]:
            st, sc = "ARRIVED",        GREEN
        elif paused:
            st, sc = "PAUSED",         YELLOW
        elif abs(herr) > MAX_STEER_ANGLE:
            st, sc = "TURNING",        RED
        else:
            st, sc = "NAVIGATING",     (100, 180, 255)
        label(screen, font_md, st,                      px, y, sc);                y += 28

        pygame.draw.line(screen, GRAY, (MAP_W, y), (WINDOW_W, y));                y += 6

        label(screen, font_sm, "VEHICLE  EG2028KSF",     px, y, ORANGE);            y += 15
        for line in [
            f"Body:     {CAR_LENGTH_M} m × {CAR_WIDTH_M} m",
            f"Wheelbase:{WHEELBASE_M} m",
            f"Min R:    {MIN_TURN_RADIUS_M} m",
            f"Max steer:{MAX_WHEEL_DEG}°",
            f"Speed:    {MAX_SPEED_MS} m/s",
            f"Arrival:  {ARRIVAL_RADIUS_M} m",
        ]:
            label(screen, font_sm, line, px, y, WHITE); y += 14
        y += 6

        pygame.draw.line(screen, GRAY, (MAP_W, y), (WINDOW_W, y));                y += 6
        label(screen, font_sm, "CONTROLS",              px, y);                    y += 15
        for line in [
            "L-click   add waypoint",
            "R-click   remove last WP",
            "Mid-drag  pan map",
            "Scroll    zoom in/out",
            "SPACE     start / pause",
            "R         reset vehicle",
            "C         clear path",
            "ESC       quit",
        ]:
            label(screen, font_sm, line, px, y, WHITE); y += 14

        # legend at bottom of panel
        ly = WINDOW_H - 20
        for i, (c, lbl_txt) in enumerate([
            (GREEN,  "Start"), (RED, "End"), (ORANGE, "Path"), (CYAN, "Driven"),
        ]):
            lx = MAP_W + 8 + i * 62
            pygame.draw.circle(screen, c, (lx + 5, ly + 4), 5)
            label(screen, font_sm, lbl_txt, lx + 13, ly, WHITE)

        # arrived banner on map
        if car["arrived"]:
            s = pygame.Surface((MAP_W, 44), pygame.SRCALPHA)
            s.fill((40, 200, 80, 90))
            screen.blit(s, (0, WINDOW_H // 2 - 22))
            label(screen, font_lg, "  ROUTE COMPLETE!", 20, WINDOW_H // 2 - 10, WHITE)

        pygame.display.flip()
        clock.tick(SIM_HZ)


if __name__ == "__main__":
    main()
