import math
import time
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple


@dataclass
class GPSData:
    latitude: float
    longitude: float
    timestamp: float


@dataclass
class ParktronicData:
    front: Optional[float] = None
    rear: Optional[float] = None
    timestamp: float = field(default_factory=time.time)


@dataclass
class VehicleState:
    obstacle_detected: bool = False
    danger_level: str = "SAFE"
    min_distance: Optional[float] = None
    speed_mps: float = 0.0
    inside_geofence: bool = True
    latitude: Optional[float] = None
    longitude: Optional[float] = None


class SensorProcessor:
    def __init__(
        self,
        critical_distance_m: float = 1.0,
        warning_distance_m: float = 2.0,
        geofence_center: Tuple[float, float] = (47.0105, 28.8638),
        geofence_radius_m: float = 100.0,
    ):
        self.critical_distance_m = critical_distance_m
        self.warning_distance_m = warning_distance_m
        self.geofence_center = geofence_center
        self.geofence_radius_m = geofence_radius_m
        self.last_gps: Optional[GPSData] = None
        self.state = VehicleState()

    def haversine_distance(
        self, lat1: float, lon1: float, lat2: float, lon2: float
    ) -> float:
        r = 6371000.0
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)

        a = (
            math.sin(dphi / 2) ** 2
            + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
        )
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return r * c

    def process_gps(self, gps: GPSData) -> None:
        self.state.latitude = gps.latitude
        self.state.longitude = gps.longitude

        center_lat, center_lon = self.geofence_center
        dist_from_center = self.haversine_distance(
            gps.latitude, gps.longitude, center_lat, center_lon
        )
        self.state.inside_geofence = dist_from_center <= self.geofence_radius_m

        if self.last_gps is not None:
            dt = gps.timestamp - self.last_gps.timestamp
            if dt > 0:
                traveled = self.haversine_distance(
                    self.last_gps.latitude,
                    self.last_gps.longitude,
                    gps.latitude,
                    gps.longitude,
                )
                self.state.speed_mps = traveled / dt

        self.last_gps = gps

    def process_parktronic(self, park: ParktronicData) -> None:
        valid_distances = [
            d for d in [park.front, park.rear] if d is not None and d >= 0
        ]

        if not valid_distances:
            self.state.obstacle_detected = False
            self.state.danger_level = "UNKNOWN"
            self.state.min_distance = None
            return

        min_distance = min(valid_distances)
        self.state.min_distance = min_distance
        self.state.obstacle_detected = min_distance <= self.warning_distance_m

        if min_distance <= self.critical_distance_m:
            self.state.danger_level = "CRITICAL"
        elif min_distance <= self.warning_distance_m:
            self.state.danger_level = "WARNING"
        else:
            self.state.danger_level = "SAFE"

    def get_decision(self) -> Dict[str, object]:
        stop_required = self.state.danger_level == "CRITICAL"
        slow_required = self.state.danger_level == "WARNING"
        geofence_violation = not self.state.inside_geofence

        return {
            "obstacle_detected": self.state.obstacle_detected,
            "danger_level": self.state.danger_level,
            "min_distance_m": self.state.min_distance,
            "speed_mps": round(self.state.speed_mps, 2),
            "inside_geofence": self.state.inside_geofence,
            "latitude": self.state.latitude,
            "longitude": self.state.longitude,
            "stop_required": stop_required,
            "slow_required": slow_required and not stop_required,
            "geofence_violation": geofence_violation,
        }


def simulate_sensor_input():
    gps_samples = [
        GPSData(47.0105, 28.8638, time.time()),
        GPSData(47.01055, 28.86385, time.time() + 1),
        GPSData(47.01060, 28.86390, time.time() + 2),
        GPSData(47.01150, 28.86550, time.time() + 3),
    ]

    park_samples = [
        ParktronicData(front=3.2, rear=2.8),
        ParktronicData(front=1.7, rear=2.6),
        ParktronicData(front=0.8, rear=2.4),
        ParktronicData(front=2.4, rear=2.3),
    ]

    return list(zip(gps_samples, park_samples))


def main():
    processor = SensorProcessor(
        critical_distance_m=1.0,
        warning_distance_m=2.0,
        geofence_center=(47.0105, 28.8638),
        geofence_radius_m=150.0,
    )

    for gps, park in simulate_sensor_input():
        processor.process_gps(gps)
        processor.process_parktronic(park)

        decision = processor.get_decision()
        print(decision)
        time.sleep(1)


if __name__ == "__main__":
    main()
