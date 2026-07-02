"""Pure navigation math: great-circle geometry and relay timing.

Great-circle distance and bearing, waypoint projection, and flight-time
estimates for relay scheduling. All functions are pure (no I/O, no ROS, no
state).
"""

import math

EARTH_RADIUS = 6_371_000  # meters

# Flight speed defaults (m/s). DJI climb/descent rate and horizontal cruise.
VERTICAL_SPEED = 4.0
HORIZONTAL_SPEED_PID = 15.0
HORIZONTAL_SPEED_NATIVE = 15.0


def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance between two GPS points, in meters."""
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    delta_lat = math.radians(lat2 - lat1)
    delta_lon = math.radians(lon2 - lon1)

    a = (math.sin(delta_lat / 2) ** 2
         + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(delta_lon / 2) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return EARTH_RADIUS * c


def calculate_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Initial bearing from point 1 to point 2, in degrees (0-360)."""
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    delta_lon = math.radians(lon2 - lon1)

    x = math.sin(delta_lon) * math.cos(lat2_rad)
    y = (math.cos(lat1_rad) * math.sin(lat2_rad)
         - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon))
    return (math.degrees(math.atan2(x, y)) + 360) % 360


def estimate_travel_time(
    distance: float,
    altitude: float = 50.0,
    horizontal_speed: float = HORIZONTAL_SPEED_PID,
    vertical_speed: float = VERTICAL_SPEED,
    *,
    wait_after_takeoff: float = 0.0,
    wait_after_climb: float = 0.0,
) -> float:
    """Estimate takeoff-to-arrival time in seconds.

    Sequence: wait after takeoff, climb to altitude, wait after climb,
    horizontal transit. The two waits default to 0 (matching the plain
    calculator); the mission controller passes the real hold times.
    Returns ``inf`` when horizontal speed is non-positive.
    """
    if horizontal_speed <= 0:
        return float('inf')
    climb_time = altitude / vertical_speed if vertical_speed > 0 else 0
    horizontal_time = distance / horizontal_speed
    return wait_after_takeoff + climb_time + wait_after_climb + horizontal_time


def calculate_relay_countdown(
    remaining_flight_time: float,
    time_to_monitoring_point: float,
    safety_buffer: float = 60.0,
) -> float:
    """Seconds until the next drone should launch (0 = launch now)."""
    return max(0.0, remaining_flight_time - time_to_monitoring_point - safety_buffer)


def calculate_drones_needed(
    total_mission_time: float,
    flight_time_per_drone: float,
    travel_time: float,
    safety_buffer: float = 60.0,
) -> int | float:
    """Minimum drones for continuous coverage.

    Returns ``inf`` if a single drone's effective monitoring time is
    non-positive (impossible with the given parameters), ``0`` if flight time
    per drone is non-positive.
    """
    if flight_time_per_drone <= 0:
        return 0

    effective_monitoring_time = flight_time_per_drone - (2 * travel_time) - safety_buffer
    if effective_monitoring_time <= 0:
        return float('inf')
    return max(1, math.ceil(total_mission_time / effective_monitoring_time))
