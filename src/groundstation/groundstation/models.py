"""Domain models for the groundstation: mission/relay state and drone data.

Previously these enums and dataclasses were split between ``mission_controller``
and ``perpetual_monitor``, which forced the two modules to import types from
each other. Collecting them here gives a single definition point and lets both
modules (and the GUI) depend on data types rather than on each other.
"""

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Dict, List


class MissionState(Enum):
    """Mission state machine states."""
    IDLE = auto()
    SETTING_RTH_ALTITUDE = auto()
    TAKING_OFF = auto()
    CLIMBING_TO_ALTITUDE = auto()
    TRANSIT_TO_MONITORING = auto()
    APPROACHING_POINT = auto()  # Within video trigger distance
    MONITORING = auto()
    WAITING_FOR_RELAY = auto()  # Drone is waiting for replacement to arrive
    CAMERA_SYNC = auto()  # Drone performing 360° yaw for video synchronization
    RETURNING_HOME = auto()
    COMPLETED = auto()
    ABORTED = auto()
    ERROR = auto()


class RelayState(Enum):
    """Relay mission state."""
    INACTIVE = auto()
    ACTIVE = auto()
    TRANSITIONING = auto()
    PAUSED = auto()
    STOPPED = auto()


class MissionMode(Enum):
    """Mission flight mode."""
    MONITORING_POINT = auto()  # Fly to defined monitoring point
    FREE_FLIGHT = auto()  # Pilot manually controls after reaching altitude


class DroneState(Enum):
    """Drone operational states."""
    DISCONNECTED = "Disconnected"
    CONNECTED = "Connected"
    IDLE = "Idle"
    TAKING_OFF = "Taking Off"
    FLYING_TO_POINT = "Flying to Point"
    MONITORING = "Monitoring"
    RETURNING_HOME = "Returning Home"
    LANDING = "Landing"
    WAITING_FOR_RELAY = "Waiting for Relay"
    CAMERA_SYNC = "Camera Sync"
    EMERGENCY = "Emergency"


class MissionPhase(Enum):
    """Mission phases for the drone."""
    NONE = "None"
    CLIMB_TO_RTH_ALTITUDE = "Climbing to RTH Altitude"
    TRANSIT_TO_MONITORING = "Transit to Monitoring Point"
    MONITORING = "Monitoring"
    RELAY_HANDOFF = "Relay Handoff"
    RETURN_HOME = "Return Home"
    LANDING = "Landing"


@dataclass
class MissionWaypoint:
    """A waypoint in the mission."""
    latitude: float
    longitude: float
    altitude: float
    action: str = "goto"  # goto, hover, record_start, record_stop
    reached: bool = False


@dataclass
class DroneMissionStatus:
    """Status of a single drone's mission."""
    namespace: str
    state: MissionState = MissionState.IDLE
    assigned_altitude: float = 50.0
    target_heading: float = 0.0  # Target heading when reaching monitoring point

    # Current target position (for trajectory re-evaluation during transit)
    target_lat: float = 0.0
    target_lon: float = 0.0
    target_alt: float = 0.0

    # Timing
    mission_start_time: float = 0.0
    transit_start_time: float = 0.0
    monitoring_start_time: float = 0.0
    rth_start_time: float = 0.0

    # Travel time tracking
    estimated_travel_time: float = 0.0
    actual_travel_time: float = 0.0
    actual_rth_time: float = 0.0

    # Video recording
    video_trigger_distance: float = 50.0
    video_started: bool = False

    # Relay handoff - drone namespace to replace when this drone arrives at monitoring point
    replacing_drone: str = ""

    # Relay handoff target - snapshot of the drone being replaced at launch time
    # Used to fly to where the old drone was instead of the monitoring point
    relay_target_lat: float = 0.0
    relay_target_lon: float = 0.0
    relay_target_heading: float = 0.0

    # Camera sync tracking (360° yaw for video synchronization)
    camera_sync_start_time: float = 0.0
    camera_sync_yaw_started: bool = False
    camera_sync_yaw_phase2_started: bool = False  # Second half of 360° rotation
    camera_sync_yaw_completed: bool = False
    camera_sync_initial_heading: float = 0.0
    camera_sync_next_state: str = "RTH"  # "RTH" or "MONITORING" - what to do after spin
    camera_sync_partner_drone: str = ""  # The OTHER drone that should RTH after spin completes
    camera_sync_top_drone_ns: str = ""  # The TOP (higher) drone namespace during sync
    camera_sync_top_drone_previous_gimbal: float = 0.0  # Previous gimbal pitch to restore after sync
    camera_sync_old_drone_gimbal: float = 0.0  # Old monitoring drone's gimbal pitch to transfer to new drone
    camera_sync_new_drone_ns: str = ""  # The NEW drone that will continue monitoring after sync

    # Non-blocking state machine timers (timestamps)
    state_entry_time: float = 0.0  # When current state was entered
    rth_altitude_cmd_count: int = 0  # Number of RTH altitude commands sent
    rth_altitude_last_cmd_time: float = 0.0  # Time of last RTH altitude command
    takeoff_cmd_sent: bool = False  # Whether takeoff command was sent
    takeoff_second_cmd_sent: bool = False  # Whether second takeoff command was sent
    climb_wait_started: bool = False  # Whether post-climb wait has started

    # Logging timers (to avoid spamming logs)
    last_alt_log_time: float = 0.0
    last_transit_log_time: float = 0.0
    last_approach_log_time: float = 0.0

    # Climb time tracking (for Free Flight dynamic travel estimation)
    climb_start_time: float = 0.0  # When climb started (after takeoff wait)
    actual_climb_time: float = 0.0  # Measured climb time for this drone

    # Transit tracking (for Free Flight dynamic travel estimation)
    horizontal_transit_start_time: float = 0.0  # When horizontal transit started (after climb)
    horizontal_transit_start_lat: float = 0.0  # Position when transit started
    horizontal_transit_start_lon: float = 0.0
    actual_horizontal_transit_time: float = 0.0  # Measured horizontal transit time
    actual_horizontal_transit_distance: float = 0.0  # Measured horizontal transit distance

    # Error handling
    error_message: str = ""
    retry_count: int = 0


@dataclass
class RelayMissionConfig:
    """Configuration for relay mission."""
    monitoring_lat: float = 0.0
    monitoring_lon: float = 0.0
    monitoring_alt: float = 50.0
    monitoring_heading: float = 0.0  # Target heading/yaw at monitoring point (degrees)

    # Mission mode
    mission_mode: MissionMode = MissionMode.MONITORING_POINT

    base_rth_altitude: float = 50.0
    altitude_separation: float = 15.0
    video_trigger_distance: float = 50.0

    # Safety parameters
    safety_buffer_seconds: float = 60.0
    min_battery_to_launch: float = 30.0
    min_satellites: int = 8
    max_retry_count: int = 3
    min_vertical_separation: float = 5.0  # meters - minimum safe altitude difference

    # Camera sync (360° yaw rotation during relay handoff)
    camera_sync_enabled: bool = True  # If False, skip 360° rotation but keep 10s waits

    # Timing
    preflight_wait_seconds: float = 3.0
    altitude_tolerance: float = 2.0  # meters
    position_tolerance: float = 5.0  # meters

    # Robustness parameters
    telemetry_timeout_seconds: float = 5.0  # Consider disconnected if no telemetry
    emergency_battery_threshold: float = 15.0  # Force RTH
    max_distance_from_home: float = 5000.0  # meters, max allowed distance
    watchdog_interval: float = 1.0  # seconds between watchdog checks


@dataclass
class DroneData:
    """Complete drone data including telemetry and mission state."""
    # Connection info
    ip_address: str = ""
    namespace: str = ""
    is_connected: bool = False
    color: str = "#FF6B6B"  # Assigned color for UI visualization

    # Telemetry
    latitude: float = 0.0
    longitude: float = 0.0
    altitude: float = 0.0
    heading: float = 0.0
    battery_level: float = 0.0
    satellite_count: int = 0
    speed: float = 0.0

    # Flight time info (critical for relay logic)
    remaining_flight_time: float = 0.0  # seconds
    time_needed_to_go_home: float = 0.0  # seconds
    time_needed_to_land: float = 0.0  # seconds
    distance_to_home: float = 0.0  # meters

    # Battery thresholds (for RTH predictor)
    battery_needed_to_go_home: float = 0.0  # percentage
    battery_needed_to_land: float = 0.0  # percentage
    flight_mode: str = "UNKNOWN"

    # Home location
    home_latitude: float = 0.0
    home_longitude: float = 0.0
    home_set: bool = False

    # Camera state
    is_recording: bool = False
    gimbal_pitch: float = 0.0
    gimbal_yaw: float = 0.0

    # Mission state
    state: DroneState = DroneState.DISCONNECTED
    mission_phase: MissionPhase = MissionPhase.NONE
    current_task: str = "None"

    # Mission status flags
    waypoint_reached: bool = False
    altitude_reached: bool = False
    yaw_reached: bool = False

    # Timestamps
    last_telemetry_update: float = 0.0
    connection_time: float = 0.0


@dataclass
class MonitoringPoint:
    """GPS coordinates for the monitoring point."""
    latitude: float = 0.0
    longitude: float = 0.0
    altitude: float = 50.0  # Default monitoring altitude
    heading: float = 0.0  # Target heading in degrees (0-360)
    is_set: bool = False
    source: str = ""  # "drone", "map", "manual"


@dataclass
class RelayMission:
    """Relay mission configuration."""
    is_active: bool = False
    rth_altitude: float = 50.0
    monitoring_point: MonitoringPoint = field(default_factory=MonitoringPoint)
    video_trigger_distance: float = 50.0  # meters before monitoring point
    altitude_separation: float = 15.0  # vertical separation between drones
    safety_buffer: float = 60.0  # seconds buffer for relay timing

    # Mission state
    active_drone: str = ""
    next_drone: str = ""
    relay_countdown: float = 0.0  # seconds until next drone should launch
    drones_in_mission: List[str] = field(default_factory=list)

    # Travel time estimation
    estimated_travel_time: float = 0.0
    actual_travel_times: Dict[str, float] = field(default_factory=dict)
