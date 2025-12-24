"""
Mission Controller for Perpetual Drone Monitoring

Handles the state machine for single drone missions and relay operations.

Author: Edouard Rolland
Project: WildDrone
"""

import time
import math
import threading
import csv
import os
from datetime import datetime
from enum import Enum, auto
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Callable, Tuple
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


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


class MissionController:
    """
    Controls the execution of drone monitoring missions.
    
    Supports:
    - Single drone missions
    - Relay missions with automatic handoff
    - Video recording triggers
    - Travel time estimation
    """
    
    EARTH_RADIUS = 6371000  # meters
    
    def __init__(self):
        self.config = RelayMissionConfig()
        self.relay_state = RelayState.INACTIVE
        self.mission_mode = MissionMode.MONITORING_POINT  # Default to monitoring point mode
        
        # Navigation mode (DJI Native removed - always use PID)
        self.use_dji_native = False  # Always False - PID navigation only
        
        # Command and event logging
        self._command_log: List[str] = []  # Log of all commands sent
        self._event_log: List[Tuple[str, str, str, str]] = []  # (timestamp, namespace, state, message)
        
        # CSV logging configuration
        self._csv_log_dir: str = os.path.expanduser("~/wildperpetua_logs")
        self._csv_session_id: str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._csv_commands_file: Optional[str] = None
        self._csv_events_file: Optional[str] = None
        self._csv_positions_file: Optional[str] = None
        self._csv_logging_enabled: bool = False
        
        # Drone tracking
        self.drone_missions: Dict[str, DroneMissionStatus] = {}
        self.drone_order: List[str] = []  # Order for relay
        self.current_drone_index: int = 0
        
        # Relay launch tracking - prevents missed launches if countdown fluctuates
        self._relay_launch_pending: bool = False
        self._pending_next_drone: str = ""
        self._pending_next_index: int = 0
        
        # Manual swap mode - bypasses countdown timer
        self._manual_swap_active: bool = False
        
        # Continuous position tracking for replacement drone
        self._last_trajectory_update_time: float = 0.0
        self._trajectory_update_interval: float = 10.0  # seconds
        
        # Vertical separation safety
        self._vertical_separation_enabled: bool = True  # Toggle to enable/disable vertical separation check
        self._vertical_separation_warning_active: bool = False
        self._vertical_separation_aborted_drone: str = ""
        self._last_vertical_separation_check: float = 0.0
        
        # Vertical separation countdown (20 seconds before RTH)
        self._vertical_separation_countdown_active: bool = False
        self._vertical_separation_countdown_start: float = 0.0
        self._vertical_separation_countdown_duration: float = 20.0  # seconds
        self._vertical_separation_violating_drones: tuple = ()  # (drone1, drone2)
        self._vertical_separation_mission_stopped: bool = False  # Prevent countdown restart after stop
        
        # Callbacks for drone commands (set by ROS node)
        self._cmd_takeoff: Optional[Callable[[str], None]] = None
        self._cmd_land: Optional[Callable[[str], None]] = None
        self._cmd_rth: Optional[Callable[[str], None]] = None
        self._cmd_goto_waypoint: Optional[Callable[[str, float, float, float, float], None]] = None
        self._cmd_goto_altitude: Optional[Callable[[str, float], None]] = None
        self._cmd_set_rth_altitude: Optional[Callable[[str, float], None]] = None
        self._cmd_start_recording: Optional[Callable[[str], None]] = None
        self._cmd_stop_recording: Optional[Callable[[str], None]] = None
        self._cmd_set_gimbal_pitch: Optional[Callable[[str, float], None]] = None
        self._cmd_goto_yaw: Optional[Callable[[str, float], None]] = None  # Set drone heading
        self._cmd_abort: Optional[Callable[[str], None]] = None
        
        # Telemetry getters (set by ROS node)
        self.get_drone_position: Optional[Callable[[str], Tuple[float, float, float]]] = None
        self.get_drone_home_position: Optional[Callable[[str], Tuple[float, float]]] = None  # home lat, lon
        self.get_drone_heading: Optional[Callable[[str], float]] = None
        self.get_drone_gimbal_pitch: Optional[Callable[[str], float]] = None
        self.get_drone_altitude: Optional[Callable[[str], float]] = None  # Get drone altitude for vertical separation check
        self.get_remaining_flight_time: Optional[Callable[[str], float]] = None
        self.get_battery_level: Optional[Callable[[str], float]] = None
        self.get_satellite_count: Optional[Callable[[str], int]] = None
        self.get_is_recording: Optional[Callable[[str], bool]] = None
        self.get_waypoint_reached: Optional[Callable[[str], bool]] = None
        self.get_altitude_reached: Optional[Callable[[str], bool]] = None
        self.get_configured_speed: Optional[Callable[[], float]] = None  # Get UI-configured speed
        self.get_connected_drones: Optional[Callable[[], List[str]]] = None  # Get list of connected drone namespaces
        self.get_flight_mode: Optional[Callable[[str], str]] = None  # Get drone flight mode (for manual control detection)
        
        # Status callback (for GUI updates)
        self.on_status_update: Optional[Callable[[str, MissionState, str], None]] = None
        self.on_relay_countdown: Optional[Callable[[float, str, dict], None]] = None  # countdown, next_drone, timing_breakdown
        self.on_mission_event: Optional[Callable[[str, str], None]] = None
        self.on_takeoff_confirmation_request: Optional[Callable[[str, Callable[[bool], None]], None]] = None  # drone_name, callback(confirmed)
        self.on_vertical_separation_warning: Optional[Callable[[str, str, float], None]] = None  # drone1, drone2, separation
        self.on_vertical_separation_alert: Optional[Callable[[str, str, float, float, float], None]] = None  # drone1, drone2, separation, alt1, alt2
        self.on_vertical_separation_countdown_start: Optional[Callable[[], None]] = None  # Start 20s countdown audio
        self.on_vertical_separation_countdown_cancel: Optional[Callable[[], None]] = None  # Cancel countdown, play respected sound
        self.on_vertical_separation_mission_stopped: Optional[Callable[[], None]] = None  # Mission stopped due to countdown expiry
        
        # Takeoff confirmation tracking for relay auto-launch
        self._takeoff_confirmation_pending: bool = False
        self._takeoff_confirmation_shown_for_drone: str = ""
        self._takeoff_confirmed: bool = False
        self._takeoff_cancelled: bool = False
        
        # Travel time history for estimation
        self.travel_time_history: List[float] = []
        
        # Flight time history (actual total flight times from completed missions)
        self.flight_time_history: List[float] = []
        
        # Mission thread
        self._mission_thread: Optional[threading.Thread] = None
        self._running = False
        self._update_interval = 0.5  # seconds
    
    # ========================================================================
    # VERTICAL SEPARATION TOGGLE
    # ========================================================================
    
    @property
    def vertical_separation_enabled(self) -> bool:
        """Check if vertical separation check is enabled."""
        return self._vertical_separation_enabled
    
    @vertical_separation_enabled.setter
    def vertical_separation_enabled(self, value: bool):
        """Enable or disable vertical separation check."""
        self._vertical_separation_enabled = value
        if not value:
            # Cancel any active countdown when disabling
            if self._vertical_separation_countdown_active:
                self._vertical_separation_countdown_active = False
                self._vertical_separation_violating_drones = ()
                if self.on_vertical_separation_countdown_cancel:
                    self.on_vertical_separation_countdown_cancel()
        logger.info(f"Vertical separation check {'enabled' if value else 'disabled'}")
    
    # ========================================================================
    # DEBUG MODE - Command wrappers with logging
    # ========================================================================
    
    def _log_command(self, cmd_name: str, namespace: str, *args):
        """Log a command being sent to a drone."""
        timestamp = time.strftime("%H:%M:%S")
        args_str = ", ".join(str(a) for a in args) if args else ""
        log_entry = f"[{timestamp}] {namespace} <- {cmd_name}({args_str})"
        self._command_log.append(log_entry)
        
        # Keep log bounded
        if len(self._command_log) > 500:
            self._command_log = self._command_log[-500:]
        
        # Always log commands
        logger.info(f"🔹 CMD: {log_entry}")
        self._emit_event(namespace, f"CMD: {cmd_name}({args_str})")
        
        # Write to CSV if enabled
        if self._csv_logging_enabled and self._csv_commands_file:
            self._write_csv_command(timestamp, namespace, cmd_name, args_str)
    
    def get_command_log(self) -> List[str]:
        """Get the command log for display."""
        return self._command_log.copy()
    
    def clear_command_log(self):
        """Clear the command log."""
        self._command_log.clear()
    
    # ========================================================================
    # CSV LOGGING
    # ========================================================================
    
    def enable_csv_logging(self, enabled: bool = True, log_dir: str = None):
        """
        Enable or disable CSV logging.
        
        Args:
            enabled: Enable CSV logging
            log_dir: Directory to save CSV files (default: ~/wildperpetua_logs)
        """
        if log_dir:
            self._csv_log_dir = os.path.expanduser(log_dir)
        
        self._csv_logging_enabled = enabled
        
        if enabled:
            # Create log directory
            os.makedirs(self._csv_log_dir, exist_ok=True)
            
            # Generate session ID and file paths
            self._csv_session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
            self._csv_commands_file = os.path.join(
                self._csv_log_dir, f"commands_{self._csv_session_id}.csv"
            )
            self._csv_events_file = os.path.join(
                self._csv_log_dir, f"events_{self._csv_session_id}.csv"
            )
            self._csv_positions_file = os.path.join(
                self._csv_log_dir, f"positions_{self._csv_session_id}.csv"
            )
            
            # Initialize CSV files with headers
            self._init_csv_files()
            
            logger.info(f"📁 CSV logging enabled: {self._csv_log_dir}")
            logger.info(f"   Commands: {self._csv_commands_file}")
            logger.info(f"   Events: {self._csv_events_file}")
            logger.info(f"   Positions: {self._csv_positions_file}")
        else:
            logger.info("📁 CSV logging disabled")
    
    def _init_csv_files(self):
        """Initialize CSV files with headers."""
        # Commands file
        with open(self._csv_commands_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'datetime', 'namespace', 'command', 'arguments'])
        
        # Events file
        with open(self._csv_events_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'datetime', 'namespace', 'state', 'message'])
        
        # Positions file
        with open(self._csv_positions_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'datetime', 'namespace', 'latitude', 'longitude', 'altitude', 'heading', 'battery', 'state'])
    
    def _write_csv_command(self, time_str: str, namespace: str, command: str, args: str):
        """Write a command to the CSV file."""
        try:
            with open(self._csv_commands_file, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    time.time(),
                    datetime.now().isoformat(),
                    namespace,
                    command,
                    args
                ])
        except Exception as e:
            logger.error(f"Failed to write command to CSV: {e}")
    
    def _write_csv_event(self, namespace: str, state: str, message: str):
        """Write an event to the CSV file."""
        if not self._csv_logging_enabled or not self._csv_events_file:
            return
        try:
            with open(self._csv_events_file, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    time.time(),
                    datetime.now().isoformat(),
                    namespace,
                    state,
                    message
                ])
        except Exception as e:
            logger.error(f"Failed to write event to CSV: {e}")
    
    def log_position_to_csv(self, namespace: str, lat: float, lon: float, alt: float, 
                            heading: float = 0.0, battery: float = 0.0):
        """Log a position update to the CSV file."""
        if not self._csv_logging_enabled or not self._csv_positions_file:
            return
        try:
            state = ""
            if namespace in self.drone_missions:
                state = self.drone_missions[namespace].state.name
            
            with open(self._csv_positions_file, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    time.time(),
                    datetime.now().isoformat(),
                    namespace,
                    f"{lat:.8f}",
                    f"{lon:.8f}",
                    f"{alt:.2f}",
                    f"{heading:.1f}",
                    f"{battery:.1f}",
                    state
                ])
        except Exception as e:
            logger.error(f"Failed to write position to CSV: {e}")
    
    def get_csv_log_files(self) -> Dict[str, str]:
        """Get paths to current CSV log files."""
        return {
            'commands': self._csv_commands_file,
            'events': self._csv_events_file,
            'positions': self._csv_positions_file,
            'directory': self._csv_log_dir
        }
    
    # Command property wrappers that add logging
    @property
    def cmd_takeoff(self):
        return self._cmd_takeoff
    
    @cmd_takeoff.setter
    def cmd_takeoff(self, func):
        if func is None:
            self._cmd_takeoff = None
        else:
            def wrapper(ns):
                self._log_command("takeoff", ns)
                return func(ns)
            self._cmd_takeoff = wrapper
    
    @property
    def cmd_land(self):
        return self._cmd_land
    
    @cmd_land.setter
    def cmd_land(self, func):
        if func is None:
            self._cmd_land = None
        else:
            def wrapper(ns):
                self._log_command("land", ns)
                return func(ns)
            self._cmd_land = wrapper
    
    @property
    def cmd_rth(self):
        return self._cmd_rth
    
    @cmd_rth.setter
    def cmd_rth(self, func):
        if func is None:
            self._cmd_rth = None
        else:
            def wrapper(ns):
                self._log_command("rth", ns)
                return func(ns)
            self._cmd_rth = wrapper
    
    @property
    def cmd_goto_waypoint(self):
        return self._cmd_goto_waypoint
    
    @cmd_goto_waypoint.setter
    def cmd_goto_waypoint(self, func):
        if func is None:
            self._cmd_goto_waypoint = None
        else:
            def wrapper(ns, lat, lon, alt, yaw, speed=None):
                self._log_command("goto_waypoint", ns, f"lat={lat:.6f}", f"lon={lon:.6f}", f"alt={alt:.1f}m", f"yaw={yaw:.1f}°", f"speed={speed}")
                return func(ns, lat, lon, alt, yaw, speed) if speed else func(ns, lat, lon, alt, yaw)
            self._cmd_goto_waypoint = wrapper
    
    @property
    def cmd_goto_waypoint_dji_native(self):
        return self._cmd_goto_waypoint_dji_native
    
    @cmd_goto_waypoint_dji_native.setter
    def cmd_goto_waypoint_dji_native(self, func):
        if func is None:
            self._cmd_goto_waypoint_dji_native = None
        else:
            def wrapper(ns, lat, lon, alt, speed=None):
                self._log_command("goto_waypoint_dji_native", ns, f"lat={lat:.6f}", f"lon={lon:.6f}", f"alt={alt:.1f}m", f"speed={speed}")
                return func(ns, lat, lon, alt, speed) if speed else func(ns, lat, lon, alt)
            self._cmd_goto_waypoint_dji_native = wrapper
    
    @property
    def cmd_goto_altitude(self):
        return self._cmd_goto_altitude
    
    @cmd_goto_altitude.setter
    def cmd_goto_altitude(self, func):
        if func is None:
            self._cmd_goto_altitude = None
        else:
            def wrapper(ns, alt):
                self._log_command("goto_altitude", ns, f"{alt:.1f}m")
                return func(ns, alt)
            self._cmd_goto_altitude = wrapper
    
    @property
    def cmd_set_rth_altitude(self):
        return self._cmd_set_rth_altitude
    
    @cmd_set_rth_altitude.setter
    def cmd_set_rth_altitude(self, func):
        if func is None:
            self._cmd_set_rth_altitude = None
        else:
            def wrapper(ns, alt):
                self._log_command("set_rth_altitude", ns, f"{alt:.1f}m")
                return func(ns, alt)
            self._cmd_set_rth_altitude = wrapper
    
    @property
    def cmd_start_recording(self):
        return self._cmd_start_recording
    
    @cmd_start_recording.setter
    def cmd_start_recording(self, func):
        if func is None:
            self._cmd_start_recording = None
        else:
            def wrapper(ns):
                self._log_command("start_recording", ns)
                return func(ns)
            self._cmd_start_recording = wrapper
    
    @property
    def cmd_stop_recording(self):
        return self._cmd_stop_recording
    
    @cmd_stop_recording.setter
    def cmd_stop_recording(self, func):
        if func is None:
            self._cmd_stop_recording = None
        else:
            def wrapper(ns):
                self._log_command("stop_recording", ns)
                return func(ns)
            self._cmd_stop_recording = wrapper
    
    @property
    def cmd_set_gimbal_pitch(self):
        return self._cmd_set_gimbal_pitch
    
    @cmd_set_gimbal_pitch.setter
    def cmd_set_gimbal_pitch(self, func):
        if func is None:
            self._cmd_set_gimbal_pitch = None
        else:
            def wrapper(ns, pitch):
                self._log_command("set_gimbal_pitch", ns, f"{pitch:.1f}°")
                return func(ns, pitch)
            self._cmd_set_gimbal_pitch = wrapper
    
    @property
    def cmd_goto_yaw(self):
        return self._cmd_goto_yaw
    
    @cmd_goto_yaw.setter
    def cmd_goto_yaw(self, func):
        if func is None:
            self._cmd_goto_yaw = None
        else:
            def wrapper(ns, yaw):
                self._log_command("goto_yaw", ns, f"{yaw:.1f}°")
                return func(ns, yaw)
            self._cmd_goto_yaw = wrapper
    
    @property
    def cmd_abort(self):
        return self._cmd_abort
    
    @cmd_abort.setter
    def cmd_abort(self, func):
        if func is None:
            self._cmd_abort = None
        else:
            def wrapper(ns):
                self._log_command("abort", ns)
                return func(ns)
            self._cmd_abort = wrapper

    # ========================================================================
    # DISTANCE CALCULATIONS
    # ========================================================================
    
    def haversine_distance(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Calculate great-circle distance between two points in meters."""
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lat = math.radians(lat2 - lat1)
        delta_lon = math.radians(lon2 - lon1)
        
        a = math.sin(delta_lat / 2) ** 2 + \
            math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(delta_lon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        
        return self.EARTH_RADIUS * c
    
    def calculate_bearing(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Calculate bearing from point 1 to point 2 in degrees."""
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lon = math.radians(lon2 - lon1)
        
        x = math.sin(delta_lon) * math.cos(lat2_rad)
        y = math.cos(lat1_rad) * math.sin(lat2_rad) - \
            math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon)
        
        bearing = math.atan2(x, y)
        return (math.degrees(bearing) + 360) % 360
    
    # Speed constants
    VERTICAL_SPEED = 4.0      # m/s - DJI climb/descent rate
    HORIZONTAL_SPEED_PID = 5.0      # m/s - PID control mode
    HORIZONTAL_SPEED_NATIVE = 10.0  # m/s - DJI native trajectory mode
    
    def estimate_travel_time(self, distance: float, altitude: float = None, 
                             horizontal_speed: float = None, vertical_speed: float = None) -> float:
        """
        Estimate travel time in seconds, including climb time and wait times.
        
        This estimation matches the actual flight sequence:
        1. Wait 5s after takeoff before climbing
        2. Climb to altitude
        3. Wait 5s after reaching altitude before transit
        4. Horizontal transit to monitoring point
        
        Args:
            distance: Horizontal distance in meters
            altitude: Target altitude in meters (uses config monitoring_alt if None)
            horizontal_speed: Horizontal flight speed in m/s 
                              (default: 5 m/s for PID mode, 10 m/s for DJI native)
            vertical_speed: Vertical climb speed in m/s (default 4 m/s)
        
        Returns:
            Estimated travel time in seconds
        """
        # Wait times matching actual implementation
        WAIT_AFTER_TAKEOFF = 5.0  # time.sleep(5.0) in TAKING_OFF state
        WAIT_AFTER_CLIMB = 5.0    # time.sleep(5.0) in CLIMBING_TO_ALTITUDE state
        
        # Use defaults if not specified
        if vertical_speed is None:
            vertical_speed = self.VERTICAL_SPEED
        if horizontal_speed is None:
            # First try to get speed from UI configuration
            if self.get_configured_speed:
                horizontal_speed = self.get_configured_speed()
            else:
                # Fallback to defaults based on navigation mode
                if self.use_dji_native:
                    horizontal_speed = self.HORIZONTAL_SPEED_NATIVE
                else:
                    horizontal_speed = self.HORIZONTAL_SPEED_PID
        
        if horizontal_speed <= 0:
            return float('inf')
        
        # Use monitoring altitude from config if not specified
        if altitude is None:
            altitude = self.config.monitoring_alt if self.config.monitoring_alt > 0 else 50.0
        
        # Time to climb to altitude (from ground)
        climb_time = altitude / vertical_speed if vertical_speed > 0 else 0
        
        # Time for horizontal translation
        horizontal_time = distance / horizontal_speed
        
        # Total travel time = waits + climb + horizontal (sequential)
        # Matches actual flight sequence for accurate first-rotation estimation
        return WAIT_AFTER_TAKEOFF + climb_time + WAIT_AFTER_CLIMB + horizontal_time
    
    def get_average_travel_time(self) -> float:
        """Get average travel time from history."""
        if self.travel_time_history:
            return sum(self.travel_time_history) / len(self.travel_time_history)
        return 0.0
    
    def get_average_flight_time(self) -> float:
        """Get average total flight time from history (takeoff to RTH)."""
        if self.flight_time_history:
            return sum(self.flight_time_history) / len(self.flight_time_history)
        return 0.0
    
    # ========================================================================
    # PREFLIGHT CHECKS
    # ========================================================================
    
    def preflight_check(self, namespace: str) -> Tuple[bool, str]:
        """
        Perform preflight checks for a drone.
        
        Returns:
            Tuple of (passed, message)
        """
        errors = []
        
        # Check battery
        if self.get_battery_level:
            battery = self.get_battery_level(namespace)
            if battery < self.config.min_battery_to_launch:
                errors.append(f"Battery too low: {battery:.1f}% (min: {self.config.min_battery_to_launch}%)")
        
        # Check satellites
        if self.get_satellite_count:
            sats = self.get_satellite_count(namespace)
            if sats < self.config.min_satellites:
                errors.append(f"Not enough satellites: {sats} (min: {self.config.min_satellites})")
        
        # Check monitoring point is set
        if not self.config.monitoring_lat and not self.config.monitoring_lon:
            errors.append("Monitoring point not set")
        
        if errors:
            return False, "; ".join(errors)
        
        return True, "Preflight checks passed"
    
    # ========================================================================
    # SINGLE DRONE MISSION
    # ========================================================================
    
    def start_single_mission(
        self, 
        namespace: str, 
        monitoring_lat: float, 
        monitoring_lon: float,
        monitoring_alt: float = 50.0,
        rth_altitude: float = 50.0
    ) -> bool:
        """
        Start a monitoring mission for a single drone.
        
        Mission sequence:
        1. Set RTH altitude
        2. Take off
        3. Climb to RTH altitude
        4. Fly to monitoring point
        5. Start video recording 50m before arrival
        6. Monitor (hover at point)
        7. When commanded: Stop recording and RTH
        """
        # Configure mission
        self.config.monitoring_lat = monitoring_lat
        self.config.monitoring_lon = monitoring_lon
        self.config.monitoring_alt = monitoring_alt
        self.config.base_rth_altitude = rth_altitude
        
        # Reset vertical separation mission stopped flag for new mission
        self._vertical_separation_mission_stopped = False
        
        # Create drone mission status
        mission = DroneMissionStatus(
            namespace=namespace,
            state=MissionState.IDLE,
            assigned_altitude=rth_altitude
        )
        self.drone_missions[namespace] = mission
        self.drone_order = [namespace]
        self.current_drone_index = 0

        # Check drone health before starting
        is_healthy, issues = self.check_drone_health(namespace)
        if not is_healthy:
            mission.state = MissionState.ERROR
            mission.error_message = ", ".join(issues)
            self._emit_event(namespace, f"Cannot start: {mission.error_message}")
            return False        # Start mission thread if not running
        if not self._running:
            self._running = True
            self._mission_thread = threading.Thread(target=self._mission_loop, daemon=True)
            self._mission_thread.start()
        
        # Begin mission
        mission.state = MissionState.SETTING_RTH_ALTITUDE
        mission.mission_start_time = time.time()
        self._emit_event(namespace, "Mission starting...")
        
        return True
    
    # ========================================================================
    # RELAY MISSION
    # ========================================================================
    
    def start_relay_mission(
        self,
        drone_list: List[str],
        monitoring_lat: float,
        monitoring_lon: float,
        monitoring_alt: float = 50.0,
        base_rth_altitude: float = 50.0
    ) -> bool:
        """
        Start a perpetual monitoring mission with relay drones.
        
        Can start with 1 drone - additional drones can join later.
        Each subsequent drone flies at altitude +15m from the previous.
        """
        if len(drone_list) < 1:
            logger.error("Need at least 1 drone for relay mission")
            return False
        
        if len(drone_list) == 1:
            logger.warning("Starting relay with 1 drone - more drones should connect before battery runs low")
        
        # Configure mission
        self.config.monitoring_lat = monitoring_lat
        self.config.monitoring_lon = monitoring_lon
        self.config.monitoring_alt = monitoring_alt
        self.config.base_rth_altitude = base_rth_altitude
        
        # Reset vertical separation mission stopped flag for new mission
        self._vertical_separation_mission_stopped = False
        
        # IMPORTANT: Clear old mission state to prevent stale waypoints
        self.drone_missions.clear()
        
        # Initialize all drones
        self.drone_order = drone_list.copy()
        for i, ns in enumerate(drone_list):
            altitude = base_rth_altitude + (i * self.config.altitude_separation)
            self.drone_missions[ns] = DroneMissionStatus(
                namespace=ns,
                state=MissionState.IDLE,
                assigned_altitude=altitude
            )
        
        # Check first drone health before starting
        first_drone = drone_list[0]
        is_healthy, issues = self.check_drone_health(first_drone)
        if not is_healthy:
            self._emit_event(first_drone, f"Cannot start: {', '.join(issues)}")
            return False
        
        # Set relay state
        self.relay_state = RelayState.ACTIVE
        self.current_drone_index = 0
        
        # Reset relay launch flags
        self._relay_launch_pending = False
        self._pending_next_drone = ""
        self._pending_next_index = 0
        self._manual_swap_active = False
        
        # Start mission thread
        if not self._running:
            self._running = True
            self._mission_thread = threading.Thread(target=self._mission_loop, daemon=True)
            self._mission_thread.start()
        
        # Start first drone
        self.drone_missions[first_drone].state = MissionState.SETTING_RTH_ALTITUDE
        self.drone_missions[first_drone].mission_start_time = time.time()
        
        logger.info(f"Relay mission started with {len(drone_list)} drones")
        return True
    
    def stop_mission(self):
        """Stop the current mission and recall all drones."""
        self.relay_state = RelayState.STOPPED
        
        for ns, mission in self.drone_missions.items():
            if mission.state not in [MissionState.IDLE, MissionState.COMPLETED, MissionState.ABORTED]:
                # Stop recording if active
                if mission.video_started and self.cmd_stop_recording:
                    self.cmd_stop_recording(ns)
                
                # Re-set RTH altitude before RTH command
                if self.cmd_set_rth_altitude:
                    self.cmd_set_rth_altitude(ns, mission.assigned_altitude)
                    time.sleep(0.3)
                
                # Command RTH
                if self.cmd_rth:
                    self.cmd_rth(ns)
                
                mission.state = MissionState.ABORTED
                self._update_status(ns, mission.state, f"Mission aborted - RTH at {mission.assigned_altitude}m")
                self._emit_event(ns, f"Mission aborted - RTH at {mission.assigned_altitude}m")
        
        logger.info("Mission stopped - all drones returning home")
    
    def abort_drone_mission(self, namespace: str):
        """Abort mission for a specific drone."""
        if namespace in self.drone_missions:
            mission = self.drone_missions[namespace]
            
            if mission.video_started and self.cmd_stop_recording:
                self.cmd_stop_recording(namespace)
            
            if self.cmd_abort:
                self.cmd_abort(namespace)
            
            mission.state = MissionState.ABORTED
            self._emit_event(namespace, "Mission aborted")
    
    def abort_drone_mission(self, namespace: str, reason: str = "Mission aborted"):
        """Abort mission for a specific drone with custom reason."""
        if namespace in self.drone_missions:
            mission = self.drone_missions[namespace]
            
            if mission.video_started and self.cmd_stop_recording:
                self.cmd_stop_recording(namespace)
            
            # Trigger RTH instead of just abort (safer)
            if self.cmd_rth:
                self.cmd_rth(namespace)
            elif self.cmd_abort:
                self.cmd_abort(namespace)
            
            mission.state = MissionState.ABORTED
            self._emit_event(namespace, reason)
    
    def pin_drone_location(self, namespace: str) -> bool:
        """
        Pin drone's current location as the new monitoring point.
        
        This is useful for free flight mode - pilot flies to a good location,
        then pins it to switch to monitoring point mode.
        
        Args:
            namespace: The drone to use as the location source
            
        Returns:
            True if location was pinned successfully
        """
        if not self.get_drone_position:
            self._emit_event(namespace, "Cannot pin location - position not available")
            return False
        
        lat, lon, alt = self.get_drone_position(namespace)
        
        if lat == 0.0 and lon == 0.0:
            self._emit_event(namespace, "Cannot pin location - invalid coordinates")
            return False
        
        # Get heading if available
        heading = 0.0
        if self.get_drone_heading:
            heading = self.get_drone_heading(namespace)
        
        # Update config with new monitoring point
        self.config.monitoring_lat = lat
        self.config.monitoring_lon = lon
        self.config.monitoring_heading = heading
        
        # Switch to monitoring point mode
        self.mission_mode = MissionMode.MONITORING_POINT
        
        # Update target for any active missions
        for ns, mission in self.drone_missions.items():
            if mission.state in [MissionState.MONITORING, MissionState.TRANSIT_TO_MONITORING, MissionState.APPROACHING_POINT]:
                mission.target_lat = lat
                mission.target_lon = lon
        
        self._emit_event(namespace, f"📍 Location pinned: ({lat:.6f}, {lon:.6f}) heading={heading:.0f}°")
        logger.info(f"Pinned location from {namespace}: ({lat}, {lon}, {alt}) heading={heading}")
        
        return True
    
    def add_drone_to_relay(self, namespace: str) -> bool:
        """
        Add a new drone to an ongoing relay mission.
        
        The drone will be queued and take over when the current drone's battery gets low.
        
        Args:
            namespace: The drone namespace to add
            
        Returns:
            True if drone was added, False otherwise
        """
        if self.relay_state != RelayState.ACTIVE:
            logger.warning(f"Cannot add drone {namespace} - no active relay mission")
            return False
        
        if namespace in self.drone_missions:
            logger.warning(f"Drone {namespace} already in relay mission")
            return False
        
        # Assign altitude: base + (position in order * separation)
        # New drones go at the end of the queue
        position = len(self.drone_order)
        altitude = self.config.base_rth_altitude + (position * self.config.altitude_separation)
        
        # Add to mission tracking
        self.drone_missions[namespace] = DroneMissionStatus(
            namespace=namespace,
            state=MissionState.IDLE,  # Will be activated when needed
            assigned_altitude=altitude
        )
        
        # Add to relay order
        self.drone_order.append(namespace)
        
        self._emit_event(namespace, f"Added to relay queue (position {position + 1}, alt {altitude}m)")
        logger.info(f"Drone {namespace} added to relay mission at position {position + 1}")
        
        return True
    
    def is_drone_in_mission(self, namespace: str) -> bool:
        """Check if a drone is part of the current mission."""
        return namespace in self.drone_missions

    # ========================================================================
    # MISSION STATE MACHINE
    # ========================================================================
    
    def _mission_loop(self):
        """Main mission control loop."""
        while self._running:
            try:
                for namespace, mission in list(self.drone_missions.items()):
                    self._update_drone_mission(namespace, mission)
                
                # Update relay logic
                if self.relay_state == RelayState.ACTIVE:
                    self._update_relay_logic()
                    # Check vertical separation during relay operations
                    self._check_vertical_separation()
                
                time.sleep(self._update_interval)
                
            except Exception as e:
                logger.error(f"Mission loop error: {e}")
    
    def _update_drone_mission(self, namespace: str, mission: DroneMissionStatus):
        """Update a single drone's mission state."""
        
        if mission.state == MissionState.IDLE:
            return
        
        elif mission.state == MissionState.SETTING_RTH_ALTITUDE:
            # Set RTH altitude before takeoff - CRITICAL for safe returns
            # Non-blocking: send commands at 0.5s intervals, then wait 1s before transition
            
            now = time.time()
            
            # Initialize state entry time on first call
            if mission.state_entry_time == 0.0:
                mission.state_entry_time = now
                mission.rth_altitude_cmd_count = 0
                mission.rth_altitude_last_cmd_time = 0.0
                self._emit_event(namespace, f"Setting RTH altitude to {mission.assigned_altitude}m")
            
            # Send RTH altitude command up to 3 times, 0.5s apart
            if self.cmd_set_rth_altitude:
                if mission.rth_altitude_cmd_count < 3:
                    if now - mission.rth_altitude_last_cmd_time >= 0.5:
                        self.cmd_set_rth_altitude(namespace, mission.assigned_altitude)
                        mission.rth_altitude_cmd_count += 1
                        mission.rth_altitude_last_cmd_time = now
                elif mission.rth_altitude_cmd_count == 3:
                    # All commands sent, emit confirmation once
                    self._emit_event(namespace, f"RTH altitude confirmed: {mission.assigned_altitude}m")
                    mission.rth_altitude_cmd_count = 4  # Mark as confirmed
            else:
                if mission.rth_altitude_cmd_count == 0:
                    self._emit_event(namespace, "WARNING: cmd_set_rth_altitude not available!")
                    mission.rth_altitude_cmd_count = 4  # Skip to wait phase
            
            self._update_status(namespace, mission.state, f"RTH altitude set to {mission.assigned_altitude}m")
            
            # After all commands sent + 1s wait, transition to takeoff
            # Total time: ~2.5s (3 commands at 0.5s intervals + 1s wait)
            time_since_last_cmd = now - mission.rth_altitude_last_cmd_time
            if mission.rth_altitude_cmd_count >= 3 and time_since_last_cmd >= 1.0:
                mission.state = MissionState.TAKING_OFF
                mission.state_entry_time = 0.0  # Reset for next state
                self._update_status(namespace, mission.state, "Taking off...")
        
        elif mission.state == MissionState.TAKING_OFF:
            # Non-blocking takeoff sequence:
            # - Send first takeoff command immediately
            # - Send second takeoff command after 2s
            # - Wait until 7s total before transitioning to climb
            
            now = time.time()
            
            # Initialize state entry time on first call
            if mission.state_entry_time == 0.0:
                mission.state_entry_time = now
                mission.takeoff_cmd_sent = False
                mission.takeoff_second_cmd_sent = False
            
            elapsed = now - mission.state_entry_time
            
            takeoff_wait = 7.0
            second_cmd_delay = 2.0
            
            # Send first takeoff command immediately
            if not mission.takeoff_cmd_sent and self.cmd_takeoff:
                self.cmd_takeoff(namespace)
                mission.takeoff_cmd_sent = True
            
            # Send second takeoff command after delay
            if elapsed >= second_cmd_delay and not mission.takeoff_second_cmd_sent and self.cmd_takeoff:
                self.cmd_takeoff(namespace)
                mission.takeoff_second_cmd_sent = True
            
            # After wait time, transition
            if elapsed >= takeoff_wait:
                # Start travel time measurement from beginning of climb
                # (includes climb + transit for accurate relay timing estimation)
                mission.transit_start_time = time.time()
                
                # Transition to climbing and send altitude command
                mission.state = MissionState.CLIMBING_TO_ALTITUDE
                mission.state_entry_time = 0.0  # Reset for next state
                mission.climb_wait_started = False  # Reset climb wait flag
                
                if self.cmd_goto_altitude:
                    self.cmd_goto_altitude(namespace, mission.assigned_altitude)
                    self._emit_event(namespace, f"Climbing to {mission.assigned_altitude}m")
                
                self._update_status(namespace, mission.state, f"Climbing to {mission.assigned_altitude}m")
        
        elif mission.state == MissionState.CLIMBING_TO_ALTITUDE:
            # Check if altitude reached - prefer using the altitude_reached flag from drone
            altitude_reached = False
            current_alt = 0.0
            target_alt = mission.assigned_altitude
            now = time.time()
            
            if self.get_altitude_reached:
                altitude_reached = altitude_reached or self.get_altitude_reached(namespace)
            
            if self.get_drone_position:
                _, _, current_alt = self.get_drone_position(namespace)
                # Fallback: also check if altitude is above target (in case flag not set)
                if current_alt >= (target_alt - self.config.altitude_tolerance):
                    altitude_reached = True
            
            # Log altitude progress periodically (every 5 seconds)
            if now - mission.last_alt_log_time > 5.0:
                mission.last_alt_log_time = now
                self._emit_event(namespace, f"Climbing: {current_alt:.1f}m / {target_alt:.1f}m (reached={altitude_reached})")
            
            if altitude_reached:
                # Non-blocking: wait 5 seconds before starting transit
                wait_time = 5.0
                if not mission.climb_wait_started:
                    mission.climb_wait_started = True
                    mission.state_entry_time = now
                    self._emit_event(namespace, f"Altitude reached, stabilizing for {wait_time}s...")
                elif now - mission.state_entry_time >= wait_time:
                    # Wait elapsed - check mission mode
                    mission.state_entry_time = 0.0  # Reset for next state
                    
                    if self.mission_mode == MissionMode.FREE_FLIGHT:
                        # FREE FLIGHT MODE behavior depends on whether this is a relay drone
                        if mission.replacing_drone:
                            # RELAY DRONE in Free Flight: Navigate to current drone's position
                            # Capture target position from the drone we're replacing
                            old_ns = mission.replacing_drone
                            if self.get_drone_position:
                                old_lat, old_lon, _ = self.get_drone_position(old_ns)
                                mission.relay_target_lat = old_lat
                                mission.relay_target_lon = old_lon
                                mission.target_lat = old_lat
                                mission.target_lon = old_lon
                                self._emit_event(namespace, f"FREE FLIGHT RELAY: Flying to {old_ns} position ({old_lat:.6f}, {old_lon:.6f})")
                            if self.get_drone_heading:
                                mission.relay_target_heading = self.get_drone_heading(old_ns)
                                mission.target_heading = mission.relay_target_heading
                            
                            # Put old drone in WAITING_FOR_RELAY state
                            old_mission = self.drone_missions.get(old_ns)
                            if old_mission:
                                old_mission.state = MissionState.WAITING_FOR_RELAY
                                self._update_status(old_ns, old_mission.state, f"Waiting for {namespace} to arrive")
                                self._emit_event(old_ns, f"Waiting for {namespace} to arrive for relay handoff")
                            
                            # Start transit to the other drone's position
                            mission.state = MissionState.TRANSIT_TO_MONITORING
                            mission.transit_start_time = time.time()
                            self._start_transit(namespace, mission)
                            self._update_status(namespace, mission.state, "Transit to relay position")
                        else:
                            # FIRST DRONE in Free Flight: Start recording and give pilot control
                            mission.state = MissionState.MONITORING
                            mission.monitoring_start_time = time.time()
                            
                            # Start video recording immediately
                            if self.cmd_start_recording and not mission.video_started:
                                self.cmd_start_recording(namespace)
                                mission.video_started = True
                                self._emit_event(namespace, "FREE FLIGHT: Recording started")
                            
                            self._emit_event(namespace, "FREE FLIGHT: Pilot has control. Drone holding position.")
                            self._update_status(namespace, mission.state, "Free flight - pilot control")
                    else:
                        # MONITORING POINT MODE: Navigate to monitoring point (default behavior)
                        mission.state = MissionState.TRANSIT_TO_MONITORING
                        # Note: transit_start_time was set at beginning of climb for accurate total travel time
                        self._start_transit(namespace, mission)
                        self._update_status(namespace, mission.state, "Transit to monitoring point")
        
        elif mission.state == MissionState.TRANSIT_TO_MONITORING:
            # Log transit progress periodically
            now = time.time()
            
            if self.get_drone_position:
                lat, lon, _ = self.get_drone_position(namespace)
                target_lat = mission.target_lat if mission.target_lat != 0 else self.config.monitoring_lat
                target_lon = mission.target_lon if mission.target_lon != 0 else self.config.monitoring_lon
                distance = self.haversine_distance(lat, lon, target_lat, target_lon)
                
                if now - mission.last_transit_log_time > 5.0:
                    mission.last_transit_log_time = now
                    self._emit_event(namespace, f"Transit: {distance:.1f}m to target")
            
            # CONTINUOUS POSITION TRACKING for relay replacement drones
            # Update target more frequently in Free Flight mode (2s) vs Monitoring Point mode (10s)
            # because in Free Flight the pilot is actively flying and position changes rapidly
            update_interval = 2.0 if self.mission_mode == MissionMode.FREE_FLIGHT else self._trajectory_update_interval
            if mission.replacing_drone and self.relay_state == RelayState.ACTIVE:
                if now - self._last_trajectory_update_time >= update_interval:
                    self._last_trajectory_update_time = now
                    self._update_replacement_target(namespace, mission)
            
            self._check_approach(namespace, mission)
        
        elif mission.state == MissionState.APPROACHING_POINT:
            # Already recording, check if reached (by flag or distance)
            reached = False
            distance = float('inf')
            now = time.time()
            
            # Check waypoint_reached flag
            if self.get_waypoint_reached and self.get_waypoint_reached(namespace):
                reached = True
            
            # Also check by distance as fallback
            if self.get_drone_position:
                lat, lon, _ = self.get_drone_position(namespace)
                target_lat = mission.target_lat if mission.target_lat != 0 else self.config.monitoring_lat
                target_lon = mission.target_lon if mission.target_lon != 0 else self.config.monitoring_lon
                distance = self.haversine_distance(lat, lon, target_lat, target_lon)
                if distance <= self.config.position_tolerance:
                    reached = True
            
            # Log progress periodically
            if now - mission.last_approach_log_time > 3.0:
                mission.last_approach_log_time = now
                self._emit_event(namespace, f"Approaching: {distance:.1f}m to target (tolerance: {self.config.position_tolerance}m)")
            
            if reached:
                mission.state = MissionState.MONITORING
                mission.monitoring_start_time = time.time()
                mission.state_entry_time = 0.0  # Reset for debug mode timing
                
                # Record actual travel time
                mission.actual_travel_time = time.time() - mission.transit_start_time
                self.travel_time_history.append(mission.actual_travel_time)
                if len(self.travel_time_history) > 10:
                    self.travel_time_history.pop(0)
                
                self._update_status(namespace, mission.state, "Monitoring point reached")
                self._emit_event(namespace, f"Monitoring started. Travel time: {mission.actual_travel_time:.1f}s")
                
                # RELAY HANDOFF: If this drone was replacing another, determine which drone does camera sync
                # The LOWEST drone always does the 360° spin for video synchronization
                if mission.replacing_drone:
                    old_ns = mission.replacing_drone
                    old_mission = self.drone_missions.get(old_ns)
                    
                    if old_mission and old_mission.state == MissionState.WAITING_FOR_RELAY:
                        # Get altitudes to determine which drone is lower
                        new_alt = mission.assigned_altitude
                        old_alt = old_mission.assigned_altitude
                        self._emit_event(namespace, f"Relay handoff: {old_ns} ({old_alt}m) <-> {namespace} ({new_alt}m)")
                        
                        # Record actual flight time for the old drone
                        if old_mission.mission_start_time:
                            actual_flight_time = time.time() - old_mission.mission_start_time
                            self.flight_time_history.append(actual_flight_time)
                            if len(self.flight_time_history) > 10:
                                self.flight_time_history.pop(0)
                            self._emit_event(old_ns, f"Flight time recorded: {actual_flight_time:.0f}s")
                        
                        # Determine which drone is LOWER - that one does the camera sync spin
                        # The OTHER drone waits, then departs (RTH) after spin completes
                        if old_alt <= new_alt:
                            # Old drone (departing) is lower or equal - it does the spin then RTH
                            sync_ns = old_ns
                            sync_mission = old_mission
                            sync_mission.camera_sync_next_state = "RTH"
                            sync_mission.camera_sync_partner_drone = ""  # No partner to notify, spinning drone goes RTH itself
                            sync_mission.camera_sync_top_drone_ns = namespace  # New drone is the top drone
                            sync_mission.camera_sync_new_drone_ns = namespace  # New drone that will continue monitoring
                            self._emit_event(old_ns, f"Lower drone ({old_alt}m) - will do 360° camera sync then RTH")
                            # Capture old drone's gimbal pitch to transfer to new drone after spin
                            if self.get_drone_gimbal_pitch:
                                sync_mission.camera_sync_old_drone_gimbal = self.get_drone_gimbal_pitch(old_ns)
                                self._emit_event(old_ns, f"Old drone gimbal pitch captured: {sync_mission.camera_sync_old_drone_gimbal:.1f}°")
                            # New drone (top drone) - set gimbal to -90° during sync
                            if self.get_drone_gimbal_pitch and self.cmd_set_gimbal_pitch:
                                sync_mission.camera_sync_top_drone_previous_gimbal = self.get_drone_gimbal_pitch(namespace)
                                self.cmd_set_gimbal_pitch(namespace, -90.0)
                                self._emit_event(namespace, f"Top drone gimbal set to -90° (was {sync_mission.camera_sync_top_drone_previous_gimbal:.1f}°)")
                        else:
                            # New drone (arriving) is lower - it does the spin, then continues monitoring
                            # Old drone (top drone) waits for spin to complete before RTH
                            sync_ns = namespace
                            sync_mission = mission
                            sync_mission.camera_sync_next_state = "MONITORING"
                            sync_mission.camera_sync_partner_drone = old_ns  # Old drone will be sent to RTH after spin
                            sync_mission.camera_sync_top_drone_ns = old_ns  # Old drone is the top drone
                            sync_mission.camera_sync_new_drone_ns = namespace  # New drone (self) will continue monitoring
                            self._emit_event(namespace, f"Lower drone ({new_alt}m) - will do 360° camera sync then monitor")
                            self._emit_event(old_ns, f"Higher drone ({old_alt}m) - waiting for spin to complete before RTH")
                            # Capture old drone's gimbal pitch to transfer to new drone after spin
                            if self.get_drone_gimbal_pitch:
                                sync_mission.camera_sync_old_drone_gimbal = self.get_drone_gimbal_pitch(old_ns)
                                self._emit_event(old_ns, f"Old drone gimbal pitch captured: {sync_mission.camera_sync_old_drone_gimbal:.1f}°")
                            # Old drone (top drone) - set gimbal to -90° during sync
                            if self.get_drone_gimbal_pitch and self.cmd_set_gimbal_pitch:
                                sync_mission.camera_sync_top_drone_previous_gimbal = self.get_drone_gimbal_pitch(old_ns)
                                self.cmd_set_gimbal_pitch(old_ns, -90.0)
                                self._emit_event(old_ns, f"Top drone gimbal set to -90° (was {sync_mission.camera_sync_top_drone_previous_gimbal:.1f}°)")
                            # Keep old drone in WAITING_FOR_RELAY - it will be sent to RTH after spin completes
                        
                        # Start camera sync on the LOWER drone
                        sync_mission.state = MissionState.CAMERA_SYNC
                        sync_mission.camera_sync_start_time = time.time()
                        sync_mission.camera_sync_yaw_started = False
                        sync_mission.camera_sync_yaw_phase2_started = False
                        sync_mission.camera_sync_yaw_completed = False
                        
                        # Store initial heading for 360° rotation
                        if self.get_drone_heading:
                            sync_mission.camera_sync_initial_heading = self.get_drone_heading(sync_ns)
                        
                        self._update_status(sync_ns, sync_mission.state, "Camera sync - waiting 10s before yaw")
                        self._emit_event(sync_ns, "Camera sync started - waiting 10 seconds before 360° yaw")
                    
                    # Clear the replacing_drone field
                    mission.replacing_drone = ""
        
        elif mission.state == MissionState.MONITORING:
            # Drone is monitoring - relay logic handles transition
            pass
        
        elif mission.state == MissionState.WAITING_FOR_RELAY:
            # Drone waiting for relay - normally waits for new drone to arrive
            # DEBUG MODE: This state transitions automatically when relay drone reaches MONITORING
            pass
        
        elif mission.state == MissionState.CAMERA_SYNC:
            # Camera synchronization sequence:
            # 1. Wait 10 seconds after relay drone arrives
            # 2. Perform 360° yaw rotation
            # 3. Wait 10 seconds after yaw completes
            # 4. Command RTH
            
            elapsed = time.time() - mission.camera_sync_start_time
            
            # Phase 1: Wait 10 seconds before starting yaw
            wait_before_yaw = 10.0
            if not mission.camera_sync_yaw_started:
                if elapsed >= wait_before_yaw:
                    mission.camera_sync_yaw_started = True
                    
                    # Check if camera sync rotation is enabled
                    if not self.config.camera_sync_enabled:
                        # Skip rotation, go directly to completed
                        self._emit_event(namespace, "Camera sync rotation disabled - skipping 360° yaw")
                        mission.camera_sync_yaw_phase2_started = True
                        mission.camera_sync_yaw_completed = True
                        mission.camera_sync_start_time = time.time()  # Reset timer for post-wait phase
                        self._update_status(namespace, mission.state, "Camera sync - waiting 10s (rotation skipped)")
                    elif self.cmd_goto_yaw:
                        # Command 360° yaw rotation
                        # Calculate target heading: current heading + 360° (full rotation)
                        target_heading = (mission.camera_sync_initial_heading + 360.0) % 360.0
                        # Since we want a full rotation, we'll do it in steps or use a special yaw command
                        # For now, we rotate to initial + 180, then to initial (which completes the circle)
                        self.cmd_goto_yaw(namespace, (mission.camera_sync_initial_heading + 180.0) % 360.0)
                        self._emit_event(namespace, "Starting 360° yaw rotation (phase 1/2)")
                        self._update_status(namespace, mission.state, "Camera sync - 360° yaw in progress")
                        mission.camera_sync_start_time = time.time()  # Reset timer for yaw phase
                    else:
                        self._emit_event(namespace, "Warning: cmd_goto_yaw not available, skipping rotation")
                        mission.camera_sync_yaw_completed = True
                        mission.camera_sync_start_time = time.time()  # Reset timer for post-wait phase
            
            # Phase 2a: Monitor yaw progress for first half of rotation (0° -> 180°)
            elif not mission.camera_sync_yaw_phase2_started:
                # Check if yaw reached (first half of rotation)
                yaw_reached = False
                if self.get_drone_heading:
                    current_heading = self.get_drone_heading(namespace)
                    target_heading = (mission.camera_sync_initial_heading + 180.0) % 360.0
                    heading_diff = abs(current_heading - target_heading)
                    # Account for wrap-around at 360°
                    if heading_diff > 180:
                        heading_diff = 360 - heading_diff
                    yaw_reached = heading_diff < 10.0  # Within 10 degrees
                
                # Also timeout after 30 seconds if yaw command not responding
                if yaw_reached or elapsed >= 30.0:
                    if elapsed >= 30.0 and not yaw_reached:
                        self._emit_event(namespace, "Yaw phase 1 timeout, continuing...")
                    
                    # Start second half of rotation (180° -> 360°/0°)
                    if self.cmd_goto_yaw:
                        self.cmd_goto_yaw(namespace, mission.camera_sync_initial_heading)
                        self._emit_event(namespace, "360° yaw rotation (phase 2/2)")
                    
                    mission.camera_sync_yaw_phase2_started = True
                    mission.camera_sync_start_time = time.time()  # Reset timer for phase 2
                    self._update_status(namespace, mission.state, "Camera sync - yaw phase 2/2")
            
            # Phase 2b: Monitor yaw progress for second half of rotation (180° -> 0°)
            elif not mission.camera_sync_yaw_completed:
                # Check if yaw reached (second half - back to initial heading)
                yaw_reached = False
                if self.get_drone_heading:
                    current_heading = self.get_drone_heading(namespace)
                    target_heading = mission.camera_sync_initial_heading
                    heading_diff = abs(current_heading - target_heading)
                    # Account for wrap-around at 360°
                    if heading_diff > 180:
                        heading_diff = 360 - heading_diff
                    yaw_reached = heading_diff < 10.0  # Within 10 degrees
                
                # Also timeout after 30 seconds if yaw command not responding
                if yaw_reached or elapsed >= 30.0:
                    if elapsed >= 30.0 and not yaw_reached:
                        self._emit_event(namespace, "Yaw phase 2 timeout, continuing...")
                    
                    mission.camera_sync_yaw_completed = True
                    mission.camera_sync_start_time = time.time()  # Reset timer for post-yaw wait
                    self._emit_event(namespace, "360° yaw rotation completed")
                    self._update_status(namespace, mission.state, "Camera sync - waiting 10s before transition")
            
            # Phase 3: Wait 10 seconds after yaw, then transition to next state (RTH or MONITORING)
            else:
                phase3_wait = 10.0
                if elapsed >= phase3_wait:
                    self._emit_event(namespace, "360° yaw wait complete")
                    
                    if mission.camera_sync_next_state == "MONITORING":
                        # This drone continues monitoring (it was the arriving lower drone)
                        mission.state = MissionState.MONITORING
                        mission.monitoring_start_time = time.time()
                        mission.state_entry_time = 0.0  # Reset for debug mode
                        
                        # Transfer old drone's gimbal pitch to the new monitoring drone
                        if self.cmd_set_gimbal_pitch and mission.camera_sync_old_drone_gimbal != 0.0:
                            self.cmd_set_gimbal_pitch(namespace, mission.camera_sync_old_drone_gimbal)
                            self._emit_event(namespace, f"Gimbal pitch inherited from previous drone: {mission.camera_sync_old_drone_gimbal:.1f}°")
                        
                        # In Free Flight mode, pilot now has control
                        if self.mission_mode == MissionMode.FREE_FLIGHT:
                            self._update_status(namespace, mission.state, "FREE FLIGHT: Camera sync complete, pilot has control")
                            self._emit_event(namespace, "FREE FLIGHT: Camera sync complete - pilot now has control")
                        else:
                            self._update_status(namespace, mission.state, "Monitoring after camera sync")
                            self._emit_event(namespace, "Camera sync complete - now monitoring")
                        
                        # Now send the partner drone (old/departing drone) to RTH
                        if mission.camera_sync_partner_drone:
                            partner_ns = mission.camera_sync_partner_drone
                            if partner_ns in self.drone_missions:
                                partner_mission = self.drone_missions[partner_ns]
                                self._emit_event(partner_ns, "Spin complete - now departing")
                                
                                # Stop recording on partner
                                if partner_mission.video_started and self.cmd_stop_recording:
                                    self.cmd_stop_recording(partner_ns)
                                    self._emit_event(partner_ns, "Recording stopped")
                                
                                # Set RTH altitude
                                if self.cmd_set_rth_altitude:
                                    self._emit_event(partner_ns, f"Setting RTH altitude: {partner_mission.assigned_altitude}m")
                                    self.cmd_set_rth_altitude(partner_ns, partner_mission.assigned_altitude)
                                    time.sleep(0.5)
                                
                                # Command RTH
                                if self.cmd_rth:
                                    self._emit_event(partner_ns, f"Commanding RTH at {partner_mission.assigned_altitude}m")
                                    self.cmd_rth(partner_ns)
                                
                                partner_mission.state = MissionState.RETURNING_HOME
                                partner_mission.state_entry_time = 0.0  # Reset for debug mode
                                partner_mission.rth_start_time = time.time()
                                self._update_status(partner_ns, partner_mission.state, "Returning home after relay handoff")
                                
                                # Clear manual swap mode - swap is complete
                                self._manual_swap_active = False
                            
                            # Clear the partner reference
                            mission.camera_sync_partner_drone = ""
                    else:
                        # This drone goes to RTH (default, it was the departing drone)
                        # Transfer old drone's gimbal pitch to the new monitoring drone
                        if mission.camera_sync_new_drone_ns and self.cmd_set_gimbal_pitch:
                            new_drone_ns = mission.camera_sync_new_drone_ns
                            if mission.camera_sync_old_drone_gimbal != 0.0:
                                self.cmd_set_gimbal_pitch(new_drone_ns, mission.camera_sync_old_drone_gimbal)
                                self._emit_event(new_drone_ns, f"Gimbal pitch inherited from previous drone: {mission.camera_sync_old_drone_gimbal:.1f}°")
                        
                        # Stop recording
                        if mission.video_started and self.cmd_stop_recording:
                            self.cmd_stop_recording(namespace)
                            self._emit_event(namespace, "Recording stopped")
                        
                        # Re-set RTH altitude before RTH command
                        if self.cmd_set_rth_altitude:
                            self._emit_event(namespace, f"Confirming RTH altitude: {mission.assigned_altitude}m")
                            self.cmd_set_rth_altitude(namespace, mission.assigned_altitude)
                            time.sleep(0.5)
                        
                        # Command RTH
                        if self.cmd_rth:
                            self._emit_event(namespace, f"Commanding RTH at {mission.assigned_altitude}m")
                            self.cmd_rth(namespace)
                        
                        mission.state = MissionState.RETURNING_HOME
                        mission.state_entry_time = 0.0  # Reset for debug mode
                        mission.rth_start_time = time.time()
                        self._update_status(namespace, mission.state, "Returning home after camera sync")
                        self._emit_event(namespace, "Camera sync complete - returning home")
                        
                        # Clear manual swap mode - swap is complete
                        self._manual_swap_active = False
        
        elif mission.state == MissionState.RETURNING_HOME:
            # Drone returning home - landing detection is handled by GUI
            pass
        
        elif mission.state == MissionState.COMPLETED:
            # Mission completed - drone has landed
            pass
    
    def _start_transit(self, namespace: str, mission: DroneMissionStatus):
        """Start transit to monitoring point (or current monitoring drone's position for relay)."""
        if not self.get_drone_position:
            return
            
        lat, lon, _ = self.get_drone_position(namespace)
        
        # Determine target position
        # Default to configured monitoring point
        target_lat = self.config.monitoring_lat
        target_lon = self.config.monitoring_lon
        # Use the drone's assigned altitude (which includes vertical separation for safety)
        target_alt = mission.assigned_altitude
        target_heading = self.config.monitoring_heading
        
        # For relay missions with a replacing_drone, use the captured snapshot position
        # This was captured in _launch_relay_drone at the moment of launch
        if mission.replacing_drone and mission.relay_target_lat != 0.0:
            target_lat = mission.relay_target_lat
            target_lon = mission.relay_target_lon
            target_heading = mission.relay_target_heading
            self._emit_event(namespace, f"Flying to captured position of {mission.replacing_drone}: ({target_lat:.6f}, {target_lon:.6f}) heading={target_heading:.1f}°")
        elif self.relay_state == RelayState.ACTIVE:
            # Fallback: try to find a currently monitoring drone (shouldn't happen in normal relay)
            current_monitoring_ns = self._get_currently_monitoring_drone(exclude=namespace)
            if current_monitoring_ns and self.get_drone_position:
                mon_lat, mon_lon, _ = self.get_drone_position(current_monitoring_ns)
                if mon_lat != 0.0 or mon_lon != 0.0:
                    target_lat = mon_lat
                    target_lon = mon_lon
                    self._emit_event(namespace, f"Target: current position of {current_monitoring_ns}")
                if self.get_drone_heading:
                    target_heading = self.get_drone_heading(current_monitoring_ns)
                    self._emit_event(namespace, f"Target heading synced to {target_heading:.1f}° from {current_monitoring_ns}")
        
        # Store target position and heading for trajectory re-evaluation
        mission.target_lat = target_lat
        mission.target_lon = target_lon
        mission.target_alt = target_alt
        mission.target_heading = target_heading
        
        # Estimate travel time
        distance = self.haversine_distance(lat, lon, target_lat, target_lon)
        mission.estimated_travel_time = self.estimate_travel_time(distance)
        
        # Send waypoint command (PID or DJI Native based on setting)
        if self.use_dji_native and self.cmd_goto_waypoint_dji_native:
            # DJI Native: faster but needs separate yaw command after arrival
            self.cmd_goto_waypoint_dji_native(
                namespace,
                target_lat,
                target_lon,
                target_alt
            )
            mode_str = "DJI Native"
        
        elif self.cmd_goto_waypoint:
            # PID: includes yaw control, use target heading
            self.cmd_goto_waypoint(
                namespace,
                target_lat,
                target_lon,
                target_alt,
                target_heading
            )
            mode_str = "PID"
        else:
            mode_str = "unknown"
        
        self._emit_event(namespace, f"Transit to monitoring point ({distance:.0f}m, ~{mission.estimated_travel_time:.0f}s) [{mode_str}] heading={target_heading:.0f}°")
    
    def _send_updated_trajectory(self, namespace: str, mission: DroneMissionStatus):
        """Send updated waypoint command during transit (for trajectory re-evaluation)."""
        target_lat = mission.target_lat
        target_lon = mission.target_lon
        target_alt = mission.target_alt
        target_heading = mission.target_heading
        
        if self.use_dji_native and self.cmd_goto_waypoint_dji_native:
            self.cmd_goto_waypoint_dji_native(
                namespace,
                target_lat,
                target_lon,
                target_alt
            )
        elif self.cmd_goto_waypoint:
            self.cmd_goto_waypoint(
                namespace,
                target_lat,
                target_lon,
                target_alt,
                target_heading
            )
    
    def _update_replacement_target(self, namespace: str, mission: DroneMissionStatus):
        """
        Update the target position for a replacement drone during transit.
        
        This is called every 10 seconds to continuously track the position
        of the drone being replaced, ensuring the replacement drone follows
        any movement of the monitoring drone.
        
        Args:
            namespace: The replacement drone namespace
            mission: The replacement drone's mission status
        """
        if not mission.replacing_drone:
            return
        
        old_ns = mission.replacing_drone
        old_mission = self.drone_missions.get(old_ns)
        
        # Only track if the old drone is still monitoring or waiting
        if not old_mission or old_mission.state not in [MissionState.MONITORING, MissionState.WAITING_FOR_RELAY]:
            return
        
        if not self.get_drone_position:
            return
        
        # Get current position of the drone being replaced
        new_lat, new_lon, _ = self.get_drone_position(old_ns)
        
        if new_lat == 0.0 and new_lon == 0.0:
            return
        
        # Check if position has changed significantly (more than 5 meters)
        old_lat = mission.relay_target_lat
        old_lon = mission.relay_target_lon
        distance_moved = self.haversine_distance(old_lat, old_lon, new_lat, new_lon)
        
        if distance_moved < 5.0:
            # Position hasn't changed significantly, no need to update
            return
        
        # Get new heading
        new_heading = mission.relay_target_heading
        if self.get_drone_heading:
            new_heading = self.get_drone_heading(old_ns)
        
        # Update target
        mission.relay_target_lat = new_lat
        mission.relay_target_lon = new_lon
        mission.relay_target_heading = new_heading
        mission.target_lat = new_lat
        mission.target_lon = new_lon
        mission.target_heading = new_heading
        
        self._emit_event(namespace, f"Target updated: {old_ns} moved {distance_moved:.1f}m → ({new_lat:.6f}, {new_lon:.6f})")
        
        # Send updated waypoint command
        self._send_updated_trajectory(namespace, mission)
    
    def _get_currently_monitoring_drone(self, exclude: str = None) -> Optional[str]:
        """
        Get the namespace of the drone currently in MONITORING state.
        
        Args:
            exclude: Optional namespace to exclude (typically the incoming relay drone)
            
        Returns:
            Namespace of the monitoring drone, or None if no drone is monitoring
        """
        for ns, mission in self.drone_missions.items():
            if ns != exclude and mission.state == MissionState.MONITORING:
                return ns
        return None
    
    def _check_approach(self, namespace: str, mission: DroneMissionStatus):
        """Check if drone is approaching video trigger distance and re-evaluate trajectory if needed."""
        if not self.get_drone_position:
            return
        
        lat, lon, _ = self.get_drone_position(namespace)
        
        # Determine target position for distance calculation
        # Priority:
        # 1. Use mission.target_lat/lon if already set (e.g., from relay transit setup)
        # 2. For relay missions, use the monitoring/waiting drone's position
        # 3. Fall back to monitoring point config
        target_lat = mission.target_lat if mission.target_lat != 0.0 else self.config.monitoring_lat
        target_lon = mission.target_lon if mission.target_lon != 0.0 else self.config.monitoring_lon
        
        if self.relay_state == RelayState.ACTIVE:
            # For relay: get target drone (could be in MONITORING or WAITING_FOR_RELAY state)
            target_drone_ns = mission.replacing_drone if mission.replacing_drone else self._get_currently_monitoring_drone(exclude=namespace)
            
            # Also check for drones in WAITING_FOR_RELAY (they were MONITORING before relay started)
            if not target_drone_ns:
                for ns, m in self.drone_missions.items():
                    if ns != namespace and m.state == MissionState.WAITING_FOR_RELAY:
                        target_drone_ns = ns
                        break
            
            if target_drone_ns and self.get_drone_position:
                mon_lat, mon_lon, _ = self.get_drone_position(target_drone_ns)
                if mon_lat != 0.0 and mon_lon != 0.0:
                    target_lat = mon_lat
                    target_lon = mon_lon
                    
                    # Re-evaluate trajectory if target drone moved > 5 meters
                    # Only for PID mode - DJI Native doesn't support live trajectory updates
                    if not self.use_dji_native:
                        if mission.target_lat != 0.0 and mission.target_lon != 0.0:
                            target_moved = self.haversine_distance(
                                mission.target_lat, mission.target_lon,
                                target_lat, target_lon
                            )
                            if target_moved > 5.0:  # Target drone moved more than 5m
                                self._emit_event(namespace, f"Target moved {target_moved:.1f}m - updating trajectory")
                                mission.target_lat = target_lat
                                mission.target_lon = target_lon
                                
                                # Also sync heading from target drone
                                if self.get_drone_heading:
                                    mission.target_heading = self.get_drone_heading(target_drone_ns)
                                
                                # Send updated waypoint command
                                self._send_updated_trajectory(namespace, mission)
        
        distance = self.haversine_distance(lat, lon, target_lat, target_lon)
        
        # Check if within video trigger distance
        if distance <= mission.video_trigger_distance and not mission.video_started:
            # Sync gimbal pitch from currently monitoring drone (for relay missions)
            self._sync_gimbal_pitch(namespace)
            
            if self.cmd_start_recording:
                self.cmd_start_recording(namespace)
            mission.video_started = True
            mission.state = MissionState.APPROACHING_POINT
            mission.state_entry_time = 0.0  # Reset for debug mode timing
            self._update_status(namespace, mission.state, f"Approaching target ({distance:.1f}m)")
            self._emit_event(namespace, f"Video recording started at {distance:.1f}m from target")
        
        # Check if reached target
        if distance <= self.config.position_tolerance:
            if not mission.video_started:
                # Sync gimbal pitch from currently monitoring drone (for relay missions)
                self._sync_gimbal_pitch(namespace)
                
                # Start recording if not already started
                if self.cmd_start_recording:
                    self.cmd_start_recording(namespace)
                mission.video_started = True
            
            # For DJI Native mode, send go-to-yaw command to achieve target heading
            if self.use_dji_native and self.cmd_goto_yaw:
                self.cmd_goto_yaw(namespace, mission.target_heading)
                self._emit_event(namespace, f"Setting heading to {mission.target_heading:.0f}°")
            
            mission.state = MissionState.MONITORING
            mission.monitoring_start_time = time.time()
            mission.state_entry_time = 0.0  # Reset for debug mode timing
            mission.actual_travel_time = time.time() - mission.transit_start_time
            self.travel_time_history.append(mission.actual_travel_time)
            self._update_status(namespace, mission.state, "Monitoring point reached")
            self._emit_event(namespace, f"Monitoring started (direct). Travel time: {mission.actual_travel_time:.1f}s")
    
    def _update_relay_logic(self):
        """Update relay timing and launch next drone if needed."""
        if self.relay_state != RelayState.ACTIVE:
            # Reset pending launch if relay is no longer active
            self._relay_launch_pending = False
            return
        
        if self.current_drone_index >= len(self.drone_order):
            self._relay_launch_pending = False
            return
        
        current_ns = self.drone_order[self.current_drone_index]
        current_mission = self.drone_missions.get(current_ns)
        
        if not current_mission or current_mission.state != MissionState.MONITORING:
            # No drone is currently monitoring - don't launch a relay
            self._relay_launch_pending = False
            return
        
        # Get remaining flight time
        if not self.get_remaining_flight_time:
            return
        
        remaining = self.get_remaining_flight_time(current_ns)
        
        # Don't process relay logic if remaining flight time is not yet available (0 or very low)
        # This prevents premature relay triggering when DJI data hasn't arrived yet
        # Use a lower threshold (30s) to allow countdown display even with partial data
        if remaining < 30:
            return
        
        # Calculate when next drone should launch
        avg_travel = self.get_average_travel_time()
        if avg_travel == 0:
            avg_travel = current_mission.estimated_travel_time
        
        # In Free Flight mode, the first drone doesn't transit, so estimated_travel_time is 0
        # Use a minimum travel time estimate (climb + transit) for relay timing
        if avg_travel == 0 and self.mission_mode == MissionMode.FREE_FLIGHT:
            # Minimum estimate: ~60s climb + ~60s transit = 120s (conservative)
            avg_travel = 120.0
        
        countdown = remaining - avg_travel - self.config.safety_buffer_seconds

        # Notify UI of countdown
        next_index = (self.current_drone_index + 1) % len(self.drone_order)
        next_ns = self.drone_order[next_index]

        # Build timing breakdown for UI display
        timing_breakdown = {
            'remaining_flight_time': remaining,
            'avg_travel_time': avg_travel,
            'safety_buffer': self.config.safety_buffer_seconds,
            'countdown': countdown
        }

        if self.on_relay_countdown:
            self.on_relay_countdown(countdown, next_ns, timing_breakdown)
        
        # Check if next drone is ready (IDLE or COMPLETED state)
        next_mission = self.drone_missions.get(next_ns)
        next_drone_ready = next_mission and next_mission.state in [MissionState.IDLE, MissionState.COMPLETED]
        
        # Show takeoff confirmation dialog 30 seconds before launch (or immediately if countdown already <= 0)
        # Only show if the next drone is actually ready to launch
        if countdown <= 30 and next_drone_ready:
            if not self._takeoff_confirmation_pending and self._takeoff_confirmation_shown_for_drone != next_ns:
                # Request user confirmation
                self._takeoff_confirmation_pending = True
                self._takeoff_confirmation_shown_for_drone = next_ns
                self._takeoff_confirmed = False
                self._takeoff_cancelled = False
                
                if self.on_takeoff_confirmation_request:
                    self._emit_event(next_ns, "Requesting takeoff confirmation from user...")
                    self.on_takeoff_confirmation_request(next_ns, self._handle_takeoff_confirmation)
        
        # Check if user cancelled the mission
        if self._takeoff_cancelled:
            self._emit_event(next_ns, "User cancelled relay mission")
            self._takeoff_cancelled = False
            self._takeoff_confirmation_pending = False
            self._takeoff_confirmation_shown_for_drone = ""
            self._relay_launch_pending = False
            # Stop relay without affecting airborne drone
            self.relay_state = RelayState.INACTIVE
            return
        
        # Mark launch as pending when countdown <= 0
        # This ensures we don't miss the launch if countdown fluctuates back to positive
        if countdown <= 0:
            if not self._relay_launch_pending:
                self._relay_launch_pending = True
                self._pending_next_drone = next_ns
                self._pending_next_index = next_index
                self._emit_event(next_ns, "Relay launch triggered - waiting for confirmation")
        
        # Try to launch if pending AND countdown is still <= 0 AND user has confirmed
        # Double-check countdown to prevent stale _relay_launch_pending from triggering launch
        if self._relay_launch_pending and countdown <= 0:
            # Wait for user confirmation before launching
            if self._takeoff_confirmation_pending and not self._takeoff_confirmed:
                return  # Still waiting for user confirmation
            
            pending_mission = self.drone_missions.get(self._pending_next_drone)
            if pending_mission and pending_mission.state == MissionState.IDLE:
                self._launch_relay_drone(self._pending_next_drone, self._pending_next_index)
                self._relay_launch_pending = False  # Reset after successful launch
                self._takeoff_confirmation_pending = False
                self._takeoff_confirmed = False
                # NOTE: Don't reset _takeoff_confirmation_shown_for_drone here
                # It will be reset when current_drone_index changes (new relay cycle)
    
    def _launch_relay_drone(self, namespace: str, index: int):
        """Launch the next drone in the relay sequence."""
        mission = self.drone_missions[namespace]
        
        # Check drone health before launching
        is_healthy, issues = self.check_drone_health(namespace)
        if not is_healthy:
            self._emit_event(namespace, f"Cannot launch: {', '.join(issues)}")
            mission.retry_count += 1
            if mission.retry_count >= self.config.max_retry_count:
                mission.state = MissionState.ERROR
            return
        
        # Calculate altitude based on drone's fixed position in the order
        # Each drone keeps the same altitude across all rotations:
        # drone_0: base_altitude, drone_1: base + 15m, drone_2: base + 30m, etc.
        drone_position = self.drone_order.index(namespace)
        mission.assigned_altitude = self.config.base_rth_altitude + (drone_position * self.config.altitude_separation)
        
        # Start mission
        mission.state = MissionState.SETTING_RTH_ALTITUDE
        mission.mission_start_time = time.time()
        
        self._emit_event(namespace, f"Relay drone launching at {mission.assigned_altitude}m")
        
        # Track which drone this one is replacing (will trigger RTH when this drone arrives)
        old_index = self.current_drone_index
        old_ns = self.drone_order[old_index]
        old_mission = self.drone_missions.get(old_ns)
        
        if old_mission and old_mission.state == MissionState.MONITORING:
            # Mark this drone as replacing the old one
            # RTH will be triggered when THIS drone reaches MONITORING state
            mission.replacing_drone = old_ns
            
            # SNAPSHOT: Capture the old drone's current position and heading
            # The replacement drone will fly to this position (not the monitoring point)
            if self.get_drone_position:
                old_lat, old_lon, _ = self.get_drone_position(old_ns)
                mission.relay_target_lat = old_lat
                mission.relay_target_lon = old_lon
                self._emit_event(namespace, f"Captured {old_ns} position: ({old_lat:.6f}, {old_lon:.6f})")
            
            if self.get_drone_heading:
                mission.relay_target_heading = self.get_drone_heading(old_ns)
                self._emit_event(namespace, f"Captured {old_ns} heading: {mission.relay_target_heading:.1f}°")
            
            self._emit_event(namespace, f"Will replace {old_ns} at its current position")
            
            # Put old drone in WAITING_FOR_RELAY state
            old_mission.state = MissionState.WAITING_FOR_RELAY
            self._update_status(old_ns, old_mission.state, f"Waiting for {namespace} to arrive")
            self._emit_event(old_ns, f"Waiting for {namespace} to arrive for relay handoff")
        
        # Update current drone index to the new drone
        self.current_drone_index = index
        
        # Reset confirmation tracking for next relay cycle
        self._takeoff_confirmation_shown_for_drone = ""
    
    def _handle_takeoff_confirmation(self, confirmed: bool):
        """Handle user's response to takeoff confirmation dialog."""
        if confirmed:
            self._takeoff_confirmed = True
            self._emit_event(self._takeoff_confirmation_shown_for_drone, "Takeoff confirmed by user")
        else:
            self._takeoff_cancelled = True
            self._emit_event(self._takeoff_confirmation_shown_for_drone, "Takeoff cancelled by user")
    
    def _check_vertical_separation(self):
        """
        Check vertical separation between drones during relay operations.
        Alert if two drones are within 5 meters vertically (critical safety issue).
        
        This is called during active relay operations when drones are in transit.
        """
        # Only check every 2 seconds to avoid spamming
        now = time.time()
        if now - self._last_vertical_separation_check < 2.0:
            return
        self._last_vertical_separation_check = now
        
        # Check if vertical separation is enabled
        if not self._vertical_separation_enabled:
            return
        
        # Get all drones that are actually airborne and in comparable flight states
        # Exclude states where drone is on the ground or climbing (at different altitudes by design):
        # - IDLE: not started
        # - SETTING_RTH_ALTITUDE: on ground, setting parameters
        # - TAKING_OFF: still on/just leaving ground
        # - CLIMBING_TO_ALTITUDE: drones are at different altitudes by design during climb
        # - RETURNING_HOME: drone is returning at its own RTH altitude
        # - COMPLETED: landed
        # - ABORTED: landed
        # - ERROR: error state
        EXCLUDED_STATES = [
            MissionState.IDLE,
            MissionState.SETTING_RTH_ALTITUDE,
            MissionState.TAKING_OFF,
            MissionState.CLIMBING_TO_ALTITUDE,
            MissionState.RETURNING_HOME,
            MissionState.COMPLETED,
            MissionState.ABORTED,
            MissionState.ERROR
        ]
        
        airborne_drones = []
        for ns, mission in self.drone_missions.items():
            if mission.state not in EXCLUDED_STATES:
                # Get altitude using get_drone_position (which is known to work for climbing)
                altitude = 0.0
                if self.get_drone_position:
                    _, _, altitude = self.get_drone_position(ns)
                    logger.debug(f"Vertical sep check: {ns} altitude from get_drone_position = {altitude:.1f}m (state={mission.state.name})")
                elif self.get_drone_altitude:
                    altitude = self.get_drone_altitude(ns)
                    logger.debug(f"Vertical sep check: {ns} altitude from get_drone_altitude = {altitude:.1f}m (state={mission.state.name})")
                else:
                    logger.warning(f"No altitude callback registered!")
                airborne_drones.append((ns, altitude, mission.state))
        
        # Check all pairs for vertical separation
        MIN_VERTICAL_SEPARATION = 5.0  # meters
        
        # Track if any violation exists this check
        violation_found = False
        violation_drones = None
        
        for i, (ns1, alt1, state1) in enumerate(airborne_drones):
            for ns2, alt2, state2 in airborne_drones[i+1:]:
                vertical_sep = abs(alt1 - alt2)
                
                if vertical_sep < MIN_VERTICAL_SEPARATION:
                    violation_found = True
                    violation_drones = (ns1, ns2, alt1, alt2, vertical_sep, state1, state2)
                    
                    # Critical safety alert!
                    warning_msg = (
                        f"CRITICAL: Vertical separation between {ns1} ({alt1:.1f}m) "
                        f"and {ns2} ({alt2:.1f}m) is only {vertical_sep:.1f}m! "
                        f"Minimum required: {MIN_VERTICAL_SEPARATION}m"
                    )
                    logger.warning(warning_msg)
                    self._emit_event(ns1, warning_msg)
                    
                    # Trigger the alert callback (for UI sound/notification)
                    if self.on_vertical_separation_alert and not self._vertical_separation_countdown_active:
                        self.on_vertical_separation_alert(ns1, ns2, vertical_sep, alt1, alt2)
                    
                    break  # Only handle one violation at a time
            if violation_found:
                break
        
        # Handle countdown logic
        current_time = time.time()
        
        # Don't start countdown if mission was already stopped due to vertical separation
        if self._vertical_separation_mission_stopped:
            return
        
        if violation_found:
            ns1, ns2, alt1, alt2, vertical_sep, state1, state2 = violation_drones
            
            if not self._vertical_separation_countdown_active:
                # Start the 20-second countdown
                self._vertical_separation_countdown_active = True
                self._vertical_separation_countdown_start = current_time
                self._vertical_separation_violating_drones = (ns1, ns2)
                
                self._emit_event(ns1, f"⏱️ 20-SECOND COUNTDOWN STARTED - RTH will trigger if separation not restored!")
                logger.warning(f"Vertical separation countdown started for {ns1} and {ns2}")
                
                # Trigger countdown audio
                logger.info(f"Countdown callback registered: {self.on_vertical_separation_countdown_start is not None}")
                if self.on_vertical_separation_countdown_start:
                    logger.info("Calling on_vertical_separation_countdown_start callback")
                    self.on_vertical_separation_countdown_start()
                else:
                    logger.warning("on_vertical_separation_countdown_start callback is NOT registered!")
            else:
                # Countdown already active - check if 20 seconds have passed
                elapsed = current_time - self._vertical_separation_countdown_start
                
                if elapsed >= self._vertical_separation_countdown_duration:
                    # 20 seconds passed - STOP ENTIRE MISSION (same as Stop button)
                    self._emit_event(ns1, f"⚠️ COUNTDOWN EXPIRED - STOPPING MISSION due to vertical separation violation!")
                    logger.warning(f"Vertical separation countdown expired - STOPPING ENTIRE MISSION")
                    
                    # Stop the entire mission (all drones RTH)
                    self.stop_mission()
                    
                    # Reset countdown state AFTER stopping mission
                    self._vertical_separation_countdown_active = False
                    self._vertical_separation_violating_drones = ()
                    
                    # Mark that mission was stopped due to vertical separation (prevent restart)
                    self._vertical_separation_mission_stopped = True
                    
                    # Notify UI to update its state
                    if self.on_vertical_separation_mission_stopped:
                        self.on_vertical_separation_mission_stopped()
        else:
            # No violation - check if we need to cancel an active countdown
            if self._vertical_separation_countdown_active:
                self._vertical_separation_countdown_active = False
                old_drones = self._vertical_separation_violating_drones
                self._vertical_separation_violating_drones = ()
                
                if old_drones:
                    self._emit_event(old_drones[0], f"✅ VERTICAL SEPARATION RESTORED - Countdown cancelled, mission continues!")
                    logger.info(f"Vertical separation restored between {old_drones[0]} and {old_drones[1]} - countdown cancelled")
                
                # Trigger the "respected" audio
                if self.on_vertical_separation_countdown_cancel:
                    self.on_vertical_separation_countdown_cancel()
    
    def force_relay_swap(self) -> tuple[bool, str]:
        """
        Force a relay swap manually, bypassing the countdown timer.
        
        Returns:
            Tuple of (success: bool, message: str)
        """
        if not self.drone_order:
            return False, "No drones in relay mission"
        
        if len(self.drone_order) < 2:
            return False, "Need at least 2 drones for relay swap"
        
        # Find the next drone in the sequence
        next_index = (self.current_drone_index + 1) % len(self.drone_order)
        next_ns = self.drone_order[next_index]
        next_mission = self.drone_missions.get(next_ns)
        
        if not next_mission:
            return False, f"Next drone {next_ns} not found"
        
        # Check if next drone is ready (IDLE state)
        if next_mission.state != MissionState.IDLE:
            return False, f"Next drone {next_ns} not ready (state: {next_mission.state.name})"
        
        # Check current drone is monitoring
        current_ns = self.drone_order[self.current_drone_index]
        current_mission = self.drone_missions.get(current_ns)
        
        if not current_mission or current_mission.state != MissionState.MONITORING:
            return False, f"Current drone {current_ns} not monitoring"
        
        # Set manual swap mode active
        self._manual_swap_active = True
        
        # Re-activate relay state in case it was cancelled
        self.relay_state = RelayState.ACTIVE
        
        # Clear any pending automatic launch to prevent double launches
        self._relay_launch_pending = False
        
        # Force launch the next drone
        self._emit_event(next_ns, "Manual swap triggered - launching relay drone")
        self._launch_relay_drone(next_ns, next_index)
        
        return True, f"Manual swap initiated: {next_ns} launching to replace {current_ns}"
    
    def is_manual_swap_active(self) -> bool:
        """Check if a manual swap is currently in progress."""
        return self._manual_swap_active
    
    def clear_manual_swap_mode(self):
        """Clear manual swap mode (called when swap completes)."""
        self._manual_swap_active = False

    # ========================================================================
    # CALLBACKS
    # ========================================================================
    
    def _update_status(self, namespace: str, state: MissionState, message: str):
        """Emit status update callback."""
        if self.on_status_update:
            self.on_status_update(namespace, state, message)
    
    def _emit_event(self, namespace: str, event: str):
        """Emit mission event callback."""
        logger.info(f"[{namespace}] {event}")
        if self.on_mission_event:
            self.on_mission_event(namespace, event)
        
        # Write to CSV
        state = ""
        if namespace in self.drone_missions:
            state = self.drone_missions[namespace].state.name
        self._write_csv_event(namespace, state, event)
    
    def _sync_gimbal_pitch(self, namespace: str):
        """
        Sync gimbal pitch from the currently monitoring drone to the incoming drone.
        
        This ensures the relay drone has the same camera angle as the drone it's replacing.
        Called when the incoming drone starts recording.
        """
        if not self.cmd_set_gimbal_pitch or not self.get_drone_gimbal_pitch:
            return
        
        # Find the currently monitoring drone (not the incoming one)
        current_monitoring_ns = None
        for ns, mission in self.drone_missions.items():
            if ns != namespace and mission.state == MissionState.MONITORING:
                current_monitoring_ns = ns
                break
        
        if current_monitoring_ns:
            # Get gimbal pitch from currently monitoring drone
            current_pitch = self.get_drone_gimbal_pitch(current_monitoring_ns)
            
            # Set the same gimbal pitch on the incoming drone
            self.cmd_set_gimbal_pitch(namespace, current_pitch)
            self._emit_event(namespace, f"Gimbal pitch synced to {current_pitch:.1f}° from {current_monitoring_ns}")
    
    # ========================================================================
    # STATE QUERIES
    # ========================================================================
    
    def get_mission_status(self, namespace: str) -> Optional[DroneMissionStatus]:
        """Get mission status for a drone."""
        return self.drone_missions.get(namespace)
    
    def get_active_drone(self) -> Optional[str]:
        """Get the currently active drone in relay mission."""
        if self.current_drone_index < len(self.drone_order):
            return self.drone_order[self.current_drone_index]
        return None
    
    def get_next_drone(self) -> Optional[str]:
        """Get the next drone in relay sequence."""
        next_index = (self.current_drone_index + 1) % len(self.drone_order)
        if next_index < len(self.drone_order):
            return self.drone_order[next_index]
        return None
    
    def get_drones_needing_reconnection(self) -> List[str]:
        """
        Get list of drones that have completed their mission and need battery swap.
        
        These are drones the operator should reconnect (with fresh battery) 
        using the SAME namespace.
        
        Returns:
            List of drone namespaces that need reconnection
        """
        needs_reconnect = []
        for ns in self.drone_order:
            mission = self.drone_missions.get(ns)
            if mission and mission.state == MissionState.COMPLETED:
                needs_reconnect.append(ns)
        return needs_reconnect
    
    def is_mission_active(self) -> bool:
        """Check if any mission is active."""
        return self.relay_state == RelayState.ACTIVE or \
               any(m.state not in [MissionState.IDLE, MissionState.COMPLETED, MissionState.ABORTED, MissionState.ERROR] 
                   for m in self.drone_missions.values())
    
    def calculate_drones_needed(self, mission_duration_hours: float = 1.0, battery_swap_time: float = 180.0) -> Tuple[int, int, float, float, bool]:
        """
        Calculate minimum drones needed for continuous coverage.
        
        Accounts for drone reuse - once a drone lands and gets a fresh battery,
        it can rejoin the rotation.
        
        Args:
            mission_duration_hours: How long to maintain coverage
            battery_swap_time: Time in seconds to swap battery and reconnect (default 3 min)
        
        Returns:
            Tuple of (drones_simultaneous, drones_total, travel_time_seconds, distance_meters, is_from_actual_data)
            - drones_simultaneous: min drones flying at the same time for continuous coverage
            - drones_total: total drones needed in rotation (including those on ground swapping batteries)
            - drones values are float('inf') if point is too far
            - is_from_actual_data is True if calculation used actual flight data
        """
        # Check if we have actual data from completed flights
        has_actual_data = len(self.travel_time_history) > 0 or len(self.flight_time_history) > 0
        
        # Get average flight time - prefer actual history, then drone telemetry, then default
        avg_flight_time = self.get_average_flight_time()
        
        if avg_flight_time == 0:
            # Try to get from connected drone's remaining flight time
            if self.get_remaining_flight_time and self.get_connected_drones:
                connected = self.get_connected_drones()
                if connected:
                    max_flight_time = 0
                    for ns in connected:
                        remaining = self.get_remaining_flight_time(ns)
                        if remaining > max_flight_time:
                            max_flight_time = remaining
                    if max_flight_time > 0:
                        avg_flight_time = max_flight_time
            
            # Final fallback: assume 25 minutes
            if avg_flight_time == 0:
                avg_flight_time = 1500  # 25 minutes
        
        # Get list of drones to check - use drone_order if mission active, else get connected drones
        drones_to_check = self.drone_order if self.drone_order else []
        if not drones_to_check and self.get_connected_drones:
            drones_to_check = self.get_connected_drones()
        
        # Always calculate distance from drone home/position to monitoring point
        distance = 0.0
        if self.get_drone_home_position and drones_to_check:
            first_drone = drones_to_check[0]
            home_lat, home_lon = self.get_drone_home_position(first_drone)
            # Require both lat AND lon to be non-zero (valid GPS, not default 0,0)
            if home_lat != 0.0 and home_lon != 0.0:
                distance = self.haversine_distance(home_lat, home_lon, 
                    self.config.monitoring_lat, self.config.monitoring_lon)
        
        # Fallback: try current drone position if no home set
        if distance == 0.0 and self.get_drone_position and drones_to_check:
            first_drone = drones_to_check[0]
            lat, lon, _ = self.get_drone_position(first_drone)
            # Require both lat AND lon to be non-zero
            if lat != 0.0 and lon != 0.0:
                distance = self.haversine_distance(lat, lon, 
                    self.config.monitoring_lat, self.config.monitoring_lon)
        
        # Get average travel time - prefer from actual flight history
        avg_travel = self.get_average_travel_time()
        
        if avg_travel == 0:
            if distance > 0:
                avg_travel = self.estimate_travel_time(distance)
            else:
                # Last fallback
                has_actual_data = False
                avg_travel = 300  # Assume 5 minutes
                distance = 3000  # ~3km at 10m/s
        
        # Effective monitoring time per drone (time actually spent at monitoring point)
        # Formula: flight_time - travel_out - travel_back - safety_buffer
        effective_monitoring = avg_flight_time - (2 * avg_travel) - self.config.safety_buffer_seconds
        
        if effective_monitoring <= 0:
            return (float('inf'), float('inf'), avg_travel, distance, has_actual_data)  # Impossible - point too far
        
        # Calculate drones flying simultaneously:
        # Need to launch next drone when: remaining_time <= travel_time + safety_buffer
        # So next drone must be in the air (travel_time + safety_buffer) before current drone leaves
        # If travel_time >= effective_monitoring, we need multiple drones in transit
        drones_simultaneous = max(1, math.ceil(avg_travel / effective_monitoring)) + 1
        
        # Full cycle time: fly out + monitor + fly back + swap battery
        cycle_time = avg_flight_time + battery_swap_time
        
        # Total drones in rotation (including those swapping batteries)
        drones_total = max(drones_simultaneous, math.ceil(cycle_time / effective_monitoring))
        
        return (drones_simultaneous, drones_total, avg_travel, distance, has_actual_data)
    
    def shutdown(self):
        """Shutdown the mission controller."""
        self._running = False
        if self._mission_thread:
            self._mission_thread.join(timeout=2)
    
    # ========================================================================
    # ROBUSTNESS & ERROR HANDLING
    # ========================================================================
    
    def check_drone_health(self, namespace: str) -> Tuple[bool, List[str]]:
        """
        Check drone health and return status.
        
        Returns:
            Tuple of (is_healthy, list of warnings/errors)
        """
        issues = []
        
        if namespace not in self.drone_missions:
            return False, ["Drone not in mission"]
        
        mission = self.drone_missions[namespace]
        
        # Check battery
        if self.get_battery_level:
            battery = self.get_battery_level(namespace)
            if battery < self.config.emergency_battery_threshold:
                issues.append(f"CRITICAL: Battery at {battery:.1f}%")
            elif battery < self.config.min_battery_to_launch:
                issues.append(f"WARNING: Low battery {battery:.1f}%")
        
        # Check satellite count
        if self.get_satellite_count:
            sats = self.get_satellite_count(namespace)
            if sats < 6:
                issues.append(f"WARNING: Only {sats} satellites")
        
        # Check if position is valid
        if self.get_drone_position:
            lat, lon, alt = self.get_drone_position(namespace)
            if lat == 0.0 and lon == 0.0:
                issues.append("WARNING: Invalid GPS position")
        
        # Check distance from home
        if self.get_drone_position:
            lat, lon, _ = self.get_drone_position(namespace)
            # Assume home at origin if not available
            if hasattr(mission, 'home_lat') and hasattr(mission, 'home_lon'):
                distance = self.haversine_distance(lat, lon, mission.home_lat, mission.home_lon)
                if distance > self.config.max_distance_from_home:
                    issues.append(f"WARNING: {distance:.0f}m from home (max: {self.config.max_distance_from_home}m)")
        
        has_critical = any("CRITICAL" in issue for issue in issues)
        return not has_critical, issues
    
    def handle_emergency(self, namespace: str, reason: str):
        """
        Handle emergency situation for a drone.
        
        Forces immediate RTH and marks mission as emergency.
        """
        if namespace not in self.drone_missions:
            return
        
        mission = self.drone_missions[namespace]
        
        # Log emergency
        self._emit_event(namespace, f"🚨 EMERGENCY: {reason}")
        logger.error(f"Emergency for {namespace}: {reason}")
        
        # Stop recording if active
        if mission.video_started and self.cmd_stop_recording:
            self.cmd_stop_recording(namespace)
        
        # Re-set RTH altitude before RTH command
        if self.cmd_set_rth_altitude:
            self.cmd_set_rth_altitude(namespace, mission.assigned_altitude)
            time.sleep(0.3)
        
        # Force RTH
        if self.cmd_rth:
            self._emit_event(namespace, f"Emergency RTH at {mission.assigned_altitude}m")
            self.cmd_rth(namespace)
        
        mission.state = MissionState.ERROR
        mission.error_message = f"Emergency: {reason}"
        self._update_status(namespace, mission.state, f"Emergency: {reason}")
        
        # If this was the active drone in relay, try to launch next
        if self.relay_state == RelayState.ACTIVE and self.get_active_drone() == namespace:
            next_drone = self.get_next_drone()
            if next_drone and next_drone in self.drone_missions:
                next_mission = self.drone_missions[next_drone]
                if next_mission.state == MissionState.IDLE:
                    self._emit_event(next_drone, "Emergency relay - launching backup")
                    next_idx = self.drone_order.index(next_drone)
                    self._launch_relay_drone(next_drone, next_idx)
    
    def handle_disconnect(self, namespace: str):
        """
        Handle drone disconnect during mission.
        
        Called when telemetry stops being received.
        """
        if namespace not in self.drone_missions:
            return
        
        mission = self.drone_missions[namespace]
        
        if mission.state in [MissionState.IDLE, MissionState.COMPLETED, MissionState.ABORTED]:
            return
        
        self._emit_event(namespace, "⚠️ Connection lost")
        logger.warning(f"Lost connection to {namespace} during mission")
        
        mission.state = MissionState.ERROR
        mission.error_message = "Connection lost"
        
        # For relay mission, launch next drone immediately
        if self.relay_state == RelayState.ACTIVE and self.get_active_drone() == namespace:
            next_drone = self.get_next_drone()
            if next_drone and next_drone in self.drone_missions:
                next_mission = self.drone_missions[next_drone]
                if next_mission.state == MissionState.IDLE:
                    self._emit_event(next_drone, "Emergency launch - previous drone disconnected")
                    next_idx = self.drone_order.index(next_drone)
                    self._launch_relay_drone(next_drone, next_idx)
    
    def recover_mission(self, namespace: str) -> bool:
        """
        Attempt to recover a mission after error.
        
        Returns:
            True if recovery successful
        """
        if namespace not in self.drone_missions:
            return False
        
        mission = self.drone_missions[namespace]
        
        if mission.state != MissionState.ERROR:
            return False
        
        # Check if drone is healthy now
        is_healthy, issues = self.check_drone_health(namespace)
        if not is_healthy:
            self._emit_event(namespace, f"Cannot recover: {', '.join(issues)}")
            return False
        
        # Reset mission state
        mission.state = MissionState.IDLE
        mission.error_message = ""
        mission.retry_count += 1
        
        if mission.retry_count > self.config.max_retry_count:
            self._emit_event(namespace, "Max retries exceeded")
            return False
        
        self._emit_event(namespace, "Mission recovered, can restart")
        return True
    
    def reset_drone_for_reuse(self, namespace: str) -> bool:
        """
        Reset a drone's mission state so it can be used again in the relay rotation.
        
        Called when a drone reconnects after battery swap.
        
        Args:
            namespace: The drone namespace
            
        Returns:
            True if reset successful, False if drone not in mission
        """
        if namespace not in self.drone_missions:
            return False
        
        mission = self.drone_missions[namespace]
        
        # Reset to IDLE so it can be launched again
        mission.state = MissionState.IDLE
        mission.error_message = ""
        mission.retry_count = 0
        mission.mission_start_time = None
        mission.monitoring_start_time = None
        mission.transit_start_time = None
        
        self._emit_event(namespace, "Drone reconnected - ready for next relay cycle")
        return True
    
    def get_mission_summary(self) -> Dict:
        """
        Get a summary of the current mission state.
        
        Returns:
            Dictionary with mission statistics and status
        """
        summary = {
            "relay_state": self.relay_state.name,
            "active_drone": self.get_active_drone(),
            "next_drone": self.get_next_drone(),
            "total_drones": len(self.drone_order),
            "drone_statuses": {},
            "average_travel_time": self.get_average_travel_time(),
            "monitoring_point": {
                "lat": self.config.monitoring_lat,
                "lon": self.config.monitoring_lon,
                "alt": self.config.monitoring_alt
            }
        }
        
        for ns, mission in self.drone_missions.items():
            summary["drone_statuses"][ns] = {
                "state": mission.state.name,
                "altitude": mission.assigned_altitude,
                "error": mission.error_message,
                "video_active": mission.video_started,
                "retries": mission.retry_count
            }
        
        return summary
    
    def adjust_safety_buffer(self, new_buffer: float):
        """Adjust the safety buffer during runtime."""
        old_buffer = self.config.safety_buffer_seconds
        self.config.safety_buffer_seconds = max(30.0, new_buffer)  # Minimum 30 seconds
        logger.info(f"Safety buffer adjusted: {old_buffer}s -> {self.config.safety_buffer_seconds}s")
    
    def set_video_trigger_distance(self, distance: float):
        """Set the distance from monitoring point to start video."""
        self.config.video_trigger_distance = max(10.0, distance)  # Minimum 10m
        
        # Update all active missions
        for mission in self.drone_missions.values():
            mission.video_trigger_distance = self.config.video_trigger_distance
    
    def notify_landing_detected(self, namespace: str, rth_duration: float = 0.0) -> bool:
        """
        Notify that a drone has landed after RTH.
        
        Called by GUI when landing is detected based on altitude stability.
        Transitions from RETURNING_HOME/ABORTED to COMPLETED.
        
        Args:
            namespace: The drone namespace
            rth_duration: Time in seconds from RTH start to landing
            
        Returns:
            True if state was updated, False otherwise
        """
        if namespace not in self.drone_missions:
            return False
        
        mission = self.drone_missions[namespace]
        
        # Only transition if drone is in a state where landing makes sense
        if mission.state not in [MissionState.RETURNING_HOME, MissionState.ABORTED, MissionState.ERROR]:
            return False
        
        mission.state = MissionState.COMPLETED
        mission.actual_rth_time = rth_duration
        
        self._emit_event(namespace, f"Landed after RTH ({rth_duration:.1f}s)")
        self._update_status(namespace, mission.state, "Mission completed - drone landed")
        
        return True
    
    def mark_drone_ready(self, namespace: str) -> bool:
        """
        Manually mark a drone as ready (IDLE) for next relay cycle.
        
        Called by GUI when user clicks the Ready button.
        Similar to notify_battery_swap but without battery detection.
        
        Args:
            namespace: The drone namespace
            
        Returns:
            True if state was updated, False otherwise
        """
        if namespace not in self.drone_missions:
            return False
        
        mission = self.drone_missions[namespace]
        
        # Only reset if drone has completed its mission
        if mission.state != MissionState.COMPLETED:
            return False
        
        # Reset mission state for reuse
        mission.state = MissionState.IDLE
        mission.error_message = ""
        mission.retry_count = 0
        mission.mission_start_time = None
        mission.monitoring_start_time = None
        mission.transit_start_time = None
        mission.video_started = False
        mission.replacing_drone = ""
        mission.relay_target_lat = 0.0
        mission.relay_target_lon = 0.0
        mission.relay_target_heading = 0.0
        
        self._emit_event(namespace, "Manually marked as ready for next relay cycle")
        self._update_status(namespace, mission.state, "Ready for next mission")
        
        return True
    
    def notify_battery_swap(self, namespace: str) -> bool:
        """
        Notify that a drone's battery has been swapped.
        
        Called by GUI when battery level increases significantly (>10%).
        Resets drone from COMPLETED to IDLE so it can rejoin the relay.
        
        Args:
            namespace: The drone namespace
            
        Returns:
            True if state was updated, False otherwise
        """
        if namespace not in self.drone_missions:
            return False
        
        mission = self.drone_missions[namespace]
        
        # Only reset if drone has completed its mission
        if mission.state != MissionState.COMPLETED:
            return False
        
        # Reset mission state for reuse
        mission.state = MissionState.IDLE
        mission.error_message = ""
        mission.retry_count = 0
        mission.mission_start_time = None
        mission.monitoring_start_time = None
        mission.transit_start_time = None
        mission.video_started = False
        mission.replacing_drone = ""
        mission.relay_target_lat = 0.0
        mission.relay_target_lon = 0.0
        mission.relay_target_heading = 0.0
        
        self._emit_event(namespace, "Battery swapped - ready for next relay cycle")
        self._update_status(namespace, mission.state, "Ready for next mission")
        
        return True
