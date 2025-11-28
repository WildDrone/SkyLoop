"""
Mission Controller for Perpetual Drone Monitoring

Handles the state machine for single drone missions and relay operations.

Author: Edouard Rolland
Project: WildDrone
"""

import time
import math
import threading
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
    PREFLIGHT_CHECK = auto()
    SETTING_RTH_ALTITUDE = auto()
    TAKING_OFF = auto()
    CLIMBING_TO_ALTITUDE = auto()
    TRANSIT_TO_MONITORING = auto()
    APPROACHING_POINT = auto()  # Within video trigger distance
    MONITORING = auto()
    WAITING_FOR_RELAY = auto()
    RETURNING_HOME = auto()
    LANDING = auto()
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
    
    base_rth_altitude: float = 50.0
    altitude_separation: float = 15.0
    video_trigger_distance: float = 50.0
    
    # Safety parameters
    safety_buffer_seconds: float = 60.0
    min_battery_to_launch: float = 30.0
    min_satellites: int = 8
    max_retry_count: int = 3
    
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
        
        # Drone tracking
        self.drone_missions: Dict[str, DroneMissionStatus] = {}
        self.drone_order: List[str] = []  # Order for relay
        self.current_drone_index: int = 0
        
        # Navigation mode: False = PID (5 m/s), True = DJI Native (10 m/s)
        self.use_dji_native: bool = False
        
        # Callbacks for drone commands (set by ROS node)
        self.cmd_takeoff: Optional[Callable[[str], None]] = None
        self.cmd_land: Optional[Callable[[str], None]] = None
        self.cmd_rth: Optional[Callable[[str], None]] = None
        self.cmd_goto_waypoint: Optional[Callable[[str, float, float, float, float], None]] = None
        self.cmd_goto_waypoint_dji_native: Optional[Callable[[str, float, float, float], None]] = None  # DJI Native mode
        self.cmd_goto_altitude: Optional[Callable[[str, float], None]] = None
        self.cmd_set_rth_altitude: Optional[Callable[[str, float], None]] = None
        self.cmd_start_recording: Optional[Callable[[str], None]] = None
        self.cmd_stop_recording: Optional[Callable[[str], None]] = None
        self.cmd_set_gimbal_pitch: Optional[Callable[[str, float], None]] = None
        self.cmd_goto_yaw: Optional[Callable[[str, float], None]] = None  # Set drone heading
        self.cmd_abort: Optional[Callable[[str], None]] = None
        
        # Telemetry getters (set by ROS node)
        self.get_drone_position: Optional[Callable[[str], Tuple[float, float, float]]] = None
        self.get_drone_home_position: Optional[Callable[[str], Tuple[float, float]]] = None  # home lat, lon
        self.get_drone_heading: Optional[Callable[[str], float]] = None
        self.get_drone_gimbal_pitch: Optional[Callable[[str], float]] = None
        self.get_remaining_flight_time: Optional[Callable[[str], float]] = None
        self.get_battery_level: Optional[Callable[[str], float]] = None
        self.get_satellite_count: Optional[Callable[[str], int]] = None
        self.get_is_recording: Optional[Callable[[str], bool]] = None
        self.get_waypoint_reached: Optional[Callable[[str], bool]] = None
        self.get_altitude_reached: Optional[Callable[[str], bool]] = None
        self.get_configured_speed: Optional[Callable[[], float]] = None  # Get UI-configured speed
        self.get_connected_drones: Optional[Callable[[], List[str]]] = None  # Get list of connected drone namespaces
        
        # Status callback (for GUI updates)
        self.on_status_update: Optional[Callable[[str, MissionState, str], None]] = None
        self.on_relay_countdown: Optional[Callable[[float, str], None]] = None
        self.on_mission_event: Optional[Callable[[str, str], None]] = None
        
        # Travel time history for estimation
        self.travel_time_history: List[float] = []
        
        # Flight time history (actual total flight times from completed missions)
        self.flight_time_history: List[float] = []
        
        # Mission thread
        self._mission_thread: Optional[threading.Thread] = None
        self._running = False
        self._update_interval = 0.5  # seconds
    
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
        Estimate travel time in seconds, including climb time.
        
        Args:
            distance: Horizontal distance in meters
            altitude: Target altitude in meters (uses config monitoring_alt if None)
            horizontal_speed: Horizontal flight speed in m/s 
                              (default: 5 m/s for PID mode, 10 m/s for DJI native)
            vertical_speed: Vertical climb speed in m/s (default 4 m/s)
        
        Returns:
            Estimated travel time in seconds
        """
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
        
        # Total travel time = climb + horizontal (sequential, not parallel)
        # Note: DJI drones typically climb first, then translate
        return climb_time + horizontal_time
    
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
        
        # Create drone mission status
        mission = DroneMissionStatus(
            namespace=namespace,
            state=MissionState.PREFLIGHT_CHECK,
            assigned_altitude=rth_altitude
        )
        self.drone_missions[namespace] = mission
        self.drone_order = [namespace]
        self.current_drone_index = 0
        
        # Preflight check
        passed, message = self.preflight_check(namespace)
        if not passed:
            mission.state = MissionState.ERROR
            mission.error_message = message
            self._emit_event(namespace, f"Preflight failed: {message}")
            return False
        
        # Start mission thread if not running
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
        
        # Initialize all drones
        self.drone_order = drone_list.copy()
        for i, ns in enumerate(drone_list):
            altitude = base_rth_altitude + (i * self.config.altitude_separation)
            self.drone_missions[ns] = DroneMissionStatus(
                namespace=ns,
                state=MissionState.IDLE,
                assigned_altitude=altitude
            )
        
        # Preflight check first drone
        first_drone = drone_list[0]
        passed, message = self.preflight_check(first_drone)
        if not passed:
            self._emit_event(first_drone, f"Preflight failed: {message}")
            return False
        
        # Set relay state
        self.relay_state = RelayState.ACTIVE
        self.current_drone_index = 0
        
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
                
                # Command RTH
                if self.cmd_rth:
                    self.cmd_rth(ns)
                
                mission.state = MissionState.ABORTED
                self._emit_event(ns, "Mission aborted - returning home")
        
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
                
                time.sleep(self._update_interval)
                
            except Exception as e:
                logger.error(f"Mission loop error: {e}")
    
    def _update_drone_mission(self, namespace: str, mission: DroneMissionStatus):
        """Update a single drone's mission state."""
        
        if mission.state == MissionState.IDLE:
            return
        
        elif mission.state == MissionState.SETTING_RTH_ALTITUDE:
            if self.cmd_set_rth_altitude:
                self.cmd_set_rth_altitude(namespace, mission.assigned_altitude)
            
            # Brief wait then takeoff
            time.sleep(0.5)
            mission.state = MissionState.TAKING_OFF
            self._update_status(namespace, mission.state, "Setting RTH altitude...")
        
        elif mission.state == MissionState.TAKING_OFF:
            if self.cmd_takeoff:
                # Send takeoff command twice for reliability
                self.cmd_takeoff(namespace)
                time.sleep(2.0)
                self.cmd_takeoff(namespace)
            
            # Wait 5 seconds before transitioning to climbing
            time.sleep(5.0)
            mission.state = MissionState.CLIMBING_TO_ALTITUDE
            self._update_status(namespace, mission.state, "Taking off...")
            
            # Send goto_altitude command to climb to target altitude
            if self.cmd_goto_altitude:
                self.cmd_goto_altitude(namespace, mission.assigned_altitude)
                self._emit_event(namespace, f"Climbing to {mission.assigned_altitude}m")
        
        elif mission.state == MissionState.CLIMBING_TO_ALTITUDE:
            # Check if altitude reached - prefer using the altitude_reached flag from drone
            altitude_reached = False
            current_alt = 0.0
            target_alt = mission.assigned_altitude
            
            if self.get_altitude_reached:
                altitude_reached = self.get_altitude_reached(namespace)
            
            if self.get_drone_position:
                _, _, current_alt = self.get_drone_position(namespace)
                # Fallback: also check if altitude is above target (in case flag not set)
                if current_alt >= (target_alt - self.config.altitude_tolerance):
                    altitude_reached = True
            
            # Log altitude progress periodically
            if not hasattr(mission, '_last_alt_log') or time.time() - mission._last_alt_log > 5.0:
                mission._last_alt_log = time.time()
                self._emit_event(namespace, f"Climbing: {current_alt:.1f}m / {target_alt:.1f}m (reached={altitude_reached})")
            
            if altitude_reached:
                # Wait 5 seconds before starting transit
                time.sleep(5.0)
                # Altitude reached, start transit
                mission.state = MissionState.TRANSIT_TO_MONITORING
                mission.transit_start_time = time.time()
                self._start_transit(namespace, mission)
                self._update_status(namespace, mission.state, "Climbing to altitude...")
        
        elif mission.state == MissionState.TRANSIT_TO_MONITORING:
            # Log transit progress periodically
            if self.get_drone_position:
                lat, lon, _ = self.get_drone_position(namespace)
                target_lat = mission.target_lat if mission.target_lat != 0 else self.config.monitoring_lat
                target_lon = mission.target_lon if mission.target_lon != 0 else self.config.monitoring_lon
                distance = self.haversine_distance(lat, lon, target_lat, target_lon)
                
                if not hasattr(mission, '_last_transit_log') or time.time() - mission._last_transit_log > 5.0:
                    mission._last_transit_log = time.time()
                    self._emit_event(namespace, f"Transit: {distance:.1f}m to target")
            
            self._check_approach(namespace, mission)
        
        elif mission.state == MissionState.APPROACHING_POINT:
            # Already recording, check if reached (by flag or distance)
            reached = False
            distance = float('inf')
            
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
            if not hasattr(mission, '_last_approach_log') or time.time() - mission._last_approach_log > 3.0:
                mission._last_approach_log = time.time()
                self._emit_event(namespace, f"Approaching: {distance:.1f}m to target (tolerance: {self.config.position_tolerance}m)")
            
            if reached:
                mission.state = MissionState.MONITORING
                mission.monitoring_start_time = time.time()
                
                # Record actual travel time
                mission.actual_travel_time = time.time() - mission.transit_start_time
                self.travel_time_history.append(mission.actual_travel_time)
                if len(self.travel_time_history) > 10:
                    self.travel_time_history.pop(0)
                
                self._update_status(namespace, mission.state, "Monitoring point reached")
                self._emit_event(namespace, f"Monitoring started. Travel time: {mission.actual_travel_time:.1f}s")
        
        elif mission.state == MissionState.MONITORING:
            # Drone is monitoring - relay logic handles transition
            pass
        
        elif mission.state == MissionState.RETURNING_HOME:
            # Drone returning home
            pass
        
        elif mission.state == MissionState.LANDING:
            # Drone landing
            pass
    
    def _start_transit(self, namespace: str, mission: DroneMissionStatus):
        """Start transit to monitoring point (or current monitoring drone's position for relay)."""
        if not self.get_drone_position:
            return
            
        lat, lon, _ = self.get_drone_position(namespace)
        
        # Determine target position
        # For relay missions, fly to where the current monitoring drone is (not just the config point)
        target_lat = self.config.monitoring_lat
        target_lon = self.config.monitoring_lon
        # Use the drone's assigned altitude (which includes vertical separation for safety)
        target_alt = mission.assigned_altitude
        
        if self.relay_state == RelayState.ACTIVE:
            # Find the currently monitoring drone and use its position (but keep our assigned altitude!)
            current_monitoring_ns = self._get_currently_monitoring_drone(exclude=namespace)
            if current_monitoring_ns and self.get_drone_position:
                mon_lat, mon_lon, _ = self.get_drone_position(current_monitoring_ns)
                if mon_lat != 0.0 or mon_lon != 0.0:
                    target_lat = mon_lat
                    target_lon = mon_lon
                    # Keep target_alt as mission.assigned_altitude for vertical separation!
                    self._emit_event(namespace, f"Target: current position of {current_monitoring_ns}")
        
        # Calculate bearing to target
        bearing = self.calculate_bearing(lat, lon, target_lat, target_lon)
        
        # Determine target heading: use monitoring drone's heading for relay, or configured heading
        target_heading = self.config.monitoring_heading
        if self.relay_state == RelayState.ACTIVE:
            current_monitoring_ns = self._get_currently_monitoring_drone(exclude=namespace)
            if current_monitoring_ns and self.get_drone_heading:
                # Sync heading from currently monitoring drone
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
                target_heading  # Use configured/synced heading instead of bearing
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
        # For relay missions, use the monitoring drone's position
        target_lat = self.config.monitoring_lat
        target_lon = self.config.monitoring_lon
        
        if self.relay_state == RelayState.ACTIVE:
            current_monitoring_ns = self._get_currently_monitoring_drone(exclude=namespace)
            if current_monitoring_ns and self.get_drone_position:
                mon_lat, mon_lon, _ = self.get_drone_position(current_monitoring_ns)
                if mon_lat != 0.0 and mon_lon != 0.0:
                    target_lat = mon_lat
                    target_lon = mon_lon
                    
                    # Re-evaluate trajectory if monitoring drone moved > 5 meters
                    # Only for PID mode - DJI Native doesn't support live trajectory updates
                    if not self.use_dji_native:
                        if mission.target_lat != 0.0 and mission.target_lon != 0.0:
                            target_moved = self.haversine_distance(
                                mission.target_lat, mission.target_lon,
                                target_lat, target_lon
                            )
                            if target_moved > 5.0:  # Monitoring drone moved more than 5m
                                self._emit_event(namespace, f"Target moved {target_moved:.1f}m - updating trajectory")
                                mission.target_lat = target_lat
                                mission.target_lon = target_lon
                                
                                # Also sync heading from monitoring drone
                                if self.get_drone_heading:
                                    mission.target_heading = self.get_drone_heading(current_monitoring_ns)
                                
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
            mission.actual_travel_time = time.time() - mission.transit_start_time
            self.travel_time_history.append(mission.actual_travel_time)
    
    def _update_relay_logic(self):
        """Update relay timing and launch next drone if needed."""
        if self.relay_state != RelayState.ACTIVE:
            return
        
        if self.current_drone_index >= len(self.drone_order):
            return
        
        current_ns = self.drone_order[self.current_drone_index]
        current_mission = self.drone_missions.get(current_ns)
        
        if not current_mission or current_mission.state != MissionState.MONITORING:
            return
        
        # Get remaining flight time
        if not self.get_remaining_flight_time:
            return
        
        remaining = self.get_remaining_flight_time(current_ns)
        
        # Calculate when next drone should launch
        avg_travel = self.get_average_travel_time()
        if avg_travel == 0:
            avg_travel = current_mission.estimated_travel_time
        
        countdown = remaining - avg_travel - self.config.safety_buffer_seconds
        
        # Notify UI of countdown
        next_index = (self.current_drone_index + 1) % len(self.drone_order)
        next_ns = self.drone_order[next_index]
        
        if self.on_relay_countdown:
            self.on_relay_countdown(countdown, next_ns)
        
        # Launch next drone if countdown <= 0
        if countdown <= 0:
            next_mission = self.drone_missions.get(next_ns)
            if next_mission and next_mission.state == MissionState.IDLE:
                self._launch_relay_drone(next_ns, next_index)
    
    def _launch_relay_drone(self, namespace: str, index: int):
        """Launch the next drone in the relay sequence."""
        mission = self.drone_missions[namespace]
        
        # Preflight check
        passed, message = self.preflight_check(namespace)
        if not passed:
            self._emit_event(namespace, f"Cannot launch: {message}")
            mission.retry_count += 1
            if mission.retry_count >= self.config.max_retry_count:
                mission.state = MissionState.ERROR
            return
        
        # Calculate altitude (15m above previous)
        mission.assigned_altitude = self.config.base_rth_altitude + (index * self.config.altitude_separation)
        
        # Start mission
        mission.state = MissionState.SETTING_RTH_ALTITUDE
        mission.mission_start_time = time.time()
        
        self._emit_event(namespace, f"Relay drone launching at {mission.assigned_altitude}m")
        
        # Update current drone to return home
        old_index = self.current_drone_index
        old_ns = self.drone_order[old_index]
        old_mission = self.drone_missions.get(old_ns)
        
        if old_mission:
            # Record actual flight time (from takeoff to return home command)
            if old_mission.mission_start_time:
                actual_flight_time = time.time() - old_mission.mission_start_time
                self.flight_time_history.append(actual_flight_time)
                if len(self.flight_time_history) > 10:
                    self.flight_time_history.pop(0)
                self._emit_event(old_ns, f"Flight time recorded: {actual_flight_time:.0f}s")
            
            # Stop recording
            if old_mission.video_started and self.cmd_stop_recording:
                self.cmd_stop_recording(old_ns)
            
            # Command RTH
            if self.cmd_rth:
                self.cmd_rth(old_ns)
            
            old_mission.state = MissionState.RETURNING_HOME
            old_mission.rth_start_time = time.time()  # Track RTH start time
            self._emit_event(old_ns, "Relay handoff - returning home")
        
        # Update current drone index
        self.current_drone_index = index
    
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
        
        # Force RTH
        if self.cmd_rth:
            self.cmd_rth(namespace)
        
        mission.state = MissionState.ERROR
        mission.error_message = f"Emergency: {reason}"
        
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
