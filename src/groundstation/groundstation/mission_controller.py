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
    
    # Timing
    mission_start_time: float = 0.0
    transit_start_time: float = 0.0
    monitoring_start_time: float = 0.0
    
    # Travel time tracking
    estimated_travel_time: float = 0.0
    actual_travel_time: float = 0.0
    
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
        
        # Callbacks for drone commands (set by ROS node)
        self.cmd_takeoff: Optional[Callable[[str], None]] = None
        self.cmd_land: Optional[Callable[[str], None]] = None
        self.cmd_rth: Optional[Callable[[str], None]] = None
        self.cmd_goto_waypoint: Optional[Callable[[str, float, float, float, float], None]] = None
        self.cmd_goto_altitude: Optional[Callable[[str, float], None]] = None
        self.cmd_set_rth_altitude: Optional[Callable[[str, float], None]] = None
        self.cmd_start_recording: Optional[Callable[[str], None]] = None
        self.cmd_stop_recording: Optional[Callable[[str], None]] = None
        self.cmd_abort: Optional[Callable[[str], None]] = None
        
        # Telemetry getters (set by ROS node)
        self.get_drone_position: Optional[Callable[[str], Tuple[float, float, float]]] = None
        self.get_drone_heading: Optional[Callable[[str], float]] = None
        self.get_remaining_flight_time: Optional[Callable[[str], float]] = None
        self.get_battery_level: Optional[Callable[[str], float]] = None
        self.get_satellite_count: Optional[Callable[[str], int]] = None
        self.get_is_recording: Optional[Callable[[str], bool]] = None
        self.get_waypoint_reached: Optional[Callable[[str], bool]] = None
        self.get_altitude_reached: Optional[Callable[[str], bool]] = None
        
        # Status callback (for GUI updates)
        self.on_status_update: Optional[Callable[[str, MissionState, str], None]] = None
        self.on_relay_countdown: Optional[Callable[[float, str], None]] = None
        self.on_mission_event: Optional[Callable[[str, str], None]] = None
        
        # Travel time history for estimation
        self.travel_time_history: List[float] = []
        
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
    
    def estimate_travel_time(self, distance: float, average_speed: float = 10.0) -> float:
        """Estimate travel time in seconds."""
        if average_speed <= 0:
            return float('inf')
        return distance / average_speed
    
    def get_average_travel_time(self) -> float:
        """Get average travel time from history."""
        if self.travel_time_history:
            return sum(self.travel_time_history) / len(self.travel_time_history)
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
        
        Each subsequent drone flies at altitude +15m from the previous.
        """
        if len(drone_list) < 2:
            logger.error("Need at least 2 drones for relay mission")
            return False
        
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
                self.cmd_takeoff(namespace)
            
            mission.state = MissionState.CLIMBING_TO_ALTITUDE
            self._update_status(namespace, mission.state, "Taking off...")
        
        elif mission.state == MissionState.CLIMBING_TO_ALTITUDE:
            # Check if altitude reached
            if self.get_drone_position:
                _, _, alt = self.get_drone_position(namespace)
                
                if alt >= (mission.assigned_altitude - self.config.altitude_tolerance):
                    # Altitude reached, start transit
                    mission.state = MissionState.TRANSIT_TO_MONITORING
                    mission.transit_start_time = time.time()
                    self._start_transit(namespace, mission)
                    self._update_status(namespace, mission.state, "Climbing to altitude...")
        
        elif mission.state == MissionState.TRANSIT_TO_MONITORING:
            self._check_approach(namespace, mission)
        
        elif mission.state == MissionState.APPROACHING_POINT:
            # Already recording, check if reached
            if self.get_waypoint_reached and self.get_waypoint_reached(namespace):
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
        """Start transit to monitoring point."""
        if self.get_drone_position:
            lat, lon, _ = self.get_drone_position(namespace)
            
            # Calculate bearing to target
            bearing = self.calculate_bearing(
                lat, lon,
                self.config.monitoring_lat, self.config.monitoring_lon
            )
            
            # Estimate travel time
            distance = self.haversine_distance(
                lat, lon,
                self.config.monitoring_lat, self.config.monitoring_lon
            )
            mission.estimated_travel_time = self.estimate_travel_time(distance)
            
            # Send waypoint command
            if self.cmd_goto_waypoint:
                self.cmd_goto_waypoint(
                    namespace,
                    self.config.monitoring_lat,
                    self.config.monitoring_lon,
                    self.config.monitoring_alt,
                    bearing
                )
            
            self._emit_event(namespace, f"Transit to monitoring point ({distance:.0f}m, ~{mission.estimated_travel_time:.0f}s)")
    
    def _check_approach(self, namespace: str, mission: DroneMissionStatus):
        """Check if drone is approaching video trigger distance."""
        if not self.get_drone_position:
            return
        
        lat, lon, _ = self.get_drone_position(namespace)
        distance = self.haversine_distance(
            lat, lon,
            self.config.monitoring_lat, self.config.monitoring_lon
        )
        
        # Check if within video trigger distance
        if distance <= mission.video_trigger_distance and not mission.video_started:
            if self.cmd_start_recording:
                self.cmd_start_recording(namespace)
            mission.video_started = True
            mission.state = MissionState.APPROACHING_POINT
            self._emit_event(namespace, f"Video recording started at {distance:.1f}m from target")
        
        # Check if reached target
        if distance <= self.config.position_tolerance:
            if not mission.video_started:
                # Start recording if not already started
                if self.cmd_start_recording:
                    self.cmd_start_recording(namespace)
                mission.video_started = True
            
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
            # Stop recording
            if old_mission.video_started and self.cmd_stop_recording:
                self.cmd_stop_recording(old_ns)
            
            # Command RTH
            if self.cmd_rth:
                self.cmd_rth(old_ns)
            
            old_mission.state = MissionState.RETURNING_HOME
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
    
    def is_mission_active(self) -> bool:
        """Check if any mission is active."""
        return self.relay_state == RelayState.ACTIVE or \
               any(m.state not in [MissionState.IDLE, MissionState.COMPLETED, MissionState.ABORTED, MissionState.ERROR] 
                   for m in self.drone_missions.values())
    
    def calculate_drones_needed(self, mission_duration_hours: float = 1.0) -> int:
        """
        Calculate minimum drones needed for continuous coverage.
        
        Args:
            mission_duration_hours: How long to maintain coverage
        
        Returns:
            Minimum number of drones needed
        """
        # Assume average flight time of 25 minutes (1500 seconds) for DJI drones
        avg_flight_time = 1500  # seconds
        
        # Get average travel time
        avg_travel = self.get_average_travel_time()
        if avg_travel == 0:
            # Estimate based on distance if no history
            if self.get_drone_position and self.drone_order:
                first_drone = self.drone_order[0]
                lat, lon, _ = self.get_drone_position(first_drone)
                distance = self.haversine_distance(lat, lon, 
                    self.config.monitoring_lat, self.config.monitoring_lon)
                avg_travel = self.estimate_travel_time(distance)
            else:
                avg_travel = 300  # Assume 5 minutes
        
        # Effective monitoring time per drone
        effective_time = avg_flight_time - (2 * avg_travel) - self.config.safety_buffer_seconds
        
        if effective_time <= 0:
            return float('inf')  # Impossible
        
        mission_duration_seconds = mission_duration_hours * 3600
        return max(1, math.ceil(mission_duration_seconds / effective_time))
    
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
