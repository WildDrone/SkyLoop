"""
Perpetual Drone Monitoring Groundstation

A ROS2-based groundstation for autonomous, perpetual monitoring of a single GPS point
using multiple drones with dynamic relay missions.

Author: Edouard Rolland
Project: WildDrone
"""

import math
import time
import subprocess
import signal
import os
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Tuple, Callable
from pathlib import Path
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool, Empty, String, Float64MultiArray, Int32
from sensor_msgs.msg import NavSatFix

from nicegui import ui, app, ui_run

from groundstation.mission_controller import MissionController, MissionState, MissionMode


# ============================================================================
# DATA STRUCTURES
# ============================================================================

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


class DroneRTHPredictor:
    """
    Predicts when DJI will trigger RTH based on battery drain.
    
    Uses linear regression on battery history to predict when battery will reach
    the RTH trigger threshold (batteryNeededToGoHome + margin).
    
    DJI returns battery as integer steps, so we only keep the FIRST point
    of each battery level to avoid skewing the regression.
    
    This is a simplified version for ROS2 integration - receives data from DroneData.
    """
    
    MAX_POINTS = 100  # Max unique battery levels to keep (100% to 0%)
    RTH_TRIGGER_MARGIN = 2  # RTH triggers when battery <= batt_needed_rth + margin
    MIN_DATAPOINTS = 3  # Minimum battery points needed before using RTH predictor
    
    # Import numpy once at class level
    import numpy as np
    import time as _time
    import csv
    import os
    from datetime import datetime as _datetime
    
    def __init__(self, namespace: str = "drone"):
        self.namespace = namespace
        
        # Data storage - only first point per battery level
        # Key: battery level (int), Value: (timestamp, batt_needed_rth)
        self.battery_points: dict = {}  # {battery_level: (timestamp, batt_needed_rth)}
        
        # Track max battery needed to go home (for conservative prediction)
        self.max_batt_needed_rth = 0.0
        
        # Track last seen battery level to detect changes
        self.last_battery_level = None
        
        # State
        self.is_active = False  # Only predict when drone is in MONITORING state
        self.start_time = None
        
        # CSV logging
        self.csv_file = None
        self.csv_writer = None
        self.csv_path = None
        
    def _init_csv_logging(self):
        """Initialize CSV file for logging RTH predictor data."""
        if self.csv_file is not None:
            return  # Already initialized
        
        # Create logs directory if it doesn't exist
        log_dir = self.os.path.expanduser("~/rth_predictor_logs")
        self.os.makedirs(log_dir, exist_ok=True)
        
        # Create filename with timestamp and drone namespace
        timestamp = self._datetime.now().strftime("%Y%m%d_%H%M%S")
        safe_ns = self.namespace.replace("/", "_")
        self.csv_path = self.os.path.join(log_dir, f"rth_predictor_{safe_ns}_{timestamp}.csv")
        
        # Open CSV file and write header
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = self.csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'elapsed_time',
            'battery_level',
            'batt_needed_to_go_home',
            'max_batt_needed_rth',
            'rth_threshold',
            'drain_rate_per_min',
            'predicted_rth_seconds',
            'data_points',
            'slope',
            'intercept'
        ])
        
    def _log_to_csv(self, battery: float, batt_needed: float):
        """Log current state to CSV file."""
        if not self.is_active or self.start_time is None:
            return
            
        if self.csv_writer is None:
            self._init_csv_logging()
        
        elapsed = self._time.time() - self.start_time
        
        # Get current prediction info
        times, batteries = self._get_regression_data()
        slope = 0.0
        intercept = 0.0
        drain_rate = 0.0
        predicted_rth = float('inf')
        rth_threshold = self.max_batt_needed_rth + self.RTH_TRIGGER_MARGIN
        
        if times is not None and len(times) >= 2:
            try:
                slope, intercept = self.np.polyfit(times, batteries, 1)
                drain_rate = -slope * 60  # %/min
                
                if slope < 0:
                    t_rth = (rth_threshold - intercept) / slope
                    predicted_rth = max(0.0, t_rth - elapsed)
            except:
                pass
        
        # Write row
        self.csv_writer.writerow([
            f"{elapsed:.2f}",
            f"{battery:.1f}",
            f"{batt_needed:.1f}",
            f"{self.max_batt_needed_rth:.1f}",
            f"{rth_threshold:.1f}",
            f"{drain_rate:.4f}",
            f"{predicted_rth:.1f}" if predicted_rth != float('inf') else "inf",
            len(self.battery_points),
            f"{slope:.6f}",
            f"{intercept:.2f}"
        ])
        self.csv_file.flush()  # Ensure data is written immediately
        
    def close_csv(self):
        """Close CSV file when done."""
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None
        
    def reset(self):
        """Reset the predictor when drone state changes."""
        # Close previous CSV if exists
        self.close_csv()
        
        self.battery_points.clear()
        self.max_batt_needed_rth = 0.0
        self.last_battery_level = None
        self.start_time = None
        self.is_active = False
    
    def update(self, battery: float, batt_needed_to_go_home: float, is_monitoring: bool):
        """
        Update predictor with new telemetry data.
        
        Only stores the FIRST timestamp for each battery level (integer).
        DJI returns battery as steps, so multiple readings at same level
        would skew the regression.
        
        Args:
            battery: Current battery percentage
            batt_needed_to_go_home: DJI's batteryNeededToGoHome value
            is_monitoring: Whether drone is currently in MONITORING state
        """
        # Start/stop tracking based on monitoring state
        if is_monitoring and not self.is_active:
            # Just started monitoring - reset and start fresh
            self.reset()
            self.is_active = True
            self.start_time = self._time.time()
        elif not is_monitoring and self.is_active:
            # Stopped monitoring - deactivate but keep data for reference
            self.is_active = False
            return
        
        if not self.is_active:
            return
        
        # Convert battery to integer (DJI uses integer steps)
        battery_int = int(battery)
        
        # Only store the FIRST point for each battery level
        if battery_int not in self.battery_points:
            elapsed = self._time.time() - self.start_time
            self.battery_points[battery_int] = (elapsed, batt_needed_to_go_home)
        
        self.last_battery_level = battery_int
        
        # Track maximum battery needed to go home (conservative approach)
        if batt_needed_to_go_home > self.max_batt_needed_rth:
            self.max_batt_needed_rth = batt_needed_to_go_home
        
        # Log data to CSV
        self._log_to_csv(battery, batt_needed_to_go_home)
    
    def _get_regression_data(self):
        """
        Get arrays of (timestamps, battery_levels) for regression.
        
        Returns:
            Tuple of (times_array, batteries_array) sorted by time,
            or (None, None) if insufficient data.
        """
        if len(self.battery_points) < 2:
            return None, None
        
        # Extract and sort by timestamp
        points = [(t, batt) for batt, (t, _) in self.battery_points.items()]
        points.sort(key=lambda x: x[0])  # Sort by timestamp
        
        times = self.np.array([p[0] for p in points])
        batteries = self.np.array([p[1] for p in points])
        
        return times, batteries
    
    def predict_rth_time(self) -> float:
        """
        Predict time until RTH is triggered in seconds.
        
        Uses only one point per battery level for accurate regression.
        
        Returns:
            Predicted seconds until RTH, or float('inf') if cannot predict.
        """
        if not self.is_active or self.start_time is None:
            return float('inf')
        
        # Get regression data (one point per battery level)
        times, batteries = self._get_regression_data()
        if times is None:
            return float('inf')
        
        # Linear regression: battery = slope*t + intercept
        try:
            slope, intercept = self.np.polyfit(times, batteries, 1)
        except:
            return float('inf')
        
        # slope should be negative (battery draining)
        if slope >= 0:
            return float('inf')  # Battery not draining
        
        # Use MAX battery needed to go home for conservative prediction
        # This ensures we don't underestimate when RTH will trigger
        rth_threshold = self.max_batt_needed_rth + self.RTH_TRIGGER_MARGIN
        
        # Current time (use actual current time relative to start, not last stored timestamp)
        current_time = self._time.time() - self.start_time
        
        # Find when battery line crosses threshold
        # battery(t) = slope * t + intercept = rth_threshold
        # t_rth = (rth_threshold - intercept) / slope
        t_rth = (rth_threshold - intercept) / slope
        
        # Time until RTH
        time_until_rth = t_rth - current_time
        
        return max(0.0, time_until_rth)
    
    def get_datapoints(self) -> int:
        """Get the number of datapoints collected for the RTH prediction regression."""
        return len(self.battery_points)
    
    def get_drain_rate(self) -> float:
        """Get battery drain rate in %/second using one point per battery level."""
        times, batteries = self._get_regression_data()
        if times is None:
            return 0.0
        
        try:
            slope, intercept = self.np.polyfit(times, batteries, 1)
            return -slope  # Negative slope = positive drain rate
        except:
            return 0.0
    
    def get_debug_info(self) -> dict:
        """
        Get detailed debug information about the prediction.
        
        Returns:
            Dict with all computation details for debugging.
        """
        # Get current battery level from the most recent point
        current_battery = self.last_battery_level if self.last_battery_level is not None else 0.0
        
        # Get current batt_needed_rth from the last stored point
        current_batt_needed = 0.0
        if self.battery_points and self.last_battery_level in self.battery_points:
            _, current_batt_needed = self.battery_points[self.last_battery_level]
        
        info = {
            'is_active': self.is_active,
            'data_points': len(self.battery_points),
            'current_battery': float(current_battery),
            'batt_needed_to_go_home': current_batt_needed,
            'max_batt_needed_to_go_home': self.max_batt_needed_rth,
            'rth_threshold': 0.0,
            'slope': 0.0,
            'intercept': 0.0,
            'drain_rate_per_min': 0.0,
            'elapsed_since_monitoring': 0.0,
            't_rth_absolute': 0.0,
            'predicted_rth_seconds': float('inf'),
            'dji_remaining_flight_time': 0.0,  # Will be filled by caller
        }
        
        if not self.is_active or self.start_time is None:
            return info
        
        # Get regression data (one point per battery level)
        times, batteries = self._get_regression_data()
        if times is None:
            return info
        
        try:
            slope, intercept = self.np.polyfit(times, batteries, 1)
        except:
            return info
        
        info['slope'] = slope
        info['intercept'] = intercept
        info['drain_rate_per_min'] = -slope * 60  # %/min
        
        # Use MAX battery needed to go home for conservative prediction
        rth_threshold = self.max_batt_needed_rth + self.RTH_TRIGGER_MARGIN
        info['rth_threshold'] = rth_threshold
        
        # Current time relative to start
        current_time = self._time.time() - self.start_time
        info['elapsed_since_monitoring'] = current_time
        
        if slope >= 0:
            return info  # Battery not draining
        
        # When does line cross threshold?
        t_rth = (rth_threshold - intercept) / slope
        info['t_rth_absolute'] = t_rth
        
        # Time until RTH
        time_until_rth = max(0.0, t_rth - current_time)
        info['predicted_rth_seconds'] = time_until_rth
        
        # Add chart data for visualization (convert to minutes for readability)
        # Battery scatter points: [[time_min, battery], ...]
        chart_battery_points = [[t / 60, b] for t, b in zip(times, batteries)]
        info['chart_battery_points'] = chart_battery_points
        
        # Regression line: from t=0 to t=t_rth (or current + 5min if t_rth is too far)
        t_end = min(t_rth, current_time + 300) if t_rth > 0 else current_time + 300  # Max 5min projection
        t_start = 0
        regression_line = [
            [t_start / 60, slope * t_start + intercept],
            [t_end / 60, slope * t_end + intercept]
        ]
        info['chart_regression_line'] = regression_line
        
        # RTH threshold line (horizontal) - using MAX threshold
        threshold_line = [
            [0, rth_threshold],
            [t_end / 60, rth_threshold]
        ]
        info['chart_threshold_line'] = threshold_line
        
        # RTH crossing point (where regression meets threshold)
        if t_rth > 0:
            info['chart_rth_point'] = [[t_rth / 60, rth_threshold]]
        else:
            info['chart_rth_point'] = []
        
        # Current time vertical marker (from battery level to 0)
        current_time_min = current_time / 60
        info['chart_current_time_line'] = [
            [current_time_min, 0],
            [current_time_min, 100]
        ]
        
        # Current position marker (where we are now on the chart)
        info['chart_current_point'] = [[current_time_min, current_battery]]
        
        return info


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


# ============================================================================
# MISSION CALCULATOR
# ============================================================================

class MissionCalculator:
    """Calculates mission parameters and relay timing."""
    
    EARTH_RADIUS = 6371000  # meters
    
    @staticmethod
    def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Calculate the great-circle distance between two GPS points in meters."""
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lat = math.radians(lat2 - lat1)
        delta_lon = math.radians(lon2 - lon1)
        
        a = math.sin(delta_lat / 2) ** 2 + \
            math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(delta_lon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        
        return MissionCalculator.EARTH_RADIUS * c
    
    @staticmethod
    def calculate_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Calculate initial bearing from point 1 to point 2 in degrees."""
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
    HORIZONTAL_SPEED_PID = 15.0     # m/s - PID control mode
    HORIZONTAL_SPEED_NATIVE = 15.0  # m/s - DJI native trajectory mode
    
    @staticmethod
    def estimate_travel_time(distance: float, altitude: float = 50.0,
                             horizontal_speed: float = 15.0, vertical_speed: float = 4.0) -> float:
        """
        Estimate travel time in seconds, including climb time.
        
        Args:
            distance: Horizontal distance in meters
            altitude: Target altitude in meters (default 50m)
            horizontal_speed: Horizontal flight speed in m/s 
                              (15 m/s for PID mode, 15 m/s for DJI native)
            vertical_speed: Vertical climb speed in m/s (default 4 m/s)
        
        Returns:
            Estimated travel time in seconds
        """
        if horizontal_speed <= 0:
            return float('inf')
        
        # Time to climb to altitude (from ground)
        climb_time = altitude / vertical_speed if vertical_speed > 0 else 0
        
        # Time for horizontal translation
        horizontal_time = distance / horizontal_speed
        
        # Total travel time = climb + horizontal (sequential)
        return climb_time + horizontal_time
    
    @staticmethod
    def calculate_relay_countdown(
        remaining_flight_time: float,
        time_to_monitoring_point: float,
        safety_buffer: float = 60.0
    ) -> float:
        """
        Calculate when the next drone should launch.
        
        Returns seconds until next drone should take off, or 0 if it should launch now.
        """
        # Time available for monitoring = remaining - time to return home - buffer
        # Next drone should arrive when current drone needs to start returning
        
        countdown = remaining_flight_time - time_to_monitoring_point - safety_buffer
        return max(0.0, countdown)
    
    @staticmethod
    def calculate_drones_needed(
        total_mission_time: float,
        flight_time_per_drone: float,
        travel_time: float,
        safety_buffer: float = 60.0
    ) -> int:
        """Calculate minimum number of drones needed for continuous coverage."""
        if flight_time_per_drone <= 0:
            return 0
        
        effective_monitoring_time = flight_time_per_drone - (2 * travel_time) - safety_buffer
        if effective_monitoring_time <= 0:
            return float('inf')  # Impossible with current parameters
        
        return max(1, math.ceil(total_mission_time / effective_monitoring_time))


# ============================================================================
# ROS2 NODE FOR PERPETUAL MONITORING
# ============================================================================

class PerpetualMonitorNode(Node):
    """
    ROS2 Node for perpetual drone monitoring.
    
    Manages multiple drones, relay missions, and provides data to the GUI.
    """
    
    _instance = None
    
    @staticmethod
    def get_instance():
        if PerpetualMonitorNode._instance is None:
            PerpetualMonitorNode._instance = PerpetualMonitorNode()
        return PerpetualMonitorNode._instance
    
    def __init__(self):
        super().__init__('perpetual_monitor_node')
        self.get_logger().info("Initializing Perpetual Monitor Node")
        
        # Drone management
        self.drones: Dict[str, DroneData] = {}
        self.drone_publishers: Dict[str, Dict[str, any]] = {}
        self.drone_subscribers: Dict[str, Dict[str, any]] = {}
        self.drone_processes: Dict[str, subprocess.Popen] = {}  # Controller node processes
        
        # RTH Predictors (one per drone for relay timing)
        self.rth_predictors: Dict[str, DroneRTHPredictor] = {}
        
        # Mission state
        self.mission = RelayMission()
        self.monitoring_point = MonitoringPoint()
        self.calculator = MissionCalculator()
        
        # Mission controller for state machine
        self.mission_controller = MissionController()
        self._setup_mission_controller_callbacks()
        
        # UI callback reference
        self.ui_handler = None
        
        # Mission update timer
        self.mission_timer = self.create_timer(1.0, self._update_mission_state)
        
        # Relay logic timer (faster updates for countdown)
        self.relay_timer = self.create_timer(0.5, self._update_relay_logic)
        
        self.get_logger().info("Perpetual Monitor Node initialized")
    
    def _setup_mission_controller_callbacks(self):
        """Set up callbacks between ROS node and mission controller."""
        mc = self.mission_controller
        
        # Command callbacks
        mc.cmd_takeoff = self.send_takeoff
        mc.cmd_land = self.send_land
        mc.cmd_rth = self.send_rth
        mc.cmd_goto_waypoint = self.send_goto_waypoint
        mc.cmd_goto_waypoint_dji_native = self.send_goto_waypoint_dji_native
        mc.cmd_goto_altitude = self.send_goto_altitude
        mc.cmd_set_rth_altitude = self.send_set_rth_altitude
        mc.cmd_start_recording = self.send_start_recording
        mc.cmd_stop_recording = self.send_stop_recording
        mc.cmd_set_gimbal_pitch = self.send_gimbal_pitch
        mc.cmd_goto_yaw = self.send_goto_yaw
        mc.cmd_abort = self.send_abort_mission
        
        # Telemetry getters
        mc.get_drone_position = lambda ns: (
            self.drones[ns].latitude, 
            self.drones[ns].longitude, 
            self.drones[ns].altitude
        ) if ns in self.drones else (0.0, 0.0, 0.0)
        
        mc.get_drone_home_position = lambda ns: (
            self.drones[ns].home_latitude,
            self.drones[ns].home_longitude
        ) if ns in self.drones else (0.0, 0.0)
        
        mc.get_drone_heading = lambda ns: self.drones[ns].heading if ns in self.drones else 0.0
        mc.get_drone_gimbal_pitch = lambda ns: self.drones[ns].gimbal_pitch if ns in self.drones else 0.0
        mc.get_remaining_flight_time = self._get_remaining_flight_time_with_prediction
        mc.get_battery_level = lambda ns: self.drones[ns].battery_level if ns in self.drones else 0.0
        mc.get_satellite_count = lambda ns: self.drones[ns].satellite_count if ns in self.drones else 0
        mc.get_is_recording = lambda ns: self.drones[ns].is_recording if ns in self.drones else False
        mc.get_waypoint_reached = lambda ns: self.drones[ns].waypoint_reached if ns in self.drones else False
        mc.get_altitude_reached = lambda ns: self.drones[ns].altitude_reached if ns in self.drones else False
        mc.get_configured_speed = self._get_active_navigation_speed  # Returns speed based on current nav mode
        mc.get_connected_drones = lambda: [ns for ns, d in self.drones.items() if d.is_connected]
        mc.get_rth_predictor_datapoints = self._get_rth_predictor_datapoints
        
        # Status callbacks
        mc.on_status_update = self._on_mission_status_update
        mc.on_relay_countdown = self._on_relay_countdown_update
        mc.on_mission_event = self._on_mission_event
    
    def _get_remaining_flight_time_with_prediction(self, namespace: str) -> float:
        """
        Get remaining flight time for a drone.
        
        Uses RTH predictor when drone is in MONITORING state for accurate relay timing.
        Falls back to DJI's remaining_flight_time when predictor has insufficient data.
        Returns -1 when collecting data (less than MIN_DATAPOINTS).
        
        Args:
            namespace: The drone's ROS namespace
            
        Returns:
            Remaining flight time in seconds, or -1 if collecting data
        """
        if namespace not in self.drones:
            return 0.0
        
        drone = self.drones[namespace]
        
        # Use RTH predictor when drone is in MONITORING state
        if drone.state == DroneState.MONITORING and namespace in self.rth_predictors:
            predictor = self.rth_predictors[namespace]
            
            # Check if we have enough datapoints for prediction
            if predictor.get_datapoints() < DroneRTHPredictor.MIN_DATAPOINTS:
                return -1  # Signal: still collecting data
            
            predicted_time = predictor.predict_rth_time()
            
            # Return prediction if valid (not infinity and reasonable)
            if predicted_time != float('inf') and predicted_time > 0:
                return predicted_time
        
        # Fallback to DJI's remaining_flight_time when predictor has insufficient data
        # This is critical - without this, returning 0 would trigger immediate RTH!
        return drone.remaining_flight_time
    
    def _get_rth_predictor_datapoints(self, namespace: str) -> int:
        """
        Get the number of datapoints collected by the RTH predictor for a drone.
        
        Args:
            namespace: The drone's ROS namespace
            
        Returns:
            Number of datapoints (battery level changes) collected
        """
        if namespace in self.rth_predictors:
            return self.rth_predictors[namespace].get_datapoints()
        return 0
    
    def _on_mission_status_update(self, namespace: str, state: MissionState, message: str):
        """Handle mission status updates from controller."""
        if namespace in self.drones:
            # Map MissionState to DroneState
            state_map = {
                MissionState.IDLE: DroneState.IDLE,
                MissionState.TAKING_OFF: DroneState.TAKING_OFF,
                MissionState.CLIMBING_TO_ALTITUDE: DroneState.TAKING_OFF,
                MissionState.TRANSIT_TO_MONITORING: DroneState.FLYING_TO_POINT,
                MissionState.APPROACHING_POINT: DroneState.FLYING_TO_POINT,
                MissionState.MONITORING: DroneState.MONITORING,
                MissionState.WAITING_FOR_RELAY: DroneState.WAITING_FOR_RELAY,
                MissionState.RETURNING_HOME: DroneState.RETURNING_HOME,
                MissionState.COMPLETED: DroneState.IDLE,
                MissionState.ABORTED: DroneState.IDLE,
                MissionState.ERROR: DroneState.EMERGENCY,
            }
            
            if state in state_map:
                self.drones[namespace].state = state_map[state]
                self.drones[namespace].current_task = message
                
                if self.ui_handler:
                    self.ui_handler.update_drone_state(namespace, state_map[state])
    
    def _on_relay_countdown_update(self, countdown: float, next_drone: str, timing_breakdown: dict = None):
        """Handle relay countdown updates."""
        self.mission.relay_countdown = countdown
        self.mission.next_drone = next_drone
        
        if self.ui_handler:
            self.ui_handler.update_relay_countdown(countdown, next_drone)
    
    def _on_mission_event(self, namespace: str, event: str):
        """Handle mission events for logging."""
        self.get_logger().info(f"[{namespace}] {event}")
    
    def _get_active_navigation_speed(self) -> float:
        """Get the active navigation speed based on current mode.
        
        Returns:
            Speed in m/s - DJI_NATIVE_SPEED if DJI Native mode is active,
            otherwise PID_SPEED.
        """
        if hasattr(self, 'mission_controller') and self.mission_controller.use_dji_native:
            return self.DJI_NATIVE_SPEED
        return self.PID_SPEED

    # ========================================================================
    # DRONE CONNECTION MANAGEMENT
    # ========================================================================
    
    def connect_drone(self, ip_address: str, namespace: str = None) -> bool:
        """
        Dynamically connect a new drone to the system.
        
        Args:
            ip_address: IP address of the drone's RC controller (empty string triggers auto-discovery)
            namespace: ROS namespace for the drone (auto-generated if not provided)
        
        Returns:
            True if connection successful, False otherwise
        """
        if namespace is None:
            namespace = f"drone_{len(self.drones) + 1}"
        
        # Allow reconnection of a previously disconnected drone while preserving its color/identity
        existing_drone = self.drones.get(namespace)
        if existing_drone and existing_drone.is_connected:
            self.get_logger().warning(f"Drone {namespace} already connected")
            return False
        
        if ip_address:
            self.get_logger().info(f"Connecting drone {namespace} at {ip_address}")
        else:
            self.get_logger().info(f"Connecting drone {namespace} with auto-discovery")
        
        # Launch dji_controller node for this drone as a subprocess
        try:
            # Build the ros2 run command with namespace and IP parameter
            cmd = [
                'ros2', 'run', 'dji_controller', 'dji_node',
                '--ros-args',
                '-r', f'__ns:=/{namespace}',
                '-p', f'ip_rc:={ip_address}'
            ]
            
            self.get_logger().info(f"Launching controller node: {' '.join(cmd)}")
            
            # Start the process
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid  # Create new process group for clean shutdown
            )
            
            # Store the process reference
            self.drone_processes[namespace] = process
            
            # Wait briefly for the node to initialize
            time.sleep(1.0)
            
            # Check if process started successfully
            if process.poll() is not None:
                # Process already terminated - connection failed
                stdout, stderr = process.communicate()
                self.get_logger().error(f"Controller node failed to start: {stderr.decode()}")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Failed to launch controller node: {e}")
            return False
        
        # Preserve prior color if this namespace existed; otherwise assign next palette color
        drone_colors = ['#FF6B6B', '#4ECDC4', '#45B7D1', '#96CEB4', '#FFEAA7', '#DDA0DD', '#98D8C8', '#F7DC6F']
        if existing_drone and existing_drone.color:
            drone_color = existing_drone.color
        else:
            color_idx = len(self.drones) % len(drone_colors)
            drone_color = drone_colors[color_idx]
        
        # Create drone data entry
        drone = DroneData(
            ip_address=ip_address,
            namespace=namespace,
            is_connected=True,
            connection_time=time.time(),
            state=DroneState.CONNECTED,
            color=drone_color
        )
        self.drones[namespace] = drone
        
        # Create RTH predictor for this drone
        self.rth_predictors[namespace] = DroneRTHPredictor(namespace=namespace)
        
        # Set up ROS subscribers for this drone's telemetry
        self._setup_drone_subscribers(namespace)
        
        # Set up ROS publishers for commands to this drone
        self._setup_drone_publishers(namespace)
        
        # If this drone is part of an active relay mission, reset its state for reuse
        if self.mission_controller.reset_drone_for_reuse(namespace):
            self.get_logger().info(f"Drone {namespace} reset for relay reuse")
        # Otherwise, if there's an active relay and this is a new drone, add it to the relay
        elif self.mission_controller.add_drone_to_relay(namespace):
            self.get_logger().info(f"Drone {namespace} automatically added to active relay mission")
        
        # Notify UI
        if self.ui_handler:
            self.ui_handler.on_drone_connected(namespace, drone)
        
        self.get_logger().info(f"Drone {namespace} connected successfully")
        return True
    
    def disconnect_drone(self, namespace: str) -> bool:
        """
        Disconnect a drone from the system.
        
        Only allows disconnection if drone is not in active flight.
        """
        if namespace not in self.drones:
            self.get_logger().warning(f"Drone {namespace} not found")
            return False
        
        drone = self.drones[namespace]
        
        # Check if drone is flying
        if drone.state in [DroneState.TAKING_OFF, DroneState.FLYING_TO_POINT, 
                          DroneState.MONITORING, DroneState.RETURNING_HOME]:
            self.get_logger().warning(f"Cannot disconnect {namespace}: drone is in flight")
            return False
        
        # Stop the controller node process
        if namespace in self.drone_processes:
            try:
                process = self.drone_processes[namespace]
                # Send SIGTERM to the process group
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                process.wait(timeout=5)
                self.get_logger().info(f"Controller node for {namespace} terminated")
            except Exception as e:
                self.get_logger().warning(f"Error stopping controller node: {e}")
                # Force kill if graceful shutdown failed
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                except:
                    pass
            del self.drone_processes[namespace]
        
        # Clean up subscribers
        if namespace in self.drone_subscribers:
            for sub in self.drone_subscribers[namespace].values():
                self.destroy_subscription(sub)
            del self.drone_subscribers[namespace]
        
        # Clean up publishers
        if namespace in self.drone_publishers:
            for pub in self.drone_publishers[namespace].values():
                self.destroy_publisher(pub)
            del self.drone_publishers[namespace]
        
        # Remove from mission if needed
        if namespace in self.mission.drones_in_mission:
            self.mission.drones_in_mission.remove(namespace)
        
        # Clear drone from mission controller
        if hasattr(self, 'mission_controller') and namespace in self.mission_controller.drone_missions:
            del self.mission_controller.drone_missions[namespace]
        
        # Clean up RTH predictor (close CSV file first). Keep entry to preserve history if needed.
        if namespace in self.rth_predictors:
            self.rth_predictors[namespace].close_csv()
            del self.rth_predictors[namespace]
        
        # Mark drone as disconnected but keep its data (color, namespace, last known state)
        drone.is_connected = False
        drone.state = DroneState.DISCONNECTED
        
        # Notify UI
        if self.ui_handler:
            self.ui_handler.on_drone_disconnected(namespace)
        
        self.get_logger().info(f"Drone {namespace} disconnected")
        return True
    
    def _setup_drone_subscribers(self, namespace: str):
        """Set up all ROS telemetry subscribers for a drone."""
        self.drone_subscribers[namespace] = {}
        
        # Location
        self.drone_subscribers[namespace]['location'] = self.create_subscription(
            NavSatFix, f"{namespace}/location",
            lambda msg, ns=namespace: self._on_location(ns, msg), 10
        )
        
        # Battery
        self.drone_subscribers[namespace]['battery'] = self.create_subscription(
            Float64, f"{namespace}/battery_level",
            lambda msg, ns=namespace: self._on_battery(ns, msg.data), 10
        )
        
        # Heading
        self.drone_subscribers[namespace]['heading'] = self.create_subscription(
            Float64, f"{namespace}/heading",
            lambda msg, ns=namespace: self._on_heading(ns, msg.data), 10
        )
        
        # Speed
        self.drone_subscribers[namespace]['speed'] = self.create_subscription(
            Float64, f"{namespace}/speed",
            lambda msg, ns=namespace: self._on_speed(ns, msg.data), 10
        )
        
        # Remaining flight time
        self.drone_subscribers[namespace]['remaining_flight_time'] = self.create_subscription(
            Float64, f"{namespace}/remaining_flight_time",
            lambda msg, ns=namespace: self._on_remaining_flight_time(ns, msg.data), 10
        )
        
        # Time needed to go home
        self.drone_subscribers[namespace]['time_to_home'] = self.create_subscription(
            Float64, f"{namespace}/time_needed_to_go_home",
            lambda msg, ns=namespace: self._on_time_to_home(ns, msg.data), 10
        )
        
        # Distance to home
        self.drone_subscribers[namespace]['distance_to_home'] = self.create_subscription(
            Float64, f"{namespace}/distance_to_home",
            lambda msg, ns=namespace: self._on_distance_to_home(ns, msg.data), 10
        )
        
        # Home location
        self.drone_subscribers[namespace]['home_location'] = self.create_subscription(
            NavSatFix, f"{namespace}/home_location",
            lambda msg, ns=namespace: self._on_home_location(ns, msg), 10
        )
        
        # Home set status
        self.drone_subscribers[namespace]['home_set'] = self.create_subscription(
            Bool, f"{namespace}/home_set",
            lambda msg, ns=namespace: self._on_home_set(ns, msg.data), 10
        )
        
        # Waypoint reached
        self.drone_subscribers[namespace]['waypoint_reached'] = self.create_subscription(
            Bool, f"{namespace}/waypoint_reached",
            lambda msg, ns=namespace: self._on_waypoint_reached(ns, msg.data), 10
        )
        
        # Altitude reached
        self.drone_subscribers[namespace]['altitude_reached'] = self.create_subscription(
            Bool, f"{namespace}/altitude_reached",
            lambda msg, ns=namespace: self._on_altitude_reached(ns, msg.data), 10
        )
        
        # Recording status
        self.drone_subscribers[namespace]['is_recording'] = self.create_subscription(
            Bool, f"{namespace}/camera/is_recording",
            lambda msg, ns=namespace: self._on_recording_status(ns, msg.data), 10
        )
        
        # Satellite count
        self.drone_subscribers[namespace]['satellites'] = self.create_subscription(
            Int32, f"{namespace}/satellite_count",
            lambda msg, ns=namespace: self._on_satellite_count(ns, msg.data), 10
        )
        
        # Gimbal pitch
        self.drone_subscribers[namespace]['gimbal_pitch'] = self.create_subscription(
            Float64, f"{namespace}/gimbal_pitch",
            lambda msg, ns=namespace: self._on_gimbal_pitch(ns, msg.data), 10
        )
        
        # Battery thresholds (for RTH predictor)
        self.drone_subscribers[namespace]['battery_needed_to_go_home'] = self.create_subscription(
            Float64, f"{namespace}/battery_needed_to_go_home",
            lambda msg, ns=namespace: self._on_battery_needed_to_go_home(ns, msg.data), 10
        )
        
        self.drone_subscribers[namespace]['battery_needed_to_land'] = self.create_subscription(
            Float64, f"{namespace}/battery_needed_to_land",
            lambda msg, ns=namespace: self._on_battery_needed_to_land(ns, msg.data), 10
        )
        
        # Flight mode (for RTH predictor - to know when drone is in MONITORING state)
        self.drone_subscribers[namespace]['flight_mode'] = self.create_subscription(
            String, f"{namespace}/flight_mode",
            lambda msg, ns=namespace: self._on_flight_mode(ns, msg.data), 10
        )
    
    def _setup_drone_publishers(self, namespace: str):
        """Set up all command publishers for a drone."""
        self.drone_publishers[namespace] = {}
        
        # Basic commands (Empty messages)
        for cmd in ['takeoff', 'land', 'rth', 'abort_mission', 
                    'enable_virtual_stick', 'camera/start_recording', 
                    'camera/stop_recording']:
            self.drone_publishers[namespace][cmd] = self.create_publisher(
                Empty, f"{namespace}/command/{cmd}", 10
            )
        
        # Waypoint command
        self.drone_publishers[namespace]['goto_waypoint'] = self.create_publisher(
            Float64MultiArray, f"{namespace}/command/goto_waypoint", 10
        )
        
        # Trajectory command
        self.drone_publishers[namespace]['goto_trajectory'] = self.create_publisher(
            String, f"{namespace}/command/goto_trajectory", 10
        )
        
        # DJI Native trajectory
        self.drone_publishers[namespace]['goto_trajectory_dji_native'] = self.create_publisher(
            String, f"{namespace}/command/goto_trajectory_dji_native", 10
        )
        
        # Altitude command
        self.drone_publishers[namespace]['goto_altitude'] = self.create_publisher(
            Float64, f"{namespace}/command/goto_altitude", 10
        )
        
        # Gimbal commands
        self.drone_publishers[namespace]['gimbal_pitch'] = self.create_publisher(
            Float64, f"{namespace}/command/gimbal_pitch", 10
        )
        self.drone_publishers[namespace]['gimbal_yaw'] = self.create_publisher(
            Float64, f"{namespace}/command/gimbal_yaw", 10
        )
        
        # Camera zoom command
        self.drone_publishers[namespace]['zoom_ratio'] = self.create_publisher(
            Float64, f"{namespace}/command/zoom_ratio", 10
        )
        
        # RTH altitude
        self.drone_publishers[namespace]['set_rth_altitude'] = self.create_publisher(
            Float64, f"{namespace}/command/set_rth_altitude", 10
        )
        
        # Yaw command
        self.drone_publishers[namespace]['goto_yaw'] = self.create_publisher(
            Float64, f"{namespace}/command/goto_yaw", 10
        )
    
    # ========================================================================
    # TELEMETRY CALLBACKS
    # ========================================================================
    
    def _on_location(self, namespace: str, msg: NavSatFix):
        if namespace in self.drones:
            drone = self.drones[namespace]
            drone.latitude = msg.latitude
            drone.longitude = msg.longitude
            drone.altitude = msg.altitude
            drone.last_telemetry_update = time.time()
            
            # Update state if was disconnected
            if drone.state == DroneState.DISCONNECTED:
                drone.state = DroneState.CONNECTED
            
            if self.ui_handler:
                self.ui_handler.update_drone_position(namespace, msg.latitude, msg.longitude, msg.altitude)
    
    def _on_battery(self, namespace: str, level: float):
        if namespace in self.drones:
            drone = self.drones[namespace]
            drone.battery_level = level
            
            # Update RTH predictor if available
            if namespace in self.rth_predictors:
                # Check if drone is in MONITORING state
                is_monitoring = drone.state == DroneState.MONITORING
                self.rth_predictors[namespace].update(
                    battery=level,
                    batt_needed_to_go_home=drone.battery_needed_to_go_home,
                    is_monitoring=is_monitoring
                )
            
            if self.ui_handler:
                self.ui_handler.update_drone_battery(namespace, level)
    
    def _on_heading(self, namespace: str, heading: float):
        if namespace in self.drones:
            self.drones[namespace].heading = heading
            if self.ui_handler:
                self.ui_handler.update_drone_heading(namespace, heading)
    
    def _on_speed(self, namespace: str, speed: float):
        if namespace in self.drones:
            self.drones[namespace].speed = speed
    
    def _on_remaining_flight_time(self, namespace: str, time_remaining: float):
        if namespace in self.drones:
            self.drones[namespace].remaining_flight_time = time_remaining
            if self.ui_handler:
                self.ui_handler.update_remaining_flight_time(namespace, time_remaining)
    
    def _on_time_to_home(self, namespace: str, time_to_home: float):
        if namespace in self.drones:
            self.drones[namespace].time_needed_to_go_home = time_to_home
    
    def _on_distance_to_home(self, namespace: str, distance: float):
        if namespace in self.drones:
            self.drones[namespace].distance_to_home = distance
    
    def _on_home_location(self, namespace: str, msg: NavSatFix):
        if namespace in self.drones:
            self.drones[namespace].home_latitude = msg.latitude
            self.drones[namespace].home_longitude = msg.longitude
    
    def _on_home_set(self, namespace: str, is_set: bool):
        if namespace in self.drones:
            self.drones[namespace].home_set = is_set
    
    def _on_waypoint_reached(self, namespace: str, reached: bool):
        if namespace in self.drones:
            self.drones[namespace].waypoint_reached = reached
            if reached:
                # Handle normal waypoint reached logic
                self._handle_waypoint_reached(namespace)
    
    def _on_altitude_reached(self, namespace: str, reached: bool):
        if namespace in self.drones:
            self.drones[namespace].altitude_reached = reached
    
    def _on_recording_status(self, namespace: str, is_recording: bool):
        if namespace in self.drones:
            self.drones[namespace].is_recording = is_recording
            if self.ui_handler:
                self.ui_handler.update_recording_status(namespace, is_recording)
    
    def _on_satellite_count(self, namespace: str, count: int):
        if namespace in self.drones:
            self.drones[namespace].satellite_count = count
    
    def _on_gimbal_pitch(self, namespace: str, pitch: float):
        if namespace in self.drones:
            self.drones[namespace].gimbal_pitch = pitch
    
    def _on_battery_needed_to_go_home(self, namespace: str, percentage: float):
        if namespace in self.drones:
            self.drones[namespace].battery_needed_to_go_home = percentage
    
    def _on_battery_needed_to_land(self, namespace: str, percentage: float):
        if namespace in self.drones:
            self.drones[namespace].battery_needed_to_land = percentage
    
    def _on_flight_mode(self, namespace: str, mode: str):
        if namespace in self.drones:
            self.drones[namespace].flight_mode = mode
    
    # ========================================================================
    # DRONE COMMANDS
    # ========================================================================
    
    def send_takeoff(self, namespace: str):
        """Command a drone to take off."""
        if namespace in self.drone_publishers:
            self.drone_publishers[namespace]['takeoff'].publish(Empty())
            self.drones[namespace].state = DroneState.TAKING_OFF
            self.get_logger().info(f"Takeoff command sent to {namespace}")
    
    def send_land(self, namespace: str):
        """Command a drone to land."""
        if namespace in self.drone_publishers:
            self.drone_publishers[namespace]['land'].publish(Empty())
            self.drones[namespace].state = DroneState.LANDING
            self.get_logger().info(f"Land command sent to {namespace}")
    
    def send_rth(self, namespace: str):
        """Command a drone to return to home."""
        if namespace in self.drone_publishers:
            self.drone_publishers[namespace]['rth'].publish(Empty())
            self.drones[namespace].state = DroneState.RETURNING_HOME
            self.get_logger().info(f"RTH command sent to {namespace}")
    
    def send_abort_mission(self, namespace: str):
        """Abort the current mission for a drone."""
        if namespace in self.drone_publishers:
            self.drone_publishers[namespace]['abort_mission'].publish(Empty())
            self.drones[namespace].state = DroneState.IDLE
            self.drones[namespace].mission_phase = MissionPhase.NONE
            self.get_logger().info(f"Abort mission command sent to {namespace}")
    
    # PID control speed (m/s) - can be set from UI
    PID_SPEED = 15.0
    
    def send_goto_waypoint(self, namespace: str, lat: float, lon: float, alt: float, yaw: float = 0.0, speed: float = None):
        """Command a drone to go to a waypoint using PID control.
        
        Args:
            namespace: Drone namespace
            lat: Target latitude
            lon: Target longitude
            alt: Target altitude
            yaw: Target yaw angle
            speed: Max speed in m/s (default: PID_SPEED = 15.0)
        """
        if speed is None:
            speed = self.PID_SPEED
            
        if namespace in self.drone_publishers:
            msg = Float64MultiArray()
            msg.data = [lat, lon, alt, yaw, speed]
            self.drone_publishers[namespace]['goto_waypoint'].publish(msg)
            self.drones[namespace].state = DroneState.FLYING_TO_POINT
            self.get_logger().info(f"Goto waypoint [PID @ {speed}m/s] sent to {namespace}: ({lat}, {lon}, {alt})")
    
    # DJI Native trajectory speed (m/s)
    DJI_NATIVE_SPEED = 15.0
    
    def send_goto_waypoint_dji_native(self, namespace: str, lat: float, lon: float, alt: float, speed: float = None):
        """Command a drone to go to a waypoint using DJI Native mode.
        
        Generates waypoints including current position for DJI native mission.
        DJI native missions require the first waypoint to be at or near the drone's current position.
        
        Args:
            namespace: Drone namespace
            lat: Target latitude
            lon: Target longitude  
            alt: Target altitude
            speed: Flight speed in m/s (default: DJI_NATIVE_SPEED = 15.0)
        """
        if speed is None:
            speed = self.DJI_NATIVE_SPEED
            
        if namespace in self.drone_publishers and namespace in self.drones:
            drone = self.drones[namespace]
            start_lat = drone.latitude
            start_lon = drone.longitude
            start_alt = drone.altitude
            
            # DJI native mission requires first waypoint at current position
            # Then we add the destination as second waypoint
            # Format: [(current_pos), (destination)]
            waypoints = [
                (start_lat, start_lon, start_alt),  # First waypoint: current position
                (lat, lon, alt)                      # Second waypoint: destination
            ]
            
            # Send as tuple: (speed, waypoints)
            msg = String()
            msg.data = str((speed, waypoints))
            self.drone_publishers[namespace]['goto_trajectory_dji_native'].publish(msg)
            self.drones[namespace].state = DroneState.FLYING_TO_POINT
            self.get_logger().info(f"Goto waypoint [DJI Native @ {speed}m/s] sent to {namespace}: from ({start_lat:.6f}, {start_lon:.6f}, {start_alt:.1f}) to ({lat:.6f}, {lon:.6f}, {alt:.1f})")
    
    def send_goto_altitude(self, namespace: str, altitude: float):
        """Command a drone to go to a specific altitude."""
        if namespace in self.drone_publishers:
            msg = Float64()
            msg.data = altitude
            self.drone_publishers[namespace]['goto_altitude'].publish(msg)
            self.get_logger().info(f"Goto altitude command sent to {namespace}: {altitude}m")
    
    def send_set_rth_altitude(self, namespace: str, altitude: float):
        """Set the RTH altitude for a drone."""
        if namespace in self.drone_publishers:
            msg = Float64()
            msg.data = altitude
            self.drone_publishers[namespace]['set_rth_altitude'].publish(msg)
            self.get_logger().info(f"RTH altitude set for {namespace}: {altitude}m")
    
    def send_start_recording(self, namespace: str):
        """Start video recording on a drone."""
        if namespace in self.drone_publishers:
            self.drone_publishers[namespace]['camera/start_recording'].publish(Empty())
            self.get_logger().info(f"Start recording command sent to {namespace}")
    
    def send_stop_recording(self, namespace: str):
        """Stop video recording on a drone."""
        if namespace in self.drone_publishers:
            self.drone_publishers[namespace]['camera/stop_recording'].publish(Empty())
            self.get_logger().info(f"Stop recording command sent to {namespace}")
    
    def send_gimbal_pitch(self, namespace: str, pitch: float):
        """Set gimbal pitch angle."""
        if namespace in self.drone_publishers:
            msg = Float64()
            msg.data = pitch
            self.drone_publishers[namespace]['gimbal_pitch'].publish(msg)
    
    def send_gimbal_yaw(self, namespace: str, yaw: float):
        """Set gimbal yaw angle."""
        if namespace in self.drone_publishers:
            msg = Float64()
            msg.data = yaw
            self.drone_publishers[namespace]['gimbal_yaw'].publish(msg)
    
    def send_zoom_ratio(self, namespace: str, zoom: float):
        """Set camera zoom ratio (1.0 to 2.0)."""
        if namespace in self.drone_publishers:
            # Clamp zoom to valid range
            zoom = max(1.0, min(2.0, zoom))
            msg = Float64()
            msg.data = zoom
            self.drone_publishers[namespace]['zoom_ratio'].publish(msg)
            self.get_logger().info(f"Zoom ratio command sent to {namespace}: {zoom}x")
    
    def send_goto_yaw(self, namespace: str, yaw: float):
        """Command drone to rotate to a specific heading/yaw."""
        if namespace in self.drone_publishers:
            msg = Float64()
            msg.data = yaw
            self.drone_publishers[namespace]['goto_yaw'].publish(msg)
            self.get_logger().info(f"Goto yaw command sent to {namespace}: {yaw}°")
    
    def send_trajectory(self, namespace: str, waypoints: List[Tuple[float, float, float]], final_yaw: float = 0.0):
        """
        Send a trajectory (list of waypoints) to a drone.
        
        Args:
            namespace: Drone namespace
            waypoints: List of (lat, lon, alt) tuples
            final_yaw: Final heading at last waypoint
        """
        if namespace in self.drone_publishers:
            msg = String()
            msg.data = str((waypoints, final_yaw))
            self.drone_publishers[namespace]['goto_trajectory'].publish(msg)
            self.drones[namespace].state = DroneState.FLYING_TO_POINT
            self.get_logger().info(f"Trajectory sent to {namespace}: {len(waypoints)} waypoints")
    
    def send_trajectory_dji_native(self, namespace: str, waypoints: List[Tuple[float, float, float]]):
        """
        Send a trajectory using DJI's native waypoint mission system.
        """
        if namespace in self.drone_publishers:
            msg = String()
            msg.data = str(waypoints)
            self.drone_publishers[namespace]['goto_trajectory_dji_native'].publish(msg)
            self.drones[namespace].state = DroneState.FLYING_TO_POINT
            self.get_logger().info(f"DJI Native trajectory sent to {namespace}")
    
    # ========================================================================
    # MONITORING POINT MANAGEMENT
    # ========================================================================
    
    def set_monitoring_point(self, lat: float, lon: float, alt: float = 50.0, heading: float = 0.0, source: str = "manual"):
        """Set the monitoring GPS point and target heading."""
        self.monitoring_point = MonitoringPoint(
            latitude=lat,
            longitude=lon,
            altitude=alt,
            heading=heading,
            is_set=True,
            source=source
        )
        self.mission.monitoring_point = self.monitoring_point
        
        # Update mission controller config for distance calculations
        if hasattr(self, 'mission_controller'):
            self.mission_controller.config.monitoring_lat = lat
            self.mission_controller.config.monitoring_lon = lon
            self.mission_controller.config.monitoring_alt = alt
            self.mission_controller.config.monitoring_heading = heading
        
        self.get_logger().info(f"Monitoring point set: ({lat}, {lon}, {alt}) heading={heading}° from {source}")
        
        if self.ui_handler:
            self.ui_handler.update_monitoring_point(lat, lon, alt)
    
    def set_monitoring_point_from_drone(self, namespace: str):
        """Set monitoring point to current position of a flying drone."""
        if namespace in self.drones:
            drone = self.drones[namespace]
            self.set_monitoring_point(
                drone.latitude, 
                drone.longitude, 
                drone.altitude,
                source=f"drone:{namespace}"
            )
    
    def clear_monitoring_point(self):
        """Clear the current monitoring point."""
        self.monitoring_point = MonitoringPoint()
        self.mission.monitoring_point = self.monitoring_point
        
        if self.ui_handler:
            self.ui_handler.clear_monitoring_point()
    
    # ========================================================================
    # MISSION CONTROL
    # ========================================================================
    
    def start_monitoring_mission(self, namespace: str, rth_altitude: float = 50.0):
        """
        Start a monitoring mission for a single drone.
        
        Uses the MissionController for state machine management:
        1. Preflight check
        2. Set RTH altitude
        3. Take off
        4. Climb to RTH altitude
        5. Fly to monitoring point
        6. Start recording 50m before arrival
        """
        # In FREE_FLIGHT mode, monitoring point is not required
        is_free_flight = self.mission_controller.mission_mode == MissionMode.FREE_FLIGHT
        
        if not is_free_flight and not self.monitoring_point.is_set:
            self.get_logger().error("Cannot start mission: monitoring point not set")
            return False
        
        if namespace not in self.drones:
            self.get_logger().error(f"Drone {namespace} not found")
            return False
        
        # Get monitoring point coordinates (use 0,0,0 for FREE_FLIGHT mode)
        mon_lat = self.monitoring_point.latitude if self.monitoring_point.is_set else 0.0
        mon_lon = self.monitoring_point.longitude if self.monitoring_point.is_set else 0.0
        mon_alt = self.monitoring_point.altitude if self.monitoring_point.is_set else rth_altitude
        
        # Use mission controller for proper state machine management
        success = self.mission_controller.start_single_mission(
            namespace,
            mon_lat,
            mon_lon,
            mon_alt,
            rth_altitude
        )
        
        if success:
            # Update local mission tracking
            self.mission.rth_altitude = rth_altitude
            self.mission.active_drone = namespace
            self.mission.is_active = True
            
            if namespace not in self.mission.drones_in_mission:
                self.mission.drones_in_mission.append(namespace)
            
            self.get_logger().info(f"Monitoring mission started for {namespace}")
        
        return success
    
    def start_relay_mission(self, drone_list: List[str], rth_altitude: float = 50.0):
        """
        Start a perpetual monitoring mission with relay drones.
        
        Args:
            drone_list: Ordered list of drones to use in relay
            rth_altitude: Base RTH altitude (each subsequent drone +15m)
        """
        # In FREE_FLIGHT mode, monitoring point is not required
        is_free_flight = self.mission_controller.mission_mode == MissionMode.FREE_FLIGHT
        
        if not is_free_flight and not self.monitoring_point.is_set:
            self.get_logger().error("Cannot start relay: monitoring point not set")
            return False
        
        if len(drone_list) < 1:
            self.get_logger().error("Need at least 1 drone for relay mission")
            return False
        
        if len(drone_list) == 1:
            self.get_logger().warning("Starting relay with 1 drone - more drones should connect before battery runs low")
        
        # Validate all drones exist
        for ns in drone_list:
            if ns not in self.drones:
                self.get_logger().error(f"Drone {ns} not found")
                return False
        
        # Get monitoring point coordinates (use 0,0,0 for FREE_FLIGHT mode)
        mon_lat = self.monitoring_point.latitude if self.monitoring_point.is_set else 0.0
        mon_lon = self.monitoring_point.longitude if self.monitoring_point.is_set else 0.0
        mon_alt = self.monitoring_point.altitude if self.monitoring_point.is_set else rth_altitude
        
        # Use mission controller for relay
        success = self.mission_controller.start_relay_mission(
            drone_list,
            mon_lat,
            mon_lon,
            mon_alt,
            rth_altitude
        )
        
        if success:
            # Update local mission tracking
            self.mission.is_active = True
            self.mission.rth_altitude = rth_altitude
            self.mission.drones_in_mission = drone_list.copy()
            self.mission.active_drone = drone_list[0]
            self.mission.next_drone = drone_list[1] if len(drone_list) > 1 else ""
            
            self.get_logger().info(f"Relay mission started with {len(drone_list)} drones")
        
        return success
    
    def stop_mission(self):
        """Stop the current mission and recall all drones."""
        # Use mission controller to stop
        self.mission_controller.stop_mission()
        
        self.mission.is_active = False
        self.mission.active_drone = ""
        self.mission.next_drone = ""
        
        self.get_logger().info("Mission stopped, all drones returning home")
    
    # ========================================================================
    # MISSION STATE MACHINE
    # ========================================================================
    
    def _update_mission_state(self):
        """Update mission state for all drones (called by timer)."""
        for namespace, drone in self.drones.items():
            if drone.mission_phase == MissionPhase.NONE:
                continue
            
            # Check for altitude reached during climb phase
            if drone.mission_phase == MissionPhase.CLIMB_TO_RTH_ALTITUDE:
                if drone.altitude >= (self.mission.rth_altitude - 2.0):  # 2m tolerance
                    self._transition_to_transit(namespace)
            
            # Check for approach to monitoring point
            elif drone.mission_phase == MissionPhase.TRANSIT_TO_MONITORING:
                distance_to_point = self.calculator.haversine_distance(
                    drone.latitude, drone.longitude,
                    self.monitoring_point.latitude, self.monitoring_point.longitude
                )
                
                # Trigger video recording 50m before
                if distance_to_point <= self.mission.video_trigger_distance and not drone.is_recording:
                    self.send_start_recording(namespace)
                    self.get_logger().info(f"Auto-start recording for {namespace} at {distance_to_point:.1f}m from target")
                
                # Check if reached
                if drone.waypoint_reached:
                    self._transition_to_monitoring(namespace)
    
    def _transition_to_transit(self, namespace: str):
        """Transition drone to transit phase."""
        drone = self.drones[namespace]
        drone.mission_phase = MissionPhase.TRANSIT_TO_MONITORING
        
        # Calculate bearing to monitoring point
        bearing = self.calculator.calculate_bearing(
            drone.latitude, drone.longitude,
            self.monitoring_point.latitude, self.monitoring_point.longitude
        )
        
        # Send waypoint command
        self.send_goto_waypoint(
            namespace,
            self.monitoring_point.latitude,
            self.monitoring_point.longitude,
            self.monitoring_point.altitude,
            self.monitoring_point.heading,
            self.PID_SPEED
        )
        
        # Record transit start time for travel time estimation
        drone._transit_start_time = time.time()
        
        self.get_logger().info(f"{namespace} transitioning to monitoring point")
    
    def _transition_to_monitoring(self, namespace: str):
        """Transition drone to monitoring phase."""
        drone = self.drones[namespace]
        drone.mission_phase = MissionPhase.MONITORING
        drone.state = DroneState.MONITORING
        
        # Calculate actual travel time
        if hasattr(drone, '_transit_start_time'):
            travel_time = time.time() - drone._transit_start_time
            self.mission.actual_travel_times[namespace] = travel_time
            self.mission.estimated_travel_time = travel_time  # Update estimate
            self.get_logger().info(f"{namespace} travel time: {travel_time:.1f}s")
        
        self.get_logger().info(f"{namespace} now monitoring")
        
        if self.ui_handler:
            self.ui_handler.update_drone_state(namespace, DroneState.MONITORING)
    
    def _handle_waypoint_reached(self, namespace: str):
        """Handle waypoint reached event."""
        if namespace in self.drones:
            drone = self.drones[namespace]
            if drone.mission_phase == MissionPhase.TRANSIT_TO_MONITORING:
                self._transition_to_monitoring(namespace)
    
    def _update_relay_logic(self):
        """Update relay countdown and launch next drone if needed."""
        if not self.mission.is_active or not self.mission.active_drone:
            return
        
        active_ns = self.mission.active_drone
        if active_ns not in self.drones:
            return
        
        active_drone = self.drones[active_ns]
        
        # Only calculate relay when drone is monitoring
        if active_drone.mission_phase != MissionPhase.MONITORING:
            return
        
        # Calculate time for next drone to travel to monitoring point
        if self.mission.next_drone and self.mission.next_drone in self.drones:
            next_drone = self.drones[self.mission.next_drone]
            
            # Distance from next drone's home to monitoring point
            distance = self.calculator.haversine_distance(
                next_drone.home_latitude, next_drone.home_longitude,
                self.monitoring_point.latitude, self.monitoring_point.longitude
            )
            
            # Use actual travel time if available, otherwise estimate
            if self.mission.actual_travel_times:
                avg_travel = sum(self.mission.actual_travel_times.values()) / len(self.mission.actual_travel_times)
            else:
                avg_travel = self.calculator.estimate_travel_time(distance)
            
            # Calculate countdown: when should next drone launch?
            # Next drone should arrive when current has just enough time to RTH
            countdown = active_drone.remaining_flight_time - avg_travel - self.mission.safety_buffer
            
            self.mission.relay_countdown = max(0.0, countdown)
            
            # Notify UI
            if self.ui_handler:
                self.ui_handler.update_relay_countdown(countdown, self.mission.next_drone)
            
            # Auto-launch next drone when countdown hits zero
            if countdown <= 0 and next_drone.state == DroneState.CONNECTED:
                self._launch_relay_drone(self.mission.next_drone)
    
    def _launch_relay_drone(self, namespace: str):
        """Launch the next relay drone."""
        if namespace not in self.drones:
            return
        
        # Calculate altitude (15m above current active drone)
        current_idx = self.mission.drones_in_mission.index(self.mission.active_drone)
        next_idx = self.mission.drones_in_mission.index(namespace)
        altitude = self.mission.rth_altitude + (next_idx * self.mission.altitude_separation)
        
        self.get_logger().info(f"Launching relay drone {namespace} at {altitude}m altitude")
        
        # Start mission for next drone
        self.start_monitoring_mission(namespace, altitude)
        
        # Update relay chain
        old_active = self.mission.active_drone
        self.mission.active_drone = namespace
        
        # Determine next drone in sequence
        if next_idx + 1 < len(self.mission.drones_in_mission):
            self.mission.next_drone = self.mission.drones_in_mission[next_idx + 1]
        else:
            # Loop back to first drone (if it's available)
            first_drone = self.mission.drones_in_mission[0]
            if first_drone in self.drones and self.drones[first_drone].state == DroneState.CONNECTED:
                self.mission.next_drone = first_drone
            else:
                self.mission.next_drone = ""
        
        # Command old active drone to return home
        self.drones[old_active].mission_phase = MissionPhase.RETURN_HOME
        self.send_stop_recording(old_active)
        self.send_rth(old_active)


# ============================================================================
# GLOBAL NODE REFERENCE
# ============================================================================

ros_node: PerpetualMonitorNode = None


def ros_main():
    """ROS2 spinning loop - runs in background thread."""
    global ros_node
    rclpy.init()
    ros_node = PerpetualMonitorNode.get_instance()
    rclpy.spin(ros_node)
    ros_node.destroy_node()
    rclpy.shutdown()
