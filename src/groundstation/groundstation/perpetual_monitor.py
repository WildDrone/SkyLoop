"""
Perpetual Drone Monitoring Groundstation

A ROS2-based groundstation for autonomous, perpetual monitoring of a single GPS point
using multiple drones with dynamic relay missions.

Author: Edouard Rolland
Project: WildDrone
"""

import threading
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
import ast

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool, Empty, String, Float64MultiArray
from sensor_msgs.msg import NavSatFix

from nicegui import ui, app, ui_run

from groundstation.mission_controller import MissionController, MissionState


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
    HORIZONTAL_SPEED_PID = 5.0      # m/s - PID control mode
    HORIZONTAL_SPEED_NATIVE = 10.0  # m/s - DJI native trajectory mode
    
    @staticmethod
    def estimate_travel_time(distance: float, altitude: float = 50.0,
                             horizontal_speed: float = 5.0, vertical_speed: float = 4.0) -> float:
        """
        Estimate travel time in seconds, including climb time.
        
        Args:
            distance: Horizontal distance in meters
            altitude: Target altitude in meters (default 50m)
            horizontal_speed: Horizontal flight speed in m/s 
                              (5 m/s for PID mode, 10 m/s for DJI native)
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
        mc.cmd_goto_altitude = self.send_goto_altitude
        mc.cmd_set_rth_altitude = self.send_set_rth_altitude
        mc.cmd_start_recording = self.send_start_recording
        mc.cmd_stop_recording = self.send_stop_recording
        mc.cmd_set_gimbal_pitch = self.send_gimbal_pitch
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
        mc.get_remaining_flight_time = lambda ns: self.drones[ns].remaining_flight_time if ns in self.drones else 0.0
        mc.get_battery_level = lambda ns: self.drones[ns].battery_level if ns in self.drones else 0.0
        mc.get_satellite_count = lambda ns: self.drones[ns].satellite_count if ns in self.drones else 0
        mc.get_is_recording = lambda ns: self.drones[ns].is_recording if ns in self.drones else False
        mc.get_waypoint_reached = lambda ns: self.drones[ns].waypoint_reached if ns in self.drones else False
        mc.get_altitude_reached = lambda ns: self.drones[ns].altitude_reached if ns in self.drones else False
        
        # Status callbacks
        mc.on_status_update = self._on_mission_status_update
        mc.on_relay_countdown = self._on_relay_countdown_update
        mc.on_mission_event = self._on_mission_event
    
    def _on_mission_status_update(self, namespace: str, state: MissionState, message: str):
        """Handle mission status updates from controller."""
        if namespace in self.drones:
            # Map MissionState to DroneState
            state_map = {
                MissionState.TAKING_OFF: DroneState.TAKING_OFF,
                MissionState.CLIMBING_TO_ALTITUDE: DroneState.TAKING_OFF,
                MissionState.TRANSIT_TO_MONITORING: DroneState.FLYING_TO_POINT,
                MissionState.APPROACHING_POINT: DroneState.FLYING_TO_POINT,
                MissionState.MONITORING: DroneState.MONITORING,
                MissionState.RETURNING_HOME: DroneState.RETURNING_HOME,
                MissionState.LANDING: DroneState.LANDING,
            }
            
            if state in state_map:
                self.drones[namespace].state = state_map[state]
                self.drones[namespace].current_task = message
                
                if self.ui_handler:
                    self.ui_handler.update_drone_state(namespace, state_map[state])
    
    def _on_relay_countdown_update(self, countdown: float, next_drone: str):
        """Handle relay countdown updates."""
        self.mission.relay_countdown = countdown
        self.mission.next_drone = next_drone
        
        if self.ui_handler:
            self.ui_handler.update_relay_countdown(countdown, next_drone)
    
    def _on_mission_event(self, namespace: str, event: str):
        """Handle mission events for logging."""
        self.get_logger().info(f"[{namespace}] {event}")
    
    # ========================================================================
    # DRONE CONNECTION MANAGEMENT
    # ========================================================================
    
    def connect_drone(self, ip_address: str, namespace: str = None) -> bool:
        """
        Dynamically connect a new drone to the system.
        
        Args:
            ip_address: IP address of the drone's RC controller
            namespace: ROS namespace for the drone (auto-generated if not provided)
        
        Returns:
            True if connection successful, False otherwise
        """
        if namespace is None:
            namespace = f"drone_{len(self.drones) + 1}"
        
        if namespace in self.drones:
            self.get_logger().warning(f"Drone {namespace} already connected")
            return False
        
        self.get_logger().info(f"Connecting drone {namespace} at {ip_address}")
        
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
        
        # Create drone data entry
        drone = DroneData(
            ip_address=ip_address,
            namespace=namespace,
            is_connected=True,
            connection_time=time.time(),
            state=DroneState.CONNECTED
        )
        self.drones[namespace] = drone
        
        # Set up ROS subscribers for this drone's telemetry
        self._setup_drone_subscribers(namespace)
        
        # Set up ROS publishers for commands to this drone
        self._setup_drone_publishers(namespace)
        
        # If this drone is part of an active relay mission, reset its state for reuse
        if self.mission_controller.reset_drone_for_reuse(namespace):
            self.get_logger().info(f"Drone {namespace} reset for relay reuse")
        
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
        
        # Remove drone data
        del self.drones[namespace]
        
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
            Float64, f"{namespace}/satellite_count",
            lambda msg, ns=namespace: self._on_satellite_count(ns, int(msg.data)), 10
        )
        
        # Gimbal pitch
        self.drone_subscribers[namespace]['gimbal_pitch'] = self.create_subscription(
            Float64, f"{namespace}/gimbal_pitch",
            lambda msg, ns=namespace: self._on_gimbal_pitch(ns, msg.data), 10
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
        
        # RTH altitude
        self.drone_publishers[namespace]['set_rth_altitude'] = self.create_publisher(
            Float64, f"{namespace}/command/set_rth_altitude", 10
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
            self.drones[namespace].battery_level = level
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
    
    def send_goto_waypoint(self, namespace: str, lat: float, lon: float, alt: float, yaw: float = 0.0):
        """Command a drone to go to a waypoint."""
        if namespace in self.drone_publishers:
            msg = Float64MultiArray()
            msg.data = [lat, lon, alt, yaw]
            self.drone_publishers[namespace]['goto_waypoint'].publish(msg)
            self.drones[namespace].state = DroneState.FLYING_TO_POINT
            self.get_logger().info(f"Goto waypoint command sent to {namespace}: ({lat}, {lon}, {alt})")
    
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
    
    def set_monitoring_point(self, lat: float, lon: float, alt: float = 50.0, source: str = "manual"):
        """Set the monitoring GPS point."""
        self.monitoring_point = MonitoringPoint(
            latitude=lat,
            longitude=lon,
            altitude=alt,
            is_set=True,
            source=source
        )
        self.mission.monitoring_point = self.monitoring_point
        
        self.get_logger().info(f"Monitoring point set: ({lat}, {lon}, {alt}) from {source}")
        
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
        if not self.monitoring_point.is_set:
            self.get_logger().error("Cannot start mission: monitoring point not set")
            return False
        
        if namespace not in self.drones:
            self.get_logger().error(f"Drone {namespace} not found")
            return False
        
        # Use mission controller for proper state machine management
        success = self.mission_controller.start_single_mission(
            namespace,
            self.monitoring_point.latitude,
            self.monitoring_point.longitude,
            self.monitoring_point.altitude,
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
        if not self.monitoring_point.is_set:
            self.get_logger().error("Cannot start relay: monitoring point not set")
            return False
        
        if len(drone_list) < 2:
            self.get_logger().error("Need at least 2 drones for relay mission")
            return False
        
        # Validate all drones exist
        for ns in drone_list:
            if ns not in self.drones:
                self.get_logger().error(f"Drone {ns} not found")
                return False
        
        # Use mission controller for relay
        success = self.mission_controller.start_relay_mission(
            drone_list,
            self.monitoring_point.latitude,
            self.monitoring_point.longitude,
            self.monitoring_point.altitude,
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
            bearing
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
