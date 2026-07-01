"""
Perpetual Drone Monitoring Groundstation

A ROS2-based groundstation for autonomous, perpetual monitoring of a single GPS point
using multiple drones with dynamic relay missions.

Author: Edouard Rolland
Project: WildDrone
"""

import time
import subprocess
import signal
import os
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool, Empty, String, Float64MultiArray, Int32
from sensor_msgs.msg import NavSatFix

from nicegui import ui, app, ui_run

from groundstation import navigation
from groundstation.models import (
    DroneState, MissionPhase, DroneData, MonitoringPoint, RelayMission,
)
from groundstation.mission_controller import MissionController, MissionState, MissionMode
from groundstation.rth_prediction import DroneRTHPredictor
from dji_controller.submodules.dji_interface import discover_drone


# ============================================================================
# MISSION CALCULATOR
# ============================================================================

class MissionCalculator:
    """Mission parameter and relay-timing calculations.

    Thin wrapper over the pure :mod:`groundstation.navigation` functions, kept
    as a class for the existing ``self.calculator.*`` call sites.
    """

    EARTH_RADIUS = navigation.EARTH_RADIUS
    VERTICAL_SPEED = navigation.VERTICAL_SPEED
    HORIZONTAL_SPEED_PID = navigation.HORIZONTAL_SPEED_PID
    HORIZONTAL_SPEED_NATIVE = navigation.HORIZONTAL_SPEED_NATIVE

    haversine_distance = staticmethod(navigation.haversine_distance)
    calculate_bearing = staticmethod(navigation.calculate_bearing)
    estimate_travel_time = staticmethod(navigation.estimate_travel_time)
    calculate_relay_countdown = staticmethod(navigation.calculate_relay_countdown)
    calculate_drones_needed = staticmethod(navigation.calculate_drones_needed)


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
            namespace: ROS namespace for the drone (auto-generated if not provided).
                       When auto-discovering (no IP), the discovered drone name is used if available.
        
        Returns:
            The namespace string if connection successful (truthy), None otherwise (falsy)
        """
        # If no IP provided, run discovery from groundstation to get both IP and drone name
        discovered_name = None
        if not ip_address:
            self.get_logger().info("No IP provided, running network discovery...")
            disc_ip, disc_name = discover_drone(timeout=5.0)
            if disc_ip:
                ip_address = disc_ip
                discovered_name = disc_name
                self.get_logger().info(
                    f"Discovered drone at {disc_ip}" +
                    (f" (name: {disc_name})" if disc_name else ""))
            else:
                self.get_logger().error("No drone found on network during discovery")
                return None
        
        # Determine namespace: prefer user-provided, then discovered name, then auto-generate
        if namespace is None:
            if discovered_name:
                # Sanitize drone name for ROS namespace (lowercase, replace spaces/special chars)
                namespace = discovered_name.lower().replace(' ', '_').replace('-', '_')
                # Remove any characters not valid in ROS names
                namespace = ''.join(c for c in namespace if c.isalnum() or c == '_')
                self.get_logger().info(f"Using discovered drone name as namespace: {namespace}")
            else:
                namespace = f"drone_{len(self.drones) + 1}"
        
        # Allow reconnection of a previously disconnected drone while preserving its color/identity
        existing_drone = self.drones.get(namespace)
        if existing_drone and existing_drone.is_connected:
            self.get_logger().warning(f"Drone {namespace} already connected")
            return None
        
        self.get_logger().info(f"Connecting drone {namespace} at {ip_address}")
        
        # Launch dji_controller node for this drone as a subprocess
        try:
            # Build the ros2 run command with namespace and IP parameter
            cmd = [
                'ros2', 'run', 'dji_controller', 'dji_node',
                '--ros-args',
                '-r', f'__ns:=/{namespace}',
            ]
            if ip_address:
                cmd.extend(['-p', f'ip_rc:={ip_address}'])
            
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
                return None
                
        except Exception as e:
            self.get_logger().error(f"Failed to launch controller node: {e}")
            return None
        
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
        return namespace
    
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
