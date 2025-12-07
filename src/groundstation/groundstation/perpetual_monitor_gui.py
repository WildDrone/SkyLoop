"""
Perpetual Monitoring GUI

NiceGUI-based web interface for the perpetual drone monitoring system.
Uses NiceGUI Events for thread-safe communication between ROS2 callbacks and UI.

Author: Edouard Rolland
Project: WildDrone
"""

import asyncio
import math
import threading
from typing import Dict
from pathlib import Path
from datetime import datetime

import rclpy
import time
from rclpy.executors import ExternalShutdownException

from nicegui import Event, app, ui, ui_run

from groundstation.perpetual_monitor import (
    PerpetualMonitorNode, DroneData, DroneState, MissionPhase,
    MonitoringPoint, RelayMission, DroneRTHPredictor
)
from groundstation.mission_controller import MissionController, MissionState


# ============================================================================
# STATIC FILES
# ============================================================================

app.add_static_files('/static', str(Path(__file__).parent / 'static'))


# ============================================================================
# ARROW DISPLAY
# ============================================================================

class Arrow:
    """Arrow marker for drone position display on map."""
    
    def __init__(self, map_ui, id: str, lat: float, lng: float, heading: float, drones_arrows: dict, color: str = '#FF6B6B'):
        """
        Initialize an arrow on the given map.

        :param map_ui: The NiceGUI Leaflet map instance.
        :param id: Unique identifier for the arrow (namespace).
        :param lat: Latitude of the arrow's initial position.
        :param lng: Longitude of the arrow's initial position.
        :param heading: Initial heading of the arrow (in degrees).
        :param drones_arrows: Dict to check for duplicate arrows.
        :param color: Primary color for the arrow (hex format).
        """
        self.map_ui = map_ui
        self.id = id
        self.lat = lat
        self.lng = lng
        self.heading = heading
        self.color = color
        # Generate darker shade for 3D effect
        self.dark_color = self._darken_color(color)

        if id in drones_arrows:
            raise ValueError(f"Arrow with id '{id}' already exists.")
    
    def _darken_color(self, hex_color: str) -> str:
        """Generate a darker shade of the given hex color."""
        # Remove # if present
        hex_color = hex_color.lstrip('#')
        # Convert to RGB
        r = int(hex_color[0:2], 16)
        g = int(hex_color[2:4], 16)
        b = int(hex_color[4:6], 16)
        # Darken by 30%
        factor = 0.7
        r = int(r * factor)
        g = int(g * factor)
        b = int(b * factor)
        return f'#{r:02x}{g:02x}{b:02x}'

    def _place_arrow(self):
        """Place the arrow on the map."""
        ui.run_javascript(
            f"place_arrow({self.map_ui.id}, {self.lat}, {self.lng}, {self.heading}, '{self.id}', '{self.color}', '{self.dark_color}')"
        )

    def update(self, lat: float, lng: float, heading: float):
        """
        Update the position and heading of the arrow.

        :param lat: New latitude.
        :param lng: New longitude.
        :param heading: New heading (in degrees).
        """
        self.lat = lat
        self.lng = lng
        self.heading = heading
        ui.run_javascript(
            f"update_arrow_test('{self.id}', {self.lat}, {self.lng}, {self.heading})"
        )

    def destroy(self):
        """Remove the arrow from the map."""
        ui.run_javascript(f"delete_arrow('{self.id}')")


# ============================================================================
# GUI NODE WITH NICEGUI EVENTS
# ============================================================================

class PerpetualMonitorGUI(PerpetualMonitorNode):
    """
    Extended ROS2 Node with NiceGUI Event-based UI updates.
    
    Uses NiceGUI Events to safely communicate between ROS callbacks and UI thread.
    This follows the recommended NiceGUI + ROS2 integration pattern.
    """
    
    _instance = None
    
    @staticmethod
    def get_instance():
        if PerpetualMonitorGUI._instance is None:
            PerpetualMonitorGUI._instance = PerpetualMonitorGUI()
        return PerpetualMonitorGUI._instance
    
    def __init__(self):
        super().__init__()
        
        # NiceGUI Events for thread-safe UI updates
        self.drone_position_update = Event()
        self.drone_heading_update = Event()
        self.drone_battery_update = Event()
        self.drone_flight_time_update = Event()
        self.drone_state_update = Event()
        self.drone_recording_update = Event()
        self.drone_satellite_update = Event()
        self.drone_speed_update = Event()
        self.drone_flight_mode_update = Event()  # Flight mode (virtual_stick, manual, etc.)
        self.drone_connected_event = Event()
        self.drone_disconnected_event = Event()
        self.monitoring_point_update = Event()
        self.relay_countdown_update = Event()
        self.rth_predictor_update = Event()  # RTH predictor info
        self.log_event = Event()
        
        # UI element references (populated when page loads)
        self.map = None
        self.drone_cards: Dict[str, ui.card] = {}
        self.drone_arrows: Dict[str, Arrow] = {}
        self.drone_labels: Dict[str, Dict[str, ui.label]] = {}
        self.drone_buttons: Dict[str, Dict[str, ui.button]] = {}
        self.drone_list_container = None
        
        # Event log message queue (for thread-safe logging)
        self.log_message_queue: list = []
        self._should_start_timer = False  # Flag for starting mission timer from UI thread
        
        # Vertical separation alert queue (for thread-safe UI updates)
        self._vertical_separation_alerts: list = []
        
        # Notification queue (for thread-safe UI notifications from background threads)
        self._notification_queue: list = []
        
        # Takeoff confirmation queue (for thread-safe dialog from background threads)
        self._takeoff_confirmation_queue: list = []
        self._takeoff_confirmation_dialog_open = False
        
        # Monitoring point marker
        self.monitoring_marker = None
        self.monitoring_circle = None
        
        # Mission display elements
        self.mission_status_label = None
        self.countdown_label = None
        self.countdown_progress = None
        self.countdown_container = Nonex  # Container for countdown (hidden during manual swap)
        self.force_swap_button = None  # Button to trigger manual swap
        self.active_drone_label = None
        self.next_drone_label = None
        self.drones_needed_label = None
        self.relay_alert_label = None
        self.relay_alert_icon = None
        self.relay_alert_container = None
        self.reconnect_label = None
        self.mission_timer_label = None
        self._mission_start_time = None
        self._mission_timer_task = None
        self._manual_swap_active = False  # True when manual swap in progress
        
        # ROS bag recording
        self._rosbag_process = None  # subprocess.Popen for ros2 bag record
        self._rosbag_recording = False
        self._rosbag_dir = "/WildPerpetua/src/rosbags"  # Mounted to host's src/rosbags/
        
        # Debug console (ROS logs)
        self.debug_mode = False
        self.debug_toggle = None
        self.debug_console = None
        self.debug_console_container = None
        self.debug_scroll = None
        self.normal_logs_container = None
        self.mission_stats_card = None  # Card to hide when debug console is shown
        
        # Event log
        self.event_log = None
        
        # Track which drones we've centered on (to center on first position)
        self._centered_on_drone: set = set()
        self.event_scroll = None
        
        # Mission statistics tracking
        self.mission_stats_container = None
        self.mission_stats_scroll = None
        self.mission_stats_history: list = []  # List of {drone, iteration, est_travel, actual_travel, actual_rth}
        self.drone_iteration_counter: Dict[str, int] = {}  # Track iteration per drone
        
        # RTH landing detection
        self.drone_rth_tracking: Dict[str, dict] = {}  # {ns: {start_time, last_alt, stable_count, detected}}
        
        # Battery swap detection (track last battery level per drone)
        self.drone_last_battery: Dict[str, float] = {}
        
        # Trajectory tracking for flight paths
        self.drone_trajectories: Dict[str, list] = {}  # {ns: [(lat, lon), ...]}
        self.drone_trajectory_lines: Dict[str, object] = {}  # {ns: leaflet polyline layer}
        self.drone_is_flying: Dict[str, bool] = {}  # {ns: True if currently flying}
        
        # State machine display elements
        self.state_machine_container = None
        self.state_machine_labels: Dict[str, Dict[str, ui.label]] = {}
        
        # Connection form elements
        self.ip_input = None
        self.namespace_input = None
        self.lat_input = None
        self.lon_input = None
        self.alt_input = None
        self.heading_input = None
        self.rth_alt_input = None
        self.safety_buffer_input = None
        self.min_battery_input = None
        self.min_satellites_input = None
        self.trajectory_mode = None
        self.trajectory_speed_slider = None
        self.trajectory_speed_label = None
        
        # Map settings
        self.map_center = (49.306260, 4.593715)  # Default center
        
        # Drone colors for visualization
        self.drone_colors = ['#FF6B6B', '#4ECDC4', '#45B7D1', '#96CEB4', '#FFEAA7', '#DDA0DD', '#98D8C8', '#F7DC6F']
        
        # Pending takeoff drone name (for dialog re-show on cancel->cancel)
        self._pending_takeoff_drone = ""
        self._pending_single_mission = False
        self._pending_relay_mission = False
        self._pending_relay_data = {}
        
        # Vertical separation countdown state
        self._vertical_separation_countdown_active = False
        
        # Register takeoff confirmation callback
        self.mission_controller.on_takeoff_confirmation_request = self._on_takeoff_confirmation_request
        
        # Register vertical separation alert callback
        self.mission_controller.on_vertical_separation_alert = self._on_vertical_separation_alert
        
        # Register vertical separation countdown callbacks
        self.mission_controller.on_vertical_separation_countdown_start = self._on_vertical_separation_countdown_start
        self.mission_controller.on_vertical_separation_countdown_cancel = self._on_vertical_separation_countdown_cancel
        self.mission_controller.on_vertical_separation_mission_stopped = self._on_vertical_separation_mission_stopped
        
        # Register drone flight mode callback for manual flight detection
        self.mission_controller.get_drone_flight_mode = self._get_drone_flight_mode
        
        # Register drone altitude callback for separation check  
        self.mission_controller.get_drone_altitude = self._get_drone_altitude
        
        # Define the main page
        @ui.page('/')
        def main_page():
            self._build_ui()
    
    # ========================================================================
    # OVERRIDE PARENT CALLBACKS TO EMIT EVENTS
    # ========================================================================
    
    def _on_location(self, namespace: str, msg):
        """Override location callback to emit event."""
        super()._on_location(namespace, msg)
        self.drone_position_update.emit({
            'namespace': namespace,
            'lat': msg.latitude,
            'lon': msg.longitude,
            'alt': msg.altitude
        })
        
        # Check for RTH landing detection
        self._check_rth_landing(namespace, msg.altitude)
    
    def _on_heading(self, namespace: str, heading: float):
        """Override heading callback to emit event."""
        if namespace in self.drones:
            self.drones[namespace].heading = heading
        self.drone_heading_update.emit({
            'namespace': namespace,
            'heading': heading
        })
    
    def _on_battery(self, namespace: str, level: float):
        """Override battery callback to emit event."""
        # IMPORTANT: Call parent to update drone data AND RTH predictor
        super()._on_battery(namespace, level)
        
        # Detect battery swap: if battery level increases significantly, drone got a new battery
        # Reset to IDLE so it can be used for next relay
        if namespace in self.drone_last_battery:
            last_level = self.drone_last_battery[namespace]
            battery_increase = level - last_level
            
            # If battery increased by more than 10%, assume battery was swapped
            if battery_increase > 10:
                self._emit_log(f"[{namespace}] Battery swap detected ({last_level:.0f}% → {level:.0f}%)")
                
                # Delegate state change to mission controller
                if self.mission_controller and self.mission_controller.notify_battery_swap(namespace):
                    # Reset RTH predictor for fresh data with new battery
                    if namespace in self.rth_predictors:
                        self.rth_predictors[namespace] = DroneRTHPredictor(namespace=namespace)
                    
                    # Emit state update to refresh GUI
                    self.drone_state_update.emit({
                        'namespace': namespace,
                        'state': DroneState.IDLE
                    })
        
        self.drone_last_battery[namespace] = level
        
        self.drone_battery_update.emit({
            'namespace': namespace,
            'level': level
        })
        
        # Also emit RTH predictor update if available
        if namespace in self.rth_predictors:
            try:
                predictor = self.rth_predictors[namespace]
                debug_info = predictor.get_debug_info()
                
                # Add DJI's remaining_flight_time for comparison
                if namespace in self.drones:
                    debug_info['dji_remaining_flight_time'] = self.drones[namespace].remaining_flight_time
                
                self.rth_predictor_update.emit({
                    'namespace': namespace,
                    'debug_info': debug_info
                })
            except Exception as e:
                # Silently ignore threading issues during RTH predictor updates
                pass
    
    def _on_remaining_flight_time(self, namespace: str, time_remaining: float):
        """Override flight time callback to emit event."""
        if namespace in self.drones:
            self.drones[namespace].remaining_flight_time = time_remaining
        self.drone_flight_time_update.emit({
            'namespace': namespace,
            'time_remaining': time_remaining
        })
    
    def _on_recording_status(self, namespace: str, is_recording: bool):
        """Override recording status callback to emit event."""
        if namespace in self.drones:
            self.drones[namespace].is_recording = is_recording
        self.drone_recording_update.emit({
            'namespace': namespace,
            'is_recording': is_recording
        })
    
    def _on_satellite_count(self, namespace: str, count: int):
        """Override satellite count callback to emit event."""
        if namespace in self.drones:
            self.drones[namespace].satellite_count = count
        self.drone_satellite_update.emit({
            'namespace': namespace,
            'count': count
        })
    
    def _on_speed(self, namespace: str, speed: float):
        """Override speed callback to emit event."""
        if namespace in self.drones:
            self.drones[namespace].speed = speed
        self.drone_speed_update.emit({
            'namespace': namespace,
            'speed': speed
        })
    
    def _on_flight_mode(self, namespace: str, mode: str):
        """Override flight mode callback to emit event and track manual control."""
        super()._on_flight_mode(namespace, mode)
        self.drone_flight_mode_update.emit({
            'namespace': namespace,
            'flight_mode': mode
        })
    
    def _on_mission_status_update(self, namespace: str, state: MissionState, message: str):
        """Override mission status callback to emit event."""
        super()._on_mission_status_update(namespace, state, message)
        
        # Track when monitoring starts (timer will be started by UI timer)
        if state == MissionState.MONITORING and self._mission_start_time is None:
            self._mission_start_time = time.time()
            self._should_start_timer = True  # Flag for UI thread to pick up
        
        # Track mission statistics per drone iteration
        self._track_mission_stats(namespace, state)
        
        # Map MissionState to DroneState
        state_map = {
            MissionState.IDLE: DroneState.IDLE,
            MissionState.SETTING_RTH_ALTITUDE: DroneState.IDLE,
            MissionState.TAKING_OFF: DroneState.TAKING_OFF,
            MissionState.CLIMBING_TO_ALTITUDE: DroneState.TAKING_OFF,
            MissionState.TRANSIT_TO_MONITORING: DroneState.FLYING_TO_POINT,
            MissionState.APPROACHING_POINT: DroneState.FLYING_TO_POINT,
            MissionState.MONITORING: DroneState.MONITORING,
            MissionState.WAITING_FOR_RELAY: DroneState.WAITING_FOR_RELAY,
            MissionState.CAMERA_SYNC: DroneState.CAMERA_SYNC,
            MissionState.RETURNING_HOME: DroneState.RETURNING_HOME,
            MissionState.COMPLETED: DroneState.IDLE,
            MissionState.ABORTED: DroneState.IDLE,
            MissionState.ERROR: DroneState.EMERGENCY,
        }
        
        if state in state_map:
            self.drone_state_update.emit({
                'namespace': namespace,
                'state': state_map[state]
            })
    
    def _track_mission_stats(self, namespace: str, state: MissionState):
        """Track mission statistics for each drone iteration."""
        if not hasattr(self, 'mission_controller') or not self.mission_controller:
            return
        
        mission = self.mission_controller.drone_missions.get(namespace)
        if not mission:
            return
        
        # When transit starts, create a new entry with estimated travel time
        if state == MissionState.TRANSIT_TO_MONITORING:
            # Increment iteration counter for this drone
            if namespace not in self.drone_iteration_counter:
                self.drone_iteration_counter[namespace] = 0
            self.drone_iteration_counter[namespace] += 1
            iteration = self.drone_iteration_counter[namespace]
            
            # Get estimated travel time from mission
            est_travel = mission.estimated_travel_time
            self._add_mission_stat(namespace, iteration, est_travel)
        
        # When monitoring starts, update with actual travel time
        elif state == MissionState.MONITORING:
            if namespace in self.drone_iteration_counter:
                iteration = self.drone_iteration_counter[namespace]
                actual_travel = mission.actual_travel_time
                self._add_mission_stat(namespace, iteration, 0, actual_travel=actual_travel)
        
        # When RTH starts (normal, aborted, or error), start tracking for landing detection
        elif state in [MissionState.RETURNING_HOME, MissionState.ABORTED, MissionState.ERROR]:
            # Only start tracking if not already tracking this drone
            if namespace not in self.drone_rth_tracking:
                self.drone_rth_tracking[namespace] = {
                    'start_time': time.time(),
                    'last_alt': None,
                    'stable_count': 0,
                    'detected': False
                }
                self._emit_log(f"[{namespace}] RTH tracking started")
    
    def _check_rth_landing(self, namespace: str, altitude: float):
        """Check if a drone in RTH state has landed based on altitude stability."""
        if namespace not in self.drone_rth_tracking:
            return
        
        tracking = self.drone_rth_tracking[namespace]
        if tracking['detected']:
            return  # Already detected landing
        
        # Need low altitude (< 3m) and stable (not changing much)
        LANDING_ALTITUDE_THRESHOLD = 3.0  # meters
        ALTITUDE_STABLE_THRESHOLD = 0.5   # meters - altitude change threshold
        STABLE_COUNT_REQUIRED = 3         # number of consecutive stable readings
        
        if tracking['last_alt'] is not None:
            alt_change = abs(altitude - tracking['last_alt'])
            
            # Check if altitude is low and stable
            if altitude < LANDING_ALTITUDE_THRESHOLD and alt_change < ALTITUDE_STABLE_THRESHOLD:
                tracking['stable_count'] += 1
                
                if tracking['stable_count'] >= STABLE_COUNT_REQUIRED:
                    # Landing detected!
                    tracking['detected'] = True
                    rth_duration = time.time() - tracking['start_time']
                    
                    # Update mission stats with RTH time
                    self._update_mission_stat_rth(namespace, rth_duration)
                    self._emit_log(f"[{namespace}] Landed after RTH ({rth_duration:.1f}s)")
                    
                    # Delegate state change to mission controller
                    if self.mission_controller:
                        self.mission_controller.notify_landing_detected(namespace, rth_duration)
                        # Emit state update to refresh GUI (subscriber will call _update_state_icons)
                        self.drone_state_update.emit({
                            'namespace': namespace,
                            'state': DroneState.IDLE
                        })
                    
                    # Clean up tracking
                    del self.drone_rth_tracking[namespace]
                    return
            else:
                # Reset stable count if altitude is changing or too high
                tracking['stable_count'] = 0
        
        tracking['last_alt'] = altitude
    
    def _on_relay_countdown_update(self, countdown: float, next_drone: str, timing_breakdown: dict = None):
        """Override relay countdown callback to emit event."""
        super()._on_relay_countdown_update(countdown, next_drone, timing_breakdown)
        self.relay_countdown_update.emit({
            'countdown': countdown,
            'next_drone': next_drone,
            'timing_breakdown': timing_breakdown or {}
        })
    
    def _on_takeoff_confirmation_request(self, drone_name: str, callback):
        """Handle takeoff confirmation request from mission controller (relay auto-launch).
        
        This is called from the mission controller thread, so we queue the request
        for processing in the UI thread (similar to other background events).
        """
        # Queue the request for the UI thread to process
        self._takeoff_confirmation_queue.append({
            'drone_name': drone_name,
            'callback': callback
        })
    
    def _process_takeoff_confirmation_sync(self, request: dict):
        """Process a takeoff confirmation request in the UI thread (sync version)."""
        try:
            drone_name = request['drone_name']
            callback = request['callback']
            
            # Show the confirmation dialog using sync version with callback storage
            self._relay_takeoff_callback = callback
            # Use main_row container to ensure proper UI context
            if hasattr(self, 'main_row') and self.main_row:
                with self.main_row:
                    self._show_relay_takeoff_dialog_sync(drone_name)
            else:
                self._show_relay_takeoff_dialog_sync(drone_name)
            # Note: callback will be called from dialog buttons
        except Exception as e:
            self._emit_log(f"[ERROR] Failed to process takeoff confirmation: {e}")
            request['callback'](False)
            self._takeoff_confirmation_dialog_open = False
    
    def _show_relay_takeoff_dialog_sync(self, drone_name: str):
        """Show relay takeoff confirmation dialog (sync version for background thread requests)."""
        with ui.dialog() as dialog, ui.card().classes('p-4'):
            ui.label(f'🚁 Relay Takeoff Confirmation').classes('text-xl font-bold text-blue-700')
            ui.separator()
            ui.label(f'Drone "{drone_name}" is ready to relay.').classes('text-lg mt-2')
            ui.label('Confirm to launch relay drone.').classes('text-sm text-gray-600 mt-1')
            
            def on_confirm():
                dialog.close()
                self._takeoff_confirmation_dialog_open = False
                # Play takeoff confirmation sound
                ui.run_javascript('''
                    var audio = new Audio("/static/take_off.mp3");
                    audio.play().catch(function(e) { console.log("Audio play failed:", e); });
                ''')
                # Call the mission controller callback
                if hasattr(self, '_relay_takeoff_callback') and self._relay_takeoff_callback:
                    self._relay_takeoff_callback(True)
                    self._relay_takeoff_callback = None
            
            def on_cancel():
                dialog.close()
                self._takeoff_confirmation_dialog_open = False
                # Call the mission controller callback with False
                if hasattr(self, '_relay_takeoff_callback') and self._relay_takeoff_callback:
                    self._relay_takeoff_callback(False)
                    self._relay_takeoff_callback = None
            
            with ui.row().classes('w-full justify-end gap-2 mt-4'):
                ui.button('Cancel', on_click=on_cancel, color='red').props('flat')
                ui.button('Confirm', on_click=on_confirm, color='primary')
        
        dialog.open()
    
    def _on_vertical_separation_alert(self, drone1: str, drone2: str, separation: float, alt1: float, alt2: float):
        """Handle vertical separation alert from mission controller.
        
        This is called from a background thread, so we queue the alert
        for processing in the UI thread.
        """
        # Queue the alert for processing in UI thread (thread-safe)
        self._vertical_separation_alerts.append({
            'drone1': drone1,
            'drone2': drone2,
            'separation': separation,
            'alt1': alt1,
            'alt2': alt2
        })
    
    def _on_vertical_separation_countdown_start(self):
        """Handle countdown start - play 20_seconds.mp3.
        
        This is called from a background thread, so we queue it for the UI thread.
        """
        self._emit_log("[DEBUG] _on_vertical_separation_countdown_start called!")
        self._vertical_separation_countdown_active = True
        self._notification_queue.append({
            'message': '⏱️ 20-SECOND COUNTDOWN STARTED!\nRTH will trigger if vertical separation not restored.',
            'type': 'warning',
            'timeout': 20000
        })
        # Queue the audio playback
        self._vertical_separation_alerts.append({
            'action': 'countdown_start'
        })
    
    def _on_vertical_separation_countdown_cancel(self):
        """Handle countdown cancel - play vertical_separation_respected.mp3.
        
        This is called from a background thread, so we queue it for the UI thread.
        """
        self._emit_log("[DEBUG] _on_vertical_separation_countdown_cancel called!")
        self._vertical_separation_countdown_active = False
        self._notification_queue.append({
            'message': '✅ Vertical separation restored!\nCountdown cancelled, mission continues.',
            'type': 'positive',
            'timeout': 5000
        })
        # Queue the audio playback
        self._vertical_separation_alerts.append({
            'action': 'countdown_cancel'
        })
    
    def _on_vertical_separation_mission_stopped(self):
        """Handle mission stopped due to vertical separation countdown expiry.
        
        This is called from a background thread, so we queue UI updates.
        """
        self._emit_log("[CRITICAL] Mission STOPPED due to vertical separation countdown expiry!")
        self._vertical_separation_countdown_active = False
        self._notification_queue.append({
            'message': '🛑 MISSION STOPPED!\nVertical separation countdown expired.\nAll drones returning home.',
            'type': 'negative',
            'timeout': 10000
        })
        # Queue the UI state update
        self._vertical_separation_alerts.append({
            'action': 'mission_stopped'
        })
    
    def _get_drone_flight_mode(self, namespace: str) -> str:
        """Get drone's current flight mode for manual flight detection."""
        if namespace in self.drones:
            return self.drones[namespace].flight_mode or ""
        return ""
    
    def _get_drone_altitude(self, namespace: str) -> float:
        """Get drone's current altitude for vertical separation check."""
        if namespace in self.drones:
            alt = self.drones[namespace].altitude or 0.0
            self.get_logger().debug(f"_get_drone_altitude({namespace}): {alt:.1f}m")
            return alt
        self.get_logger().warning(f"_get_drone_altitude({namespace}): drone not found!")
        return 0.0
    
    def connect_drone(self, ip_address: str, namespace: str = None) -> bool:
        """Override connect_drone to emit event on success."""
        result = super().connect_drone(ip_address, namespace)
        if result:
            # Find the namespace that was used
            ns = namespace if namespace else f"drone_{len(self.drones)}"
            if ns in self.drones:
                self.drone_connected_event.emit({
                    'namespace': ns,
                    'drone': self.drones[ns]
                })
                self._emit_log(f"[CONNECTED] {ns} at {ip_address}")
                
                # Check if drone was auto-added to relay
                if self.mission_controller.is_drone_in_mission(ns):
                    position = len(self.mission_controller.drone_order)
                    # Queue notification for UI thread (thread-safe)
                    self._notification_queue.append({
                        'message': f'{ns} auto-added to relay queue (position {position})',
                        'type': 'positive',
                        'timeout': 5000
                    })
                    self._emit_log(f"[RELAY] {ns} auto-joined relay mission")
        return result
    
    def disconnect_drone(self, namespace: str) -> bool:
        """Override disconnect_drone to emit event."""
        result = super().disconnect_drone(namespace)
        if result:
            self.drone_disconnected_event.emit({'namespace': namespace})
            self._emit_log(f"[DISCONNECTED] {namespace}")
        return result
    
    def set_monitoring_point(self, lat: float, lon: float, alt: float, heading: float = 0.0, source: str = "manual"):
        """Override to emit event."""
        super().set_monitoring_point(lat, lon, alt, heading, source)
        self.monitoring_point_update.emit({
            'lat': lat,
            'lon': lon,
            'alt': alt,
            'heading': heading
        })
        self._emit_log(f"[POINT] Set: ({lat:.6f}, {lon:.6f}, {alt:.1f}m) heading={heading:.0f}°")
    
    def clear_monitoring_point(self):
        """Override to emit event."""
        super().clear_monitoring_point()
        self.monitoring_point_update.emit({'clear': True})
        self._emit_log("[POINT] Cleared")
    
    def _emit_log(self, message: str):
        """Emit a log event."""
        self.log_event.emit({'message': message})
    
    # ========================================================================
    # UI CONSTRUCTION
    # ========================================================================
    
    def _build_ui(self):
        """Build the main UI layout with event subscriptions."""
        
        # Clear UI references on page refresh (map and arrows need to be recreated)
        self.drone_cards.clear()
        self.drone_arrows.clear()
        self.drone_labels.clear()
        self.drone_buttons.clear()
        self.monitoring_marker = None
        self.monitoring_circle = None
        
        # Clear trajectory data on page refresh (polylines need to be recreated with new map)
        self.drone_trajectory_lines.clear()
        # Keep drone_trajectories data so we can redraw if needed
        # Keep drone_is_flying state
        
        # Add CSS and JS (cache-busting with version parameter)
        ui.add_head_html("""
            <script src='/static/arrows.js?v=2'></script>
            <style>
                .drone-card { 
                    min-width: 280px; 
                    transition: all 0.3s ease;
                }
                .drone-card:hover {
                    transform: translateY(-2px);
                    box-shadow: 0 4px 12px rgba(0,0,0,0.15);
                }
                .countdown-display { 
                    font-size: 1.5rem; 
                    font-weight: bold; 
                }
                .mission-status { 
                    padding: 10px; 
                    border-radius: 8px; 
                }
                @keyframes blink {
                    0%, 100% { opacity: 1; }
                    50% { opacity: 0.5; }
                }
                .pulse {
                    animation: pulse 2s infinite;
                }
                @keyframes pulse {
                    0% { box-shadow: 0 0 0 0 rgba(76, 175, 80, 0.4); }
                    70% { box-shadow: 0 0 0 10px rgba(76, 175, 80, 0); }
                    100% { box-shadow: 0 0 0 0 rgba(76, 175, 80, 0); }
                }
            </style>
        """)
        
        with ui.row().classes('w-full h-full').style('display: flex; height: 95vh; gap: 10px; padding: 10px;') as self.main_row:
            # Left panel: Drone management
            self._build_left_panel()
            
            # Right panel: Map and mission control
            self._build_right_panel()
        
        # Subscribe to events for UI updates
        self._setup_event_subscriptions()
        
        # Timer to process log message queue (runs in UI thread)
        ui.timer(0.5, self._process_log_queue)
        
        # Populate drone list after map is created (so arrows can be placed)
        self._refresh_drone_list()
    
    def _setup_event_subscriptions(self):
        """Set up event subscriptions for UI updates."""
        
        @self.drone_position_update.subscribe
        def on_position(data: dict):
            ns = data['namespace']
            lat = data['lat']
            lon = data['lon']
            alt = data['alt']
            
            # Center map on first valid position from any new drone
            if ns not in self._centered_on_drone and self.map:
                if lat != 0.0 or lon != 0.0:
                    self.map.set_center((lat, lon))
                    self.map.set_zoom(17)
                    self._centered_on_drone.add(ns)
            
            # Update arrow on map
            if ns in self.drone_arrows:
                heading = self.drones[ns].heading if ns in self.drones else 0.0
                self.drone_arrows[ns].update(lat, lon, heading)
            
            # Update trajectory if drone is flying
            if self.drone_is_flying.get(ns, False) and (lat != 0.0 or lon != 0.0):
                self._update_drone_trajectory(ns, lat, lon)
            
            # Update altitude label
            if ns in self.drone_labels and 'altitude' in self.drone_labels[ns]:
                self.drone_labels[ns]['altitude'].text = f"{alt:.1f}m"
        
        @self.drone_heading_update.subscribe
        def on_heading(data: dict):
            ns = data['namespace']
            heading = data['heading']
            
            # Update arrow rotation
            if ns in self.drone_arrows and ns in self.drones:
                drone = self.drones[ns]
                if drone.latitude != 0 and drone.longitude != 0:
                    self.drone_arrows[ns].update(drone.latitude, drone.longitude, heading)
        
        @self.drone_battery_update.subscribe
        def on_battery(data: dict):
            ns = data['namespace']
            battery = data['level']
            
            if ns in self.drone_labels and 'battery' in self.drone_labels[ns]:
                color = 'green' if battery > 50 else 'orange' if battery > 20 else 'red'
                self.drone_labels[ns]['battery'].text = f"{battery:.0f}%"
                self.drone_labels[ns]['battery'].style(f'color: {color}; font-weight: bold')
        
        @self.drone_flight_time_update.subscribe
        def on_flight_time(data: dict):
            ns = data['namespace']
            time_remaining = data['time_remaining']
            
            if ns in self.drone_labels and 'flight_time' in self.drone_labels[ns]:
                minutes = int(time_remaining // 60)
                seconds = int(time_remaining % 60)
                color = 'green' if time_remaining > 300 else 'orange' if time_remaining > 120 else 'red'
                self.drone_labels[ns]['flight_time'].text = f"{minutes}:{seconds:02d}"
                self.drone_labels[ns]['flight_time'].style(f'color: {color}; font-weight: bold')
        
        @self.drone_recording_update.subscribe
        def on_recording(data: dict):
            ns = data['namespace']
            is_recording = data['is_recording']
            
            if ns in self.drone_labels and 'recording' in self.drone_labels[ns]:
                if is_recording:
                    self.drone_labels[ns]['recording'].text = "REC"
                    self.drone_labels[ns]['recording'].style('color: #c62828; font-weight: bold;')
                    if 'recording_icon' in self.drone_labels[ns]:
                        self.drone_labels[ns]['recording_icon'].style('font-size: 28px; color: #c62828; animation: blink 1s infinite;')
                else:
                    self.drone_labels[ns]['recording'].text = "OFF"
                    self.drone_labels[ns]['recording'].style('color: #bdbdbd; font-weight: bold;')
                    if 'recording_icon' in self.drone_labels[ns]:
                        self.drone_labels[ns]['recording_icon'].style('font-size: 28px; color: #bdbdbd; animation: none;')
        
        @self.drone_satellite_update.subscribe
        def on_satellite(data: dict):
            ns = data['namespace']
            count = data['count']
            
            if ns in self.drone_labels and 'satellites' in self.drone_labels[ns]:
                self.drone_labels[ns]['satellites'].text = f"{count}"
                # Color code based on count
                if count >= 10:
                    color = '#2e7d32'  # green
                elif count >= 6:
                    color = '#ef6c00'  # orange
                else:
                    color = '#c62828'  # red
                self.drone_labels[ns]['satellites'].style(f'color: {color}')
        
        @self.drone_speed_update.subscribe
        def on_speed(data: dict):
            ns = data['namespace']
            speed = data['speed']
            
            if ns in self.drone_labels and 'speed' in self.drone_labels[ns]:
                self.drone_labels[ns]['speed'].text = f"{speed:.1f}m/s"
        
        @self.drone_flight_mode_update.subscribe
        def on_flight_mode(data: dict):
            """Update manual flight indicator when flight mode changes."""
            ns = data['namespace']
            mode = data['flight_mode']
            
            if ns in self.drone_labels and 'manual_indicator' in self.drone_labels[ns]:
                # Show indicator if NOT in virtual_stick mode (pilot has manual control)
                is_manual = mode.lower() != 'virtual_stick' and mode.lower() != 'unknown'
                if is_manual:
                    self.drone_labels[ns]['manual_indicator'].style('display: inline-flex')
                    self._emit_log(f"[{ns}] Manual control detected: {mode}")
                else:
                    self.drone_labels[ns]['manual_indicator'].style('display: none')
        
        @self.rth_predictor_update.subscribe
        def on_rth_predictor(data: dict):
            ns = data['namespace']
            debug_info = data.get('debug_info', {})
            
            predicted_rth = debug_info.get('predicted_rth_seconds', float('inf'))
            drain_rate = debug_info.get('drain_rate_per_min', 0.0) / 60  # Convert back to %/s for existing code
            is_active = debug_info.get('is_active', False)
            data_points = debug_info.get('data_points', 0)
            
            if ns not in self.drone_labels:
                return
            
            labels = self.drone_labels[ns]
            
            # Update RTH debug panel if this is the active monitoring drone
            # (Do this FIRST, before any early returns, so debug info is always shown)
            if hasattr(self, 'rth_debug_container') and self.rth_debug_container:
                # Check if this drone is in MONITORING state
                if ns in self.drones and self.drones[ns].state == DroneState.MONITORING:
                    self._update_rth_debug_panel(ns, debug_info)
            
            # Show/hide the RTH predictor row based on active state
            if 'rth_predictor_row' in labels:
                if is_active and data_points >= 2:
                    labels['rth_predictor_row'].style('display: flex')
                else:
                    labels['rth_predictor_row'].style('display: none')
                    return
            
            # Update predicted RTH time
            if 'rth_predicted' in labels:
                if predicted_rth != float('inf') and predicted_rth > 0:
                    minutes = int(predicted_rth // 60)
                    seconds = int(predicted_rth % 60)
                    color = '#2e7d32' if predicted_rth > 300 else '#f57c00' if predicted_rth > 120 else '#c62828'
                    labels['rth_predicted'].text = f"{minutes}:{seconds:02d}"
                    labels['rth_predicted'].style(f'color: {color}; font-weight: bold')
                else:
                    labels['rth_predicted'].text = "--:--"
                    labels['rth_predicted'].style('color: #9e9e9e')
            
            # Update drain rate (%/min)
            if 'rth_drain_rate' in labels:
                drain_per_min = drain_rate * 60  # Convert from %/s to %/min
                if drain_per_min > 0:
                    labels['rth_drain_rate'].text = f"{drain_per_min:.2f}%/min"
                else:
                    labels['rth_drain_rate'].text = "--%/min"
            
            # Update data points count
            if 'rth_data_points' in labels:
                labels['rth_data_points'].text = f"{data_points} pts"
        
        @self.drone_state_update.subscribe
        def on_state(data: dict):
            ns = data['namespace']
            state = data['state']
            
            # Track flying state for trajectory
            flying_states = [
                DroneState.TAKING_OFF,
                DroneState.FLYING_TO_POINT,
                DroneState.MONITORING,
                DroneState.WAITING_FOR_RELAY,
                DroneState.CAMERA_SYNC,
                DroneState.RETURNING_HOME,
            ]
            
            was_flying = self.drone_is_flying.get(ns, False)
            is_flying = state in flying_states
            
            # Start new trajectory on takeoff
            if not was_flying and is_flying:
                self._start_drone_trajectory(ns)
            
            # Clear trajectory with fade on landing
            if was_flying and not is_flying:
                self._fade_and_clear_trajectory(ns)
            
            self.drone_is_flying[ns] = is_flying
            
            if ns in self.drone_labels and 'state' in self.drone_labels[ns]:
                # Use simple text labels - icons are in the card header
                state_colors = {
                    DroneState.DISCONNECTED: "background: #ffebee; color: #c62828;",
                    DroneState.CONNECTED: "background: #e8f5e9; color: #2e7d32;",
                    DroneState.IDLE: "background: #f5f5f5; color: #616161;",
                    DroneState.TAKING_OFF: "background: #e3f2fd; color: #1565c0;",
                    DroneState.FLYING_TO_POINT: "background: #e3f2fd; color: #1565c0;",
                    DroneState.MONITORING: "background: #f3e5f5; color: #7b1fa2;",
                    DroneState.WAITING_FOR_RELAY: "background: #fff8e1; color: #f9a825;",
                    DroneState.CAMERA_SYNC: "background: #e8f5e9; color: #2e7d32;",
                    DroneState.RETURNING_HOME: "background: #fff3e0; color: #ef6c00;",
                    DroneState.LANDING: "background: #e0f2f1; color: #00695c;",
                    DroneState.EMERGENCY: "background: #ffebee; color: #c62828;"
                }
                style = state_colors.get(state, "background: #f5f5f5; color: #616161;")
                self.drone_labels[ns]['state'].text = state.value
                self.drone_labels[ns]['state'].style(style)
                
                # Highlight active monitoring drone
                if state == DroneState.MONITORING and ns in self.drone_cards:
                    self.drone_cards[ns].style('border: 3px solid #4CAF50; box-shadow: 0 0 10px #4CAF50')
                elif ns in self.drone_cards:
                    self.drone_cards[ns].style('border: 1px solid #ddd; box-shadow: none')
            
            # Update state machine display icons (if they exist)
            self._update_state_icons(ns)
        
        @self.drone_connected_event.subscribe
        def on_connected(data: dict):
            self._refresh_drone_list()
            self._build_state_machine_display()  # Update state machine display
            # Try to center map on the newly connected drone (if it has position)
            ns = data.get('namespace')
            drone = data.get('drone')
            if drone and self.map and ns not in self._centered_on_drone:
                lat = drone.latitude
                lon = drone.longitude
                # Only center if we have valid coordinates (not 0,0)
                if lat != 0.0 or lon != 0.0:
                    self.map.set_center((lat, lon))
                    self.map.set_zoom(17)
                    self._centered_on_drone.add(ns)
            # Note: If drone doesn't have position yet, position_update handler will center on first GPS fix
        
        @self.drone_disconnected_event.subscribe
        def on_disconnected(data: dict):
            ns = data['namespace']
            if ns in self.drone_cards:
                self.drone_cards[ns].delete()
                del self.drone_cards[ns]
            if ns in self.drone_arrows:
                self.drone_arrows[ns].destroy()
                del self.drone_arrows[ns]
            if ns in self.drone_labels:
                del self.drone_labels[ns]
            # Clean up trajectory data
            if ns in self.drone_trajectory_lines and self.drone_trajectory_lines[ns]:
                try:
                    self.map.remove_layer(self.drone_trajectory_lines[ns])
                except:
                    pass
                del self.drone_trajectory_lines[ns]
            if ns in self.drone_trajectories:
                del self.drone_trajectories[ns]
            if ns in self.drone_is_flying:
                del self.drone_is_flying[ns]
            self._build_state_machine_display()  # Update state machine display
        
        @self.monitoring_point_update.subscribe
        def on_monitoring_point(data: dict):
            if data.get('clear'):
                if self.monitoring_marker:
                    self.map.remove_layer(self.monitoring_marker)
                    self.monitoring_marker = None
                if self.monitoring_circle:
                    self.map.remove_layer(self.monitoring_circle)
                    self.monitoring_circle = None
                # Reset input fields
                if self.lat_input:
                    self.lat_input.value = '0.0'
                if self.lon_input:
                    self.lon_input.value = '0.0'
                if self.heading_input:
                    self.heading_input.value = '0'
            else:
                lat, lon, alt = data['lat'], data['lon'], data['alt']
                heading = data.get('heading', 0)
                
                # Update input fields
                if self.lat_input:
                    self.lat_input.value = f"{lat:.6f}"
                if self.lon_input:
                    self.lon_input.value = f"{lon:.6f}"
                if self.alt_input:
                    self.alt_input.value = f"{alt:.0f}"
                if self.heading_input:
                    self.heading_input.value = f"{heading:.0f}"
                
                if self.map:
                    if self.monitoring_marker:
                        self.map.remove_layer(self.monitoring_marker)
                    if self.monitoring_circle:
                        self.map.remove_layer(self.monitoring_circle)
                    
                    self.monitoring_marker = self.map.marker(latlng=[lat, lon])
                    self.monitoring_circle = self.map.generic_layer(
                        name='circle',
                        args=[[lat, lon], {'radius': 50, 'color': 'green', 'fillOpacity': 0.2, 'weight': 2}]
                    )
        
        @self.relay_countdown_update.subscribe
        def on_countdown(data: dict):
            countdown = data['countdown']
            next_drone = data['next_drone']
            timing_breakdown = data.get('timing_breakdown', {})
            
            # Check if manual swap just completed - restore UI
            if self._manual_swap_active and self.mission_controller and not self.mission_controller.is_manual_swap_active():
                self._manual_swap_active = False
                # Restore countdown display
                if self.countdown_container:
                    self.countdown_container.style('display: flex; background: #fff3e0;')
                # Restore force swap button
                if self.force_swap_button:
                    self.force_swap_button.props(remove='disabled')
                    self.force_swap_button.text = 'Force Swap'
                self._emit_log("[SWAP] Swap completed - returning to automatic countdown mode")
            
            # Don't update countdown display if manual swap is in progress
            if self._manual_swap_active:
                return
            
            # Update timing breakdown display
            if timing_breakdown and hasattr(self, 'timing_breakdown_container'):
                self.timing_breakdown_container.style('display: flex;')
                
                remaining = timing_breakdown.get('remaining_flight_time', 0)
                travel = timing_breakdown.get('avg_travel_time', 0)
                buffer = timing_breakdown.get('safety_buffer', 0)
                
                if hasattr(self, 'remaining_time_label'):
                    self.remaining_time_label.text = f"To RTH: {int(remaining//60)}:{int(remaining%60):02d}"
                if hasattr(self, 'travel_time_label'):
                    self.travel_time_label.text = f"Travel: {int(travel//60)}:{int(travel%60):02d}"
                if hasattr(self, 'buffer_time_label'):
                    self.buffer_time_label.text = f"Buffer: {int(buffer//60)}:{int(buffer%60):02d}"
            
            if self.countdown_label:
                if countdown > 0:
                    minutes = int(countdown // 60)
                    seconds = int(countdown % 60)
                    self.countdown_label.text = f"{minutes}:{seconds:02d}"
                    self.countdown_label.style('color: #e65100; font-weight: bold')
                    
                    if self.countdown_progress:
                        self.countdown_progress.value = max(0, min(1, countdown / 300))
                    
                    # Threshold-based preparation alerts - use the alert container
                    if hasattr(self, 'relay_alert_container') and self.relay_alert_container:
                        if countdown <= 60:  # 1 minute - CONNECT NOW
                            self.relay_alert_container.style('background: #ffebee; border-left: 4px solid #f44336; display: block;')
                            self.relay_alert_label.text = f"CONNECT {next_drone} NOW!"
                            self.relay_alert_label.style('color: #c62828; animation: blink 0.5s infinite')
                            self.relay_alert_icon.style('color: #c62828; animation: blink 0.5s infinite')
                        elif countdown <= 180:  # 3 minutes - GET READY
                            self.relay_alert_container.style('background: #fff3e0; border-left: 4px solid #ff9800; display: block;')
                            self.relay_alert_label.text = f"GET {next_drone} READY"
                            self.relay_alert_label.style('color: #e65100;')
                            self.relay_alert_icon.style('color: #e65100;')
                        elif countdown <= 300:  # 5 minutes - PREPARE
                            self.relay_alert_container.style('background: #e8f5e9; border-left: 4px solid #4caf50; display: block;')
                            self.relay_alert_label.text = f"Prepare {next_drone}"
                            self.relay_alert_label.style('color: #2e7d32;')
                            self.relay_alert_icon.style('color: #2e7d32;')
                        else:
                            self.relay_alert_container.style('display: none;')
                else:
                    self.countdown_label.text = "LAUNCHING!"
                    self.countdown_label.style('color: #c62828; font-weight: bold; animation: blink 0.5s infinite')
                    if hasattr(self, 'relay_alert_container') and self.relay_alert_container:
                        self.relay_alert_container.style('background: #ffebee; border-left: 4px solid #f44336; display: block;')
                        self.relay_alert_label.text = f"LAUNCHING {next_drone}!"
                        self.relay_alert_label.style('color: #c62828; font-weight: bold; animation: blink 0.5s infinite')
                        self.relay_alert_icon.style('color: #c62828; animation: blink 0.5s infinite')
            
            if self.next_drone_label:
                self.next_drone_label.text = f"Next: {next_drone}"
            
            # Show drones needing reconnection (battery swap)
            if self.reconnect_label:
                needs_reconnect = self.mission_controller.get_drones_needing_reconnection()
                if needs_reconnect:
                    self.reconnect_label.text = f"Swap battery & reconnect: {', '.join(needs_reconnect)}"
                    self.reconnect_label.style('color: #ffcc00; font-weight: bold')
                else:
                    self.reconnect_label.text = ""
        
        @self.log_event.subscribe
        def on_log(data: dict):
            message = data['message']
            timestamp = datetime.now().strftime("%H:%M:%S")
            # Queue the message for the UI timer to process
            self.log_message_queue.append(f"[{timestamp}] {message}")
    
    def _process_log_queue(self):
        """Process queued log messages (called by UI timer, runs in UI thread)."""
        # Check if mission timer should be started
        if self._should_start_timer:
            self._should_start_timer = False
            self._start_mission_timer()
        
        # Process vertical separation alerts (these need UI context)
        while self._vertical_separation_alerts:
            alert = self._vertical_separation_alerts.pop(0)
            self._show_vertical_separation_alert(alert)
        
        # Update vertical separation card
        self._update_vertical_separation_card()
        
        # Process takeoff confirmation requests (from background threads)
        if self._takeoff_confirmation_queue and not self._takeoff_confirmation_dialog_open:
            request = self._takeoff_confirmation_queue.pop(0)
            self._takeoff_confirmation_dialog_open = True
            self._process_takeoff_confirmation_sync(request)
        
        # Process queued notifications (from background threads)
        while self._notification_queue:
            notif = self._notification_queue.pop(0)
            ui.notify(
                notif.get('message', ''),
                type=notif.get('type', 'info'),
                timeout=notif.get('timeout', 3000)
            )
        
        if not self.event_log or not self.log_message_queue:
            return
        
        # Process all queued messages
        while self.log_message_queue:
            message = self.log_message_queue.pop(0)
            with self.event_log:
                ui.label(message).classes('text-sm')
        
        # Scroll to bottom
        if self.event_scroll:
            self.event_scroll.scroll_to(percent=1.0)
    
    def _show_vertical_separation_alert(self, alert: dict):
        """Show vertical separation alert in UI thread.
        
        Can handle different alert types:
        - Regular alert: drone1, drone2, separation, alt1, alt2
        - Countdown start: action='countdown_start'
        - Countdown cancel: action='countdown_cancel'
        """
        action = alert.get('action')
        
        if action == 'countdown_start':
            # Play 20-second countdown audio
            ui.run_javascript('''
                // Stop any existing countdown audio first
                if (window.countdownAudio) {
                    window.countdownAudio.pause();
                    window.countdownAudio.currentTime = 0;
                }
                window.countdownAudio = new Audio("/static/20_seconds.mp3");
                window.countdownAudio.play().catch(function(e) { console.log("Countdown audio play failed:", e); });
            ''')
            self._emit_log(f"[ALERT] ⏱️ 20-SECOND COUNTDOWN STARTED - RTH will trigger if separation not restored!")
            return
        
        elif action == 'countdown_cancel':
            # Stop countdown audio and play "respected" sound
            ui.run_javascript('''
                // Stop countdown audio
                if (window.countdownAudio) {
                    window.countdownAudio.pause();
                    window.countdownAudio.currentTime = 0;
                    window.countdownAudio = null;
                }
                // Play separation respected sound
                var audio = new Audio("/static/vertical_separation_respected.mp3");
                audio.play().catch(function(e) { console.log("Respected audio play failed:", e); });
            ''')
            self._emit_log(f"[ALERT] ✅ VERTICAL SEPARATION RESTORED - Countdown cancelled!")
            return
        
        elif action == 'mission_stopped':
            # Stop countdown audio and update UI to stopped state
            ui.run_javascript('''
                // Stop countdown audio
                if (window.countdownAudio) {
                    window.countdownAudio.pause();
                    window.countdownAudio.currentTime = 0;
                    window.countdownAudio = null;
                }
            ''')
            # Update UI state same as Stop button
            self._stop_mission_timer()
            if self.mission_status_label:
                self.mission_status_label.text = "STOPPED (Vert.Sep)"
                self.mission_status_label.style('background: #ffebee; color: #c62828;')
            if self.countdown_label:
                self.countdown_label.text = "--:--"
            if self.active_drone_label:
                self.active_drone_label.text = "--"
                self.active_drone_label.style('background: #e0e0e0; color: #424242;')
            self._emit_log(f"[CRITICAL] 🛑 MISSION STOPPED - Vertical separation countdown expired!")
            return
        
        # Regular vertical separation alert
        drone1 = alert.get('drone1', 'unknown')
        drone2 = alert.get('drone2', 'unknown')
        separation = alert.get('separation', 0)
        alt1 = alert.get('alt1', 0)
        alt2 = alert.get('alt2', 0)
        
        # Play warning sound (not the countdown)
        ui.run_javascript('''
            var audio = new Audio("/static/vertical_separation.mp3");
            audio.play().catch(function(e) { console.log("Audio play failed:", e); });
        ''')
        
        # Show critical notification
        ui.notify(
            f'⚠️ CRITICAL: Vertical separation alert!\n'
            f'{drone1} ({alt1:.1f}m) and {drone2} ({alt2:.1f}m)\n'
            f'Separation: {separation:.1f}m (min 5m required)',
            type='negative',
            position='top',
            timeout=10000,
            close_button=True
        )
        
        self._emit_log(f"[ALERT] VERTICAL SEPARATION: {drone1}={alt1:.1f}m, {drone2}={alt2:.1f}m, sep={separation:.1f}m")

    def _update_vertical_separation_card(self):
        """Update the vertical separation information card."""
        if not hasattr(self, 'vertical_sep_status_badge'):
            return
        
        # Get all airborne drones with their altitudes
        airborne_drones = []
        from groundstation.mission_controller import MissionState
        
        for ns, mission in self.mission_controller.drone_missions.items():
            if mission.state not in [MissionState.IDLE, MissionState.ERROR, MissionState.COMPLETED]:
                altitude = self._get_drone_altitude(ns)
                airborne_drones.append((ns, altitude, mission.state))
        
        # Calculate minimum vertical separation between any pair
        min_separation = float('inf')
        violation_pair = None
        MIN_VERTICAL_SEPARATION = 5.0
        
        for i, (ns1, alt1, _) in enumerate(airborne_drones):
            for ns2, alt2, _ in airborne_drones[i+1:]:
                sep = abs(alt1 - alt2)
                if sep < min_separation:
                    min_separation = sep
                    violation_pair = (ns1, ns2, alt1, alt2)
        
        # Update status badge and current separation
        if len(airborne_drones) < 2:
            # Not enough drones to compare
            self.vertical_sep_status_badge.set_text('N/A')
            self.vertical_sep_status_badge.props('color=grey')
            self.vertical_sep_current_label.text = '--'
            self.vertical_sep_current_label.style('color: #9e9e9e;')
        elif min_separation < MIN_VERTICAL_SEPARATION:
            # Violation!
            self.vertical_sep_status_badge.set_text('⚠️ ALERT')
            self.vertical_sep_status_badge.props('color=red')
            self.vertical_sep_current_label.text = f'{min_separation:.1f}m'
            self.vertical_sep_current_label.style('color: #c62828;')
        elif min_separation < MIN_VERTICAL_SEPARATION * 2:
            # Warning (within 10m)
            self.vertical_sep_status_badge.set_text('CAUTION')
            self.vertical_sep_status_badge.props('color=orange')
            self.vertical_sep_current_label.text = f'{min_separation:.1f}m'
            self.vertical_sep_current_label.style('color: #e65100;')
        else:
            # OK
            self.vertical_sep_status_badge.set_text('OK')
            self.vertical_sep_status_badge.props('color=green')
            self.vertical_sep_current_label.text = f'{min_separation:.1f}m' if min_separation != float('inf') else '--'
            self.vertical_sep_current_label.style('color: #2e7d32;')
        
        # Update airborne drones list
        if hasattr(self, 'vertical_sep_drones_list'):
            self.vertical_sep_drones_list.clear()
            with self.vertical_sep_drones_list:
                if not airborne_drones:
                    ui.label("No drones airborne").classes('text-xs text-gray-400 italic')
                else:
                    for ns, alt, state in sorted(airborne_drones, key=lambda x: x[1], reverse=True):
                        # Color based on state
                        color = '#424242'
                        icon_name = 'flight'
                        if state in [MissionState.TAKING_OFF, MissionState.CLIMBING_TO_ALTITUDE]:
                            color = '#1976d2'  # Blue for climbing
                            icon_name = 'trending_up'
                        elif state == MissionState.MONITORING:
                            color = '#2e7d32'  # Green for monitoring
                            icon_name = 'location_on'
                        elif state == MissionState.RETURNING_HOME:
                            color = '#f57c00'  # Orange for RTH
                            icon_name = 'home'
                        
                        with ui.row().classes('items-center gap-1 py-0'):
                            ui.icon(icon_name, size='xs').style(f'color: {color};')
                            ui.label(f'{ns}').classes('text-xs font-bold').style(f'color: {color}; min-width: 50px;')
                            ui.label(f'{alt:.0f}m').classes('text-xs font-mono font-bold').style(f'color: {color};')
        
        # Update countdown row visibility and progress
        if self._vertical_separation_countdown_active and hasattr(self, 'vertical_sep_countdown_row'):
            self.vertical_sep_countdown_row.style('display: flex;')
            
            # Calculate countdown progress
            import time
            elapsed = time.time() - self.mission_controller._vertical_separation_countdown_start
            remaining = max(0, 20.0 - elapsed)
            progress = elapsed / 20.0
            
            self.vertical_sep_countdown_label.text = f'RTH in: {remaining:.0f}s'
            self.vertical_sep_countdown_progress.value = progress
        elif hasattr(self, 'vertical_sep_countdown_row'):
            self.vertical_sep_countdown_row.style('display: none;')

    def _build_state_machine_display(self):
        """Build the state machine visualization."""
        if not self.state_machine_container:
            return
        
        self.state_machine_container.clear()
        self.state_machine_labels.clear()
        
        # Show current mission mode indicator
        from groundstation.mission_controller import MissionMode
        is_free_flight = (hasattr(self, 'mission_controller') and 
                          self.mission_controller.mission_mode == MissionMode.FREE_FLIGHT)
        
        with self.state_machine_container:
            # Mission mode badge
            if is_free_flight:
                with ui.row().classes('w-full items-center gap-1 mb-1'):
                    ui.badge('🆓 FREE FLIGHT', color='orange').classes('text-xs')
                    ui.label('Pilot controls after altitude').classes('text-xs text-gray-500')
            else:
                with ui.row().classes('w-full items-center gap-1 mb-1'):
                    ui.badge('📍 MONITORING', color='blue').classes('text-xs')
                    ui.label('Auto-navigate to point').classes('text-xs text-gray-500')
        
        # Define states based on mission mode
        if is_free_flight:
            # Free Flight: first drone goes to pilot control after climb
            # Relay drones go through transit/approach/sync before pilot control
            # Flow: First drone: IDLE→...→CLIMB→MONITORING→WAITING→RTH
            #       Relay drone: IDLE→...→CLIMB→TRANSIT→APPROACH→SYNC→MONITORING→RTH
            states = [
                ('IDLE', 'hourglass_empty', 'Idle'),
                ('SETTING_RTH_ALTITUDE', 'height', 'RTH'),
                ('TAKING_OFF', 'flight_takeoff', 'T/O'),
                ('CLIMBING_TO_ALTITUDE', 'trending_up', 'Climb'),
                ('MONITORING', 'sports_esports', 'Pilot'),     # First drone: pilot control after climb
                ('WAITING_FOR_RELAY', 'swap_horiz', 'Wait'),   # First drone waiting for relay to arrive
                ('TRANSIT_TO_MONITORING', 'flight', 'Relay'),  # Relay drone flying to first drone
                ('APPROACHING_POINT', 'gps_fixed', 'Appr'),    # Relay drone approaching for handoff
                ('CAMERA_SYNC', '360', 'Sync'),                # Camera sync before handoff
                ('RETURNING_HOME', 'home', 'RTH'),
                ('COMPLETED', 'check_circle', 'Done'),
            ]
        else:
            # Monitoring Point: full automated flow
            states = [
                ('IDLE', 'hourglass_empty', 'Idle'),
                ('SETTING_RTH_ALTITUDE', 'height', 'RTH'),
                ('TAKING_OFF', 'flight_takeoff', 'T/O'),
                ('CLIMBING_TO_ALTITUDE', 'trending_up', 'Climb'),
                ('TRANSIT_TO_MONITORING', 'flight', 'Transit'),
                ('APPROACHING_POINT', 'gps_fixed', 'Appr'),
                ('MONITORING', 'videocam', 'Mon'),
                ('WAITING_FOR_RELAY', 'swap_horiz', 'Wait'),
                ('CAMERA_SYNC', '360', 'Sync'),
                ('RETURNING_HOME', 'home', 'RTH'),
                ('COMPLETED', 'check_circle', 'Done'),
            ]
        
        with self.state_machine_container:
            # Ultra-compact state legend - icons only in a single row
            with ui.row().classes('w-full justify-between gap-0'):
                for state_name, icon, label in states:
                    with ui.column().classes('items-center').style('min-width: 28px;'):
                        ui.icon(icon).style('font-size: 12px; color: #9e9e9e;')
                        ui.label(label).style('font-size: 8px; color: #9e9e9e;')
            
            # Show drones in mission from mission controller
            drones_in_mission = list(self.mission_controller.drone_missions.keys())
            if drones_in_mission:
                for namespace in drones_in_mission:
                    self._add_drone_state_row(namespace, states)
            elif self.drones:
                ui.label("Mission not started").classes('text-gray-400 italic text-xs mt-1')
            else:
                ui.label("No drones connected").classes('text-gray-400 italic text-xs mt-1')
    
    def _add_drone_state_row(self, namespace: str, states: list):
        """Add a state row for a drone."""
        mission = self.mission_controller.get_mission_status(namespace)
        current_state = mission.state.name if mission else 'IDLE'
        
        self.state_machine_labels[namespace] = {}
        
        with ui.row().classes('w-full items-center gap-0 py-1').style('border-top: 1px solid #e0e0e0;'):
            # Drone name - compact
            ui.label(namespace).classes('font-bold text-xs').style('min-width: 60px; max-width: 60px; overflow: hidden; text-overflow: ellipsis;')
            
            # State indicators - smaller and tighter
            with ui.row().classes('flex-grow justify-between gap-0'):
                for state_name, icon, label in states:
                    is_current = (current_state == state_name)
                    is_past = self._is_state_past(current_state, state_name, states)
                    is_error = current_state in ['ERROR', 'ABORTED']
                    
                    if is_current:
                        color = '#4CAF50'  # green - current
                        bg = '#e8f5e9'
                    elif is_past:
                        color = '#2196F3'  # blue - completed
                        bg = '#e3f2fd'
                    elif is_error and state_name == current_state:
                        color = '#f44336'  # red - error
                        bg = '#ffebee'
                    else:
                        color = '#bdbdbd'  # grey - not reached
                        bg = '#fafafa'
                    
                    state_icon = ui.icon(icon).style(f'font-size: 14px; color: {color}; background: {bg}; border-radius: 50%; padding: 2px;')
                    state_icon.tooltip(f"{label}: {state_name}")
                    self.state_machine_labels[namespace][state_name] = state_icon
    
    def _is_state_past(self, current: str, check: str, states: list) -> bool:
        """Check if a state has been passed."""
        state_order = [s[0] for s in states]
        try:
            current_idx = state_order.index(current)
            check_idx = state_order.index(check)
            return check_idx < current_idx
        except ValueError:
            return False
    
    def _update_state_icons(self, namespace: str):
        """Update state machine icons for a specific drone (thread-safe)."""
        if namespace not in self.state_machine_labels:
            return
        
        mission = self.mission_controller.get_mission_status(namespace)
        if not mission:
            return
        
        current_state = mission.state.name
        
        states = [
            ('IDLE', 'hourglass_empty', 'Waiting'),
            ('SETTING_RTH_ALTITUDE', 'height', 'Set RTH Alt'),
            ('TAKING_OFF', 'flight_takeoff', 'Takeoff'),
            ('CLIMBING_TO_ALTITUDE', 'trending_up', 'Climbing'),
            ('TRANSIT_TO_MONITORING', 'flight', 'Transit'),
            ('APPROACHING_POINT', 'gps_fixed', 'Approaching'),
            ('MONITORING', 'videocam', 'Monitoring'),
            ('WAITING_FOR_RELAY', 'swap_horiz', 'Waiting Relay'),
            ('CAMERA_SYNC', '360', 'Camera Sync'),
            ('RETURNING_HOME', 'home', 'RTH'),
            ('COMPLETED', 'check_circle', 'Done'),
        ]
        
        # Update existing icons with new colors
        for state_name, icon, label in states:
            if state_name not in self.state_machine_labels[namespace]:
                continue
            
            is_current = (current_state == state_name)
            is_past = self._is_state_past(current_state, state_name, states)
            is_error = current_state in ['ERROR', 'ABORTED']
            
            # Special case: when IDLE, reset all icons to grey (ready for new mission)
            # IDLE is special - it means drone is ready, not that it's progressing through mission
            if current_state == 'IDLE':
                if state_name == 'IDLE':
                    color = '#4CAF50'  # green - current/ready
                    bg = '#e8f5e9'
                else:
                    color = '#bdbdbd'  # grey - not started
                    bg = '#fafafa'
            elif is_current:
                color = '#4CAF50'  # green - current
                bg = '#e8f5e9'
            elif is_past:
                color = '#2196F3'  # blue - completed
                bg = '#e3f2fd'
            elif is_error:
                color = '#f44336'  # red - error
                bg = '#ffebee'
            else:
                color = '#bdbdbd'  # grey - not reached
                bg = '#fafafa'
            
            # Update the icon style
            self.state_machine_labels[namespace][state_name].style(
                f'font-size: 22px; color: {color}; background: {bg}; border-radius: 50%; padding: 4px'
            )
    
    def _build_left_panel(self):
        """Build the left panel with drone management."""
        with ui.card().classes('h-full').style('flex: 1.2; min-width: 350px; overflow-y: auto;'):
            with ui.row().classes('items-center gap-3 w-full'):
                ui.image('/static/logo.png').classes('w-16 h-16')
                ui.label("WildPerpetua").classes('text-2xl font-bold').style('flex-grow: 1')
                self.debug_toggle = ui.button(icon='bug_report', on_click=self._toggle_debug_mode).props('flat dense').tooltip('Toggle ROS Console')
                ui.button(icon='restart_alt', on_click=self._restart_groundstation).props('flat dense color=negative').tooltip('Restart Groundstation')
            
            ui.separator()
            
            # Connection form
            with ui.expansion("Add New Drone", icon='add_circle').classes('w-full'):
                with ui.row().classes('w-full gap-2'):
                    with ui.column().classes('flex-grow'):
                        self.ip_input = ui.input(
                            label='IP Address',
                            placeholder='192.168.x.x',
                            validation={'Invalid IP': lambda v: self._validate_ip(v)}
                        ).classes('w-full')
                    
                    with ui.column().style('width: 140px'):
                        with ui.row().classes('items-end gap-1'):
                            self.namespace_input = ui.input(
                                label='Name',
                                placeholder='drone_1',
                                value=self._get_next_drone_name()
                            ).classes('w-full').style('flex: 1;')
                            ui.button(icon='add', on_click=self._increment_drone_name).props('flat dense size=sm').tooltip('Next drone name')
                
                with ui.row().classes('w-full gap-2 mt-2'):
                    ui.button('Connect', icon='link', on_click=self._connect_drone_ui).props('color=primary')
                    ui.button('Refresh', icon='refresh', on_click=self._refresh_drone_list).props('flat')
            
            ui.separator()
            
            # Mission Status Card - Compact layout
            with ui.card().classes('w-full p-3'):
                # Header row with status indicators inline
                with ui.row().classes('items-center gap-3 w-full'):
                    ui.icon('analytics').classes('text-xl text-primary')
                    ui.label("Mission Status").classes('text-lg font-bold')
                    ui.space()
                    self.mission_status_label = ui.label("Inactive").classes('text-sm font-bold px-2 py-1 rounded').style('background: #e0e0e0; color: #424242;')
                    self.active_drone_label = ui.label("--").classes('text-sm font-bold px-2 py-1 rounded').style('background: #e3f2fd; color: #1565c0;')
                
                # Timer and Countdown in one row
                with ui.row().classes('w-full items-center gap-4 mt-2'):
                    # Mission duration
                    with ui.row().classes('items-center gap-2 p-2 rounded flex-1').style('background: #f5f5f5;'):
                        ui.icon('timer').classes('text-xl text-gray-600')
                        self.mission_timer_label = ui.label("00:00:00").classes('text-xl font-bold font-mono').style('color: #1976d2;')
                    
                    # Relay countdown (hidden during manual swap)
                    with ui.row().classes('items-center gap-2 p-2 rounded flex-1').style('background: #fff3e0;') as self.countdown_container:
                        ui.icon('schedule').classes('text-xl').style('color: #e65100;')
                        self.countdown_label = ui.label("--:--").classes('text-xl font-bold font-mono').style('color: #e65100;')
                    
                    # Force Swap button (always visible, next to countdown)
                    self.force_swap_button = ui.button('Force Swap', icon='swap_horiz', on_click=self._force_swap_clicked).props('color=warning no-caps dense size=sm').classes('ml-2').tooltip('Manually trigger relay swap')
                
                # Progress bar for countdown
                self.countdown_progress = ui.linear_progress(value=0).props('instant-feedback color=orange').classes('mt-1')
                
                # Timing breakdown row (hidden by default, shows formula: Countdown = ToRTH - Travel - Buffer)
                with ui.row().classes('w-full gap-1 mt-1 text-xs').style('display: none;') as self.timing_breakdown_container:
                    with ui.row().classes('items-center gap-1 p-1 rounded').style('background: #e3f2fd;'):
                        ui.icon('battery_alert', size='xs').style('color: #1565c0;')
                        self.remaining_time_label = ui.label("To RTH: --").style('color: #1565c0; font-size: 11px;')
                    with ui.row().classes('items-center gap-1 p-1 rounded').style('background: #fff3e0;'):
                        ui.icon('route', size='xs').style('color: #e65100;')
                        self.travel_time_label = ui.label("Travel: --").style('color: #e65100; font-size: 11px;')
                    with ui.row().classes('items-center gap-1 p-1 rounded').style('background: #fce4ec;'):
                        ui.icon('security', size='xs').style('color: #c2185b;')
                        self.buffer_time_label = ui.label("Buffer: --").style('color: #c2185b; font-size: 11px;')
                
                # Relay alert (hidden by default)
                with ui.row().classes('w-full items-center gap-2 mt-2 p-2 rounded').style('background: #fff3e0; border-left: 3px solid #ff9800; display: none;') as self.relay_alert_container:
                    self.relay_alert_icon = ui.icon('notifications_active').classes('text-xl').style('color: #e65100;')
                    self.relay_alert_label = ui.label("").classes('font-bold text-sm').style('color: #bf360c;')
                
                # Bottom row: Battery swap and Drones needed side by side
                with ui.row().classes('w-full gap-2 mt-2'):
                    with ui.row().classes('items-center gap-2 p-2 rounded flex-1').style('background: #e8f5e9;'):
                        ui.icon('battery_charging_full').classes('text-lg').style('color: #2e7d32;')
                        self.reconnect_label = ui.label("None").classes('text-sm').style('color: #2e7d32;')
                    
                    with ui.row().classes('items-center gap-2 p-2 rounded flex-1').style('background: #e3f2fd;'):
                        ui.icon('group').classes('text-lg').style('color: #1565c0;')
                        self.drones_needed_label = ui.label("--").classes('text-sm').style('color: #1565c0;')
            
            ui.separator()
            
            # Drone list container
            ui.label("Connected Drones").classes('text-lg font-bold')
            self.drone_list_container = ui.column().classes('w-full gap-2')
            
            # Note: _refresh_drone_list() is called after map is created in _build_ui()
    
    def _build_right_panel(self):
        """Build the right panel with map and mission control."""
        with ui.card().classes('h-full').style('flex: 3; display: flex; flex-direction: column;'):
            # Map container
            with ui.card().classes('w-full').style('flex: 1; min-height: 400px;'):
                self.map = ui.leaflet(
                    center=self.map_center,
                    zoom=15
                ).style('width: 100%; height: 100%;')
                
                # Map click handler
                self.map.on('map-click', self._on_map_click)
            
            # Control panels - reorganized into 2 rows for better space usage
            # Row 1: Monitoring Point + Trajectory/Speed + Vertical Separation + Mission Buttons
            with ui.row().classes('w-full gap-2 items-stretch mt-2'):
                # Card 1: Monitoring Point + Trajectory (merged)
                with ui.card().classes('p-2').style('flex: 1.4; background: linear-gradient(135deg, #fff5f5 0%, #ffffff 100%); border-left: 3px solid #e53935;'):
                    with ui.row().classes('items-center gap-1 pb-1').style('border-bottom: 1px solid #ffcdd2;'):
                        ui.icon('place', size='sm').style('color: #e53935;')
                        ui.label("Navigation").classes('text-xs font-bold')
                        ui.space()
                        ui.button(icon='push_pin', on_click=self._set_monitoring_point_manual).props('round dense size=xs color=red').tooltip('Set Point')
                        ui.button(icon='delete_outline', on_click=self._clear_monitoring_point_ui).props('round dense flat size=xs').tooltip('Clear')
                        ui.button(icon='cancel', on_click=self._abort_trajectories).props('round dense flat size=xs color=orange').tooltip('Abort Trajectory')
                    # Monitoring Point coordinates
                    with ui.grid(columns=4).classes('w-full gap-1 mt-1'):
                        self.lat_input = ui.input(label='Lat', value='0.0').props('dense outlined').classes('w-full')
                        self.lon_input = ui.input(label='Lon', value='0.0').props('dense outlined').classes('w-full')
                        self.alt_input = ui.input(label='Alt', value='50').props('dense outlined').classes('w-full')
                        self.heading_input = ui.input(label='Hdg', value='0').props('dense outlined').classes('w-full')
                    # Trajectory speed (inline)
                    with ui.row().classes('w-full items-center gap-1 mt-1'):
                        ui.icon('speed', size='xs').style('color: #8e24aa;')
                        ui.label('Speed:').classes('text-xs text-gray-600')
                        self.trajectory_speed_slider = ui.slider(
                            min=1, max=12, value=10, step=1,
                            on_change=self._on_trajectory_speed_change
                        ).props('dense').classes('flex-grow')
                        self.trajectory_speed_label = ui.label('10').classes('text-xs font-mono font-bold')
                
                # Card 2: Vertical Separation (compact)
                with ui.card().classes('p-2').style('flex: 0.7; background: linear-gradient(135deg, #fff8e1 0%, #ffffff 100%); border-left: 3px solid #ff9800;') as self.vertical_sep_card:
                    with ui.row().classes('items-center gap-1 pb-1').style('border-bottom: 1px solid #ffe0b2;'):
                        ui.icon('height', size='sm').style('color: #e65100;')
                        ui.label("Vert. Sep.").classes('text-xs font-bold')
                        ui.space()
                        self.vertical_sep_enabled_switch = ui.switch(value=True, on_change=self._on_vertical_sep_toggle).props('dense size=xs').tooltip('Enable/Disable vertical separation check')
                        self.vertical_sep_status_badge = ui.badge('OK', color='green').classes('text-xs')
                    # Content container (hideable when disabled)
                    with ui.column().classes('w-full gap-1') as self.vertical_sep_content:
                        # Current vs Min in one row
                        with ui.row().classes('w-full items-center justify-around mt-1'):
                            with ui.row().classes('items-center gap-1'):
                                self.vertical_sep_current_label = ui.label('--').classes('text-lg font-bold').style('color: #424242;')
                            ui.icon('compare_arrows', size='sm').style('color: #bdbdbd;')
                            ui.label('5m').classes('text-lg font-bold').style('color: #2e7d32;')
                        # Drones list (no scroll, all visible)
                        self.vertical_sep_drones_list = ui.column().classes('w-full gap-0 mt-1')
                        # Countdown (hidden)
                        with ui.row().classes('w-full items-center gap-1 p-1 rounded').style('background: #ffebee; border: 1px solid #ef5350; display: none;') as self.vertical_sep_countdown_row:
                            ui.icon('warning', size='xs').style('color: #c62828;')
                            self.vertical_sep_countdown_label = ui.label('RTH: --s').classes('text-xs font-bold').style('color: #c62828;')
                            self.vertical_sep_countdown_progress = ui.linear_progress(value=0).props('instant-feedback color=red size=xs').classes('flex-1')
                    # Disabled message (hidden by default)
                    self.vertical_sep_disabled_msg = ui.label('Check disabled').classes('w-full text-center text-sm mt-2').style('color: #9e9e9e; display: none;')
                
                # Card 3: Mission Control (compact)
                with ui.card().classes('p-2').style('flex: 1.3; background: linear-gradient(135deg, #e3f2fd 0%, #ffffff 100%); border-left: 3px solid #1976d2;'):
                    with ui.row().classes('items-center gap-1 pb-1').style('border-bottom: 1px solid #bbdefb;'):
                        ui.icon('flag', size='sm').style('color: #1976d2;')
                        ui.label("Mission").classes('text-xs font-bold')
                        ui.space()
                        self.rosbag_switch = ui.switch('🤖 ROS', value=False, on_change=self._on_rosbag_change).props('dense size=xs').tooltip('Record ROS Bag')
                    # Mode toggle + params in compact grid
                    self.mission_mode_toggle = ui.toggle(
                        {1: '📍 Monitor', 2: '🆓 Free'}, 
                        value=1,
                        on_change=self._on_mission_mode_change
                    ).props('dense spread no-caps size=xs').classes('w-full mt-1')
                    with ui.grid(columns=4).classes('w-full gap-1 mt-1'):
                        self.rth_alt_input = ui.input(label='RTH', value='50').props('dense outlined').classes('w-full')
                        self.safety_buffer_input = ui.input(label='Buf', value='60').props('dense outlined').classes('w-full')
                        self.min_battery_input = ui.input(label='Bat%', value='30').props('dense outlined').classes('w-full')
                        self.min_satellites_input = ui.input(label='Sats', value='8').props('dense outlined').classes('w-full')
                    with ui.row().classes('w-full gap-1 mt-1'):
                        ui.button('Single', icon='play_arrow', on_click=self._start_single_mission).props('color=green no-caps dense size=xs').style('flex: 1;')
                        ui.button('Relay', icon='sync', on_click=self._start_relay_mission).props('color=primary no-caps dense size=xs').style('flex: 1;')
                        ui.button('Stop', icon='stop', on_click=self._stop_mission_ui).props('color=red no-caps dense size=xs').style('flex: 1;')
                
                # Card 4: State Machine (compact)
                with ui.card().classes('p-2').style('flex: 1.5; background: linear-gradient(135deg, #e8f5e9 0%, #ffffff 100%); border-left: 3px solid #43a047;'):
                    with ui.row().classes('items-center gap-1 pb-1').style('border-bottom: 1px solid #c8e6c9;'):
                        ui.icon('account_tree', size='sm').style('color: #43a047;')
                        ui.label("State Machine").classes('text-xs font-bold')
                    self.state_machine_container = ui.column().classes('w-full gap-0 mt-1')
                    with self.state_machine_container:
                        self._build_state_machine_display()
            
            # Bottom row: Event Log and Mission Statistics side by side (or Debug Console when enabled)
            # Normal view container
            with ui.column().classes('w-full gap-2') as self.normal_logs_container:
                with ui.row().classes('w-full gap-2 items-stretch'):
                    # Event Log
                    with ui.card().classes('p-2').style('flex: 1;'):
                        with ui.row().classes('items-center gap-2 pb-1').style('border-bottom: 1px solid #e0e0e0;'):
                            ui.icon('list_alt').classes('text-lg text-primary')
                            ui.label("Event Log").classes('text-sm font-bold')
                        with ui.scroll_area().classes('w-full').style('height: 100px;').props('id=event-log') as self.event_scroll:
                            self.event_log = ui.column().classes('w-full gap-0')
                    
                    # Mission Statistics (hidden when debug console is shown)
                    with ui.card().classes('p-2').style('flex: 1;') as self.mission_stats_card:
                        with ui.row().classes('items-center gap-2 w-full pb-1').style('border-bottom: 1px solid #e0e0e0;'):
                            ui.icon('analytics').classes('text-lg text-primary')
                            ui.label("Mission Statistics").classes('text-sm font-bold')
                            ui.space()
                            ui.button(icon='delete', on_click=self._clear_mission_stats).props('flat dense size=sm').tooltip('Clear')
                        
                        # Header row
                        with ui.row().classes('w-full text-xs font-bold text-gray-500 gap-0 mt-1'):
                            ui.label("Drone").style('flex: 2;')
                            ui.label("#").style('flex: 0.8; text-align: center;')
                            ui.label("Est.").style('flex: 1.2; text-align: center;')
                            ui.label("Travel").style('flex: 1.2; text-align: center;')
                            ui.label("RTH").style('flex: 1.2; text-align: center;')
                        
                        with ui.scroll_area().classes('w-full').style('height: 80px;') as self.mission_stats_scroll:
                            self.mission_stats_container = ui.column().classes('w-full gap-0')
                    
                    # ROS Console (hidden by default, shown in place of Mission Stats)
                    with ui.card().classes('p-2').style('flex: 1; display: none;') as self.debug_console_container:
                        with ui.row().classes('items-center gap-2 w-full pb-1').style('border-bottom: 1px solid #ffcc80;'):
                            ui.icon('terminal').classes('text-lg').style('color: #e65100;')
                            ui.label("ROS Console").classes('text-sm font-bold').style('color: #e65100;')
                            ui.space()
                            ui.button(icon='delete', on_click=self._clear_debug_console).props('flat dense size=sm').tooltip('Clear console')
                        
                        with ui.scroll_area().classes('w-full').style('height: 100px; background: #1e1e1e; border-radius: 4px;') as self.debug_scroll:
                            self.debug_console = ui.column().classes('w-full gap-0 p-2')
                
                # RTH Prediction Debug Panel (expansion)
                with ui.expansion("RTH Prediction Debug", icon='bug_report').classes('w-full').style('background: #fff8e1;') as self.rth_debug_expansion:
                    with ui.card().classes('w-full p-2').style('background: #fffde7;') as self.rth_debug_container:
                        # Formula explanation
                        with ui.row().classes('w-full items-center gap-2 pb-2').style('border-bottom: 1px solid #ffe082;'):
                            ui.icon('calculate').style('color: #f57c00;')
                            ui.label("Countdown = Predicted RTH - Travel Time - Buffer").classes('text-xs font-mono').style('color: #e65100;')
                        
                        # Debug info grid
                        with ui.grid(columns=2).classes('w-full gap-x-4 gap-y-1 mt-2'):
                            # Left column - Battery & Threshold
                            ui.label("Current Battery:").classes('text-xs font-bold')
                            self.rth_debug_battery = ui.label("--").classes('text-xs font-mono')
                            
                            ui.label("Batt Needed (now):").classes('text-xs font-bold')
                            self.rth_debug_batt_needed = ui.label("--").classes('text-xs font-mono')
                            
                            ui.label("Batt Needed (MAX):").classes('text-xs font-bold').style('color: #d32f2f;')
                            self.rth_debug_batt_max = ui.label("--").classes('text-xs font-mono font-bold').style('color: #d32f2f;')
                            
                            ui.label("RTH Threshold:").classes('text-xs font-bold').tooltip('MAX BattNeeded + 2%')
                            self.rth_debug_threshold = ui.label("--").classes('text-xs font-mono')
                            
                            ui.label("Drain Rate:").classes('text-xs font-bold')
                            self.rth_debug_drain = ui.label("--").classes('text-xs font-mono')
                            
                            ui.label("Data Points:").classes('text-xs font-bold')
                            self.rth_debug_points = ui.label("--").classes('text-xs font-mono')
                            
                            ui.label("Monitoring Time:").classes('text-xs font-bold')
                            self.rth_debug_elapsed = ui.label("--").classes('text-xs font-mono')
                        
                        ui.separator().classes('my-2')
                        
                        # Comparison section
                        with ui.row().classes('w-full gap-4'):
                            with ui.column().classes('flex-1'):
                                ui.label("Predicted RTH:").classes('text-xs font-bold').style('color: #1565c0;')
                                self.rth_debug_predicted = ui.label("--").classes('text-lg font-bold font-mono').style('color: #1565c0;')
                            with ui.column().classes('flex-1'):
                                ui.label("DJI Flight Time:").classes('text-xs font-bold').style('color: #7b1fa2;')
                                self.rth_debug_dji = ui.label("--").classes('text-lg font-bold font-mono').style('color: #7b1fa2;')
                            with ui.column().classes('flex-1'):
                                ui.label("Difference:").classes('text-xs font-bold').style('color: #d32f2f;')
                                self.rth_debug_diff = ui.label("--").classes('text-lg font-bold font-mono').style('color: #d32f2f;')
                        
                        # Regression details
                        ui.separator().classes('my-2')
                        with ui.row().classes('w-full gap-2'):
                            ui.label("Regression:").classes('text-xs font-bold')
                            self.rth_debug_regression = ui.label("battery(t) = slope × t + intercept").classes('text-xs font-mono').style('color: #616161;')
                        
                        # Live regression chart
                        ui.separator().classes('my-2')
                        with ui.row().classes('w-full items-center gap-2'):
                            ui.icon('show_chart').style('color: #1565c0;')
                            ui.label("Live Battery Regression").classes('text-xs font-bold')
                        
                        # Chart using ECharts
                        self.rth_chart = ui.echart({
                            'animation': False,
                            'grid': {'left': '12%', 'right': '5%', 'top': '15%', 'bottom': '18%'},
                            'legend': {
                                'show': True,
                                'top': 0,
                                'textStyle': {'fontSize': 9},
                                'itemWidth': 12,
                                'itemHeight': 8
                            },
                            'xAxis': {
                                'type': 'value',
                                'name': 'Time (min)',
                                'nameLocation': 'middle',
                                'nameGap': 22,
                                'min': 0,
                                'axisLabel': {'fontSize': 9}
                            },
                            'yAxis': {
                                'type': 'value',
                                'name': 'Battery %',
                                'nameLocation': 'middle',
                                'nameGap': 32,
                                'min': 0,
                                'max': 100,
                                'axisLabel': {'fontSize': 9}
                            },
                            'series': [
                                {
                                    'name': 'Battery',
                                    'type': 'scatter',
                                    'data': [],
                                    'symbolSize': 5,
                                    'itemStyle': {'color': '#1976d2'}
                                },
                                {
                                    'name': 'Regression',
                                    'type': 'line',
                                    'data': [],
                                    'lineStyle': {'color': '#4caf50', 'width': 2},
                                    'symbol': 'none'
                                },
                                {
                                    'name': 'MAX Threshold',
                                    'type': 'line',
                                    'data': [],
                                    'lineStyle': {'color': '#f44336', 'width': 2, 'type': 'dashed'},
                                    'symbol': 'none'
                                },
                                {
                                    'name': 'RTH Point',
                                    'type': 'scatter',
                                    'data': [],
                                    'symbolSize': 14,
                                    'symbol': 'triangle',
                                    'itemStyle': {'color': '#f44336'}
                                },
                                {
                                    'name': 'Now',
                                    'type': 'line',
                                    'data': [],
                                    'lineStyle': {'color': '#ff9800', 'width': 2, 'type': 'dotted'},
                                    'symbol': 'none'
                                },
                                {
                                    'name': 'Current Batt',
                                    'type': 'scatter',
                                    'data': [],
                                    'symbolSize': 10,
                                    'symbol': 'circle',
                                    'itemStyle': {'color': '#ff9800', 'borderColor': '#fff', 'borderWidth': 2}
                                }
                            ],
                            'tooltip': {
                                'trigger': 'item',
                                'formatter': '{a}: {c}'
                            }
                        }).classes('w-full').style('height: 200px;')

    def _build_drone_card(self, namespace: str, drone: DroneData):
        """Build a compact card for a single drone."""
        # Use the drone's assigned color (stored in DroneData)
        color = drone.color if hasattr(drone, 'color') and drone.color else self.drone_colors[0]
        
        with ui.card().classes('drone-card w-full p-3') as card:
            self.drone_cards[namespace] = card
            self.drone_labels[namespace] = {}
            self.drone_buttons[namespace] = {}
            
            # Header: color dot, name, state, battery
            with ui.row().classes('w-full items-center gap-3'):
                ui.icon('circle').style(f'color: {color}; font-size: 20px')
                ui.label(f"{namespace}").classes('font-bold text-xl').style('flex: 1')
                # Manual flight indicator (shown when NOT in virtual_stick mode)
                manual_badge = ui.badge('🎮 MANUAL', color='orange').classes('ml-2').tooltip('Pilot has manual control (not virtual stick)')
                manual_badge.style('display: none')  # Hidden by default
                self.drone_labels[namespace]['manual_indicator'] = manual_badge
                self.drone_labels[namespace]['state'] = ui.label(f"{drone.state.value}").classes('text-base px-3 py-1 rounded bg-gray-200')
                with ui.row().classes('items-center gap-1'):
                    ui.icon('battery_full').style('font-size: 28px')
                    self.drone_labels[namespace]['battery'] = ui.label(f"{drone.battery_level:.0f}%").classes('text-lg font-bold')
            
            # Telemetry + Gimbal layout: telemetry on left (one line), gimbal centered on right
            with ui.row().classes('w-full items-center gap-4 mt-2'):
                # Left: All telemetry stats in one row
                with ui.row().classes('items-center gap-5 text-base text-gray-700').style('flex: 1'):
                    with ui.row().classes('items-center gap-0'):
                        ui.icon('height').style('font-size: 28px')
                        self.drone_labels[namespace]['altitude'] = ui.label(f"{drone.altitude:.0f}m").classes('text-lg')
                    with ui.row().classes('items-center gap-0'):
                        ui.icon('speed').style('font-size: 28px')
                        self.drone_labels[namespace]['speed'] = ui.label(f"{drone.speed:.1f}m/s").classes('text-lg')
                    with ui.row().classes('items-center gap-0'):
                        ui.icon('satellite_alt').style('font-size: 28px')
                        self.drone_labels[namespace]['satellites'] = ui.label(f"{drone.satellite_count}").classes('text-lg')
                    with ui.row().classes('items-center gap-0').tooltip('Remaining flight time'):
                        ui.icon('hourglass_bottom').style('font-size: 28px')
                        self.drone_labels[namespace]['flight_time'] = ui.label("--:--").classes('text-lg')
                    with ui.row().classes('items-center gap-1').tooltip('Recording Status'):
                        rec_icon = ui.icon('videocam').style('font-size: 28px; color: #c62828; animation: blink 1s infinite;' if drone.is_recording else 'font-size: 28px; color: #bdbdbd;')
                        self.drone_labels[namespace]['recording'] = ui.label("REC" if drone.is_recording else "OFF").classes('text-lg font-bold').style('color: #c62828;' if drone.is_recording else 'color: #bdbdbd;')
                        self.drone_labels[namespace]['recording_icon'] = rec_icon
                
                # Right: Gimbal knob (centered vertically and horizontally)
                with ui.column().classes('items-center justify-center').style('min-width: 90px'):
                    gimbal_knob = ui.knob(min=-90, max=0, value=0, step=5, show_value=True).props('size="80px" thickness=0.20 color="primary" font-size="16px"').tooltip('Gimbal Pitch')
                    
                    def update_gimbal(e, ns=namespace):
                        val = float(e.args)
                        self.send_gimbal_pitch(ns, val)
                    gimbal_knob.on('update:model-value', update_gimbal)
            
            # RTH Predictor row (only visible when active)
            with ui.row().classes('w-full items-center gap-2 mt-1') as rth_row:
                rth_row.style('display: none')  # Hidden by default
                self.drone_labels[namespace]['rth_predictor_row'] = rth_row
                ui.icon('analytics').style('font-size: 20px; color: #1976d2')
                ui.label('RTH Predictor:').classes('text-sm text-gray-600')
                with ui.row().classes('items-center gap-1').tooltip('Predicted time until RTH triggers'):
                    ui.icon('timer').style('font-size: 18px; color: #1976d2')
                    self.drone_labels[namespace]['rth_predicted'] = ui.label('--:--').classes('text-sm font-bold').style('color: #1976d2')
                with ui.row().classes('items-center gap-1').tooltip('Battery drain rate'):
                    ui.icon('trending_down').style('font-size: 18px; color: #f57c00')
                    self.drone_labels[namespace]['rth_drain_rate'] = ui.label('--%/min').classes('text-sm').style('color: #f57c00')
                with ui.row().classes('items-center gap-1').tooltip('Data points collected'):
                    ui.icon('show_chart').style('font-size: 18px; color: #388e3c')
                    self.drone_labels[namespace]['rth_data_points'] = ui.label('0 pts').classes('text-sm').style('color: #388e3c')
            
            # Hidden position label (for data, not display)
            self.drone_labels[namespace]['position'] = ui.label().classes('hidden')
            
            # All controls in one row with bigger buttons
            with ui.row().classes('w-full gap-1 mt-2'):
                ui.button(icon='flight_takeoff', on_click=lambda ns=namespace: self.send_takeoff(ns)).props('flat').tooltip('Take Off')
                ui.button(icon='flight_land', on_click=lambda ns=namespace: self.send_land(ns)).props('flat').tooltip('Land')
                ui.button(icon='home', on_click=lambda ns=namespace: self.send_rth(ns)).props('flat').tooltip('Return to Home')
                ui.button(icon='warning', on_click=lambda ns=namespace: self.send_abort_mission(ns)).props('flat color=negative').tooltip('Abort Mission')
                ui.button(icon='videocam', on_click=lambda ns=namespace: self.send_start_recording(ns)).props('flat color=red').tooltip('Start Recording')
                ui.button(icon='stop', on_click=lambda ns=namespace: self.send_stop_recording(ns)).props('flat').tooltip('Stop Recording')
                ui.button(icon='push_pin', on_click=lambda ns=namespace: self._pin_drone_location(ns)).props('flat color=purple').tooltip('📍 Pin current location (Free Flight → Monitoring Point)')
                self.drone_buttons[namespace]['ready'] = ui.button(icon='check_circle', on_click=lambda ns=namespace: self._mark_drone_ready(ns)).props('flat color=green').tooltip('Mark as Ready (battery swapped)')
                ui.button(icon='link_off', on_click=lambda ns=namespace: self._disconnect_drone_ui(ns)).props('flat color=negative').tooltip('Disconnect')
            
            # Create arrow on map
            self._add_drone_arrow(namespace, drone.latitude, drone.longitude, drone.heading, color)
    
    def _add_drone_arrow(self, namespace: str, lat: float, lon: float, heading: float, color: str = '#FF6B6B'):
        """Add a drone arrow to the map."""
        if self.map and namespace not in self.drone_arrows:
            try:
                arrow = Arrow(
                    self.map, namespace, lat, lon, heading,
                    drones_arrows=self.drone_arrows,
                    color=color
                )
                self.drone_arrows[namespace] = arrow
                arrow._place_arrow()
            except ValueError as e:
                self.get_logger().warning(f"Could not create arrow: {e}")
    
    # ========================================================================
    # UI EVENT HANDLERS
    # ========================================================================
    
    def _get_next_drone_name(self) -> str:
        """Get the next suggested drone name based on existing drones."""
        # Find the highest drone number currently in use
        max_num = 0
        for ns in self.drones.keys():
            # Try to extract number from drone_X format
            if ns.startswith('drone_'):
                try:
                    num = int(ns.replace('drone_', ''))
                    max_num = max(max_num, num)
                except ValueError:
                    pass
        return f'drone_{max_num + 1}'
    
    def _increment_drone_name(self):
        """Increment the drone name in the input field."""
        current = self.namespace_input.value.strip() if self.namespace_input.value else ''
        
        # Try to extract and increment the number
        if current.startswith('drone_'):
            try:
                num = int(current.replace('drone_', ''))
                self.namespace_input.value = f'drone_{num + 1}'
                return
            except ValueError:
                pass
        
        # If current value doesn't match pattern, use next available
        self.namespace_input.value = self._get_next_drone_name()
    
    def _validate_ip(self, value: str) -> bool:
        """Validate IP address format. Returns True if valid."""
        if not value:
            return True  # Empty is OK (will be caught in connect)
        value = value.strip()
        parts = value.split('.')
        if len(parts) != 4:
            return False
        for part in parts:
            try:
                num = int(part)
                if num < 0 or num > 255:
                    return False
            except ValueError:
                return False
        return True
    
    def _connect_drone_ui(self):
        """Handle drone connection request from UI."""
        ip = self.ip_input.value.strip() if self.ip_input.value else ''
        if not ip:
            ui.notify('Please enter an IP address', type='warning')
            return
        if not self._validate_ip(ip):
            ui.notify('Please enter a valid IP address', type='warning')
            return
        
        namespace = self.namespace_input.value.strip() or None
        
        if self.connect_drone(ip, namespace):
            ui.notify(f'Drone connected at {ip}', type='positive')
            self.ip_input.value = ''
            # Auto-suggest next drone name
            self.namespace_input.value = self._get_next_drone_name()
        else:
            ui.notify('Failed to connect drone', type='negative')
    
    def _disconnect_drone_ui(self, namespace: str):
        """Handle drone disconnection request from UI."""
        if self.disconnect_drone(namespace):
            ui.notify(f'{namespace} disconnected', type='positive')
            self._refresh_drone_list()
        else:
            ui.notify(f'Cannot disconnect {namespace} (may be in flight)', type='warning')
    
    def _pin_drone_location(self, namespace: str):
        """Pin drone's current location as monitoring point and switch to Monitoring Point mode."""
        if not self.mission_controller:
            ui.notify('No mission controller active', type='warning')
            return
        
        if self.mission_controller.pin_drone_location(namespace):
            # Update mission mode toggle in UI
            if hasattr(self, 'mission_mode_toggle') and self.mission_mode_toggle:
                self.mission_mode_toggle.value = 1  # Switch to "Monitor" mode
            
            # Update monitoring point on map
            lat, lon, alt = 0.0, 0.0, 0.0
            if namespace in self.drones:
                lat = self.drones[namespace].latitude
                lon = self.drones[namespace].longitude
                alt = self.drones[namespace].altitude
                heading = self.drones[namespace].heading
                self.set_monitoring_point(lat, lon, alt, heading, source=f"pinned:{namespace}")
            
            ui.notify(f'📍 Location pinned from {namespace}! Mode switched to Monitoring Point.', type='positive')
            self._emit_log(f"[PIN] Location pinned from {namespace}: ({lat:.6f}, {lon:.6f}, {alt:.1f}m)")
        else:
            ui.notify(f'Failed to pin location from {namespace}', type='warning')
    
    def _mark_drone_ready(self, namespace: str):
        """Mark a drone as ready (IDLE) for next relay cycle."""
        if not self.mission_controller:
            ui.notify('No mission controller active', type='warning')
            return
        
        if self.mission_controller.mark_drone_ready(namespace):
            ui.notify(f'{namespace} marked as ready', type='positive')
            # Emit state update to refresh GUI
            self.drone_state_update.emit({
                'namespace': namespace,
                'state': DroneState.IDLE
            })
        else:
            ui.notify(f'{namespace} cannot be marked ready (not in COMPLETED state)', type='warning')
    
    # ========================================================================
    # TRAJECTORY TRACKING
    # ========================================================================
    
    def _start_drone_trajectory(self, namespace: str):
        """Start tracking a new trajectory for a drone (called on takeoff)."""
        # Clear any existing trajectory
        self.drone_trajectories[namespace] = []
        
        # Remove existing polyline if any
        if namespace in self.drone_trajectory_lines and self.drone_trajectory_lines[namespace]:
            try:
                self.map.remove_layer(self.drone_trajectory_lines[namespace])
            except:
                pass
            self.drone_trajectory_lines[namespace] = None
        
        self._emit_log(f"[{namespace}] Trajectory tracking started")
    
    def _update_drone_trajectory(self, namespace: str, lat: float, lon: float):
        """Add a point to the drone's trajectory and update the map."""
        if namespace not in self.drone_trajectories:
            self.drone_trajectories[namespace] = []
        
        trajectory = self.drone_trajectories[namespace]
        
        # Only add point if it's far enough from the last one (at least 2 meters)
        # This reduces lag by limiting the number of points
        MIN_DISTANCE_METERS = 2.0
        MAX_POINTS = 500  # Limit total trajectory points
        
        should_add = False
        if not trajectory:
            should_add = True
        else:
            last_lat, last_lon = trajectory[-1]
            # Quick distance check (approximate, good enough for filtering)
            lat_diff = abs(lat - last_lat) * 111320  # meters per degree lat
            lon_diff = abs(lon - last_lon) * 111320 * abs(math.cos(math.radians(lat)))
            distance = math.sqrt(lat_diff**2 + lon_diff**2)
            should_add = distance >= MIN_DISTANCE_METERS
        
        if should_add:
            trajectory.append((lat, lon))
            
            # Trim old points if too many
            if len(trajectory) > MAX_POINTS:
                trajectory[:] = trajectory[-MAX_POINTS:]
            
            # Update polyline on map (need at least 2 points)
            if len(trajectory) >= 2 and self.map:
                # Get drone color
                color = '#FF6B6B'  # default
                if namespace in self.drone_arrows:
                    color = self.drone_arrows[namespace].color
                
                # Remove old polyline
                if namespace in self.drone_trajectory_lines and self.drone_trajectory_lines[namespace]:
                    try:
                        self.map.remove_layer(self.drone_trajectory_lines[namespace])
                    except:
                        pass
                
                # Create new polyline with all points
                self.drone_trajectory_lines[namespace] = self.map.generic_layer(
                    name='polyline',
                    args=[
                        [[p[0], p[1]] for p in trajectory],
                        {'color': color, 'weight': 3, 'opacity': 0.8}
                    ]
                )
    
    def _fade_and_clear_trajectory(self, namespace: str):
        """Fade out and clear a drone's trajectory (called on landing)."""
        if namespace not in self.drone_trajectory_lines or not self.drone_trajectory_lines[namespace]:
            # No trajectory to clear
            if namespace in self.drone_trajectories:
                self.drone_trajectories[namespace] = []
            return
        
        self._emit_log(f"[{namespace}] Trajectory tracking stopped - fading out")
        
        # Fade out over 3 seconds using opacity steps
        polyline = self.drone_trajectory_lines[namespace]
        trajectory = self.drone_trajectories.get(namespace, [])
        
        if not trajectory or len(trajectory) < 2:
            # Nothing to fade, just clear
            try:
                self.map.remove_layer(polyline)
            except:
                pass
            self.drone_trajectory_lines[namespace] = None
            self.drone_trajectories[namespace] = []
            return
        
        # Get drone color
        color = '#FF6B6B'
        if namespace in self.drone_arrows:
            color = self.drone_arrows[namespace].color
        
        # Create fade-out animation using timers
        def fade_step(opacity: float):
            if namespace not in self.drone_trajectory_lines:
                return
            
            try:
                # Remove old polyline
                if self.drone_trajectory_lines[namespace]:
                    self.map.remove_layer(self.drone_trajectory_lines[namespace])
                
                if opacity > 0:
                    # Create new polyline with reduced opacity
                    self.drone_trajectory_lines[namespace] = self.map.generic_layer(
                        name='polyline',
                        args=[
                            [[p[0], p[1]] for p in trajectory],
                            {'color': color, 'weight': 3, 'opacity': opacity}
                        ]
                    )
                else:
                    # Final step - clear everything
                    self.drone_trajectory_lines[namespace] = None
                    self.drone_trajectories[namespace] = []
            except:
                pass
        
        # Schedule fade steps (3 seconds total, 6 steps)
        ui.timer(0.5, lambda: fade_step(0.6), once=True)
        ui.timer(1.0, lambda: fade_step(0.4), once=True)
        ui.timer(1.5, lambda: fade_step(0.3), once=True)
        ui.timer(2.0, lambda: fade_step(0.2), once=True)
        ui.timer(2.5, lambda: fade_step(0.1), once=True)
        ui.timer(3.0, lambda: fade_step(0.0), once=True)
    
    def _refresh_drone_list(self):
        """Refresh the drone list display."""
        if self.drone_list_container is None:
            return
        
        # Clear the tracking dictionaries for cards being removed
        self.drone_cards.clear()
        self.drone_labels.clear()
        self.drone_buttons.clear()
        # Note: Don't clear drone_arrows here - they're on the map
            
        self.drone_list_container.clear()
        
        if not self.drones:
            with self.drone_list_container:
                ui.label("No drones connected").classes('text-gray-500 italic')
        else:
            for namespace, drone in self.drones.items():
                with self.drone_list_container:
                    self._build_drone_card(namespace, drone)
        
        # Update drones needed estimate
        self._update_drones_needed()
    
    def _update_drones_needed(self):
        """Update the estimate of drones needed for continuous coverage."""
        if self.drones_needed_label and self.monitoring_point.is_set:
            result = self.mission_controller.calculate_drones_needed()
            simultaneous, total, travel_time, distance, has_actual_data = result
            
            # Format travel time
            travel_min = int(travel_time // 60)
            travel_sec = int(travel_time % 60)
            distance_km = distance / 1000
            
            connected = len(self.drones)
            
            # Check if we have a valid distance (not fallback 3km/5min)
            is_fallback = (distance == 3000 and travel_time == 300)
            
            # Indicator for estimate source
            source_indicator = "📊" if has_actual_data else "~"  # 📊 = actual data, ~ = estimated
            
            if is_fallback:
                self.drones_needed_label.text = f"Waiting for drone GPS... ({connected} connected)"
                self.drones_needed_label.style('color: white;')
            elif simultaneous == float('inf'):
                self.drones_needed_label.text = f"Point too far! ({distance_km:.1f}km, {travel_min}:{travel_sec:02d} travel)"
                self.drones_needed_label.style('color: #c62828;')  # error red
            else:
                # Show simultaneous (flying) and total (rotation) separately
                info = f"{source_indicator}{simultaneous} flying, {total} total ({distance_km:.1f}km, {travel_min}min)"
                if connected >= total:
                    self.drones_needed_label.text = f"{info} ✓ {connected} ready"
                    self.drones_needed_label.style('color: #2e7d32;')  # success green
                elif connected >= simultaneous:
                    self.drones_needed_label.text = f"{info} ⚠ {connected} connected (need {total})"
                    self.drones_needed_label.style('color: #ef6c00;')  # warning orange
                else:
                    self.drones_needed_label.text = f"{info} ❌ only {connected} (need {simultaneous}+ flying)"
                    self.drones_needed_label.style('color: #c62828;')  # error red
    
    def _on_map_click(self, e):
        """Handle map click for setting monitoring point."""
        try:
            if 'latlng' in e.args:
                lat = e.args['latlng']['lat']
                lon = e.args['latlng']['lng']
            elif 'lat' in e.args:
                lat = e.args['lat']
                lon = e.args['lng']
            else:
                return
        except (KeyError, TypeError):
            return
        
        try:
            alt = float(self.alt_input.value)
        except ValueError:
            alt = 50.0
        
        try:
            heading = float(self.heading_input.value) if self.heading_input else 0.0
            heading = heading % 360
        except ValueError:
            heading = 0.0
        
        self.set_monitoring_point(lat, lon, alt, heading, source="map")
        
        self.lat_input.value = f"{lat:.6f}"
        self.lon_input.value = f"{lon:.6f}"
        
        ui.notify(f'Monitoring point set', type='positive')
        self._update_drones_needed()
    
    def _set_monitoring_point_manual(self):
        """Set monitoring point from manual input."""
        try:
            lat = float(self.lat_input.value)
            lon = float(self.lon_input.value)
            alt = float(self.alt_input.value)
            heading = float(self.heading_input.value) if self.heading_input else 0.0
            
            # Normalize heading to 0-360
            heading = heading % 360
            
            self.set_monitoring_point(lat, lon, alt, heading, source="manual")
            ui.notify(f'Monitoring point set (heading={heading:.0f}°)', type='positive')
            self._update_drones_needed()
        except ValueError:
            ui.notify('Invalid coordinates', type='warning')
    
    def _clear_monitoring_point_ui(self):
        """Clear the monitoring point from UI."""
        self.clear_monitoring_point()
        self.lat_input.value = '0.0'
        self.lon_input.value = '0.0'
        ui.notify('Monitoring point cleared', type='info')
    
    def _start_single_mission(self):
        """Start a single drone monitoring mission."""
        from groundstation.mission_controller import MissionMode
        
        # Check if monitoring point is required (not in Free Flight mode)
        is_free_flight = (hasattr(self, 'mission_controller') and 
                          self.mission_controller.mission_mode == MissionMode.FREE_FLIGHT)
        
        if not self.monitoring_point.is_set and not is_free_flight:
            ui.notify('Please set a monitoring point first (or use Free Flight mode)', type='warning')
            return
        
        if not self.drones:
            ui.notify('No drones connected', type='warning')
            return
        
        drone_ns = list(self.drones.keys())[0]
        
        # Show takeoff confirmation dialog using run_javascript to stay in UI context
        self._pending_takeoff_drone = drone_ns
        self._pending_single_mission = True
        self._show_takeoff_dialog_sync(drone_ns)
    
    def _do_start_single_mission(self, drone_ns: str):
        """Actually start the single mission after confirmation."""
        try:
            rth_alt = float(self.rth_alt_input.value)
        except ValueError:
            rth_alt = 50.0
        
        try:
            min_battery = float(self.min_battery_input.value)
            self.mission_controller.config.min_battery_to_launch = min_battery
        except ValueError:
            pass
        
        try:
            min_sats = int(self.min_satellites_input.value)
            self.mission_controller.config.min_satellites = min_sats
        except ValueError:
            pass
        
        if self.start_monitoring_mission(drone_ns, rth_alt):
            # Reset mission timer (will start when drone reaches monitoring point)
            self._stop_mission_timer()
            self._mission_start_time = None
            if self.mission_timer_label:
                self.mission_timer_label.text = "00:00:00"
            
            # Build state machine display for drones in mission
            self._build_state_machine_display()
            
            self.mission_status_label.text = "Single Drone"
            self.mission_status_label.style('background: #e8f5e9; color: #2e7d32;')  # green
            self.active_drone_label.text = drone_ns
            self.active_drone_label.style('background: #e8f5e9; color: #2e7d32;')
            ui.notify(f'Mission started', type='positive')
            self._emit_log(f"Single mission started with {drone_ns}")
        else:
            ui.notify('Failed to start mission', type='negative')
    
    def _start_relay_mission(self):
        """Start a relay mission with all connected drones."""
        from groundstation.mission_controller import MissionMode
        
        # Check if monitoring point is required (not in Free Flight mode)
        is_free_flight = (hasattr(self, 'mission_controller') and 
                          self.mission_controller.mission_mode == MissionMode.FREE_FLIGHT)
        
        if not self.monitoring_point.is_set and not is_free_flight:
            ui.notify('Please set a monitoring point first (or use Free Flight mode)', type='warning')
            return
        
        if len(self.drones) < 1:
            ui.notify('No drones connected', type='warning')
            return
        
        # Check if point is reachable (skip in Free Flight mode)
        if not is_free_flight:
            result = self.mission_controller.calculate_drones_needed()
            simultaneous, total, travel_time, distance, has_actual_data = result
            connected = len(self.drones)
            
            if simultaneous == float('inf'):
                ui.notify(f'Point too far! ({distance/1000:.1f}km) - cannot maintain coverage', type='negative')
                return
            
            # Info message about drone requirements (non-blocking)
            if connected < simultaneous:
                ui.notify(
                    f'Need {simultaneous} drones flying simultaneously. Connect more drones soon!',
                    type='warning',
                    timeout=5000
                )
            elif connected < total:
                ui.notify(
                    f'Starting with {connected} drones. {total} recommended for full rotation.',
                    type='info',
                    timeout=3000
                )
            
            travel_time_val = travel_time
            distance_val = distance
        else:
            # Free Flight mode - no distance calculation needed
            travel_time_val = 0
            distance_val = 0
        
        drone_list = list(self.drones.keys())
        first_drone = drone_list[0]
        
        # Store pending relay mission data and show confirmation dialog
        self._pending_takeoff_drone = first_drone
        self._pending_relay_mission = True
        self._pending_relay_data = {'drone_list': drone_list, 'travel_time': travel_time_val, 'distance': distance_val}
        self._show_takeoff_dialog_sync(first_drone)
    
    def _do_start_relay_mission(self, drone_list: list, travel_time: float, distance: float):
        """Actually start the relay mission after confirmation."""
        try:
            rth_alt = float(self.rth_alt_input.value)
        except ValueError:
            rth_alt = 50.0
        
        try:
            buffer = float(self.safety_buffer_input.value)
            self.mission_controller.config.safety_buffer_seconds = buffer
        except ValueError:
            pass
        
        try:
            min_battery = float(self.min_battery_input.value)
            self.mission_controller.config.min_battery_to_launch = min_battery
        except ValueError:
            pass
        
        try:
            min_sats = int(self.min_satellites_input.value)
            self.mission_controller.config.min_satellites = min_sats
        except ValueError:
            pass
        
        if self.start_relay_mission(drone_list, rth_alt):
            # Reset mission timer (will start when drone reaches monitoring point)
            self._stop_mission_timer()
            self._mission_start_time = None
            if self.mission_timer_label:
                self.mission_timer_label.text = "00:00:00"
            
            # Build state machine display for drones in mission
            self._build_state_machine_display()
            
            travel_min = int(travel_time // 60)
            self.mission_status_label.text = f"Relay ({len(drone_list)})"
            self.mission_status_label.style('background: #e3f2fd; color: #1565c0;')  # blue
            self.active_drone_label.text = drone_list[0]
            self.active_drone_label.style('background: #e3f2fd; color: #1565c0;')
            ui.notify(f'Relay mission started with {len(drone_list)} drones (~{travel_min}min to point)', type='positive')
            self._emit_log(f"Relay mission started: {', '.join(drone_list)} - {distance/1000:.1f}km to point")
        else:
            ui.notify('Failed to start relay mission', type='negative')
    
    def _force_swap_clicked(self):
        """Handle Force Swap button click - manually trigger relay swap."""
        if not self.mission_controller:
            ui.notify('Mission controller not available', type='warning')
            return
        
        # Check if a swap is already in progress
        if self.mission_controller.is_manual_swap_active():
            ui.notify('Swap already in progress', type='warning')
            return
        
        # Get the next drone to take off
        next_drone = self.mission_controller.get_next_drone()
        if not next_drone:
            ui.notify('No next drone available for swap', type='warning')
            return
        
        # Show takeoff confirmation dialog
        self._show_force_swap_dialog(next_drone)
    
    def _show_force_swap_dialog(self, drone_name: str):
        """Show takeoff confirmation dialog for force swap."""
        with ui.dialog() as dialog, ui.card().classes('p-4'):
            ui.label(f'🚁 Force Swap - Takeoff Confirmation').classes('text-xl font-bold text-orange-700')
            ui.separator()
            ui.label(f'Drone "{drone_name}" will take off for relay swap.').classes('text-lg mt-2')
            ui.label('This will immediately launch the next drone in sequence.').classes('text-sm text-gray-600 mt-1')
            
            def on_confirm():
                dialog.close()
                # Play takeoff confirmation sound
                ui.run_javascript('''
                    var audio = new Audio("/static/take_off.mp3");
                    audio.play().catch(function(e) { console.log("Audio play failed:", e); });
                ''')
                # Execute the force swap
                self._do_force_swap()
            
            def on_cancel():
                dialog.close()
                ui.notify('Force swap cancelled', type='info')
                self._emit_log("[SWAP] Force swap cancelled by user")
            
            with ui.row().classes('w-full justify-end gap-2 mt-4'):
                ui.button('Cancel', on_click=on_cancel, color='red').props('flat')
                ui.button('Confirm Takeoff', on_click=on_confirm, color='primary')
        
        dialog.open()
    
    def _do_force_swap(self):
        """Execute the force swap after confirmation."""
        # Try to force the swap
        success, message = self.mission_controller.force_relay_swap()
        
        if success:
            self._manual_swap_active = True
            ui.notify(message, type='positive')
            self._emit_log(f"[SWAP] {message}")
            
            # Hide countdown, show "SWAPPING" status
            if self.countdown_container:
                self.countdown_container.style('display: none;')
            if self.force_swap_button:
                self.force_swap_button.props('disabled')
                self.force_swap_button.text = 'Swapping...'
        else:
            ui.notify(message, type='warning')
            self._emit_log(f"[SWAP] Failed: {message}")
    
    def _start_mission_timer(self):
        """Start the mission elapsed time timer."""
        if self._mission_timer_task is None:
            self._mission_timer_task = ui.timer(1.0, self._update_mission_timer)
    
    def _stop_mission_timer(self):
        """Stop the mission elapsed time timer."""
        if self._mission_timer_task is not None:
            self._mission_timer_task.cancel()
            self._mission_timer_task = None
    
    def _update_mission_timer(self):
        """Update the mission timer display."""
        if self._mission_start_time is not None and self.mission_timer_label:
            elapsed = time.time() - self._mission_start_time
            hours = int(elapsed // 3600)
            minutes = int((elapsed % 3600) // 60)
            seconds = int(elapsed % 60)
            self.mission_timer_label.text = f"{hours:02d}:{minutes:02d}:{seconds:02d}"
    
    def _stop_mission_ui(self):
        """Stop the current mission from UI."""
        self.stop_mission()
        self._stop_mission_timer()
        # Keep the final time displayed, just stop updating
        self.mission_status_label.text = "Stopped"
        self.mission_status_label.style('background: #ffebee; color: #c62828;')  # red
        self.countdown_label.text = "--:--"
        self.active_drone_label.text = "--"
        self.active_drone_label.style('background: #e0e0e0; color: #424242;')
        ui.notify('Mission stopped', type='info')
        self._emit_log("Mission stopped - drones returning home")
    
    def _on_rosbag_change(self, e):
        """Handle ROS bag recording toggle change."""
        import subprocess
        import os
        from datetime import datetime
        
        enabled = e.value
        
        if enabled:
            # Start recording
            try:
                # Create rosbags directory if it doesn't exist
                os.makedirs(self._rosbag_dir, exist_ok=True)
                
                # Generate bag name with timestamp
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                bag_name = f"wildperpetua_{timestamp}"
                bag_path = os.path.join(self._rosbag_dir, bag_name)
                
                # Start ros2 bag record in background
                self._rosbag_process = subprocess.Popen(
                    ['ros2', 'bag', 'record', '-a', '-o', bag_path],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE
                )
                self._rosbag_recording = True
                
                ui.notify(f'🎥 Recording started: {bag_name}', type='positive', timeout=5000)
                self._emit_log(f"[ROSBAG] Recording started - saving to src/rosbags/{bag_name}")
                
            except Exception as ex:
                ui.notify(f'Failed to start recording: {ex}', type='negative')
                self._emit_log(f"[ROSBAG] Error: {ex}")
                self.rosbag_switch.value = False
                self._rosbag_recording = False
        else:
            # Stop recording
            if self._rosbag_process:
                try:
                    self._rosbag_process.terminate()
                    self._rosbag_process.wait(timeout=5)
                    self._rosbag_process = None
                    self._rosbag_recording = False
                    
                    ui.notify('🎥 Recording stopped', type='info')
                    self._emit_log("[ROSBAG] Recording stopped")
                    
                except Exception as ex:
                    ui.notify(f'Error stopping recording: {ex}', type='warning')
                    self._emit_log(f"[ROSBAG] Error stopping: {ex}")
    
    def _on_vertical_sep_toggle(self, e):
        """Handle vertical separation check toggle change."""
        enabled = e.value
        self.mission_controller.vertical_separation_enabled = enabled
        
        if enabled:
            ui.notify('✅ Vertical separation check enabled', type='positive')
            self._emit_log("[SAFETY] Vertical separation check ENABLED")
            self.vertical_sep_status_badge.set_text('OK')
            self.vertical_sep_status_badge.props('color=green')
            # Show content, hide disabled message
            self.vertical_sep_content.style('display: block;')
            self.vertical_sep_disabled_msg.style('display: none;')
        else:
            ui.notify('⚠️ Vertical separation check disabled', type='warning')
            self._emit_log("[SAFETY] Vertical separation check DISABLED - manual monitoring required!")
            self.vertical_sep_status_badge.set_text('OFF')
            self.vertical_sep_status_badge.props('color=grey')
            # Hide content, show disabled message
            self.vertical_sep_content.style('display: none;')
            self.vertical_sep_disabled_msg.style('display: block;')
    
    def _on_trajectory_mode_change(self, e):
        """Handle trajectory mode toggle change (deprecated - PID only now)."""
        # PID is always used now (DJI Native removed for safety)
        if hasattr(self, 'mission_controller'):
            self.mission_controller.use_dji_native = False
        
        speed = self.trajectory_speed_slider.value if self.trajectory_speed_slider else 10
        ui.notify(f'Navigation mode: PID ({speed} m/s)', type='info')
        self._emit_log(f"[CONFIG] Navigation mode set to PID ({speed} m/s)")
    
    def _on_trajectory_speed_change(self, e):
        """Handle trajectory speed slider change."""
        speed = e.value
        
        # Update label
        if self.trajectory_speed_label:
            self.trajectory_speed_label.set_text(f'{speed} m/s')
        
        # Update speed for both modes (self IS the ROS node)
        self.DJI_NATIVE_SPEED = float(speed)
        self.PID_SPEED = float(speed)
    
    def _on_mission_mode_change(self, e):
        """Handle mission mode toggle change (Monitoring Point vs Free Flight)."""
        from groundstation.mission_controller import MissionMode
        
        if e.value == 1:
            mode = MissionMode.MONITORING_POINT
            mode_name = "📍 Monitoring Point"
            mode_desc = "Drone flies to monitoring point and hovers"
        else:
            mode = MissionMode.FREE_FLIGHT
            mode_name = "🆓 Free Flight"
            mode_desc = "Pilot controls drone after reaching altitude"
        
        # Update mission controller's mode
        if hasattr(self, 'mission_controller'):
            self.mission_controller.mission_mode = mode
        
        # Update state machine display to show new mode
        self._build_state_machine_display()
        
        ui.notify(f'{mode_name}: {mode_desc}', type='info')
        self._emit_log(f"[CONFIG] Mission mode set to {mode_name}")
    
    def _abort_trajectories(self):
        """Abort all trajectories."""
        for namespace in self.drones.keys():
            self.send_abort_mission(namespace)
        ui.notify('All trajectories aborted', type='info')
        self._emit_log("[ABORT] All trajectories aborted")
    
    def _update_rth_debug_panel(self, namespace: str, debug_info: dict):
        """Update the RTH prediction debug panel with current values."""
        # Current battery
        current_batt = debug_info.get('current_battery', 0)
        if hasattr(self, 'rth_debug_battery'):
            self.rth_debug_battery.text = f"{current_batt:.1f}%"
        
        # Battery needed to go home (current value)
        batt_needed = debug_info.get('batt_needed_to_go_home', 0)
        if hasattr(self, 'rth_debug_batt_needed'):
            self.rth_debug_batt_needed.text = f"{batt_needed:.1f}%"
        
        # Battery needed to go home (MAX value - used for prediction)
        max_batt_needed = debug_info.get('max_batt_needed_to_go_home', 0)
        if hasattr(self, 'rth_debug_batt_max'):
            self.rth_debug_batt_max.text = f"{max_batt_needed:.1f}%"
        
        # RTH threshold (MAX batt_needed + 2%)
        threshold = debug_info.get('rth_threshold', 0)
        if hasattr(self, 'rth_debug_threshold'):
            self.rth_debug_threshold.text = f"{threshold:.1f}%"
            # Highlight if close to threshold
            if current_batt > 0 and (current_batt - threshold) < 10:
                self.rth_debug_threshold.style('color: #d32f2f; font-weight: bold;')
            else:
                self.rth_debug_threshold.style('color: inherit;')
        
        # Drain rate
        drain_rate = debug_info.get('drain_rate_per_min', 0)
        if hasattr(self, 'rth_debug_drain'):
            self.rth_debug_drain.text = f"{drain_rate:.2f}%/min"
        
        # Data points
        data_points = debug_info.get('data_points', 0)
        if hasattr(self, 'rth_debug_points'):
            self.rth_debug_points.text = f"{data_points}"
        
        # Elapsed monitoring time
        elapsed = debug_info.get('elapsed_since_monitoring', 0)
        if hasattr(self, 'rth_debug_elapsed'):
            mins = int(elapsed // 60)
            secs = int(elapsed % 60)
            self.rth_debug_elapsed.text = f"{mins}:{secs:02d}"
        
        # Predicted RTH time
        predicted = debug_info.get('predicted_rth_seconds', float('inf'))
        if hasattr(self, 'rth_debug_predicted'):
            if predicted != float('inf') and predicted > 0:
                mins = int(predicted // 60)
                secs = int(predicted % 60)
                self.rth_debug_predicted.text = f"{mins}:{secs:02d}"
            else:
                self.rth_debug_predicted.text = "--:--"
        
        # DJI remaining flight time
        dji_time = debug_info.get('dji_remaining_flight_time', 0)
        if hasattr(self, 'rth_debug_dji'):
            mins = int(dji_time // 60)
            secs = int(dji_time % 60)
            self.rth_debug_dji.text = f"{mins}:{secs:02d}"
        
        # Difference (predicted - DJI)
        if hasattr(self, 'rth_debug_diff'):
            if predicted != float('inf') and predicted > 0 and dji_time > 0:
                diff = predicted - dji_time
                mins = int(abs(diff) // 60)
                secs = int(abs(diff) % 60)
                sign = "+" if diff >= 0 else "-"
                self.rth_debug_diff.text = f"{sign}{mins}:{secs:02d}"
                # Color based on sign
                if diff < 0:
                    self.rth_debug_diff.style('color: #d32f2f;')  # Red - prediction is shorter
                else:
                    self.rth_debug_diff.style('color: #2e7d32;')  # Green - prediction is longer
            else:
                self.rth_debug_diff.text = "--:--"
        
        # Regression equation
        slope = debug_info.get('slope', 0)
        intercept = debug_info.get('intercept', 0)
        if hasattr(self, 'rth_debug_regression'):
            if slope != 0:
                self.rth_debug_regression.text = f"batt(t) = {slope:.4f}×t + {intercept:.2f}"
            else:
                self.rth_debug_regression.text = "Not enough data"
        
        # Update live chart
        if hasattr(self, 'rth_chart') and self.rth_chart:
            battery_points = debug_info.get('chart_battery_points', [])
            regression_line = debug_info.get('chart_regression_line', [])
            threshold_line = debug_info.get('chart_threshold_line', [])
            rth_point = debug_info.get('chart_rth_point', [])
            current_time_line = debug_info.get('chart_current_time_line', [])
            current_point = debug_info.get('chart_current_point', [])
            
            # Calculate x-axis max (add some padding)
            if battery_points:
                max_time = max(p[0] for p in battery_points)
                if rth_point:
                    max_time = max(max_time, rth_point[0][0])
                x_max = max(5, max_time + 2)  # At least 5 min, plus 2 min padding
            else:
                x_max = 10
            
            # Update chart options
            self.rth_chart.options['xAxis']['max'] = x_max
            self.rth_chart.options['series'][0]['data'] = battery_points  # Battery scatter (blue dots)
            self.rth_chart.options['series'][1]['data'] = regression_line  # Regression line (green)
            self.rth_chart.options['series'][2]['data'] = threshold_line  # MAX RTH threshold (red dashed)
            self.rth_chart.options['series'][3]['data'] = rth_point  # RTH crossing point (red triangle)
            self.rth_chart.options['series'][4]['data'] = current_time_line  # Current time vertical (orange dotted)
            self.rth_chart.options['series'][5]['data'] = current_point  # Current position (orange circle)
            self.rth_chart.update()
    
    def _clear_mission_stats(self):
        """Clear mission statistics history."""
        self.mission_stats_history.clear()
        self.drone_iteration_counter.clear()
        self.drone_rth_tracking.clear()
        self._refresh_mission_stats_display()
        ui.notify('Mission statistics cleared', type='info')
    
    def _toggle_debug_mode(self):
        """Toggle ROS console on/off."""
        self.debug_mode = not self.debug_mode
        
        if self.debug_mode:
            # Hide Mission Statistics, show ROS console
            if self.mission_stats_card:
                self.mission_stats_card.style('display: none;')
            if self.debug_console_container:
                self.debug_console_container.style('display: block; flex: 1;')
            if self.debug_toggle:
                self.debug_toggle.props('color=orange')
            ui.notify('ROS Console enabled', type='warning')
            
            # Set up logging handler to capture output
            self._setup_debug_logging()
            self._add_debug_log('ROS Console enabled - capturing logs', 'INFO')
        else:
            # Show Mission Statistics, hide ROS console
            if self.mission_stats_card:
                self.mission_stats_card.style('display: block; flex: 1;')
            if self.debug_console_container:
                self.debug_console_container.style('display: none;')
            if self.debug_toggle:
                self.debug_toggle.props('color=')
            ui.notify('ROS Console disabled', type='info')
            
            # Remove logging handler
            self._remove_debug_logging()
    
    def _setup_debug_logging(self):
        """Set up logging handlers to capture all console output."""
        import logging
        import sys
        import os
        import io
        import threading
        
        # Create a custom handler that adds to our debug console
        class DebugUIHandler(logging.Handler):
            def __init__(self, gui_instance):
                super().__init__()
                self.gui = gui_instance
            
            def emit(self, record):
                try:
                    msg = self.format(record)
                    self.gui._add_debug_log(msg, record.levelname)
                except Exception:
                    pass
        
        # Store handler reference for later removal
        self._debug_handler = DebugUIHandler(self)
        self._debug_handler.setFormatter(logging.Formatter('%(asctime)s [%(levelname)s] %(name)s: %(message)s', datefmt='%H:%M:%S'))
        self._debug_handler.setLevel(logging.DEBUG)  # Capture all levels
        
        # Add to root logger to capture all logs
        root_logger = logging.getLogger()
        root_logger.addHandler(self._debug_handler)
        # Ensure root logger level allows debug messages through
        if root_logger.level > logging.DEBUG or root_logger.level == 0:
            self._original_root_level = root_logger.level
            root_logger.setLevel(logging.DEBUG)
        
        # Capture stdout/stderr at the file descriptor level to catch ROS2/rclpy output
        # Save original file descriptors
        self._original_stdout_fd = os.dup(1)
        self._original_stderr_fd = os.dup(2)
        
        # Create pipes
        self._stdout_read_fd, self._stdout_write_fd = os.pipe()
        self._stderr_read_fd, self._stderr_write_fd = os.pipe()
        
        # Redirect stdout/stderr to our pipes
        os.dup2(self._stdout_write_fd, 1)
        os.dup2(self._stderr_write_fd, 2)
        
        # Also update Python's sys.stdout/stderr to use the new fd
        sys.stdout = io.TextIOWrapper(os.fdopen(self._stdout_write_fd, 'wb', 0), write_through=True)
        sys.stderr = io.TextIOWrapper(os.fdopen(self._stderr_write_fd, 'wb', 0), write_through=True)
        
        # Start reader threads
        self._stop_capture = False
        
        def read_output(read_fd, original_fd, default_level):
            reader = os.fdopen(read_fd, 'r')
            while not self._stop_capture:
                try:
                    line = reader.readline()
                    if line:
                        # Write to original output
                        os.write(original_fd, line.encode())
                        
                        # Determine level from content
                        line_stripped = line.strip()
                        if not line_stripped:
                            continue
                        
                        level = default_level
                        line_lower = line_stripped.lower()
                        if '[error]' in line_lower or 'error' in line_lower or 'exception' in line_lower:
                            level = 'ERROR'
                        elif '[warn]' in line_lower or 'warning' in line_lower:
                            level = 'WARNING'
                        elif '[info]' in line_lower:
                            level = 'INFO'
                        elif '[debug]' in line_lower:
                            level = 'DEBUG'
                        
                        self._add_debug_log(line_stripped, level)
                except Exception:
                    break
        
        self._stdout_thread = threading.Thread(target=read_output, args=(self._stdout_read_fd, self._original_stdout_fd, 'INFO'), daemon=True)
        self._stderr_thread = threading.Thread(target=read_output, args=(self._stderr_read_fd, self._original_stderr_fd, 'ERROR'), daemon=True)
        self._stdout_thread.start()
        self._stderr_thread.start()
    
    def _remove_debug_logging(self):
        """Remove the debug logging handler and restore stdout/stderr."""
        import logging
        import sys
        import os
        
        # Stop capture threads
        self._stop_capture = True
        
        if hasattr(self, '_debug_handler'):
            logging.getLogger().removeHandler(self._debug_handler)
            
        # Restore original log level if we changed it
        if hasattr(self, '_original_root_level'):
            logging.getLogger().setLevel(self._original_root_level)
            del self._original_root_level
        
        # Restore original file descriptors
        if hasattr(self, '_original_stdout_fd'):
            os.dup2(self._original_stdout_fd, 1)
            os.close(self._original_stdout_fd)
        if hasattr(self, '_original_stderr_fd'):
            os.dup2(self._original_stderr_fd, 2)
            os.close(self._original_stderr_fd)
        
        # Restore Python stdout/stderr
        sys.stdout = sys.__stdout__
        sys.stderr = sys.__stderr__
    
    def _add_debug_log(self, message: str, level: str = 'INFO'):
        """Add a message to the debug console."""
        if not self.debug_mode or not self.debug_console:
            return
        
        # Color based on level
        level_colors = {
            'DEBUG': '#9e9e9e',    # grey
            'INFO': '#4fc3f7',     # light blue
            'WARNING': '#ffb74d',  # orange
            'ERROR': '#ef5350',    # red
            'CRITICAL': '#f44336', # bright red
        }
        color = level_colors.get(level, '#ffffff')
        
        # Add to console (limit to last 200 lines)
        with self.debug_console:
            ui.label(message).classes('text-xs font-mono').style(f'color: {color}; white-space: pre-wrap; word-break: break-all;')
        
        # Remove old entries if too many
        if len(self.debug_console.default_slot.children) > 200:
            self.debug_console.default_slot.children[0].delete()
        
        # Scroll to bottom
        if hasattr(self, 'debug_scroll'):
            self.debug_scroll.scroll_to(percent=1.0)
    
    def _clear_debug_console(self):
        """Clear the debug console."""
        if self.debug_console:
            self.debug_console.clear()
        ui.notify('Console cleared', type='info')

    def _format_time(self, seconds: float) -> str:
        """Format seconds as MM:SS."""
        if seconds <= 0:
            return "--:--"
        minutes = int(seconds // 60)
        secs = int(seconds % 60)
        return f"{minutes}:{secs:02d}"
    
    def _add_mission_stat(self, drone: str, iteration: int, est_travel: float, actual_travel: float = 0.0, actual_rth: float = 0.0):
        """Add or update a mission statistic entry."""
        # Check if entry exists for this drone and iteration
        for stat in self.mission_stats_history:
            if stat['drone'] == drone and stat['iteration'] == iteration:
                # Update existing entry
                if actual_travel > 0:
                    stat['actual_travel'] = actual_travel
                if actual_rth > 0:
                    stat['actual_rth'] = actual_rth
                self._refresh_mission_stats_display()
                return
        
        # Add new entry
        self.mission_stats_history.append({
            'drone': drone,
            'iteration': iteration,
            'est_travel': est_travel,
            'actual_travel': actual_travel,
            'actual_rth': actual_rth
        })
        self._refresh_mission_stats_display()
    
    def _update_mission_stat_rth(self, drone: str, actual_rth: float):
        """Update RTH time for the most recent mission of a drone."""
        # Find the most recent entry for this drone
        for stat in reversed(self.mission_stats_history):
            if stat['drone'] == drone and stat['actual_rth'] == 0:
                stat['actual_rth'] = actual_rth
                self._refresh_mission_stats_display()
                return
    
    def _refresh_mission_stats_display(self):
        """Refresh the mission statistics display."""
        if not self.mission_stats_container:
            return
        
        self.mission_stats_container.clear()
        
        if not self.mission_stats_history:
            with self.mission_stats_container:
                ui.label("No mission data yet").classes('text-gray-500 italic text-sm')
            return
        
        with self.mission_stats_container:
            for stat in self.mission_stats_history:
                with ui.row().classes('w-full text-base gap-0 px-1 py-1').style('border-bottom: 1px solid #eee'):
                    ui.label(stat['drone']).style('flex: 2; min-width: 60px; overflow: hidden; text-overflow: ellipsis')
                    ui.label(str(stat['iteration'])).style('flex: 0.8; text-align: center; min-width: 30px')
                    ui.label(self._format_time(stat['est_travel'])).style('flex: 1.2; text-align: center; min-width: 50px; color: #666')
                    
                    # Actual travel - color based on comparison with estimate
                    travel_text = self._format_time(stat['actual_travel'])
                    if stat['actual_travel'] > 0:
                        diff = stat['actual_travel'] - stat['est_travel']
                        if diff > 30:  # More than 30s slower
                            travel_color = '#c62828'  # red
                        elif diff < -10:  # More than 10s faster
                            travel_color = '#2e7d32'  # green
                        else:
                            travel_color = '#1565c0'  # blue
                    else:
                        travel_color = '#999'
                    ui.label(travel_text).style(f'flex: 1.2; text-align: center; min-width: 50px; color: {travel_color}; font-weight: bold')
                    
                    # RTH time
                    rth_text = self._format_time(stat['actual_rth'])
                    rth_color = '#1565c0' if stat['actual_rth'] > 0 else '#999'
                    ui.label(rth_text).style(f'flex: 1.2; text-align: center; min-width: 50px; color: {rth_color}; font-weight: bold')
        
        # Scroll to bottom
        if self.mission_stats_scroll:
            self.mission_stats_scroll.scroll_to(percent=1.0)

    async def _show_takeoff_confirmation_dialog(self, drone_name: str) -> bool:
        """Show takeoff confirmation dialog and return user's choice."""
        result = {'confirmed': None}
        
        with ui.dialog() as dialog, ui.card().classes('p-4'):
            ui.label(f'🚁 Takeoff Confirmation').classes('text-xl font-bold text-blue-700')
            ui.separator()
            ui.label(f'Drone "{drone_name}" will take off.').classes('text-lg mt-2')
            ui.label('Confirm to proceed with takeoff.').classes('text-sm text-gray-600 mt-1')
            
            with ui.row().classes('w-full justify-end gap-2 mt-4'):
                ui.button('Cancel', on_click=lambda: self._handle_takeoff_cancel(dialog, result), color='red').props('flat')
                ui.button('Confirm', on_click=lambda: self._handle_takeoff_confirm(dialog, result), color='primary')
        
        dialog.open()
        
        # Wait for user response
        while result['confirmed'] is None:
            await asyncio.sleep(0.1)
        
        return result['confirmed']
    
    async def _show_abort_confirmation_dialog(self) -> bool:
        """Show abort confirmation dialog and return user's choice."""
        result = {'confirmed': None}
        
        with ui.dialog() as dialog, ui.card().classes('p-4'):
            ui.label('⚠️ Abort Mission?').classes('text-xl font-bold text-red-700')
            ui.separator()
            ui.label('Are you sure you want to abort the mission?').classes('text-lg mt-2')
            ui.label('The relay system will stop. Airborne drones will continue unaffected.').classes('text-sm text-gray-600 mt-1')
            
            with ui.row().classes('w-full justify-end gap-2 mt-4'):
                ui.button('Cancel', on_click=lambda: self._handle_abort_cancel(dialog, result)).props('flat')
                ui.button('Confirm Abort', on_click=lambda: self._handle_abort_confirm(dialog, result), color='red')
        
        dialog.open()
        
        # Wait for user response
        while result['confirmed'] is None:
            await asyncio.sleep(0.1)
        
        return result['confirmed']
    
    def _show_takeoff_dialog_sync(self, drone_name: str):
        """Show takeoff confirmation dialog synchronously (non-async version)."""
        with ui.dialog() as dialog, ui.card().classes('p-4'):
            ui.label(f'🚁 Takeoff Confirmation').classes('text-xl font-bold text-blue-700')
            ui.separator()
            ui.label(f'Drone "{drone_name}" will take off.').classes('text-lg mt-2')
            ui.label('Confirm to proceed with takeoff.').classes('text-sm text-gray-600 mt-1')
            
            def on_confirm():
                dialog.close()
                # Play takeoff confirmation sound
                ui.run_javascript('''
                    var audio = new Audio("/static/take_off.mp3");
                    audio.play().catch(function(e) { console.log("Audio play failed:", e); });
                ''')
                # Start the appropriate mission
                if self._pending_single_mission:
                    self._pending_single_mission = False
                    self._do_start_single_mission(self._pending_takeoff_drone)
                elif self._pending_relay_mission:
                    self._pending_relay_mission = False
                    data = self._pending_relay_data
                    self._do_start_relay_mission(data['drone_list'], data['travel_time'], data['distance'])
            
            def on_cancel():
                dialog.close()
                self._show_abort_dialog_sync()
            
            with ui.row().classes('w-full justify-end gap-2 mt-4'):
                ui.button('Cancel', on_click=on_cancel, color='red').props('flat')
                ui.button('Confirm', on_click=on_confirm, color='primary')
        
        dialog.open()
    
    def _show_abort_dialog_sync(self):
        """Show abort confirmation dialog synchronously."""
        with ui.dialog() as dialog, ui.card().classes('p-4'):
            ui.label('⚠️ Abort Mission?').classes('text-xl font-bold text-red-700')
            ui.separator()
            ui.label('Are you sure you want to abort the mission?').classes('text-lg mt-2')
            ui.label('The relay system will stop. Airborne drones will continue unaffected.').classes('text-sm text-gray-600 mt-1')
            
            def on_confirm_abort():
                dialog.close()
                self._pending_single_mission = False
                self._pending_relay_mission = False
                ui.notify('Mission cancelled', type='warning')
                self._emit_log("[MISSION] User aborted mission from takeoff confirmation")
            
            def on_cancel_abort():
                dialog.close()
                # Re-show the takeoff dialog
                self._show_takeoff_dialog_sync(self._pending_takeoff_drone)
            
            with ui.row().classes('w-full justify-end gap-2 mt-4'):
                ui.button('Cancel', on_click=on_cancel_abort).props('flat')
                ui.button('Confirm Abort', on_click=on_confirm_abort, color='red')
        
        dialog.open()
    
    def _handle_takeoff_confirm(self, dialog, result):
        """Handle confirm button click in takeoff dialog."""
        result['confirmed'] = True
        dialog.close()
        # Play takeoff confirmation sound
        ui.run_javascript('''
            var audio = new Audio("/static/take_off.mp3");
            audio.play().catch(function(e) { console.log("Audio play failed:", e); });
        ''')
    
    def _handle_takeoff_cancel(self, dialog, result):
        """Handle cancel button click in takeoff dialog - show abort confirmation."""
        dialog.close()
        
        async def show_abort():
            abort_confirmed = await self._show_abort_confirmation_dialog()
            if abort_confirmed:
                result['confirmed'] = False
                self._emit_log("[MISSION] User aborted mission from takeoff confirmation")
            else:
                # User cancelled the abort, show takeoff dialog again
                result['confirmed'] = None
                confirmed = await self._show_takeoff_confirmation_dialog(self._pending_takeoff_drone if hasattr(self, '_pending_takeoff_drone') else 'drone')
                result['confirmed'] = confirmed
        
        asyncio.create_task(show_abort())
    
    def _handle_abort_confirm(self, dialog, result):
        """Handle confirm button click in abort dialog."""
        result['confirmed'] = True
        dialog.close()
    
    def _handle_abort_cancel(self, dialog, result):
        """Handle cancel button click in abort dialog."""
        result['confirmed'] = False
        dialog.close()
    
    def _restart_groundstation(self):
        """Reset the groundstation state (soft restart)."""
        # Confirm with user
        with ui.dialog() as dialog, ui.card():
            ui.label('Reset Groundstation?').classes('text-lg font-bold')
            ui.label('This will stop all missions and disconnect all drones.').classes('text-sm text-gray-600')
            ui.label('The page will refresh after reset.').classes('text-sm text-gray-500')
            with ui.row().classes('w-full justify-end gap-2 mt-4'):
                ui.button('Cancel', on_click=dialog.close).props('flat')
                ui.button('Reset', on_click=lambda: self._do_soft_restart(dialog), color='negative')
        
        dialog.open()
    
    def _do_soft_restart(self, dialog):
        """Perform a soft restart - reset internal state without killing the process."""
        import os
        import signal
        
        dialog.close()
        
        self._emit_log("[SYSTEM] Resetting groundstation state...")
        ui.notify('Resetting groundstation...', type='warning', timeout=2000)
        
        # Stop any active missions
        try:
            self.stop_mission()
        except:
            pass
        
        # Shutdown mission controller thread
        if hasattr(self, 'mission_controller'):
            self.mission_controller.shutdown()
        
        # Kill all drone controller processes
        for ns, process in list(self.drone_processes.items()):
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                process.wait(timeout=2)
            except:
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                except:
                    pass
        self.drone_processes.clear()
        
        # Clean up ROS subscribers and publishers
        for ns in list(self.drone_subscribers.keys()):
            for sub in self.drone_subscribers[ns].values():
                try:
                    self.destroy_subscription(sub)
                except:
                    pass
        self.drone_subscribers.clear()
        
        for ns in list(self.drone_publishers.keys()):
            for pub in self.drone_publishers[ns].values():
                try:
                    self.destroy_publisher(pub)
                except:
                    pass
        self.drone_publishers.clear()
        
        # Clear drone data
        self.drones.clear()
        
        # Reset mission state
        self.mission = RelayMission()
        self.monitoring_point = MonitoringPoint()
        
        # Reinitialize mission controller
        self.mission_controller = MissionController()
        self._setup_mission_controller_callbacks()
        self.mission_controller.on_takeoff_confirmation_request = self._on_takeoff_confirmation_request
        
        # Reset UI state
        self._mission_start_time = None
        self._stop_mission_timer()
        
        self._emit_log("[SYSTEM] Groundstation reset complete")
        
        # Refresh the page
        ui.timer(0.5, lambda: ui.run_javascript('location.reload()'), once=True)


# ============================================================================
# ENTRY POINTS
# ============================================================================

def main() -> None:
    """ROS entry point - empty to enable NiceGUI auto-reloading."""
    pass


def ros_main() -> None:
    """Initialize ROS2 and spin the node."""
    rclpy.init()
    node = PerpetualMonitorGUI.get_instance()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


# Start ROS2 in background thread on app startup
app.on_startup(lambda: threading.Thread(target=ros_main, daemon=True).start())

# Handle ROS2 module naming conventions
ui_run.APP_IMPORT_STRING = f'{__name__}:app'

ui.run(
    uvicorn_reload_dirs=str(Path(__file__).parent.resolve()),
    favicon='https://fonts.gstatic.com/s/i/short-term/release/materialsymbolsoutlined/flight/default/48px.svg',
    port=8086,
    title='Perpetual Drone Monitoring'
)
