"""
Perpetual Monitoring GUI

NiceGUI-based web interface for the perpetual drone monitoring system.
Uses NiceGUI Events for thread-safe communication between ROS2 callbacks and UI.

Based on the NiceGUI ROS2 integration pattern.

Author: Edouard Rolland
Project: WildDrone
"""

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
    MonitoringPoint, RelayMission
)
from groundstation.mission_controller import MissionState


# ============================================================================
# STATIC FILES
# ============================================================================

app.add_static_files('/static', str(Path(__file__).parent / 'static'))


# ============================================================================
# ARROW DISPLAY
# ============================================================================

class Arrow:
    """Arrow marker for drone position display on map."""
    
    def __init__(self, map_ui, id: str, lat: float, lng: float, heading: float, drones_arrows: dict):
        """
        Initialize an arrow on the given map.

        :param map_ui: The NiceGUI Leaflet map instance.
        :param id: Unique identifier for the arrow (namespace).
        :param lat: Latitude of the arrow's initial position.
        :param lng: Longitude of the arrow's initial position.
        :param heading: Initial heading of the arrow (in degrees).
        :param drones_arrows: Dict to check for duplicate arrows.
        """
        self.map_ui = map_ui
        self.id = id
        self.lat = lat
        self.lng = lng
        self.heading = heading

        if id in drones_arrows:
            raise ValueError(f"Arrow with id '{id}' already exists.")

    def _place_arrow(self):
        """Place the arrow on the map."""
        ui.run_javascript(
            f"place_arrow({self.map_ui.id}, {self.lat}, {self.lng}, {self.heading}, '{self.id}')"
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
        self.drone_connected_event = Event()
        self.drone_disconnected_event = Event()
        self.monitoring_point_update = Event()
        self.relay_countdown_update = Event()
        self.log_event = Event()
        
        # UI element references (populated when page loads)
        self.map = None
        self.drone_cards: Dict[str, ui.card] = {}
        self.drone_arrows: Dict[str, Arrow] = {}
        self.drone_labels: Dict[str, Dict[str, ui.label]] = {}
        self.drone_buttons: Dict[str, Dict[str, ui.button]] = {}
        self.drone_list_container = None
        
        # Monitoring point marker
        self.monitoring_marker = None
        self.monitoring_circle = None
        
        # Mission display elements
        self.mission_status_label = None
        self.countdown_label = None
        self.countdown_progress = None
        self.active_drone_label = None
        self.next_drone_label = None
        self.drones_needed_label = None
        self.relay_alert_label = None
        self.relay_alert_icon = None
        self._relay_alert_visible = False
        self.reconnect_label = None
        self.mission_timer_label = None
        self._mission_start_time = None
        self._mission_timer_task = None
        
        # Event log
        self.event_log = None
        self.event_scroll = None
        
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
        self.map_center = (0.025324, 36.868363)  # Default center
        
        # Drone colors for visualization
        self.drone_colors = ['#FF6B6B', '#4ECDC4', '#45B7D1', '#96CEB4', '#FFEAA7', '#DDA0DD', '#98D8C8', '#F7DC6F']
        
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
        if namespace in self.drones:
            self.drones[namespace].battery_level = level
        self.drone_battery_update.emit({
            'namespace': namespace,
            'level': level
        })
    
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
    
    def _on_mission_status_update(self, namespace: str, state: MissionState, message: str):
        """Override mission status callback to emit event."""
        super()._on_mission_status_update(namespace, state, message)
        
        # Start mission timer when first drone reaches monitoring point
        if state == MissionState.MONITORING and self._mission_start_time is None:
            self._mission_start_time = time.time()
            self._start_mission_timer()
        
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
            self.drone_state_update.emit({
                'namespace': namespace,
                'state': state_map[state]
            })
    
    def _on_relay_countdown_update(self, countdown: float, next_drone: str):
        """Override relay countdown callback to emit event."""
        super()._on_relay_countdown_update(countdown, next_drone)
        self.relay_countdown_update.emit({
            'countdown': countdown,
            'next_drone': next_drone
        })
    
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
                    ui.notify(
                        f'{ns} auto-added to relay queue (position {position})',
                        type='positive',
                        timeout=5000
                    )
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
        
        # Add CSS and JS
        ui.add_head_html("""
            <script src='/static/arrows.js'></script>
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
        
        with ui.row().classes('w-full h-full').style('display: flex; height: 95vh; gap: 10px; padding: 10px;'):
            # Left panel: Drone management
            self._build_left_panel()
            
            # Right panel: Map and mission control
            self._build_right_panel()
        
        # Subscribe to events for UI updates
        self._setup_event_subscriptions()
        
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
            
            # Update arrow on map
            if ns in self.drone_arrows:
                heading = self.drones[ns].heading if ns in self.drones else 0.0
                self.drone_arrows[ns].update(lat, lon, heading)
            
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
                self.drone_labels[ns]['flight_time'].text = f"⏱️ {minutes}:{seconds:02d}"
                self.drone_labels[ns]['flight_time'].style(f'color: {color}; font-weight: bold')
        
        @self.drone_recording_update.subscribe
        def on_recording(data: dict):
            ns = data['namespace']
            is_recording = data['is_recording']
            
            if ns in self.drone_labels and 'recording' in self.drone_labels[ns]:
                if is_recording:
                    self.drone_labels[ns]['recording'].text = "REC"
                    self.drone_labels[ns]['recording'].style('color: red; font-weight: bold;')
                    if 'recording_icon' in self.drone_labels[ns]:
                        self.drone_labels[ns]['recording_icon'].style('font-size: 14px; color: red;')
                else:
                    self.drone_labels[ns]['recording'].text = ""
                    self.drone_labels[ns]['recording'].style('color: gray;')
                    if 'recording_icon' in self.drone_labels[ns]:
                        self.drone_labels[ns]['recording_icon'].style('font-size: 14px; color: gray;')
        
        @self.drone_state_update.subscribe
        def on_state(data: dict):
            ns = data['namespace']
            state = data['state']
            
            if ns in self.drone_labels and 'state' in self.drone_labels[ns]:
                # Use simple text labels - icons are in the card header
                state_colors = {
                    DroneState.DISCONNECTED: "background: #ffebee; color: #c62828;",
                    DroneState.CONNECTED: "background: #e8f5e9; color: #2e7d32;",
                    DroneState.IDLE: "background: #f5f5f5; color: #616161;",
                    DroneState.TAKING_OFF: "background: #e3f2fd; color: #1565c0;",
                    DroneState.FLYING_TO_POINT: "background: #e3f2fd; color: #1565c0;",
                    DroneState.MONITORING: "background: #f3e5f5; color: #7b1fa2;",
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
        
        @self.drone_connected_event.subscribe
        def on_connected(data: dict):
            self._refresh_drone_list()
        
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
        
        @self.monitoring_point_update.subscribe
        def on_monitoring_point(data: dict):
            if data.get('clear'):
                if self.monitoring_marker:
                    self.map.remove_layer(self.monitoring_marker)
                    self.monitoring_marker = None
                if self.monitoring_circle:
                    self.map.remove_layer(self.monitoring_circle)
                    self.monitoring_circle = None
            else:
                lat, lon, alt = data['lat'], data['lon'], data['alt']
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
            
            if self.countdown_label:
                if countdown > 0:
                    minutes = int(countdown // 60)
                    seconds = int(countdown % 60)
                    self.countdown_label.text = f"{next_drone} in {minutes}:{seconds:02d}"
                    self.countdown_label.style('color: white; font-weight: bold')
                    
                    if self.countdown_progress:
                        self.countdown_progress.value = max(0, min(1, countdown / 300))
                    
                    # Threshold-based preparation alerts
                    if hasattr(self, 'relay_alert_label') and self.relay_alert_label:
                        if countdown <= 60:  # 1 minute - CONNECT NOW
                            self._relay_alert_visible = True
                            self.relay_alert_label.text = f"CONNECT {next_drone} NOW!"
                            self.relay_alert_label.style('color: #ff4444; animation: blink 0.5s infinite')
                            self.relay_alert_icon.style('color: #ff4444; animation: blink 0.5s infinite')
                        elif countdown <= 180:  # 3 minutes - GET READY
                            self._relay_alert_visible = True
                            self.relay_alert_label.text = f"GET {next_drone} READY"
                            self.relay_alert_label.style('color: #ffaa00')
                            self.relay_alert_icon.style('color: #ffaa00')
                        elif countdown <= 300:  # 5 minutes - PREPARE
                            self._relay_alert_visible = True
                            self.relay_alert_label.text = f"Prepare {next_drone}"
                            self.relay_alert_label.style('color: #88ff88')
                            self.relay_alert_icon.style('color: #88ff88')
                        else:
                            self._relay_alert_visible = False
                else:
                    self.countdown_label.text = f"{next_drone} LAUNCHING!"
                    self.countdown_label.style('color: #ff4444; font-weight: bold; animation: blink 0.5s infinite')
                    if hasattr(self, 'relay_alert_label') and self.relay_alert_label:
                        self._relay_alert_visible = True
                        self.relay_alert_label.text = f"LAUNCH {next_drone}!"
                        self.relay_alert_label.style('color: #ff4444; font-weight: bold; animation: blink 0.5s infinite')
                        self.relay_alert_icon.style('color: #ff4444; animation: blink 0.5s infinite')
            
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
            if self.event_log:
                timestamp = datetime.now().strftime("%H:%M:%S")
                with self.event_log:
                    ui.label(f"[{timestamp}] {message}").classes('text-sm')
                if self.event_scroll:
                    self.event_scroll.scroll_to(percent=1.0)
    
    def _build_left_panel(self):
        """Build the left panel with drone management."""
        with ui.card().classes('h-full').style('flex: 1.2; min-width: 350px; overflow-y: auto;'):
            with ui.row().classes('items-center gap-2'):
                ui.icon('flight').classes('text-3xl text-primary')
                ui.label("WildPerpetua").classes('text-2xl font-bold')
            
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
                    
                    with ui.column().style('width: 100px'):
                        self.namespace_input = ui.input(
                            label='Name',
                            placeholder='drone_1'
                        ).classes('w-full')
                
                with ui.row().classes('w-full gap-2 mt-2'):
                    ui.button('Connect', icon='link', on_click=self._connect_drone_ui).props('color=primary')
                    ui.button('Refresh', icon='refresh', on_click=self._refresh_drone_list).props('flat')
            
            ui.separator()
            
            # Mission Status Card
            with ui.card().classes('w-full').style('background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: white;'):
                with ui.row().classes('items-center gap-2'):
                    ui.icon('analytics').classes('text-2xl')
                    ui.label("Mission Status").classes('text-lg font-bold')
                
                with ui.grid(columns=2).classes('w-full gap-2'):
                    self.mission_status_label = ui.label("Inactive").classes('font-bold')
                    self.active_drone_label = ui.label("Active: --")
                
                # Mission elapsed time
                with ui.row().classes('items-center gap-2 mt-2'):
                    ui.icon('timer').classes('text-xl')
                    self.mission_timer_label = ui.label("00:00:00").classes('font-bold text-lg font-mono')
                
                self.countdown_label = ui.label("").classes('countdown-display mt-2')
                self.countdown_progress = ui.linear_progress(value=0).props('instant-feedback').classes('mt-1')
                
                # Relay preparation alert with icon
                with ui.row().classes('items-center gap-2 mt-2').bind_visibility_from(self, '_relay_alert_visible'):
                    self.relay_alert_icon = ui.icon('notifications_active').classes('text-2xl')
                    self.relay_alert_label = ui.label("").classes('font-bold text-lg')
                
                # Show which drones need reconnection (battery swap)
                with ui.row().classes('items-center gap-2 mt-2'):
                    ui.icon('battery_charging_full').classes('text-xl')
                    self.reconnect_label = ui.label("").classes('text-sm')
                
                self.drones_needed_label = ui.label("").classes('text-sm mt-1')
            
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
            
            # Control panels row
            with ui.row().classes('w-full gap-2 items-stretch').style('flex-wrap: wrap;'):
                # Monitoring Point Control
                with ui.card().classes('').style('flex: 1; min-width: 280px;'):
                    with ui.row().classes('items-center gap-2'):
                        ui.icon('place').classes('text-xl text-primary')
                        ui.label("Monitoring Point").classes('text-lg font-bold')
                    
                    with ui.row().classes('w-full gap-2 items-end flex-wrap'):
                        self.lat_input = ui.input(label='Lat', value='0.0').style('width: 100px')
                        self.lon_input = ui.input(label='Lon', value='0.0').style('width: 100px')
                        self.alt_input = ui.input(label='Alt (m)', value='50').style('width: 80px')
                        self.heading_input = ui.input(label='Heading (°)', value='0').style('width: 90px').tooltip('Target drone heading at monitoring point (0-360°)')
                        ui.button(icon='push_pin', on_click=self._set_monitoring_point_manual).props('color=primary dense').tooltip('Set Point')
                        ui.button(icon='delete', on_click=self._clear_monitoring_point_ui).props('dense').tooltip('Clear')
                    
                    with ui.row().classes('items-center gap-1 text-xs text-gray-500'):
                        ui.icon('touch_app').style('font-size: 14px')
                        ui.label("Click on map to set point")
                
                # Mission Control
                with ui.card().classes('').style('flex: 1; min-width: 280px;'):
                    with ui.row().classes('items-center gap-2'):
                        ui.icon('flag').classes('text-xl text-primary')
                        ui.label("Mission Control").classes('text-lg font-bold')
                    
                    with ui.row().classes('w-full gap-2 items-end'):
                        self.rth_alt_input = ui.input(label='RTH Alt (m)', value='50').style('width: 100px')
                        self.safety_buffer_input = ui.input(label='Buffer (s)', value='60').style('width: 80px')
                    
                    with ui.row().classes('w-full gap-2 items-end'):
                        self.min_battery_input = ui.input(label='Min Battery (%)', value='30').style('width: 110px')
                        self.min_satellites_input = ui.input(label='Min Sats', value='8').style('width: 80px')
                    
                    with ui.row().classes('w-full gap-2 mt-2'):
                        ui.button('Single', icon='play_arrow', on_click=self._start_single_mission).props('color=green').tooltip('Start single drone mission')
                        ui.button('Relay', icon='sync', on_click=self._start_relay_mission).props('color=primary').tooltip('Start relay mission')
                        ui.button('Stop', icon='stop', on_click=self._stop_mission_ui).props('color=red').tooltip('Stop all missions')
                
                # Trajectory Options
                with ui.card().classes('').style('flex: 1; min-width: 280px;'):
                    with ui.row().classes('items-center gap-2'):
                        ui.icon('route').classes('text-xl text-primary')
                        ui.label("Trajectory").classes('text-lg font-bold')
                    
                    self.trajectory_mode = ui.toggle(
                        ['PID', 'DJI Native'], 
                        value='PID',
                        on_change=self._on_trajectory_mode_change
                    ).props('dense')
                    
                    # Speed slider for DJI Native mode
                    with ui.row().classes('w-full items-center gap-2 mt-2'):
                        ui.icon('speed').classes('text-lg')
                        self.trajectory_speed_slider = ui.slider(
                            min=1, max=12, value=10, step=1,
                            on_change=self._on_trajectory_speed_change
                        ).props('dense label').classes('flex-grow')
                        self.trajectory_speed_label = ui.label('10 m/s').classes('text-sm font-mono w-16')
                    
                    with ui.row().classes('w-full gap-2 mt-2'):
                        ui.button('Abort', icon='cancel', on_click=self._abort_trajectories).props('dense color=red')
            
            # Event Log
            with ui.card().classes('w-full').style('max-height: 150px;'):
                with ui.row().classes('items-center gap-2'):
                    ui.icon('list_alt').classes('text-xl text-primary')
                    ui.label("Event Log").classes('text-lg font-bold')
                with ui.scroll_area().classes('w-full').style('max-height: 100px').props('id=event-log') as self.event_scroll:
                    self.event_log = ui.column().classes('w-full gap-1')
    
    def _build_drone_card(self, namespace: str, drone: DroneData):
        """Build a compact card for a single drone."""
        color_idx = list(self.drones.keys()).index(namespace) % len(self.drone_colors)
        color = self.drone_colors[color_idx]
        
        with ui.card().classes('drone-card w-full p-2') as card:
            self.drone_cards[namespace] = card
            self.drone_labels[namespace] = {}
            self.drone_buttons[namespace] = {}
            
            # Compact header: color dot, name, state, battery
            with ui.row().classes('w-full items-center gap-2'):
                ui.icon('circle').style(f'color: {color}; font-size: 14px')
                ui.label(f"{namespace}").classes('font-bold').style('flex: 1')
                self.drone_labels[namespace]['state'] = ui.label(f"{drone.state.value}").classes('text-xs px-1 rounded bg-gray-200')
                with ui.row().classes('items-center gap-0'):
                    ui.icon('battery_full').style('font-size: 18px')
                    self.drone_labels[namespace]['battery'] = ui.label(f"{drone.battery_level:.0f}%").classes('text-xs font-bold')
            
            # Compact stats row
            with ui.row().classes('w-full items-center gap-3 text-xs text-gray-600'):
                with ui.row().classes('items-center gap-0'):
                    ui.icon('height').style('font-size: 18px')
                    self.drone_labels[namespace]['altitude'] = ui.label(f"{drone.altitude:.0f}m")
                with ui.row().classes('items-center gap-0'):
                    ui.icon('satellite_alt').style('font-size: 18px')
                    self.drone_labels[namespace]['satellites'] = ui.label(f"{drone.satellite_count}")
                with ui.row().classes('items-center gap-0'):
                    ui.icon('timer').style('font-size: 18px')
                    self.drone_labels[namespace]['flight_time'] = ui.label("--:--")
                with ui.row().classes('items-center gap-0'):
                    rec_icon = ui.icon('fiber_manual_record').style('font-size: 18px; color: red;' if drone.is_recording else 'font-size: 18px; color: gray;')
                    self.drone_labels[namespace]['recording'] = ui.label("REC" if drone.is_recording else "").style('color: red; font-weight: bold;' if drone.is_recording else 'color: gray;')
                    self.drone_labels[namespace]['recording_icon'] = rec_icon
            
            # Hidden position label (for data, not display)
            self.drone_labels[namespace]['position'] = ui.label().classes('hidden')
            
            # All controls in one compact row
            with ui.row().classes('w-full gap-0 mt-1'):
                ui.button(icon='flight_takeoff', on_click=lambda ns=namespace: self.send_takeoff(ns)).props('flat dense').tooltip('Take Off')
                ui.button(icon='flight_land', on_click=lambda ns=namespace: self.send_land(ns)).props('flat dense').tooltip('Land')
                ui.button(icon='home', on_click=lambda ns=namespace: self.send_rth(ns)).props('flat dense').tooltip('Return to Home')
                ui.button(icon='warning', on_click=lambda ns=namespace: self.send_abort_mission(ns)).props('flat dense color=negative').tooltip('Abort Mission')
                ui.button(icon='videocam', on_click=lambda ns=namespace: self.send_start_recording(ns)).props('flat dense color=red').tooltip('Start Recording')
                ui.button(icon='stop', on_click=lambda ns=namespace: self.send_stop_recording(ns)).props('flat dense').tooltip('Stop Recording')
                ui.button(icon='my_location', on_click=lambda ns=namespace: self.set_monitoring_point_from_drone(ns)).props('flat dense').tooltip('Use as monitoring point')
                ui.button(icon='link_off', on_click=lambda ns=namespace: self._disconnect_drone_ui(ns)).props('flat dense color=negative').tooltip('Disconnect')
            
            # Compact gimbal control
            with ui.row().classes('w-full items-center gap-1'):
                ui.icon('camera').style('font-size: 20px; color: gray')
                gimbal_slider = ui.slider(min=-90, max=0, value=0, step=5).props('dense').style('flex: 1')
                self.drone_labels[namespace]['gimbal'] = ui.label('0°').classes('text-xs').style('min-width: 30px')
                
                def update_gimbal(e, ns=namespace):
                    val = float(e.args)
                    if ns in self.drone_labels and 'gimbal' in self.drone_labels[ns]:
                        self.drone_labels[ns]['gimbal'].text = f"{int(val)}°"
                    self.send_gimbal_pitch(ns, val)
                gimbal_slider.on('update:model-value', update_gimbal)
            
            # Create arrow on map
            self._add_drone_arrow(namespace, drone.latitude, drone.longitude, drone.heading, color)
    
    def _add_drone_arrow(self, namespace: str, lat: float, lon: float, heading: float, color: str = 'red'):
        """Add a drone arrow to the map."""
        if self.map and namespace not in self.drone_arrows:
            try:
                arrow = Arrow(
                    self.map, namespace, lat, lon, heading,
                    drones_arrows=self.drone_arrows
                )
                self.drone_arrows[namespace] = arrow
                arrow._place_arrow()
            except ValueError as e:
                self.get_logger().warning(f"Could not create arrow: {e}")
    
    # ========================================================================
    # UI EVENT HANDLERS
    # ========================================================================
    
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
            self.namespace_input.value = ''
        else:
            ui.notify('Failed to connect drone', type='negative')
    
    def _disconnect_drone_ui(self, namespace: str):
        """Handle drone disconnection request from UI."""
        if self.disconnect_drone(namespace):
            ui.notify(f'{namespace} disconnected', type='positive')
            self._refresh_drone_list()
        else:
            ui.notify(f'Cannot disconnect {namespace} (may be in flight)', type='warning')
    
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
            needed, travel_time, distance = result
            
            # Format travel time
            travel_min = int(travel_time // 60)
            travel_sec = int(travel_time % 60)
            distance_km = distance / 1000
            
            connected = len(self.drones)
            
            # Check if we have a valid distance (not fallback 3km/5min)
            is_fallback = (distance == 3000 and travel_time == 300)
            
            if is_fallback:
                self.drones_needed_label.text = f"Waiting for drone GPS... ({connected} connected)"
                self.drones_needed_label.style('color: #757575;')  # grey
            elif needed == float('inf'):
                self.drones_needed_label.text = f"Point too far! ({distance_km:.1f}km, {travel_min}:{travel_sec:02d} travel)"
                self.drones_needed_label.style('color: #c62828;')  # error red
            else:
                info = f"Need {needed} drones ({distance_km:.1f}km, ~{travel_min}min travel)"
                if connected >= needed:
                    self.drones_needed_label.text = f"{info} ✓ {connected} connected"
                    self.drones_needed_label.style('color: #2e7d32;')  # success green
                else:
                    self.drones_needed_label.text = f"{info} ⚠ only {connected} connected"
                    self.drones_needed_label.style('color: #ef6c00;')  # warning orange
    
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
        if not self.monitoring_point.is_set:
            ui.notify('Please set a monitoring point first', type='warning')
            return
        
        if not self.drones:
            ui.notify('No drones connected', type='warning')
            return
        
        drone_ns = list(self.drones.keys())[0]
        
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
            
            self.mission_status_label.text = "Single Drone"
            self.mission_status_label.style('color: #2e7d32;')  # green
            self.active_drone_label.text = f"Active: {drone_ns}"
            ui.notify(f'Mission started', type='positive')
            self._emit_log(f"Single mission started with {drone_ns}")
        else:
            ui.notify('Failed to start mission', type='negative')
    
    def _start_relay_mission(self):
        """Start a relay mission with all connected drones."""
        if not self.monitoring_point.is_set:
            ui.notify('Please set a monitoring point first', type='warning')
            return
        
        if len(self.drones) < 1:
            ui.notify('No drones connected', type='warning')
            return
        
        # Warn if only 1 drone - relay needs more drones to connect later
        if len(self.drones) == 1:
            ui.notify(
                'Starting relay with 1 drone. Connect more drones before battery runs low!',
                type='warning',
                timeout=5000
            )
        
        # Check if we have enough drones for the distance
        result = self.mission_controller.calculate_drones_needed()
        needed, travel_time, distance = result
        connected = len(self.drones)
        
        if needed == float('inf'):
            ui.notify(f'Point too far! ({distance/1000:.1f}km) - cannot maintain coverage', type='negative')
            return
        
        if connected < needed:
            # Show warning but allow proceeding (operator may have spare batteries ready)
            ui.notify(
                f'Warning: Need {needed} drones for continuous coverage, only {connected} connected. '
                f'Coverage gaps may occur!', 
                type='warning',
                timeout=5000
            )
        
        drone_list = list(self.drones.keys())
        
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
            
            travel_min = int(travel_time // 60)
            self.mission_status_label.text = f"Relay ({len(drone_list)} drones)"
            self.mission_status_label.style('color: #1565c0;')  # blue
            self.active_drone_label.text = f"Active: {drone_list[0]}"
            ui.notify(f'Relay mission started with {len(drone_list)} drones (~{travel_min}min to point)', type='positive')
            self._emit_log(f"Relay mission started: {', '.join(drone_list)} - {distance/1000:.1f}km to point")
        else:
            ui.notify('Failed to start relay mission', type='negative')
    
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
        self.mission_status_label.style('color: #c62828;')  # red
        self.countdown_label.text = ""
        ui.notify('Mission stopped', type='info')
        self._emit_log("Mission stopped - drones returning home")
    
    def _on_trajectory_mode_change(self, e):
        """Handle trajectory mode toggle change."""
        use_dji_native = (e.value == 'DJI Native')
        
        # Update mission controller's navigation mode (self IS the ROS node)
        if hasattr(self, 'mission_controller'):
            self.mission_controller.use_dji_native = use_dji_native
        
        # Get current speed from slider
        speed = self.trajectory_speed_slider.value if self.trajectory_speed_slider else 10
        mode_name = f"DJI Native ({speed} m/s)" if use_dji_native else f"PID ({speed} m/s)"
        ui.notify(f'Navigation mode: {mode_name}', type='info')
        self._emit_log(f"[CONFIG] Navigation mode set to {mode_name}")
    
    def _on_trajectory_speed_change(self, e):
        """Handle trajectory speed slider change."""
        speed = e.value
        
        # Update label
        if self.trajectory_speed_label:
            self.trajectory_speed_label.set_text(f'{speed} m/s')
        
        # Update speed for both modes (self IS the ROS node)
        self.DJI_NATIVE_SPEED = float(speed)
        self.PID_SPEED = float(speed)
        
        self._emit_log(f"[CONFIG] Navigation speed set to {speed} m/s")
    
    def _abort_trajectories(self):
        """Abort all trajectories."""
        for namespace in self.drones.keys():
            self.send_abort_mission(namespace)
        ui.notify('All trajectories aborted', type='info')
        self._emit_log("[ABORT] All trajectories aborted")


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
