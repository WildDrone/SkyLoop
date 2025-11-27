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
        
        # Event log
        self.event_log = None
        self.event_scroll = None
        
        # Connection form elements
        self.ip_input = None
        self.namespace_input = None
        self.lat_input = None
        self.lon_input = None
        self.alt_input = None
        self.rth_alt_input = None
        self.safety_buffer_input = None
        self.trajectory_mode = None
        
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
    
    def _on_position(self, namespace: str, msg):
        """Override position callback to emit event."""
        super()._on_position(namespace, msg)
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
                self._emit_log(f"🔗 Drone {ns} connected at {ip_address}")
        return result
    
    def disconnect_drone(self, namespace: str) -> bool:
        """Override disconnect_drone to emit event."""
        result = super().disconnect_drone(namespace)
        if result:
            self.drone_disconnected_event.emit({'namespace': namespace})
            self._emit_log(f"🔌 Drone {namespace} disconnected")
        return result
    
    def set_monitoring_point(self, lat: float, lon: float, alt: float, source: str = "manual"):
        """Override to emit event."""
        super().set_monitoring_point(lat, lon, alt, source)
        self.monitoring_point_update.emit({
            'lat': lat,
            'lon': lon,
            'alt': alt
        })
        self._emit_log(f"📍 Monitoring point set: ({lat:.6f}, {lon:.6f}, {alt:.1f}m)")
    
    def clear_monitoring_point(self):
        """Override to emit event."""
        super().clear_monitoring_point()
        self.monitoring_point_update.emit({'clear': True})
        self._emit_log("📍 Monitoring point cleared")
    
    def _emit_log(self, message: str):
        """Emit a log event."""
        self.log_event.emit({'message': message})
    
    # ========================================================================
    # UI CONSTRUCTION
    # ========================================================================
    
    def _build_ui(self):
        """Build the main UI layout with event subscriptions."""
        
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
    
    def _setup_event_subscriptions(self):
        """Set up event subscriptions for UI updates."""
        
        @self.drone_position_update.subscribe
        def on_position(data: dict):
            ns = data['namespace']
            lat, lon, alt = data['lat'], data['lon'], data['alt']
            
            if ns in self.drone_arrows:
                heading = self.drones[ns].heading if ns in self.drones else 0
                arrow = self.drone_arrows[ns]
                arrow.update(lat, lon, heading)
            
            if ns in self.drone_labels:
                if 'position' in self.drone_labels[ns]:
                    self.drone_labels[ns]['position'].text = f"📍 {lat:.6f}, {lon:.6f}"
                if 'altitude' in self.drone_labels[ns]:
                    self.drone_labels[ns]['altitude'].text = f"📏 {alt:.1f} m"
        
        @self.drone_heading_update.subscribe
        def on_heading(data: dict):
            ns = data['namespace']
            heading = data['heading']
            
            if ns in self.drones and ns in self.drone_arrows:
                arrow = self.drone_arrows[ns]
                arrow.update(arrow.lat, arrow.lng, heading)
        
        @self.drone_battery_update.subscribe
        def on_battery(data: dict):
            ns = data['namespace']
            level = data['level']
            
            if ns in self.drone_labels and 'battery' in self.drone_labels[ns]:
                color = 'green' if level > 50 else 'orange' if level > 20 else 'red'
                self.drone_labels[ns]['battery'].text = f"🔋 {level:.1f}%"
                self.drone_labels[ns]['battery'].style(f'color: {color}')
        
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
        
        @self.drone_state_update.subscribe
        def on_state(data: dict):
            ns = data['namespace']
            state = data['state']
            
            if ns in self.drone_labels and 'state' in self.drone_labels[ns]:
                state_icons = {
                    DroneState.DISCONNECTED: "❌",
                    DroneState.CONNECTED: "🟢",
                    DroneState.IDLE: "⚪",
                    DroneState.TAKING_OFF: "🛫",
                    DroneState.FLYING_TO_POINT: "✈️",
                    DroneState.MONITORING: "👁️",
                    DroneState.RETURNING_HOME: "🏠",
                    DroneState.LANDING: "🛬",
                    DroneState.EMERGENCY: "🚨"
                }
                icon = state_icons.get(state, "❓")
                self.drone_labels[ns]['state'].text = f"{icon} {state.value}"
                
                # Highlight active monitoring drone
                if state == DroneState.MONITORING and ns in self.drone_cards:
                    self.drone_cards[ns].style('border: 3px solid #4CAF50; box-shadow: 0 0 10px #4CAF50')
                elif ns in self.drone_cards:
                    self.drone_cards[ns].style('border: 1px solid #ddd; box-shadow: none')
        
        @self.drone_recording_update.subscribe
        def on_recording(data: dict):
            ns = data['namespace']
            is_recording = data['is_recording']
            
            if ns in self.drone_labels and 'recording' in self.drone_labels[ns]:
                if is_recording:
                    self.drone_labels[ns]['recording'].text = "🔴 RECORDING"
                    self.drone_labels[ns]['recording'].style('color: red; font-weight: bold; animation: blink 1s infinite')
                else:
                    self.drone_labels[ns]['recording'].text = "⚫ Standby"
                    self.drone_labels[ns]['recording'].style('color: gray; animation: none')
        
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
                    self.countdown_label.text = f"⏳ {next_drone} in {minutes}:{seconds:02d}"
                    self.countdown_label.style('color: blue; font-weight: bold')
                    
                    if self.countdown_progress:
                        self.countdown_progress.value = max(0, min(1, countdown / 300))
                else:
                    self.countdown_label.text = f"🚀 {next_drone} LAUNCHING!"
                    self.countdown_label.style('color: red; font-weight: bold; animation: blink 0.5s infinite')
            
            if self.next_drone_label:
                self.next_drone_label.text = f"Next: {next_drone}"
        
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
            ui.label("🚁 Drone Fleet").classes('text-2xl font-bold')
            
            ui.separator()
            
            # Connection form
            with ui.expansion("➕ Add New Drone", icon='add_circle').classes('w-full'):
                with ui.row().classes('w-full items-end gap-2'):
                    self.ip_input = ui.input(
                        label='IP Address',
                        placeholder='192.168.x.x',
                        validation={'Invalid IP': lambda v: self._validate_ip(v)}
                    ).classes('flex-grow')
                    
                    self.namespace_input = ui.input(
                        label='Name',
                        placeholder='drone_1'
                    ).style('width: 100px')
                
                with ui.row().classes('w-full gap-2 mt-2'):
                    ui.button('🔗 Connect', on_click=self._connect_drone_ui).props('color=primary')
                    ui.button('🔄 Refresh', on_click=self._refresh_drone_list).props('flat')
            
            ui.separator()
            
            # Mission Status Card
            with ui.card().classes('w-full').style('background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: white;'):
                ui.label("📊 Mission Status").classes('text-lg font-bold')
                
                with ui.grid(columns=2).classes('w-full gap-2'):
                    self.mission_status_label = ui.label("⏸️ Inactive").classes('font-bold')
                    self.active_drone_label = ui.label("Active: --")
                
                self.countdown_label = ui.label("").classes('countdown-display mt-2')
                self.countdown_progress = ui.linear_progress(value=0).props('instant-feedback').classes('mt-1')
                
                self.drones_needed_label = ui.label("").classes('text-sm mt-1')
            
            ui.separator()
            
            # Drone list container
            ui.label("Connected Drones").classes('text-lg font-bold')
            self.drone_list_container = ui.column().classes('w-full gap-2')
            
            # Populate with existing drones
            self._refresh_drone_list()
    
    def _build_right_panel(self):
        """Build the right panel with map and mission control."""
        with ui.card().classes('h-full').style('flex: 3; display: flex; flex-direction: column;'):
            # Map container
            with ui.card().classes('w-full').style('flex: 1; min-height: 400px;'):
                self.map = ui.leaflet(
                    center=self.map_center,
                    zoom=15
                ).style('width: 100%; height: 100%;')
                
                # Add tile layer
                self.map.tile_layer(
                    url_template="http://127.0.0.1:8098/map/{z}/{x}/{y}.png",
                    options={'maxZoom': 20}
                )
                
                # Map click handler
                self.map.on('click', self._on_map_click)
            
            # Control panels row
            with ui.row().classes('w-full gap-2').style('flex-wrap: wrap;'):
                # Monitoring Point Control
                with ui.card().classes('').style('flex: 1; min-width: 300px;'):
                    ui.label("📍 Monitoring Point").classes('text-lg font-bold')
                    
                    with ui.row().classes('w-full gap-2 items-end flex-wrap'):
                        self.lat_input = ui.input(label='Lat', value='0.0').style('width: 100px')
                        self.lon_input = ui.input(label='Lon', value='0.0').style('width: 100px')
                        self.alt_input = ui.input(label='Alt (m)', value='50').style('width: 80px')
                        ui.button('📌', on_click=self._set_monitoring_point_manual).props('color=primary dense').tooltip('Set Point')
                        ui.button('🗑️', on_click=self._clear_monitoring_point_ui).props('dense').tooltip('Clear')
                    
                    ui.label("💡 Click on map to set point").classes('text-xs text-gray-500')
                
                # Mission Control
                with ui.card().classes('').style('flex: 1; min-width: 300px;'):
                    ui.label("🎯 Mission Control").classes('text-lg font-bold')
                    
                    with ui.row().classes('w-full gap-2 items-end'):
                        self.rth_alt_input = ui.input(label='RTH Alt (m)', value='50').style('width: 100px')
                        self.safety_buffer_input = ui.input(label='Buffer (s)', value='60').style('width: 80px')
                    
                    with ui.row().classes('w-full gap-2 mt-2'):
                        ui.button('▶️ Single', on_click=self._start_single_mission).props('color=green').tooltip('Start single drone mission')
                        ui.button('🔄 Relay', on_click=self._start_relay_mission).props('color=blue').tooltip('Start relay mission')
                        ui.button('⏹️ Stop', on_click=self._stop_mission_ui).props('color=red').tooltip('Stop all missions')
                
                # Trajectory Options
                with ui.card().classes('').style('flex: 1; min-width: 250px;'):
                    ui.label("🛤️ Trajectory").classes('text-lg font-bold')
                    
                    self.trajectory_mode = ui.toggle(
                        ['PID', 'DJI Native'], 
                        value='PID'
                    ).props('dense')
                    
                    with ui.row().classes('w-full gap-2 mt-2'):
                        ui.button('📍 Go to Point', on_click=self._goto_point_selected).props('dense')
                        ui.button('🛑 Abort', on_click=self._abort_trajectories).props('dense color=red')
            
            # Event Log
            with ui.card().classes('w-full').style('max-height: 150px;'):
                ui.label("📋 Event Log").classes('text-lg font-bold')
                with ui.scroll_area().classes('w-full').style('max-height: 100px').props('id=event-log') as self.event_scroll:
                    self.event_log = ui.column().classes('w-full gap-1')
    
    def _build_drone_card(self, namespace: str, drone: DroneData):
        """Build a card for a single drone."""
        color_idx = list(self.drones.keys()).index(namespace) % len(self.drone_colors)
        color = self.drone_colors[color_idx]
        
        with ui.card().classes('drone-card w-full') as card:
            self.drone_cards[namespace] = card
            self.drone_labels[namespace] = {}
            self.drone_buttons[namespace] = {}
            
            # Header with name and status
            with ui.row().classes('w-full items-center justify-between'):
                with ui.row().classes('items-center gap-2'):
                    ui.html(f'<div style="width: 12px; height: 12px; border-radius: 50%; background: {color}"></div>', sanitize=False)
                    ui.label(f"{namespace}").classes('text-lg font-bold')
                self.drone_labels[namespace]['state'] = ui.label(f"🟢 {drone.state.value}").classes('text-sm')
            
            # Telemetry grid
            with ui.grid(columns=3).classes('w-full gap-1 text-sm'):
                self.drone_labels[namespace]['battery'] = ui.label(f"🔋 {drone.battery_level:.1f}%")
                self.drone_labels[namespace]['altitude'] = ui.label(f"📏 {drone.altitude:.1f} m")
                self.drone_labels[namespace]['flight_time'] = ui.label("⏱️ --:--")
                self.drone_labels[namespace]['satellites'] = ui.label(f"🛰️ {drone.satellite_count}")
                self.drone_labels[namespace]['recording'] = ui.label("⚫ Standby").classes('col-span-2')
            
            self.drone_labels[namespace]['position'] = ui.label(
                f"📍 {drone.latitude:.6f}, {drone.longitude:.6f}"
            ).classes('text-xs text-gray-500')
            
            # Control buttons
            with ui.row().classes('w-full gap-1 mt-2'):
                ui.button('🛫', on_click=lambda ns=namespace: self.send_takeoff(ns)).props('dense size=sm').tooltip('Take Off')
                ui.button('🛬', on_click=lambda ns=namespace: self.send_land(ns)).props('dense size=sm').tooltip('Land')
                ui.button('🏠', on_click=lambda ns=namespace: self.send_rth(ns)).props('dense size=sm color=primary').tooltip('RTH')
                ui.button('⚠️', on_click=lambda ns=namespace: self.send_abort_mission(ns)).props('dense size=sm color=negative').tooltip('Abort')
                ui.button('🔴', on_click=lambda ns=namespace: self.send_start_recording(ns)).props('dense size=sm').tooltip('Record')
                ui.button('⏹️', on_click=lambda ns=namespace: self.send_stop_recording(ns)).props('dense size=sm').tooltip('Stop Rec')
            
            # Quick actions row
            with ui.row().classes('w-full gap-1'):
                ui.button('📍 Use Pos', on_click=lambda ns=namespace: self.set_monitoring_point_from_drone(ns)).props('dense size=sm flat').tooltip('Set as monitoring point')
                ui.button('❌ Disconnect', on_click=lambda ns=namespace: self._disconnect_drone_ui(ns)).props('dense size=sm flat color=negative')
            
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
        """Validate IP address format."""
        if not value:
            return True
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
        ip = self.ip_input.value
        if not ip or not self._validate_ip(ip):
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
            
        self.drone_list_container.clear()
        
        with self.drone_list_container:
            if not self.drones:
                ui.label("No drones connected").classes('text-gray-500 italic')
            else:
                for namespace, drone in self.drones.items():
                    self._build_drone_card(namespace, drone)
        
        # Update drones needed estimate
        self._update_drones_needed()
    
    def _update_drones_needed(self):
        """Update the estimate of drones needed for continuous coverage."""
        if self.drones_needed_label and self.monitoring_point.is_set:
            needed = self.mission_controller.calculate_drones_needed(1.0)
            if needed == float('inf'):
                self.drones_needed_label.text = "⚠️ Cannot calculate (point too far)"
            else:
                connected = len(self.drones)
                status = "✅" if connected >= needed else "⚠️"
                self.drones_needed_label.text = f"{status} Need {needed} drones for 1hr coverage ({connected} connected)"
    
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
        
        self.set_monitoring_point(lat, lon, alt, source="map")
        
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
            
            self.set_monitoring_point(lat, lon, alt, source="manual")
            ui.notify(f'Monitoring point set', type='positive')
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
        
        if self.start_monitoring_mission(drone_ns, rth_alt):
            self.mission_status_label.text = "▶️ Single Drone"
            self.active_drone_label.text = f"Active: {drone_ns}"
            ui.notify(f'Mission started', type='positive')
            self._emit_log(f"▶️ Single mission started with {drone_ns}")
        else:
            ui.notify('Failed to start mission', type='negative')
    
    def _start_relay_mission(self):
        """Start a relay mission with all connected drones."""
        if not self.monitoring_point.is_set:
            ui.notify('Please set a monitoring point first', type='warning')
            return
        
        if len(self.drones) < 2:
            ui.notify('Need at least 2 drones for relay mission', type='warning')
            return
        
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
        
        if self.start_relay_mission(drone_list, rth_alt):
            self.mission_status_label.text = f"🔄 Relay ({len(drone_list)})"
            self.active_drone_label.text = f"Active: {drone_list[0]}"
            ui.notify(f'Relay mission started with {len(drone_list)} drones', type='positive')
            self._emit_log(f"🔄 Relay mission started: {', '.join(drone_list)}")
        else:
            ui.notify('Failed to start relay mission', type='negative')
    
    def _stop_mission_ui(self):
        """Stop the current mission from UI."""
        self.stop_mission()
        self.mission_status_label.text = "⏹️ Stopped"
        self.countdown_label.text = ""
        ui.notify('Mission stopped', type='info')
        self._emit_log("⏹️ Mission stopped - drones returning home")
    
    def _goto_point_selected(self):
        """Send selected drones to the monitoring point."""
        if not self.monitoring_point.is_set:
            ui.notify('Set a monitoring point first', type='warning')
            return
        
        for namespace in self.drones.keys():
            if self.trajectory_mode.value == 'DJI Native':
                waypoints = [(
                    self.monitoring_point.latitude,
                    self.monitoring_point.longitude,
                    self.monitoring_point.altitude
                )]
                self.send_trajectory_dji_native(namespace, waypoints)
            else:
                self.send_goto_waypoint(
                    namespace,
                    self.monitoring_point.latitude,
                    self.monitoring_point.longitude,
                    self.monitoring_point.altitude,
                    0.0
                )
            self._emit_log(f"📍 {namespace} going to monitoring point")
    
    def _abort_trajectories(self):
        """Abort all trajectories."""
        for namespace in self.drones.keys():
            self.send_abort_mission(namespace)
        ui.notify('All trajectories aborted', type='info')
        self._emit_log("🛑 All trajectories aborted")


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
    favicon='🚁',
    port=8086,
    title='Perpetual Drone Monitoring'
)
