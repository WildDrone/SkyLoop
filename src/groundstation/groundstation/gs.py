import threading
from pathlib import Path
from folium import CircleMarker
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool

from nicegui import ui, app, ui_run
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Empty, String
import ast

###### ARROW DISPLAY ######
# Add the static files route

app.add_static_files(
    '/static', '/WildPerpetua/src/groundstation/groundstation/static')


class Arrow:
    def __init__(self, map_ui, id, lat, lng, heading, drones_arrows: dict):
        """
        Initialize an arrow on the given map.

        :param map_ui: The NiceGUI Leaflet map instance.
        :param id: Unique identifier for the arrow.
        :param lat: Latitude of the arrow's initial position.
        :param lng: Longitude of the arrow's initial position.
        :param heading: Initial heading of the arrow (in degrees).
        """
        self.map_ui = map_ui
        self.id = id
        self.lat = lat
        self.lng = lng
        self.heading = heading

        if id in drones_arrows:
            raise ValueError(
                "Condition non remplie, annulation de l'initialisation.")

    def _place_arrow(self):
        """Place the arrow on the map."""
        ui.run_javascript(
            f"place_arrow({self.map_ui.id}, {self.lat}, {self.lng}, {self.heading}, '{self.id}')"
        )

    def update(self, lat, lng, heading):
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


class ROSNode(Node):
    """ROS2 Node that handles subscriptions and publications only."""

    _instance = None

    @staticmethod
    def get_instance():
        if ROSNode._instance is None:
            ROSNode._instance = ROSNode()
        return ROSNode._instance

    def __init__(self) -> None:
        super().__init__('wildview_ground_station')

        # Get the parameters from the launch file
        # Default namespaces and IPs for 4 drones
        default_namespaces = 'drone1,drone2,drone3,drone4'
        default_ip_rcs = '192.168.1.1,192.168.1.2,192.168.1.3,192.168.1.4'
        
        namespaces_param = self.declare_parameter(
            'namespaces', default_namespaces).get_parameter_value().string_value
        ip_rcs_param = self.declare_parameter(
            'ip_rcs', default_ip_rcs).get_parameter_value().string_value
        
        # Handle empty parameters by using defaults
        namespaces = namespaces_param.split(',') if namespaces_param else default_namespaces.split(',')
        ip_rcs = ip_rcs_param.split(',') if ip_rcs_param else default_ip_rcs.split(',')
        
        # Filter out any empty strings
        namespaces = [ns.strip() for ns in namespaces if ns.strip()]
        ip_rcs = [ip.strip() for ip in ip_rcs if ip.strip()]

        self.get_logger().info(f'Namespaces: {namespaces}')
        self.get_logger().info(f'IP RCs: {ip_rcs}')

        # Create a list of streaming addresses for the drones
        self.streaming_adress = []
        port_base = 8000

        for ip_rc in ip_rcs:
            try:
                ip_parts = list(map(int, ip_rc.split('.')))
                http_port = port_base + (ip_parts[2] * 256 + ip_parts[3]) % 1000
                self.streaming_adress.append(
                    f'http://localhost:{http_port}/video_feed')
            except (ValueError, IndexError) as e:
                self.get_logger().warning(f'Invalid IP address format: {ip_rc}, using default port')
                self.streaming_adress.append(f'http://localhost:{port_base}/video_feed')

        self.centroid_lat = 0.025324 
        self.centroid_lng = 36.868363
        self.namespaces = namespaces

        # Data storage for UI updates
        self.battery_levels = {}
        self.altitudes = {}
        self.positions = {}  # {namespace: {'lat': float, 'lng': float}}
        self.headings = {}
        self.recording_status = {}
        self.current_state = "State Machine not Launched"
        self.herd_configuration = []
        self.waypoints = []
        self.trajectories = {}

        # UI reference (set by the page)
        self.ui_handler = None

        # Initialize subscribers
        self._setup_subscribers()
        self._setup_publishers()

    def _setup_subscribers(self):
        """Set up all ROS2 subscribers."""
        self.battery_subscribers = {}
        self.navsat_subscribers = {}
        self.heading_subscribers = {}
        self.trajectory_subscribers = {}
        self.recording_subscribers = {}

        for drone_namespace in self.namespaces:
            # Battery level subscription
            self.battery_subscribers[drone_namespace] = self.create_subscription(
                Float64,
                f"{drone_namespace}/battery_level",
                lambda msg, ns=drone_namespace: self._on_battery(ns, msg.data),
                10
            )

            # Recording status subscription
            self.recording_subscribers[drone_namespace] = self.create_subscription(
                Bool,
                f"{drone_namespace}/camera/is_recording",
                lambda msg, ns=drone_namespace: self._on_recording(ns, msg.data),
                10
            )

            # NavSatFix subscription
            self.navsat_subscribers[drone_namespace] = self.create_subscription(
                NavSatFix,
                f"{drone_namespace}/location",
                lambda msg, ns=drone_namespace: self._on_navsat(ns, msg),
                10
            )

            # Heading subscription
            self.heading_subscribers[drone_namespace] = self.create_subscription(
                Float64,
                f"{drone_namespace}/heading",
                lambda msg, ns=drone_namespace: self._on_heading(ns, msg.data),
                10
            )

            # Trajectory subscription
            self.trajectory_subscribers[drone_namespace] = self.create_subscription(
                String,
                f"{drone_namespace}/trajectory_to_next_wp",
                lambda msg, ns=drone_namespace: self._on_trajectory(ns, msg.data),
                10
            )

        # Global subscribers
        self.animal_subscriber = self.create_subscription(
            String, 'herd_configuration', self._on_herd_config, 10
        )

        self.state_machine_subscriber = self.create_subscription(
            String, 'state_machine_current_state', 
            lambda msg: self._on_state_change(msg.data), 10
        )

        self.pso_waypoint_subscriber = self.create_subscription(
            String, 'swarm_configuration_pso', self._on_waypoints, 10
        )

    def _setup_publishers(self):
        """Set up all ROS2 publishers."""
        self.publisher_user = self.create_publisher(Bool, 'user_validation_bool', 10)
        self.publish_user_trajectory = self.create_publisher(Bool, 'user_validation_trajectory_bool', 10)
        self.publisher_abort_mission = self.create_publisher(Bool, 'abort_mission_bool', 10)
        self.publisher_waypoints_reached = self.create_publisher(Bool, 'waypoints_reached_bool_user', 10)
        self.publisher_leader = self.create_publisher(Bool, 'leader_validation_bool', 10)

    # Callback methods that store data and notify UI
    def _on_battery(self, namespace: str, level: float):
        self.battery_levels[namespace] = level
        if self.ui_handler:
            self.ui_handler.update_battery(namespace, level)

    def _on_recording(self, namespace: str, is_recording: bool):
        self.recording_status[namespace] = is_recording
        if self.ui_handler:
            self.ui_handler.update_recording(namespace, is_recording)

    def _on_navsat(self, namespace: str, msg: NavSatFix):
        self.positions[namespace] = {'lat': msg.latitude, 'lng': msg.longitude}
        self.altitudes[namespace] = msg.altitude
        if self.ui_handler:
            self.ui_handler.update_position(namespace, msg.latitude, msg.longitude, msg.altitude)

    def _on_heading(self, namespace: str, heading: float):
        self.headings[namespace] = heading
        if self.ui_handler:
            self.ui_handler.update_heading(namespace, heading)

    def _on_trajectory(self, namespace: str, data: str):
        try:
            waypoints = ast.literal_eval(data)
            self.trajectories[namespace] = waypoints
            if self.ui_handler:
                self.ui_handler.update_trajectory(namespace, waypoints)
        except Exception as e:
            self.get_logger().error(f"Error parsing trajectory: {e}")

    def _on_herd_config(self, msg: String):
        try:
            raw = ast.literal_eval(msg.data)
            self.herd_configuration = [
                {'x': a['x'], 'z': a['z'], 'heading': a['direction']} 
                for a in raw
            ]
            if self.ui_handler:
                self.ui_handler.update_animals(self.herd_configuration)
        except Exception as e:
            self.get_logger().error(f"Error parsing herd config: {e}")

    def _on_state_change(self, state: str):
        self.current_state = state
        if self.ui_handler:
            self.ui_handler.update_state(state)

    def _on_waypoints(self, msg: String):
        try:
            self.waypoints = ast.literal_eval(msg.data)
            if self.ui_handler:
                self.ui_handler.update_waypoints(self.waypoints)
        except Exception as e:
            self.get_logger().error(f"Error parsing waypoints: {e}")

    # Command methods
    def send_drone_command(self, namespace: str, command: str):
        topic_name = f"{namespace}/command/{command}"
        publisher = self.create_publisher(Empty, topic_name, 10)
        publisher.publish(Empty())
        self.get_logger().info(f"Command '{command}' sent to '{namespace}'")

    def send_mode_command(self, mode: str):
        publisher = self.create_publisher(String, 'change_mode', 10)
        publisher.publish(String(data=mode))
        self.get_logger().info(f"Mode command '{mode}' sent")

    def send_user_validation_force(self):
        self.publisher_user.publish(Bool(data=True))
        self.get_logger().info("User validation sent - Force Mode")

    def send_leader_validation(self):
        self.publisher_leader.publish(Bool(data=True))
        self.get_logger().info("Leader validation sent")

    def send_user_validation_trajectory(self):
        self.publish_user_trajectory.publish(Bool(data=True))
        self.get_logger().info("User validation sent - Trajectory mode")

    def send_waypoints_reached(self):
        self.publisher_waypoints_reached.publish(Bool(data=True))
        self.get_logger().info("Waypoints reached signal sent")

    def send_abort_mission(self):
        self.publisher_abort_mission.publish(Bool(data=True))
        self.get_logger().info("Abort mission signal sent")


class UIHandler:
    """Handles UI updates from the main thread."""
    
    def __init__(self, ros_node: ROSNode):
        self.ros_node = ros_node
        self.namespaces = ros_node.namespaces
        
        # UI element references
        self.drone_info_labels = {}
        self.drone_recording_labels = {}
        self.state_label = None
        self.map = None
        self.markers = []
        self.drone_arrows = {}
        self.markers_next_waypoint = {}
        self.markers_trajectory = {}

    def update_battery(self, namespace: str, level: float):
        if namespace in self.drone_info_labels:
            current_text = self.drone_info_labels[namespace].text
            parts = current_text.split("||")
            if len(parts) >= 3:
                altitude_part = parts[2]
                self.drone_info_labels[namespace].text = f"Drone {namespace[-1]} || Batterie : {level:.2f} % || {altitude_part.strip()}"

    def update_recording(self, namespace: str, is_recording: bool):
        if namespace in self.drone_recording_labels:
            self.drone_recording_labels[namespace].text = f"{namespace} : Recording : {is_recording}"

    def update_position(self, namespace: str, lat: float, lng: float, altitude: float):
        if namespace in self.drone_info_labels:
            current_text = self.drone_info_labels[namespace].text
            parts = current_text.split("||")
            if len(parts) >= 2:
                battery_part = parts[1]
                self.drone_info_labels[namespace].text = f"Drone {namespace[-1]} || {battery_part.strip()} || Altitude : {altitude:.2f} m"
        
        if namespace in self.drone_arrows:
            heading = self.ros_node.headings.get(namespace, 0.0)
            self.drone_arrows[namespace].lat = lat
            self.drone_arrows[namespace].lng = lng
            self.drone_arrows[namespace].update(lat, lng, heading)

    def update_heading(self, namespace: str, heading: float):
        if namespace in self.drone_arrows:
            arrow = self.drone_arrows[namespace]
            arrow.update(arrow.lat, arrow.lng, heading)

    def update_trajectory(self, namespace: str, waypoints: list):
        try:
            latlngs = [(wp['lat'], wp['lng']) for wp in waypoints]
            if namespace in self.markers_trajectory:
                self.map.remove_layer(self.markers_trajectory[namespace])
            self.markers_trajectory[namespace] = self.map.generic_layer(
                name='polyline',
                args=[latlngs, {'color': 'blue', 'weight': 5}]
            )
        except Exception as e:
            print(f"Error updating trajectory: {e}")

    def update_animals(self, herd_configuration: list):
        # Remove old markers
        for marker in self.markers:
            self.map.remove_layer(marker)
        self.markers.clear()

        # Add new markers
        for i, animal in enumerate(herd_configuration):
            marker = self.map.generic_layer(
                name='circleMarker',
                args=[[animal['x'], animal['z']], {'color': 'red', 'weight': 2, 'id': i}],
            )
            self.markers.append(marker)

    def update_state(self, state: str):
        if self.state_label:
            self.state_label.text = f"State Machine: {state}"

    def update_waypoints(self, waypoints: list):
        for i, waypoint in enumerate(waypoints):
            lat, lng = waypoint['lat'], waypoint['lng']
            if i not in self.markers_next_waypoint:
                self.markers_next_waypoint[i] = self.map.marker(latlng=[lat, lng])
            else:
                self.markers_next_waypoint[i].move(lat=lat, lng=lng)


# Global ROS node reference
ros_node: ROSNode = None


def ros_main() -> None:
    """ROS2 spinning loop - runs in background thread."""
    global ros_node
    rclpy.init()
    ros_node = ROSNode.get_instance()
    rclpy.spin(ros_node)
    ros_node.destroy_node()
    rclpy.shutdown()


@ui.page('/')
def main_page():
    """Main UI page - created in main thread."""
    global ros_node
    
    # Wait briefly for ROS node to initialize
    import time
    max_wait = 5
    waited = 0
    while ros_node is None and waited < max_wait:
        time.sleep(0.1)
        waited += 0.1
    
    if ros_node is None:
        ui.label("Error: ROS node not initialized").classes('text-red text-2xl')
        return
    
    # Create UI handler and link to ROS node
    ui_handler = UIHandler(ros_node)
    ros_node.ui_handler = ui_handler
    namespaces = ros_node.namespaces
    
    # Add JavaScript for arrows
    ui.add_head_html("<script src='/static/arrows.js'></script>")
    
    # Main UI structure
    with ui.row().classes('w-full h-full').style('display: flex; height: 95vh; gap: 10px;'):

        # Left column: Drone feeds and controls
        with ui.card().classes('h-full').style('flex: 1; margin-right: 10px;'):

            for i in range(min(4, len(namespaces))):
                drone_namespace = namespaces[i]

                # Text labels
                ui_handler.drone_info_labels[drone_namespace] = ui.label(
                    f"Drone {i+1} || Batterie : --- % || Altitude : --- m"
                ).classes('text-xl')

                with ui.card().classes('w-full h-full').style('flex: 1; display: flex; flex-direction: row; gap: 10px;'):
                    with ui.card().classes('w-full h-full').style('flex: 1; display: flex; justify-content: center; align-items: center;'):
                        with ui.column().classes('display: flex; flex-direction: column;').style('gap: 10px;'):
                            ui.button("Take Off").props('color=green').style(
                                'object-fit: contain; width: 100%;').on(
                                'click', lambda ns=drone_namespace: ros_node.send_drone_command(ns, 'takeoff'))
                            ui.button("RTH").props('color=blue').style(
                                'object-fit: contain; width: 100%;').on(
                                'click', lambda ns=drone_namespace: ros_node.send_drone_command(ns, 'rth'))
                            ui.button("Land").props('color=orange').style(
                                'object-fit: contain; width: 100%;').on(
                                'click', lambda ns=drone_namespace: ros_node.send_drone_command(ns, 'land'))
                            ui.button("Abort Mission").props('color=red').style(
                                'object-fit: contain; width: 100%;').on(
                                'click', lambda ns=drone_namespace: ros_node.send_drone_command(ns, 'abort_mission'))

        # Right column: Satellite map and controls
        with ui.card().classes('h-full').style('flex: 4; display: flex; flex-direction: column; gap: 10px;'):

            # Satellite map with animal markers
            with ui.card().classes('w-full').style('aspect-ratio: 20/9; display: flex: 1;'):

                with ui.row().style('flex: 1; justify-content: center; align-items: center;'):
                    for i, namespace in enumerate(namespaces):
                        ui_handler.drone_recording_labels[namespace] = ui.label(
                            f"{namespace} : Recording : No Connection").classes('text-2xl text-center')

                ui_handler.map = ui.leaflet(
                    center=(ros_node.centroid_lat, ros_node.centroid_lng), zoom=15
                ).style('width: 100%; height: 100%;')

                ui_handler.map.tile_layer(
                    url_template="http://127.0.0.1:8098/map/{z}/{x}/{y}.png",
                    options={'maxZoom': 100},
                )

                # Create arrows for each drone
                for namespace in namespaces:
                    ui_handler.drone_arrows[namespace] = Arrow(
                        ui_handler.map, namespace, 0.0, 0.0, 0.0, 
                        drones_arrows=ui_handler.drone_arrows
                    )

            # Swarm control section
            with ui.card().classes('w-full h-full').style('flex: 1; align-items: center;'):
                with ui.row().classes('w-full h-full').style('display: flex; gap: 10px;'):

                    with ui.card().classes('h-full').style('flex: 0.5; margin-right: 10px; align-items: center; justify-content: center;'):
                        ui.label("State Control").classes('text-center; text-2xl')
                        with ui.row().classes('items-start').style('gap: 10px;'):
                            ui.button("Use Leader Position as Herd Detection").props('color=green').style(
                                'object-fit: contain; width: 100%;').on(
                                'click', lambda: ros_node.send_leader_validation())
                            ui_handler.state_label = ui.label("State Machine not Launched").classes('text-2xl text-center')

                    with ui.card().classes('w-full h-full').style('flex: 0.5; display: flex; justify-content: center; align-items: center;'):
                        with ui.column().classes('display: flex; flex-direction: column;').style('gap: 10px;'):
                            ui.label("PSO Input Type").classes('text-center; text-2xl')
                            ui.button("Leader").props('color=green').style(
                                'object-fit: contain; width: 100%;').on(
                                'click', lambda: ros_node.send_mode_command('LEADER_DEPLOYMENT'))
                            ui.button("Labelling").props('color=blue').style(
                                'object-fit: contain; width: 100%;').on(
                                'click', lambda: ros_node.send_mode_command('LABELLING_DEPLOYMENT'))
                            ui.button("Machine Vision").props('color=orange').style(
                                'object-fit: contain; width: 100%;').on(
                                'click', lambda: ros_node.send_mode_command('MACHINE_VISION_DEPLOYMENT'))

                    with ui.card().classes('h-full').style('flex: 1; margin-right: 10px; align-items: center; justify-content: center;'):
                        ui.label("PSO Control Panel").classes('text-center; text-2xl')
                        with ui.row().classes('w-full h-full').style('display: flex; gap: 10px;'):
                            with ui.card().classes('h-full').style('flex: 1; margin-right: 10px; align-items: center; justify-content: center;'):
                                with ui.row().classes('items-start').style('gap: 10px;'):
                                    ui.button("Go to next monitoring waypoint").props('color=green').style(
                                        'object-fit: contain; width: 100%;').on(
                                        'click', lambda: ros_node.send_user_validation_trajectory())
                                    ui.button("Force Go to next monitoring waypoint").props('color=red').style(
                                        'object-fit: contain; width: 100%;').on(
                                        'click', lambda: ros_node.send_user_validation_force())
                                    ui.button("All waypoints reached").props('color=orange').style(
                                        'object-fit: contain; width: 100%;').on(
                                        'click', lambda: ros_node.send_waypoints_reached())

                            with ui.card().classes('h-full').style('flex: 1; margin-right: 10px; align-items: center; justify-content: center;'):
                                ui.label("Data Collection").classes('text-center; text-2xl')
                                with ui.column().classes('display: flex; flex-direction: column;').style('gap: 10px;'):
                                    ui.button("Start Video Recording").props('color=green').style(
                                        'object-fit: contain; width: 100%;').on(
                                        'click', lambda: [ros_node.send_drone_command(ns, 'camera/start_recording') for ns in namespaces])
                                    ui.button("Stop Video Recording").props('color=red').style(
                                        'object-fit: contain; width: 100%;').on(
                                        'click', lambda: [ros_node.send_drone_command(ns, 'camera/stop_recording') for ns in namespaces])


def main() -> None:
    pass


# Start the ROS2 node in a background thread
@app.on_startup
def on_startup():
    threading.Thread(target=ros_main, daemon=True).start()


# Handle ROS2 module naming conventions
ui_run.APP_IMPORT_STRING = f'{__name__}:app'
ui.run(uvicorn_reload_dirs=str(
    Path(__file__).parent.resolve()), favicon='🐾', port=8085)
