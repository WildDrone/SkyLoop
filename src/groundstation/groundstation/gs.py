import threading
from pathlib import Path
from folium import CircleMarker
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool

from nicegui import ui, app, Client, ui_run
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Empty, String
import ast

###### ARROW DISPLAY ######
# Add the static files route


class Arrow:
    def __init__(self, map_ui, id, lat, lng, heading, client: Client, drones_arrows: dict):
        """
        Initialize an arrow on the given map.

        :param map_ui: The NiceGUI Leaflet map instance.
        :param id: Unique identifier for the arrow.
        :param lat: Latitude of the arrow's initial position.
        :param lng: Longitude of the arrow's initial position.
        :param heading: Initial heading of the arrow (in degrees).
        :param client: The NiceGUI client to run JavaScript in the correct context.
        """
        self.map_ui = map_ui
        self.id = id
        self.lat = lat
        self.lng = lng
        self.heading = heading
        self.client = client  # Store the client instance
        self._place_arrow()

        if id in drones_arrows:
            raise ValueError(
                "Condition non remplie, annulation de l'initialisation.")

    def _place_arrow(self):
        """Place the arrow on the map."""
        self.client.run_javascript(
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
        self.client.run_javascript(
            f"update_arrow_test('{self.id}', {self.lat}, {self.lng}, {self.heading})"
        )

    def destroy(self):
        """Remove the arrow from the map."""
        self.client.run_javascript(f"delete_arrow('{self.id}')")


app.add_static_files(
    '/static', 'src/groundstation/groundstation/static')


def setup_ui():
    # Initialize the UI setup in the main thread
    ui.add_head_html("<script src='/static/arrows.js'></script>")


class NiceGuiNode(Node):

    ###### SINGLETON PATTERN ######
    _instance = None

    def get_instance():
        if NiceGuiNode._instance is None:
            NiceGuiNode._instance = NiceGuiNode()
            return NiceGuiNode._instance
        else:
            return NiceGuiNode._instance

    def __init__(self) -> None:
        super().__init__('wildview_ground_station')

        # Get the parameters from the launch file
        namespaces = self.declare_parameter(
            'namespaces', '').get_parameter_value().string_value.split(',')
        ip_rcs = self.declare_parameter(
            'ip_rcs', '').get_parameter_value().string_value.split(',')

        self.get_logger().info(f'Namespaces: {namespaces}')
        self.get_logger().info(f'IP RCs: {ip_rcs}')

        # Create a list of streaming addresses for the drones, based on the logic implement in the RTSP Node
        self.streaming_adress = []
        port_base = 8000

        for ip_rc in ip_rcs:
            ip_parts = list(map(int, ip_rc.split('.')))
            http_port = port_base + (ip_parts[2] * 256 + ip_parts[3]) % 1000
            self.streaming_adress.append(
                f'http://localhost:{http_port}/video_feed')

        # Initialize dictionaries to store labels and subscribers
        self.battery_labels = {}
        self.altitude_labels = {}
        self.drone_info_labels = {}
        self.drone_recording_subscribers = {}

        self.battery_subscribers = {}
        self.navsat_subscribers = {}
        self.heading_subscribers = {}
        self.drones_heading_subscribers = {}

        self.trajectory_subscriber = {}

        self.centroid_lat = 0.025324 
        self.centroid_lng = 36.868363

        self.namespaces = namespaces

        # Initialise the subscribers for the drones
        for i in range(len(namespaces)):

            drone_namespace = namespaces[i]

            topic_name_battery = f"{drone_namespace}/battery_level"
            topic_name_navsat = f"{drone_namespace}/location"
            topic_name_heading = f"{drone_namespace}/heading"
            topic_name_trajectory = f"{drone_namespace}/trajectory_to_next_wp"
            topic_name_recording = f"{drone_namespace}/camera/is_recording"

            # Battery level subscription
            self.battery_subscribers[drone_namespace] = self.create_subscription(
                Float64,
                topic_name_battery,
                lambda msg, namespace=drone_namespace: self.update_battery_display(
                    namespace, msg.data),
                10
            )

            self.drone_recording_subscribers[drone_namespace] = self.create_subscription(
                Bool,
                topic_name_recording,
                lambda msg, namespace=drone_namespace: self.update_recording_display(
                    namespace, msg.data),
                10
            )

            # NavSatFix subscription for altitude and position
            self.navsat_subscribers[drone_namespace] = self.create_subscription(
                NavSatFix,
                topic_name_navsat,
                lambda msg, namespace=drone_namespace: self.update_navsat_display(
                    namespace, msg),
                10
            )

            # FLoat64 subscription for heading
            self.drones_heading_subscribers[drone_namespace] = self.create_subscription(
                Float64,
                topic_name_heading,
                lambda msg, namespace=drone_namespace: self.update_drone_heading(
                    namespace, msg.data),
                10
            )

            self.trajectory_subscriber[drone_namespace] = self.create_subscription(
                String, topic_name_trajectory,
                lambda msg, namespace=drone_namespace: self.update_trajectory(
                    namespace, msg.data),
                10
            )

        # Subscriber for animal coordinates
        self.animal_subscriber = self.create_subscription(
            String, 'herd_configuration', self.update_animal_positions, 10
        )

        self.state_machine_subscriber = self.create_subscription(
            String, 'state_machine_current_state', lambda msg: self.update_state_display(msg.data), 10
        )

        self.pso_waypoint_subscriber = self.create_subscription(
            String, 'swarm_configuration_pso', self.update_next_waypoint, 10)

        self.publisher_user = self.create_publisher(
            Bool, 'user_validation_bool', 10)

        self.publish_user_trajectory = self.create_publisher(
            Bool, 'user_validation_trajectory_bool', 10)

        self.publisher_abort_mission = self.create_publisher(
            Bool, 'abort_mission_bool', 10)

        self.publisher_waypoints_reached = self.create_publisher(
            Bool, 'waypoints_reached_bool_user', 10)

        self.publisher_leader = self.create_publisher(
            Bool, 'leader_validation_bool', 10)

        ###### ARROW DISPLAY ######
        self.drone_arrows = {}

        # Initialize the map and animal markers
        with Client.auto_index_client:

            # Main UI structure
            with ui.row().classes('w-full h-full').style('display: flex; height: 95vh; gap: 10px;'):

                # Left column: Drone feeds and controls
                with ui.card().classes('h-full').style('flex: 1; margin-right: 10px;'):

                    for i in range(4):

                        drone_namespace = namespaces[i]

                        # Text labels
                        self.drone_info_labels[drone_namespace] = ui.label(
                            f"Drone {i+1} || Batterie : --- % || Altitude : --- m"
                        ).classes('text-xl')

                        with ui.card().classes('w-full h-full').style('flex: 1; display: flex; flex-direction: row; gap: 10px;'):

                            #with ui.card().classes('w-full h-full').style('flex: 1; display: flex; flex-direction: column; gap: 10px;'):
                            #    ui.image(self.streaming_adress[i]).style(
                            #        'height: 100%; object-fit: contain; border: none;'
                            #    )

                            with ui.card().classes('w-full h-full').style('flex: 1; display: flex; justify-content: center; align-items: center;'):

                                with ui.column().classes('display: flex; flex-direction: column;').style('gap: 10px;'):

                                    ui.button(f"Take Off").props('color=green').style(
                                        'object-fit: contain; width: 100%;').on('click', lambda namespace=drone_namespace: self.send_drone_command(namespace, 'takeoff'))
                                    ui.button(f"RTH").props('color=blue').style(
                                        'object-fit: contain; width: 100%;').on('click', lambda namespace=drone_namespace: self.send_drone_command(namespace, 'rth'))
                                    ui.button(f"Land").props('color=orange').style(
                                        'object-fit: contain; width: 100%;').on('click', lambda namespace=drone_namespace: self.send_drone_command(namespace, 'land'))
                                    ui.button(f"Abort Mission").props('color=red').style(
                                        'object-fit: contain; width: 100%;').on('click', lambda namespace=drone_namespace: self.send_drone_command(namespace, 'abort_mission'))

                # Right column: Satellite map and controls
                with ui.card().classes('h-full').style('flex: 4; display: flex; flex-direction: column; gap: 10px;'):

                    # Satellite map with animal markers
                    with ui.card().classes('w-full').style('aspect-ratio: 20/9; display: flex: 1;'):

                        with ui.row().style('flex: 1; justify-content: center; align-items: center;'):
                            # Initialisation des labels de statut d'enregistrement pour chaque drone
                            self.drone_recording_labels = {}
                            for i, namespace in enumerate(namespaces):
                                self.drone_recording_labels[namespace] = ui.label(
                                    f"{namespace} : Recording : No Connection").classes('text-2xl text-center')

                        self.map = ui.leaflet(center=(self.centroid_lat, self.centroid_lng), zoom=15).style(
                            'width: 100%; height: 100%;'
                        )

                        # Ajouter une couche satellite Esri
                        self.map.tile_layer(
                            url_template="http://127.0.0.1:8098/map/{z}/{x}/{y}.png",
                            options={
                                'maxZoom': 100
                            },
                        )

                        self.markers = []  # List to store map markers
                        self.drone_arrows = {}  # Dictionary to store drone arrows
                        self.markers_next_waypoint = {}  # Dictionary to store next waypoint markers
                        self.markers_trajectory = {}

                        for namespace in namespaces:
                            self.drone_arrows[namespace] = Arrow(
                                self.map, namespace, 0.0, 0.0, 0.0, client=Client.auto_index_client, drones_arrows=self.drone_arrows)

                    # Swarm control section
                    with ui.card().classes('w-full h-full').style('flex: 1; align-items: center;'):
                        with ui.row().classes('w-full h-full').style('display: flex; gap: 10px;'):

                            # Left column: Drone feeds and controls
                            with ui.card().classes('h-full').style('flex: 0.5; margin-right: 10px; align-items: center; justify-content: center;'):
                                ui.label("State Control").classes(
                                    'text-center; text-2xl')
                                with ui.row().classes('items-start').style('gap: 10px;'):
                                    ui.button("Use Leader Position as Herd Detection").props('color=green').style(
                                        'object-fit: contain; width: 100%;').on('click', lambda: self.send_leader_validation())
                                    self.state_label = ui.label("State Machine not Launched").classes('text-2xl text-center')

                            with ui.card().classes('w-full h-full').style('flex: 0.5; display: flex; justify-content: center; align-items: center;'):

                                with ui.column().classes('display: flex; flex-direction: column;').style('gap: 10px;'):
                                    ui.label("PSO Input Type").classes(
                                        'text-center; text-2xl')
                                    ui.button(f"Leader").props('color=green').style(
                                        'object-fit: contain; width: 100%;').on('click', lambda: self.send_mode_command('LEADER_DEPLOYMENT'))
                                    ui.button(f"Labelling").props('color=blue').style(
                                        'object-fit: contain; width: 100%;').on('click', lambda: self.send_mode_command('LABELLING_DEPLOYMENT'))
                                    ui.button(f"Machine Vision").props('color=orange').style(
                                        'object-fit: contain; width: 100%;').on('click', lambda: self.send_mode_command('MACHINE_VISION_DEPLOYMENT'))

                            with ui.card().classes('h-full').style('flex: 1; margin-right: 10px; align-items: center; justify-content: center;'):
                                ui.label("PSO Control Panel").classes(
                                    'text-center; text-2xl')
                                with ui.row().classes('w-full h-full').style('display: flex; gap: 10px;'):
                                    with ui.card().classes('h-full').style('flex: 1; margin-right: 10px; align-items: center; justify-content: center;'):

                                        with ui.row().classes('items-start').style('gap: 10px;'):
                                            ui.button("Go to next monitoring waypoint").props('color=green').style(
                                                'object-fit: contain; width: 100%;').on('click', lambda: self.send_user_validation_trajectory())

                                            ui.button("Force Go to next monitoring waypoint").props('color=red').style(
                                                'object-fit: contain; width: 100%;').on('click', lambda: self.send_user_validation_force())

                                            ui.button("All waypoints reached").props('color=orange').style(
                                                'object-fit: contain; width: 100%;').on('click', lambda: self.send_waypoints_reached())

                                    with ui.card().classes('h-full').style('flex: 1; margin-right: 10px; align-items: center; justify-content: center;'):
                                        ui.label("Data Collection").classes(
                                            'text-center; text-2xl')
                                        with ui.column().classes('display: flex; flex-direction: column;').style('gap: 10px;'):

                                            ui.button("Start Video Recording").props('color=green').style(
                                                'object-fit: contain; width: 100%;').on('click', lambda: self.send_start_video_recording())

                                            ui.button("Stop Video Recording").props('color=red').style(
                                                'object-fit: contain; width: 100%;').on('click', lambda: self.send_stop_video_recording())

                                            
        NiceGuiNode._instance = self

    def update_animal_positions(self, msg: String):
        """Callback to update animal positions on the map."""

        # Convertir le message String en liste de dictionnaires
        herd_configuration_raw = ast.literal_eval(msg.data)

        herd_configuration = []
        for animal in herd_configuration_raw:
            animal_dict = {
                'x': animal['x'],
                'z': animal['z'],
                'heading': animal['direction']
            }
            herd_configuration.append(animal_dict)

        if not self.markers:
            #here we do nothing as we don't have any markers. 
            pass
        
        else:
            # We remove the markers from the map
            # we remove layer by layer
            for marker in self.markers:
                self.map.remove_layer(marker)
            self.markers.clear()

        # Then, we create a vector layer to display the animals
        for i, animal in enumerate(herd_configuration):
            latitude = animal['x']
            longitude = animal['z']
            # Add or update markers on the map
            polyline_layer_i = self.map.generic_layer(
                name='circleMarker',
                args=[[latitude, longitude], {'color': 'red', 'weight': 2, 'id': i}],
            )
            self.markers.append(polyline_layer_i)

        self.centroid_lat = sum(
            animal['x'] for animal in herd_configuration) / len(herd_configuration)
        self.centroid_lng = sum(
            animal['z'] for animal in herd_configuration) / len(herd_configuration)

    def update_next_waypoint(self, msg: String):
        """
        Updates the next waypoints for the drones on the map using the incoming ROS message.

        :param msg: ROS message containing waypoint data in string format.
                    Example format:
                    [{'lat': <latitude>, 'lng': <longitude>, 'y': <altitude>, 'heading': <heading>}, ...]
        """
        try:
            # Parse the incoming waypoint data
            waypoints = ast.literal_eval(msg.data)
            self.get_logger().info(f"Received waypoint data: {waypoints}")

            # Loop through waypoints and update their markers
            for i, waypoint in enumerate(waypoints):
                latitude = waypoint['lat']
                longitude = waypoint['lng']

                # Add or update markers on the map
                if i not in self.markers_next_waypoint:
                    self.markers_next_waypoint[i] = self.map.marker(
                        latlng=[latitude, longitude])
                else:
                    self.markers_next_waypoint[i].move(
                        lat=latitude, lng=longitude)

        except Exception as e:
            self.get_logger().error(f"Error updating waypoints: {str(e)}")

    def update_trajectory(self, namespace, msg: String):
        """
        Updates the next waypoints for the drones on the map using the incoming ROS message.

        :param msg: ROS message containing waypoint data in string format.
                    Example format:
                    [{'lat': <latitude>, 'lng': <longitude>, 'y': <altitude>, 'heading': <heading>}, ...]
        """
        try:
            # Parse the incoming waypoint data
            waypoints = ast.literal_eval(msg)
            self.get_logger().info(
                f"Received waypoint data from the trajectory: {waypoints}")

            # Extract latitude and longitude from waypoints
            latlngs = [(waypoint['lat'], waypoint['lng'])
                       for waypoint in waypoints]

            # Ajout de la polyline rouge à la carte en utilisant generic_layer
            if 'trajectory_polyline' not in self.markers_trajectory:
                self.markers_trajectory[namespace] = self.map.generic_layer(
                    name='polyline',
                    args=[latlngs, {'color': 'blue', 'weight': 5}]
                )
            else:
                self.map.remove_layer(
                    self.markers_trajectory[namespace])
                self.markers_trajectory['trajectory_polyline'] = self.map.generic_layer(
                    name='polyline',
                    args=[latlngs, {'color': 'blue', 'weight': 5}]
                )

        except Exception as e:
            self.get_logger().error(f"Error updating waypoints: {str(e)}")

    def update_drone_positions(self, namespace: str, lat: float, lng: float, heading: float):

        if namespace in self.drone_arrows:

            self.drone_arrows[namespace].update(lat, lng, heading)

    def update_battery_display(self, namespace: str, battery_level: float):
        """Update the unified display with battery level."""
        if namespace in self.drone_info_labels:
            # Update the label with new battery data
            current_text = self.drone_info_labels[namespace].text
            altitude_part = current_text.split(
                "||")[2]  # Preserve the altitude part
            self.drone_info_labels[namespace].text = f"Drone {namespace[-1]} || Batterie : {battery_level:.2f} % || {altitude_part.strip()}"

    def update_drone_heading(self, namespace: str, heading: float):
        """Callback to handle heading updates."""
        if namespace in self.drone_arrows:
            current_lat = self.drone_arrows[namespace].lat
            current_lng = self.drone_arrows[namespace].lng
            self.update_drone_positions(
                namespace, current_lat, current_lng, heading)

    def update_navsat_display(self, namespace: str, navsat_msg: NavSatFix):
        """Callback to handle NavSatFix messages."""
        lat = navsat_msg.latitude
        lng = navsat_msg.longitude
        altitude = navsat_msg.altitude

        # Update the altitude display
        if namespace in self.drone_info_labels:
            current_text = self.drone_info_labels[namespace].text
            battery_part = current_text.split("||")[1]
            self.drone_info_labels[namespace].text = (
                f"Drone {namespace[-1]} || {battery_part.strip()} || Altitude : {altitude:.2f} m"
            )

        # Update the drone position on the map (heading will be updated in another callback)
        if namespace in self.drone_arrows:
            current_heading = self.drone_arrows[namespace].heading
            self.update_drone_positions(namespace, lat, lng, current_heading)

    def update_recording_display(self, namespace: str, is_recording: bool):
        """
        Met à jour l'affichage du statut d'enregistrement pour un drone spécifique.

        :param namespace: Namespace du drone (identifiant unique).
        :param is_recording: Booléen indiquant si l'enregistrement est actif.
        """
        if namespace in self.drone_recording_labels:
            # Mettre à jour le texte du label correspondant
            self.drone_recording_labels[namespace].text = f"{namespace} : Recording : {is_recording}"
        else:
            self.get_logger().warning(
                f"Namespace {namespace} non trouvé pour la mise à jour du statut d'enregistrement.")
    
    def update_state_display(self, state: str):
        self.state_label.text = f"State Machine: {state}"

    def send_drone_command(self, namespace: str, command: str):
        """Publish a command to a drone."""
        topic_name = f"{namespace}/command/{command}"
        publisher = self.create_publisher(Empty, topic_name, 10)
        publisher.publish(Empty())
        self.get_logger().info(
            f"Command '{command}' sent to the drone in the following namespace: '{namespace}'.")

    def send_mode_command(self, mode: str):
        topic_name = 'change_mode'
        publisher = self.create_publisher(String, topic_name, 10)
        publisher.publish(String(data=mode))
        self.get_logger().info(
            f"Command '{mode}' sent to the swarm controller.")

    def send_swarm_command(self, command: str):
        """
        Send a command to all drones in the swarm.

        :param command: The command to send (e.g., 'start_mission', 'rth', 'land', 'abort_mission').
        """
        for namespace in self.battery_subscribers.keys():
            topic_name = f"{namespace}/command/{command}"
            publisher = self.create_publisher(Empty, topic_name, 10)
            publisher.publish(Empty())
            self.get_logger().info(
                f"Swarm command '{command}' sent to the drone in namespace: '{namespace}'.")

    def send_user_validation_force(self):
        self.publisher_user.publish(Bool(data=True))
        self.get_logger().info(
            f"User validation sent to the swarm controller - Force Mode")

    def send_leader_validation(self):
        self.publisher_leader.publish(Bool(data=True))
        self.get_logger().info(
            f"Leader validation sent to the swarm controller")

    def send_user_validation_trajectory(self):
        self.publish_user_trajectory.publish(Bool(data=True))
        self.get_logger().info(
            f"User validation sent to the swarm controller, with the trajectory mode")

    def send_waypoints_reached(self):
        self.publisher_waypoints_reached.publish(Bool(data=True))
        self.get_logger().info(
            f"Waypoints reached signal sent to the state machine.")

    def send_abort_mission(self):
        self.publisher_abort_mission.publish(Bool(data=True))
        self.get_logger().info("Abort mission signal sent to the state machine.")

    def send_start_video_recording(self):
        self.get_logger().info("Start video recording")
        for namespace in self.namespaces:
            self.send_drone_command(namespace, 'camera/start_recording')

    def send_stop_video_recording(self):
        self.get_logger().info("Stop video recording")
        for namespace in self.namespaces:
            self.send_drone_command(namespace, 'camera/stop_recording')


def main() -> None:
    pass


def ros_main() -> None:
    rclpy.init()
    node = NiceGuiNode.get_instance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


@app.on_connect
async def modify_ui_after_run(self):
    node = NiceGuiNode.get_instance()
    for namesace, arrow in node.drone_arrows.items():
        arrow._place_arrow()


# Start the ROS2 node and setup UI in a separate thread for NiceGUI
@app.on_startup
def my_startup():
    threading.Thread(target=ros_main, daemon=True).start()
    setup_ui()


# Handle ROS2 module naming conventions
ui_run.APP_IMPORT_STRING = f'{__name__}:app'
ui.run(uvicorn_reload_dirs=str(
    Path(__file__).parent.resolve()), favicon='🐾', port=8085)
