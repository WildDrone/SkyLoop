from launch import LaunchDescription
from launch_ros.actions import Node
import subprocess
from launch.actions import ExecuteProcess


def generate_launch_description():
    
    ld = LaunchDescription()

    drones = [
        {'namespace': 'drone_1', 'ip_rc': '192.110.110.10'},
        {'namespace': 'drone_2', 'ip_rc': '192.168.233.28'},
        {'namespace': 'drone_3', 'ip_rc': '192.168.233.32'},
        {'namespace': 'drone_4', 'ip_rc': '192.168.233.160'}
    ]

    # Add RTSP streaming node

    drone = drones[0]
    
    rtsp_node = Node(
        package='drone_videofeed',
        executable='rtsp_node',
        namespace=drone['namespace'],
        parameters=[{'ip_rc': drone['ip_rc']}]
    )
    
    ld.add_action(rtsp_node)

    # Add drone nodes dynamically
    for drone in drones:
        
        if drone['namespace'] == 'drone_1':
            node = Node(
            package='codrone_controller',
            executable='drone_control.py',
            namespace=drone['namespace']
        )
        
        else:
            node = Node(
                package='dji_controller',
                executable='dji_node',
                namespace=drone['namespace'],
                parameters=[{'ip_rc': drone['ip_rc']}],
            )
        ld.add_action(node)

    codrone_camera = Node(
            package='codrone_controller',
            executable='camera_control.py',
        )
    
    ld.add_action(codrone_camera)

    namespaces = [drone['namespace'] for drone in drones]
    ip_rcs = [drone['ip_rc'] for drone in drones]
    
    # Here we add the map server
    map_server = ExecuteProcess(
        cmd=['python3', 'src/groundstation/offline_map/map_server.py'],
        output='screen'
    )
    
    ld.add_action(map_server)

    # Add Groundstation node
    groundstation_node = Node(
        package='groundstation',
        executable='groundstation_node',
        parameters=[
            # Pass as a comma-separated string
            {'namespaces': ','.join(namespaces)},
            # Pass as a comma-separated string
            {'ip_rcs': ','.join(ip_rcs)}
        ]
    )
    ld.add_action(groundstation_node)

    return ld
