"""
Perpetual Monitoring Launch File

Launches the perpetual drone monitoring system with dynamic drone connection.

Author: Edouard Rolland
Project: WildDrone
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    """Generate launch description for perpetual monitoring."""
    
    ld = LaunchDescription()

    # Map server for offline tiles
    map_server = ExecuteProcess(
        cmd=['python3', 'src/groundstation/offline_map/map_server.py'],
        output='screen'
    )
    ld.add_action(map_server)

    # Perpetual monitoring groundstation (GUI)
    # Note: Drones are connected dynamically through the GUI
    groundstation_node = Node(
        package='groundstation',
        executable='perpetual_monitor_node',
        name='perpetual_monitor',
        output='screen'
    )
    ld.add_action(groundstation_node)

    return ld
