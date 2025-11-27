"""
Perpetual Monitoring Launch File

Launches the perpetual drone monitoring system with dynamic drone connection.

Author: Edouard Rolland
Project: WildDrone
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for perpetual monitoring."""
    
    ld = LaunchDescription()

    # Kill any process using port 8086 (the NiceGUI port) to ensure clean startup
    cleanup = ExecuteProcess(
        cmd=['bash', '-c', 
             'fuser -k 8086/tcp 2>/dev/null || true; sleep 0.5; echo "Port 8086 cleared"'],
        name='cleanup',
        output='screen'
    )
    
    # Perpetual monitoring groundstation (GUI)
    # Note: Drones are connected dynamically through the GUI
    groundstation_node = Node(
        package='groundstation',
        executable='perpetual_monitor_node',
        name='perpetual_monitor',
        output='screen'
    )
    
    # Run cleanup first
    ld.add_action(cleanup)
    
    # Start groundstation only after cleanup exits
    ld.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action=cleanup,
            on_exit=[groundstation_node]
        )
    ))

    return ld
