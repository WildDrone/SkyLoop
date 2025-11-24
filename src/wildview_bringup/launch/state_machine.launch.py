import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(get_package_share_directory('wildview_bringup'), 'config', 'parameters.yaml') 

    # Add mission_control node
    mission_control_node = Node(
        package='mission_control',
        executable='mission_control_node',
        name = 'state_machine',
        parameters=[config]
        )
    
    ld.add_action(mission_control_node)
        
    return ld
