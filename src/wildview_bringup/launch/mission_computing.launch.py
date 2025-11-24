import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(get_package_share_directory('wildview_bringup'), 'config', 'parameters.yaml') 

    # Add animal localisation node
    animal_localisation_node = Node(
        package='animal_localisation',
        executable='animal_localisation_node')
    ld.add_action(animal_localisation_node)
        
    pso_controller_node = Node(
        package='pso_controller',
        executable='pso_controller',
        name = 'pso_controller',
        parameters=[config])
    
    ld.add_action(pso_controller_node)

    path_planning_node = Node(
        package='path_planning',
        executable='path_planning',
        name = 'path_planning')

    ld.add_action(path_planning_node)    
    
    return ld
