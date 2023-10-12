import launch
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    #parameters = os.path.join(get_package_share_directory('pushcorp_driver'), 'config/network_parameters.yaml')
    
    parameters = {
        'ip': '192.168.0.12',
        'port': 1993
    }    
    
    # Launch AFD Position Publisher
    afd_position_publisher = Node(
        package='pushcorp_driver',
        executable='afd_position_publisher',
        name='afd_position_publisher',
        output='screen',
        parameters=[parameters]
    )

    # Launch AFD Force Publisher
    afd_force_publisher = Node(
        package='pushcorp_driver',
        executable='afd_force_publisher',
        name='afd_force_publisher',
        output='screen',
        parameters=[parameters] 
    )

    ld.add_action(afd_position_publisher)
    ld.add_action(afd_force_publisher)
    return ld

