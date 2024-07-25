import launch
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    parameters = {
        'ip': '192.168.0.12',
        'port': 1993,
        'period': 0.1
    }    
    
    # Launch AFD driver
    afd_driver = Node(
        package='pushcorp_driver',
        executable='afd_driver',
        name='afd_driver',
        output='screen',
        parameters=[parameters]
    )

    ld.add_action(afd_driver)
    return ld

