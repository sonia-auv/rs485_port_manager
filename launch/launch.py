import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    auv = os.getenv("AUV", None)

    if auv is None:
        raise Exception("env var AUV not set")

    
    return LaunchDescription(
        [
            Node(
                package="rs485_port_manager",
                executable="rs485_port_manager",
            )
        ]
    )
