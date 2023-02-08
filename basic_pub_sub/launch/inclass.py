#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

# from ament_index_python import get_package_share_directory

# config_dir = get_package_share_directory("basic_pub_sub")

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='basic_pub_sub',
            namespace='basic_pub_sub_ns',
            executable='talker',
            name="talker"
        ),
        Node(
            package='basic_pub_sub',
            namespace='basic_pub_sub_ns',
            executable='listener',
            name="listener"
        )
    ])
