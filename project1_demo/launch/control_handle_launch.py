import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():

    ld = LaunchDescription()
    stop_button_launch_arg = DeclareLaunchArgument \
    (
        'stop_button',
        default_value = TextSubstitution(text='X')
    )

    node = Node(

        package    = "project1_demo",
        executable = "control_handle",
        parameters = 
        [
            {
                'stop_button' : LaunchConfiguration('stop_button'),
            }
        ],
        name = "sim",
    )

    ld.add_action([stop_button_launch_arg,node])
    return ld
