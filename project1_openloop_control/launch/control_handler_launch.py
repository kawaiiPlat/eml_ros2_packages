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

    joy_node = Node(
        package="joy",
        executable="joy_node",
    )

    control_handle_node = Node(

        package    = "project1_openloop_control",
        executable = "control_handler",
        parameters = 
        [
            {
                'stop_button' : LaunchConfiguration('stop_button'),
            }
        ],
    )

    ld.add_action(stop_button_launch_arg)
    ld.add_action(joy_node)
    ld.add_action(control_handle_node)
    return ld
