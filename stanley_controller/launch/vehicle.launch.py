from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

config_dir_rviz = get_package_share_directory("gps_nav")
config_dir = get_package_share_directory("stanley_controller")

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gps_nav',
            executable='route_pose_provider',
            name='route_pose_provider',
            parameters = [
                {'want_loop': False},
                {'state_defs': '{0:\'OFF\', 1:\'ON\', 2:\'OUTSIDE\', 3:\'ENTRY_EXTENSION_PT\', 4:\'EXIT_EXTENSION_PT\', 5:\'EXIT_TURN_PT\', 6:\'START\', 7:\'END\', 8:\'UTURN_PT1\', 9:\'UTURN_PT2\', 10:\'UTURN_PT3\', 11:\'CORNER\', 12:\'END_EXTENSION\'}'},
                {'pose_filename': config_dir + '/data/carrot_poses.txt'}
            ]
        ),
        Node(
            package='gps_nav',
            executable='goal_pose_creator',
            name='goal_pose_creator',
            output='screen'
        ),
        Node(
            package='stanley_controller',
            executable='vehicle_controller_Stanley',
            name='vehicle_controller_Stanley',
            output='screen' 
        ),
        Node(
            package='gps_nav',
            executable='motion_spec_provider',
            name='motion_spec_provider',
            output='screen',
            parameters = [
                {'look_ahead_dist': 3.0},  # meters
                {'speed': 2.0}  # meters/sec
            ] 
        ),
        Node(
            package='pose_estimator',
            executable='pose_estimator',
            name='pose_estimator',
            output='screen',
        ),
        
    ])