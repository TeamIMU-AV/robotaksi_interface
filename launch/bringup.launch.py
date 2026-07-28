import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('robotaksi_interface'),
        'config',
        'vehicle_params.yaml'
    )

    return LaunchDescription([
        # Teleop nodes running in background (output='log')
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='log'
        ),
        Node(
            package='robotaksi_teleop',
            executable='joystick_teleop',
            name='joystick_teleop',
            output='log',
            parameters=[config_file]
        ),
        # Interface running in foreground (output='screen')
        Node(
            package='robotaksi_interface',
            executable='bridge_node',
            name='bridge_node',
            output='screen',
            parameters=[config_file]
        )
    ])
