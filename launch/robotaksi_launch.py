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
        Node(
            package='robotaksi_interface',
            executable='bridge_node',
            name='bridge_node',
            output='screen',
            parameters=[config_file]
        ),
        # Teleop klavye node'unu ayrı bir terminalde çalıştırmak daha mantıklıdır,
        # Ancak burada twist_keyboard için bir yönlendirme bırakılabilir.
    ])
