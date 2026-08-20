"""Serial bridge to the vehicle's control board.

Values come from config/robotaksi_interface.yaml, the vehicle's physical
measurements from robotaksi_bringup/config/vehicle_params.yaml, and the topic
names from robotaksi_bringup/config/global_params.yaml. Nothing is written in
this file and robotaksi_bringup passes nothing in, so launching this on its own
gives exactly what the full bringup gives.

The two files under robotaksi_bringup are read out of its share directory.
robotaksi_bringup already depends on this package, so that dependency cannot be
declared in reverse -- colcon would reject the cycle. In this workspace bringup
is always built, so the files are always present.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

import yaml


def load_params(config_path, block):
    with open(config_path, 'r', encoding='utf-8') as config_file:
        config = yaml.safe_load(config_file) or {}
    if block not in config or 'ros__parameters' not in (config[block] or {}):
        raise RuntimeError(
            f"{config_path} has no '{block}: ros__parameters:' block")
    return config[block]['ros__parameters']


def generate_launch_description():
    pkg_interface = get_package_share_directory('robotaksi_interface')
    pkg_bringup = get_package_share_directory('robotaksi_bringup')

    interface_config_path = os.path.join(
        pkg_interface, 'config', 'robotaksi_interface.yaml')
    global_config_path = os.path.join(pkg_bringup, 'config', 'global_params.yaml')
    vehicle_config_path = os.path.join(pkg_bringup, 'config', 'vehicle_params.yaml')

    launch_params = load_params(interface_config_path, 'robotaksi_interface_launch')
    global_params = load_params(global_config_path, 'robotaksi_global')
    vehicle_params = load_params(vehicle_config_path, 'robotaksi_vehicle')

    bridge_node = Node(
        package='robotaksi_interface',
        executable='bridge_node',
        name='bridge_node',
        output='screen',
        parameters=[
            interface_config_path,
            {
                'use_sim_time': global_params['use_sim_time'],
                'odom_frame': global_params['odom_frame'],
                # The car's real measurements, shared with cmd_vel_mux and
                # joystick_teleop so all three clamp to the same limits.
                'wheelbase_m': vehicle_params['wheelbase_m'],
                'track_width_m': vehicle_params['track_width_m'],
                'max_steer_angle_deg': vehicle_params['max_steer_angle_deg'],
                'max_steer_speed_deg_s': vehicle_params['max_steer_speed_deg_s'],
            },
        ],
        remappings=[
            # The board holds the last commanded target with no timeout of its
            # own, so this must be the arbitrated topic, never a raw source.
            ('cmd_vel', global_params['cmd_vel_topic']),
            ('odom', global_params['wheel_odom_topic']),
            ('aux_cmd', global_params['aux_cmd_topic']),
            ('/tf', launch_params['tf_sink_topic']),
        ],
    )

    return LaunchDescription([bridge_node])
