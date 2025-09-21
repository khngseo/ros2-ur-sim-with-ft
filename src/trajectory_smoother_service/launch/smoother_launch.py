#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = PathJoinSubstitution([
        FindPackageShare('trajectory_smoother_service'),
        'config',
        'smoother_parameters.yaml'
    ])

    params_file = LaunchConfiguration('smoother_params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'smoother_params_file',
            default_value=default_params,
            description='YAML file with parameters for the trajectory smoother service.'
        ),
        Node(
            package='trajectory_smoother_service',
            executable='trajectory_smoother_node',
            name='trajectory_smoother',
            output='screen',
            parameters=[params_file]
        )
    ])
