# Copyright (c) 2020, Stratom Inc.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('sick_tim'),
        'cfg',
        'sick_tcp.yaml'
    )

    robot_description_xacro = os.path.join(
        get_package_share_directory('sick_tim'),
        'urdf',
        'example.urdf.xacro'
    )
    print(config)
    print(robot_description_xacro)

    # Follow node
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['xacro', ' ', robot_description_xacro])}]
    )

    # RViz
    sick_tim = Node(
        package='sick_tim',
        executable='sick_tim551_2050001',
        parameters=[config]
    )

    return LaunchDescription([
        rsp,
        sick_tim
    ])
