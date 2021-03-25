"""
<?xml version="1.0"?>
<launch>
  <param name="robot_description" command="$(find xacro)/xacro --inorder '$(find sick_tim)/urdf/example.urdf.xacro'" />
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />

  <node name="sick_tim571_2050101" pkg="sick_tim" type="sick_tim551_2050001" respawn="false" output="screen">
    <!-- default values: -->
    <!--
      <param name="min_ang" type="double" value="-2.35619449019" />
      <param name="max_ang" type="double" value="2.35619449019" />
      <param name="intensity" type="bool" value="True" />
      <param name="skip" type="int" value="0" />
      <param name="frame_id" type="str" value="laser" />
      <param name="time_offset" type="double" value="-0.001" />
      <param name="publish_datagram" type="bool" value="False" />
      <param name="subscribe_datagram" type="bool" value="false" />
      <param name="device_number" type="int" value="0" />
      <param name="range_min" type="double" value="0.05" />
    -->
    <param name="range_max" type="double" value="25.0" />

    <!-- Older versions of the scanner firmware report an incorrect time_increment.
         If you get a warning about time_increment, uncomment this. -->
    <!-- <param name="time_increment" type="double" value="0.000061722" /> -->

    <!-- Uncomment this to enable TCP instead of USB connection; 'hostname' is the host name or IP address of the laser scanner
    In cases where a race condition exists and the computer boots up before the TIM is ready, increase 'timelimit.'
         <param name="hostname" type="string" value="192.168.0.1" />
         <param name="port" type="string" value="2112" />
         <param name="timelimit" type="int" value="5" />
    -->
  </node>
</launch>
"""

# Copyright (c) 2020, Stratom Inc.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_smm_gazebo = get_package_share_directory('smm_gazebo')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    config = os.path.join(
        get_package_share_directory('ar3p_smm_finder'),
        'config',
        'smm.param.yaml'
    )

    # Follow node
    finder = Node(
        package='ar3p_smm_finder',
        node_executable='smm_composition',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, config]
    )

    # RViz
    rviz = Node(
        package='rviz2',
        node_executable='rviz2',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-d', os.path.join(pkg_smm_gazebo, 'rviz', 'smm_sim.rviz'), '-f', '/world'],
    )

    return LaunchDescription([
        rviz,
        maruco,
        finder,
        tf,
        tf_solution
    ])
