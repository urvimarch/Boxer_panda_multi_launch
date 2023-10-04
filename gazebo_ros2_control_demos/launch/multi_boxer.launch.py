#!/usr/bin/python3
# -*- coding: utf-8 -*-
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

    pkg_box_bot_gazebo = get_package_share_directory('gazebo_ros2_control_demos')
    pkg_box_bot_description = get_package_share_directory('gazebo_ros2_control_demos')

    # Start World
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
             ) 

    spawn_robot_world_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_box_bot_description, 'launch', 'boxer_panda_robot_1.launch.py'),
        )
    )   

    spawn_robot_world_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_box_bot_description, 'launch', 'boxer_panda_robot_2.launch.py'),
        )
    )  

    return LaunchDescription([
        gazebo,
        spawn_robot_world_1,
        spawn_robot_world_2

    ])