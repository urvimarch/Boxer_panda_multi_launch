#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file_name = 'empty_world.world'
    world = os.path.join(get_package_share_directory('gazebo_ros2_control_demos'), 'worlds', world_file_name)
    robot_desc_path = os.path.join(get_package_share_directory("gazebo_ros2_control_demos"), "urdf", "boxer_panda.xacro.urdf")
    doc = xacro.parse(open(robot_desc_path))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
             )  

    entity_name_0="tb3_0"
    entity_name_1="tb3_1"

    return LaunchDescription([
        #ExecuteProcess(
        #    cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_init.so'],
        #    output='screen'),

        gazebo,
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=entity_name_0,
            parameters=[{'frame_prefix': entity_name_0+'/', 'use_sim_time': use_sim_time, 'robot_description': ParameterValue(doc.toxml()), 'robot_name': ParameterValue(entity_name_0)}],
            output="screen"
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=entity_name_1,
            parameters=[{'frame_prefix': entity_name_1+'/', 'use_sim_time': use_sim_time, 'robot_description': ParameterValue(doc.toxml()), 'robot_name': ParameterValue(entity_name_0)}],
            output="screen"
        ),

        Node(
            package='gazebo_ros', 
            executable='spawn_entity.py',
            name='spawn_robot',
            namespace=entity_name_0,
            arguments=['-topic', 'robot_description','-entity', 'entity_name_0_boxer__panda_bot', '-x', '0.0','-y', '0.0','-z', '0.0'],
            output='screen'
        ),

        Node(
            package='gazebo_ros', 
            executable='spawn_entity.py',
            name='spawn_robot',
            namespace=entity_name_1,
            arguments=['-topic', 'robot_description','-entity', 'entity_name_1_boxer__panda_bot','-x', '5.0','-y', '0.0','-z', '0.0'],
            output='screen'),
    ])
