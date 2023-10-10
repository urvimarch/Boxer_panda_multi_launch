# Copyright (c) 2023
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.launch_context import LaunchContext
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from launch.launch_description_sources import PythonLaunchDescriptionSource
#import yaml
import xacro

#https://github.com/bponsler/ros2-support

def generate_launch_description():

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
             )

    package_path = get_package_share_directory("gazebo_ros2_control_demos")
    
    declare_use_sim_time = DeclareLaunchArgument(
        name="use_sim_time", default_value='true', description="Use simulator time"
    )
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    
    '''
    gazebo_server = ExecuteProcess(
        cmd=[
            "gzserver",
            "--verbose",
            "-u",
            "-s", "libgazebo_ros_factory.so",
            "-s", "libgazebo_ros_init.so",
            #world,
        ],
        output="screen",
    )
    gazebo_client = ExecuteProcess(cmd=["gzclient"], output="screen")
    '''



    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(gazebo)
    #ld.add_action(gazebo_client)

    robots = [
            {'name': 'robot1', 'x_pose': '-1.5', 'y_pose': '-1.50', 'Y':'0.0'},
            {'name': 'robot2', 'x_pose': '-1.5', 'y_pose': '1.5', 'Y':'0.0'},
            {'name': 'robot3', 'x_pose': '1.5', 'y_pose': '0.0', 'Y':'-3.14'},
            #{'name': 'robot4', 'x_pose': '1.5', 'y_pose': '1.5', 'Y':'-3.14'},
            # …
            # …
        ]

    # Multiple ARMs in gazebo must be spawned in a serial fashion due to 
    # a global namespace dependency introduced by ros2_control.
    # robot_final_action is the last action from previous robot and 
    # new robot will only be spawned once previous action is completed
    robot_final_action = None
    for robot in robots:    
        robot_final_action = spawn_robot(
            ld,
            robot["name"] ,
            use_sim_time,
            robot["x_pose"],
            robot["y_pose"],
            robot["Y"],
            robot_final_action,
        )


    return ld


def spawn_robot(
        ld, robot_name, use_sim_time, x, y, Y,
        previous_final_action=None):

    package_path = get_package_share_directory("gazebo_ros2_control_demos")
    namespace = "/" + robot_name

    param_substitutions = {"use_sim_time": use_sim_time}
    configured_params = RewrittenYaml(
        source_file=package_path
        + "/config/boxer_panda_controller.yaml",
        root_key=robot_name,
        param_rewrites=param_substitutions,
        convert_types=True,
    )
    
    context = LaunchContext()
    controller_paramfile = configured_params.perform(context)

    xacro_path = os.path.join(package_path, "urdf", "multi_boxer_panda.xacro.urdf")
    print(xacro_path)

    

    robot_doc = xacro.process_file(
        xacro_path,
        mappings={
            "name": robot_name,
            "namespace": namespace,
            "sim_gazebo": "1",
            "simulation_controllers": controller_paramfile,
            #"safety_limits": "true",
            #"prefix": "",
            #"pedestal_height": "0.1",
        },
    )
    print(1)
    robot_urdf = robot_doc.toprettyxml(indent="  ")


    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    robot_params = {"robot_description": robot_urdf,
                    "use_sim_time": use_sim_time}
    robot_state_publisher = Node(
        package="robot_state_publisher",
        namespace=namespace,
        executable="robot_state_publisher",
        output="screen",
        remappings=remappings,
        parameters=[robot_params],
    )
    
    controller_run_state = 'active'

    robot_spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name='spawn_robot',
        namespace=namespace,        
        arguments=[
            "-topic", namespace + "/robot_description",
            "-entity", robot_name,
          #  "-robot_namespace", namespace,
            "-x", x,
            "-y", y,
            "-z", "0.0",
            "-Y", Y,
            "-unpause",
        ],
        output="screen",
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            controller_run_state,
            "joint_state_broadcaster",
            "-c",
            namespace + "/controller_manager",
        ],
        output="screen",
    )


    load_diff_drive_base_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', controller_run_state,'diff_drive_base_controller' , "-c",
            namespace + "/controller_manager"],
        output="screen",
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', controller_run_state,'joint_trajectory_controller', "-c",
            namespace + "/controller_manager"],
        output="screen",
    )

     
    if previous_final_action is not None:
        spawn_entity = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=previous_final_action,
                on_exit=[robot_spawn_entity],
            )
        )
    else:
        spawn_entity = robot_spawn_entity

    state_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_spawn_entity,
            on_exit=[load_joint_state_controller],
        )
    )
    
    diff_drive_controller_event = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_diff_drive_base_controller],
            )
        )
    joint_trajectory_controller_event = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_diff_drive_base_controller,
                on_exit=[load_joint_trajectory_controller],
            )
        )
    
    ld.add_action(robot_state_publisher)
    #ld.add_action(robot_move_group_node)
    ld.add_action(spawn_entity)
    ld.add_action(state_controller_event)
    ld.add_action(diff_drive_controller_event)
    ld.add_action(joint_trajectory_controller_event)

    return load_joint_trajectory_controller


