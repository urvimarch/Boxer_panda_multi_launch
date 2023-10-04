import launch
from ament_index_python.packages import get_package_share_directory
from launch import action
from launch import actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
import xacro

def generate_launch_description():
    
    # extracting the robot definition from the xacro file
    franka_xacro_file = os.path.join(get_package_share_directory('gazebo_ros2_control_demos'), 'urdf',
                                     'boxer_panda.xacro.urdf')
    robot_description_content = xacro.process_file(franka_xacro_file).toxml()

    #rviz_file = os.path.join(get_package_share_directory('boxer_franka_description'), 'rviz',
                             #'boxer_panda_config.rviz')

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
            'robot_description': robot_description_content
            }]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        #arguments=['-d', rviz_file],
    )

        
    return launch.LaunchDescription([

        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),

        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ])