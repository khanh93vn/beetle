import os

import launch
import launch_ros

from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    beetle_desc_dir = FindPackageShare(package='beetle_description').find('beetle_description')
    robot_description = Command(['xacro ', os.path.join(beetle_desc_dir, 'urdf/beetle.urdf')])
    default_rviz_config_file = os.path.join(beetle_desc_dir, 'rviz/default_view.rviz')

    # Create launch configuration variables
    use_js_pub_gui = LaunchConfiguration('use_js_pub_gui')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    # Define launch arguments
    declare_use_js_pub_gui_cmd = DeclareLaunchArgument(
        name='use_js_pub_gui', default_value='True',
        description='Flag to enable joint_state_publisher_gui'
    )
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file', default_value=default_rviz_config_file,
        description='Absolute path to rviz config file'
    )

    # Define actions
    start_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(use_js_pub_gui)
    )
    start_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(use_js_pub_gui)
    )
    start_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )
    start_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    # Create launch description
    ld = launch.LaunchDescription()

    # Declare launch options
    ld.add_action(declare_use_js_pub_gui_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    # Add conditioned actions
    ld.add_action(start_joint_state_publisher)
    ld.add_action(start_joint_state_publisher_gui)

    # Add actions
    ld.add_action(start_robot_state_publisher)
    ld.add_action(start_rviz)

    return ld
