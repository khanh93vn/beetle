import os

import launch

from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

DEVICE = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_54E1003696-if00"
BAUDRATE = 921600

def generate_launch_description():
    #beetle_desc_dir = FindPackageShare(package='beetle_description').find('beetle_description')
    beetle_desc_dir = FindPackageShare(package='sdv_description').find('sdv_description')

    beetle_nav_dir = FindPackageShare(package='beetle_navigation2').find('beetle_navigation2')
    default_rviz_config_file = os.path.join(beetle_nav_dir, 'rviz/default_view.rviz')

    # Create launch configuration variables
    use_rviz = LaunchConfiguration('use_rviz')
    params_file = LaunchConfiguration('params_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    #robot_description = Command(['xacro ', os.path.join(beetle_desc_dir, 'urdf/beetle.urdf'),
    #                            ' beetle_controller_yaml_file:=', params_file])
    robot_description = Command(['xacro ', os.path.join(beetle_desc_dir, 'urdf/sdv.urdf'),
                               ' beetle_controller_yaml_file:=', params_file])
    
    # Define launch arguments
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Start with RViz if true')
    # declare_params_file_cmd = DeclareLaunchArgument(
    #     name='params_file', default_value=default_controller_yaml_file,
    #     description='Absolute path to controller yaml config file')
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file', default_value=default_rviz_config_file,
        description='Absolute path to rviz config file')

    # Define actions
    # start_micro_ros_agent = Node(
    #     package='micro_ros_agent',
    #     executable='micro_ros_agent',
    #     name='micro_ros_agent',
    #     arguments=[
    #         "serial", "--dev", DEVICE,
    #         "--baudrate", str(BAUDRATE), "-v6"])
    start_msg_forwarder = Node(
        package='beetle_msg_forwarder',
        executable='beetle_msg_forwarder',
        name='beetle_msg_forwarder')
    start_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': False}])
    start_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False,
                     'robot_description': robot_description}])
    start_rviz = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': False}])

    # Declare event handlers

    # Create launch description
    ld = launch.LaunchDescription()

    # Declare launch options
    ld.add_action(declare_use_rviz_cmd)
    # ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    # Register event handlers

    # Add actions
    #ld.add_action(start_micro_ros_agent)
    ld.add_action(start_msg_forwarder)
    ld.add_action(start_joint_state_publisher)
    ld.add_action(start_robot_state_publisher)
    ld.add_action(start_rviz)

    return ld
