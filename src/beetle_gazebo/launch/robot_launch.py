import os

import launch
import launch_ros

from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    beetle_desc_dir = FindPackageShare(package='beetle_description').find('beetle_description')
    beetle_gazebo_dir = FindPackageShare(package='beetle_description').find('beetle_gazebo')
    robot_description = Command(['xacro ', os.path.join(beetle_desc_dir, 'urdf/beetle.urdf')])
    default_rviz_config_file = os.path.join(beetle_gazebo_dir, 'rviz/default_view.rviz')

    # Create launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    log_level = LaunchConfiguration('log_level')

    # Define launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Start with RViz if true')
    declare_rviz_config_file = DeclareLaunchArgument(
        name='rviz_config_file', default_value=default_rviz_config_file,
        description='Absolute path to rviz config file')
    declare_log_level = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')
    pose = {'x': LaunchConfiguration('x_pose', default='0.00'),
            'y': LaunchConfiguration('y_pose', default='0.00'),
            'z': LaunchConfiguration('z_pose', default='0.70'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
            'Y': LaunchConfiguration('yaw', default='0.00')}

    # Define actions
    start_gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen')
    start_gazebo_spawner = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'beetle',
                   '-topic', 'robot_description',
                   '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
                   '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']],
        output='screen')
    start_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}])
    start_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': robot_description}])
    start_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}])
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen')
    load_ackermann_drive_base_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'ackermann_drive_base_controller'],
        output='screen')

    # Declare event handlers
    load_joint_state_ctrl_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_gazebo_spawner,
            on_exit=[load_joint_state_controller]))
    load_ackermann_drive_base_ctrl_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_controller,
            on_exit=[load_ackermann_drive_base_controller]))

    # Create launch description
    ld = launch.LaunchDescription()

    # Declare launch options
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_rviz_config_file)
    ld.add_action(declare_log_level)

    # Register event handlers
    ld.add_action(load_joint_state_ctrl_event)
    ld.add_action(load_ackermann_drive_base_ctrl_event)

    # Add conditioned actions

    # Add actions
    ld.add_action(start_gazebo)
    ld.add_action(start_gazebo_spawner)
    ld.add_action(start_joint_state_publisher)
    ld.add_action(start_robot_state_publisher)
    ld.add_action(start_rviz)

    return ld
