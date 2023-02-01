import os

import launch

from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    beetle_desc_dir = FindPackageShare(package='beetle_description').find('beetle_description')
    beetle_gazebo_dir = FindPackageShare(package='beetle_gazebo').find('beetle_gazebo')
    beetle_nav_dir = FindPackageShare(package='beetle_navigation2').find('beetle_navigation2')
    launch_dir = os.path.join(beetle_gazebo_dir, 'launch')
    default_controller_yaml_file = os.path.join(beetle_gazebo_dir, 'config/ackermann_controller.yaml')
    default_rviz_config_file = os.path.join(beetle_nav_dir, 'rviz/default_view.rviz')

    # Create launch configuration variables
    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    params_file = LaunchConfiguration('params_file')
    world_file = LaunchConfiguration('world_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    api_key = LaunchConfiguration('api_key')
    log_level = LaunchConfiguration('log_level')

    robot_description = Command(['xacro ', os.path.join(beetle_desc_dir, 'urdf/beetle.urdf'),
                                ' beetle_controller_yaml_file:=', params_file])
    default_world_file = Command(['xacro ', os.path.join(beetle_gazebo_dir, 'worlds/empty.world')])

    # Define launch arguments
    headless = LaunchConfiguration('headless')
    declare_headless_cmd = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Whether to start gzclient')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Start with RViz if true')
    declare_params_file_cmd = DeclareLaunchArgument(
        name='params_file', default_value=default_controller_yaml_file,
        description='Absolute path to controller yaml config file')
    declare_world_file_cmd = DeclareLaunchArgument(
        name='world_file', default_value=default_world_file,
        description='Absolute path to world file to launch with Gazebo')
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file', default_value=default_rviz_config_file,
        description='Absolute path to rviz config file')
    declare_api_key_cmd = DeclareLaunchArgument(
        name='api_key', default_value='',
        description='Google API key to download map')
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')
    pose = {'x': LaunchConfiguration('x_pose', default='0.00'),
            'y': LaunchConfiguration('y_pose', default='0.00'),
            'z': LaunchConfiguration('z_pose', default='0.70'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
            'Y': LaunchConfiguration('yaw', default='1.57')}

    # Define actions
    start_gazebo_server = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world_file],
        cwd=[launch_dir], output='screen')
    start_gazebo_client = ExecuteProcess(
        condition=UnlessCondition(headless),
        cmd=['gzclient'],
        cwd=[launch_dir], output='screen')
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
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
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

    load_ackermann_drive_base_ctrl_event = RegisterEventHandler(
    event_handler=OnProcessExit(
        target_action=start_gazebo_spawner,
        on_exit=[load_ackermann_drive_base_controller]))
    load_joint_state_ctrl_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_controller,
            on_exit=[load_joint_state_controller]))

    # Create launch description
    ld = launch.LaunchDescription()

    # Declare launch options
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_world_file_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_api_key_cmd)
    ld.add_action(declare_log_level_cmd)

    # Register event handlers
    ld.add_action(load_ackermann_drive_base_ctrl_event)
    # ld.add_action(load_joint_state_ctrl_event)

    # Add actions
    ld.add_action(start_gazebo_server)
    ld.add_action(start_gazebo_client)
    ld.add_action(start_gazebo_spawner)
    ld.add_action(start_joint_state_publisher)
    ld.add_action(start_robot_state_publisher)
    ld.add_action(start_rviz)

    return ld
