import os

import launch
import launch_ros

from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess,
    IncludeLaunchDescription, RegisterEventHandler)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # beetle_desc_dir = FindPackageShare(package='beetle_description').find('beetle_description')
    beetle_gazebo_dir = FindPackageShare(package='beetle_gazebo').find('beetle_gazebo')
    beetle_nav_dir = FindPackageShare(package='beetle_navigation2').find('beetle_navigation2')
    # robot_description = Command(['xacro ', os.path.join(beetle_desc_dir, 'urdf/beetle.urdf')])
    simulator_launch_dir = os.path.join(beetle_gazebo_dir, 'launch')
    navigation_launch_dir = os.path.join(beetle_nav_dir, 'launch')
    default_rviz_config_file = os.path.join(beetle_nav_dir, 'rviz/default_view.rviz')

    # Create launch configuration variables
    autostart = LaunchConfiguration('autostart')
    use_simulator = LaunchConfiguration('use_simulator')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    params_file = LaunchConfiguration('params_file')
    map_yaml_file = LaunchConfiguration('map')
    log_level = LaunchConfiguration('log_level')

    # Define launch arguments
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')
    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='true',
        description='Whether to start the simulator')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Start with RViz if true')
    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='False',
        description='Use composed bringup if True')
    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(beetle_nav_dir, 'config', 'navigation_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file', default_value=default_rviz_config_file,
        description='Absolute path to rviz config file')
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(beetle_nav_dir, 'maps', 'map.yaml'),
        description='Full path to map yaml file to load')
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    # Define actions
    start_simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulator_launch_dir, 'robot.launch.py')),
        condition=IfCondition(use_simulator),
        launch_arguments={'use_sim_time': use_sim_time,
                          'use_rviz': use_rviz,
                          'rviz_config_file': rviz_config_file,
                          'log_level': log_level}.items())

    start_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_launch_dir, 'localization.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time,
                          'autostart': autostart,
                          'use_composition': use_composition,
                          'params_file': params_file,
                          'use_respawn': use_respawn,
                          'map': map_yaml_file,
                          'log_level': log_level}.items())

    start_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_launch_dir, 'navigation.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time,
                          'log_level': log_level}.items())

    # Declare event handlers

    # Create launch description
    ld = launch.LaunchDescription()

    # Declare launch options
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_log_level_cmd)

    # Register event handlers

    # Add conditioned actions
    ld.add_action(start_simulator)
    # ld.add_action(start_robot_interfacing)

    # Add actions
    ld.add_action(start_localization)
    # ld.add_action(start_navigation)

    return ld
