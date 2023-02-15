import os

import launch
import launch_ros

from launch.actions import (
    DeclareLaunchArgument, EmitEvent, ExecuteProcess,
    GroupAction, IncludeLaunchDescription, RegisterEventHandler,
    SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command, LaunchConfiguration, PythonExpression)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    beetle_gazebo_dir = FindPackageShare(package='beetle_gazebo').find('beetle_gazebo')
    beetle_nav_dir = FindPackageShare(package='beetle_navigation2').find('beetle_navigation2')
    bringup_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup')
    default_world_file = os.path.join(beetle_gazebo_dir, 'worlds/empty.world')
    default_rviz_config_file = os.path.join(beetle_nav_dir, 'rviz/default_view.rviz')
    default_nav_to_pose_bt_xml = os.path.join(beetle_nav_dir, 'behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml')
    default_nav_through_poses_bt_xml = os.path.join(beetle_nav_dir, 'behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml')

    # Create launch configuration variables
    autostart = LaunchConfiguration('autostart')
    use_ekf = LaunchConfiguration('use_ekf')
    use_simulator = LaunchConfiguration('use_simulator')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    params_file = LaunchConfiguration('params_file')
    world_file = LaunchConfiguration('world_file')
    map_yaml_file = LaunchConfiguration('map')
    log_level = LaunchConfiguration('log_level')

    # Rewrite params in files
    param_substitutions = {
        # 'use_sim_time': PythonExpression([use_sim_time, " and " , use_simulator]),
        # 'use_odom_tf': PythonExpression(["not ", use_ekf]),
        'yaml_filename': map_yaml_file,
        'default_nav_to_pose_bt_xml': default_nav_to_pose_bt_xml,
        'default_nav_through_poses_bt_xml': default_nav_through_poses_bt_xml}

    configured_params = RewrittenYaml(
        source_file=params_file,
        # root_key='/',
        param_rewrites=param_substitutions,
        convert_types=True)

    # Define launch arguments
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='True',
        description='Automatically startup the nav2 stack')
    declare_use_ekf_cmd = DeclareLaunchArgument(
        'use_ekf', default_value='False',
        description='Use Extended Kalman Filter to fuse sensors')
    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='False',
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
    declare_world_file_cmd = DeclareLaunchArgument(
        name='world_file', default_value=default_world_file,
        description='Absolute path to world file to launch with Gazebo')
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
            os.path.join(beetle_gazebo_dir, 'launch', 'robot.launch.py')),
        condition=IfCondition(use_simulator),
        launch_arguments={'use_sim_time': use_sim_time,
                          'use_rviz': use_rviz,
                          'params_file': params_file,
                          'world_file': world_file,
                          'rviz_config_file': rviz_config_file,
                          'log_level': log_level}.items())
    start_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(beetle_nav_dir, 'launch', 'driver.launch.py')),
        condition=UnlessCondition(use_simulator),
        launch_arguments={'use_rviz': use_rviz,
                          'params_file': params_file,
                          'rviz_config_file': rviz_config_file}.items())
    bringup_cmd_group = GroupAction([
        Node(
            condition=IfCondition(use_composition),
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[configured_params, {'autostart': autostart}],
            arguments=['--ros-args', '--log-level', log_level],
            output='screen'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(beetle_nav_dir, 'launch', 'localization.launch.py')),
            # launch_arguments={'use_sim_time': PythonExpression([use_sim_time, " and ", use_simulator]),
            launch_arguments={'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'use_ekf': use_ekf,
                              'use_composition': use_composition,
                              'params_file': params_file,
                              'use_respawn': use_respawn,
                              'map': map_yaml_file,
                              'container_name': 'nav2_container'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'navigation_launch.py')),
            launch_arguments={'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': configured_params,
                              'use_composition': use_composition,
                              'use_respawn': use_respawn,
                              'container_name': 'nav2_container'}.items())])

    # Declare event handlers

    # Create launch description
    ld = launch.LaunchDescription()

    # Declare launch options
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_ekf_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_world_file_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_log_level_cmd)

    # Register event handlers

    # Add conditioned actions
    ld.add_action(start_simulator)
    ld.add_action(start_robot)

    # Add actions
    ld.add_action(bringup_cmd_group)

    return ld
