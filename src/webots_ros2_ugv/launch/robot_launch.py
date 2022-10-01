import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from webots_ros2_driver.webots_launcher import WebotsLauncher

import pathlib
import launch


def generate_launch_description():
    ugv_dir = get_package_share_directory('webots_ros2_ugv')
    robot_description = pathlib.Path(os.path.join(ugv_dir, 'resource', 'ugv_webots.urdf')).read_text()
    rviz_config_path = pathlib.Path(os.path.join(ugv_dir, 'rviz', 'urdf_config.rviz')).read_text()
    param_file=os.path.join(ugv_dir, 'config/nav2_params.yaml')
    lifecycle_nodes = ['sensor_driver',
                       'controller_server',
                       'smoother_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower']

    webots = WebotsLauncher(
        world=os.path.join(ugv_dir, 'worlds', 'khoa_cn.wbt')
    )

    my_robot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
        ]
    )
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )
    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    # )
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', rviz_config_path],
    # )
    ekf_node_odom = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node_odom',
       output='screen',
       parameters=[os.path.join(ugv_dir, 'config/localization_params.yaml'),
                   {'use_sim_time': True}],
       remappings=[('odometry/filtered', 'odometry/local')],
    )
    ekf_node_map = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node_map',
       output='screen',
       parameters=[os.path.join(ugv_dir, 'config/localization_params.yaml'),
                   {'use_sim_time': True}],
       remappings=[('odometry/filtered', 'odometry/global')],
    )
    navsat_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[os.path.join(ugv_dir, 'config/localization_params.yaml'),
                    {'use_sim_time': True}],
        remappings=[('imu/data', 'imu'),
                    ('gps/fix', 'vehicle/gps'),
                    ('gps/filtered', 'gps/filtered'),
                    ('odometry/gps', 'odometry/gps'),
                    ('odometry/filtered', 'odometry/global')],
    )
    # nav2_bringup = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [get_package_share_directory('nav2_bringup'), '/launch/navigation_launch.py']),
    #     launch_arguments={'params_file': param_file,
    # )

    return LaunchDescription([
        webots,
        my_robot_driver,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
        robot_state_publisher_node,
        ekf_node_odom,
        ekf_node_map,
        navsat_node,
    ])
