import rclpy

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg._quaternion import Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

rclpy.init()

nav = BasicNavigator()

initial_pose = PoseStamped()
initial_pose.header.frame_id = 'map'
initial_pose.header.stamp = nav.get_clock().now().to_msg()
nav.setInitialPose(initial_pose)

pose = PoseStamped()
pose.header.frame_id = 'map'
pose.header.stamp = nav.get_clock().now().to_msg()
path = nav.
