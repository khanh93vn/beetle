"""
Script for testing beetle control performance.
"""

import rclpy

from math import pi, sin, cos
from itertools import product

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg._quaternion import Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import quaternion_about_axis, euler_from_quaternion

# Experiment samples
sample_distances = [2.0, 5.0]
sample_directions = [i*(pi/3) for i in range(4)]
sample_headings = [i*(pi/4) for i in range(8)]


def quaternion_from_yaw(yaw):
    q = Quaternion()
    q.x, q.y, q.z, q.w = quaternion_about_axis(yaw, (0, 0, 1))
    return q

def yaw_from_pose(pose):
    q = pose.orientation
    return euler_from_quaternion((q.x, q.y, q.z, q.w))[2]


def main():
    try:
        rclpy.init()
    except RuntimeError as e:
        if 'must only be called once' not in str(e):
            raise e

    navigator = BasicNavigator()

    # Set initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 3.45
    initial_pose.pose.position.y = 2.15
    initial_pose.pose.orientation.z = 1.0
    initial_pose.pose.orientation.w = 0.0
    navigator.setInitialPose(initial_pose)

    # Generate goals
    relative_goals = [
        (r*cos(phi), r*sin(phi), delta_theta)
        for r, phi, delta_theta in product(
            sample_distances, sample_directions, sample_headings)]

    # Begin testing
    experimental_results = []
    current_pose = initial_pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    for delta_x, delta_y, delta_theta in relative_goals:
        # Set goal pose
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = current_pose.pose.position.x + delta_x
        goal_pose.pose.position.y = current_pose.pose.position.y + delta_y
        goal_pose.pose.orientation = quaternion_from_yaw(
            yaw_from_pose(current_pose.pose) + delta_theta)
        navigator.goToPose(goal_pose)

        while not navigator.isTaskComplete():
            pass

        # Process the result
        result = navigator.getResult()
        current_pose = feedback._current_pose
        if result in (TaskResult.SUCCEEDED, TaskResult.FAILED):
            # Append result
            print("Succeeded" if result == TaskResult.SUCCEEDED else "Failed", end='')
            print(result)
        elif result == TaskResult.CANCELED:
            raise RuntimeError('task should not be cancelled')
        else:
            raise TypeError('Goal has an invalid return status!')

    exit(0)


if __name__ == '__main__':
    main()
