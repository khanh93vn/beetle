import os

from math import pi, sin, cos, hypot, degrees
from itertools import product
from time import sleep

import rclpy

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg._quaternion import Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import quaternion_about_axis, euler_from_quaternion
from gazebo_msgs.srv import GetEntityState, SetEntityState


# Constants --------------------------------------------------------------------
data_file_path = os.path.join(
    os.environ["HOME"], "Documents/data/beetle_evaluation_data.csv")

# Experiment samples
sample_distances = [2.5, 5.0]
sample_directions = [-i*(pi/3) for i in range(4)]
sample_headings = [i*(pi/4) for i in range(8)]
timer_period = 1.0  # seconds

# ------------------------------------------------------------------------------
# Class BeetleEvaluationManager

class BeetleEvaluationManager(Node):
    robot_name = 'beetle'
    marker_name = 'arrow'
    camera_name = 'user_camera'
    camera_orientation = Quaternion(
        x=0.0, y=0.7071067811865475, z=0.0, w=0.7071067811865476)

    def __init__(self):
        super().__init__('beetle_evaluation_manager')

        # Initialize clients
        self._initialize_node()

        # Initialize navigator
        self._initialize_navigator()

        # Reset experiments
        self.reset_experiments()

    def _initialize_node(self):
        self.get_client = \
            self.create_client(GetEntityState, '/get_entity_state')
        self.set_client = \
            self.create_client(SetEntityState, '/set_entity_state')

    def _initialize_navigator(self):
        self.navigator = BasicNavigator()

        # Set initial pose
        self.get_logger().info("Đang giúp xe set trạng thái đầu...")
        self.current_pose = PoseStamped()
        self.current_pose.header.frame_id = 'map'
        self.current_pose.header.stamp = \
            self.navigator.get_clock().now().to_msg()
        self.current_pose.pose = self.get_entity_state(self.robot_name).pose
        self.navigator.setInitialPose(self.current_pose)
        self.get_logger().info("OK")

    def reset_experiments(self):
        # Generate goals
        self.experiments = [*product(
                sample_distances, sample_directions, sample_headings)]

        # Prepare for experiments
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map'
        self.current_experiment_index = 0
        self.auto = False

    def loop(self):
        if self.navigator.isTaskComplete():
            if self.current_experiment_index > 0:
                self.collect_result()
            if not self.auto:
                n = input(">>Start next experiment<<")
                try:
                    n = int(n) - 1
                    if 0 <= n < len(self.experiments):
                        self.current_experiment_index = n
                except ValueError:
                    if n == '#':
                        self.auto = True

            self.begin_experiment(self.current_experiment_index)
            self.current_experiment_index += 1

    def begin_experiment(self, experiment_index):
        self.get_logger().info(f"Chuẩn bị cho thử nghiệm #{experiment_index+1}")
        r, phi, delta_theta = self.experiments[experiment_index]
        # pose = self.get_entity_state(self.robot_name).pose
        self.current_pose.header.stamp = \
            self.navigator.get_clock().now().to_msg()
        pose = self.current_pose.pose

        yaw = yaw_from_pose(pose)

        # x, y offsets
        delta_x, delta_y = r*cos(yaw + phi), r*sin(yaw + phi)

        # Set goal pose
        self.goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.goal_pose.pose.position.x = pose.position.x + delta_x
        self.goal_pose.pose.position.y = pose.position.y + delta_y
        self.goal_pose.pose.orientation = quaternion_from_yaw(yaw + delta_theta)
        accepted = self.navigator.goToPose(self.goal_pose)
        self.start_time = self.navigator.get_clock().now().to_msg().sec
        goal_str = f"({delta_x:.2f}, {delta_y:.2f}, " \
                   f" {round(degrees(delta_theta))}°)"
        if accepted:
            info = f"Bắt đầu thử nghiệm #{experiment_index+1}: " \
                   f"Di chuyển đến "
        else:
            info = f"Thử nghiệm #{experiment_index+1} thất bại: " \
                   f"Hệ thống từ chối hoạch định đến "

        self.get_logger().info(info + goal_str)

        # Set marker pose
        self.set_entity_state(
            self.marker_name, self.goal_pose.pose.position.x,
            self.goal_pose.pose.position.y, q=self.goal_pose.pose.orientation,
        )

        # Set camera pose
        self.set_entity_state(
            self.camera_name,
            x=(pose.position.x + self.goal_pose.pose.position.x)/2,
            y=(pose.position.x + self.goal_pose.pose.position.x)/2,
            z=30.0, q=self.camera_orientation,
        )

    def collect_result(self):
        result = self.navigator.getResult()
        self.current_pose = self.navigator.getFeedback().current_pose

        end_time = self.get_clock().now().to_msg().sec
        pose = self.get_entity_state(self.robot_name).pose
        distance_error = hypot(
            self.goal_pose.pose.position.y - pose.position.y,
            self.goal_pose.pose.position.x - pose.position.x)
        heading_error = abs(
            yaw_from_pose(pose) - yaw_from_pose(self.goal_pose.pose))

        if result == TaskResult.SUCCEEDED:
            success = True
            info = f"""Đã đến đích!
                Sai số:
                - Khoảng cách: {distance_error} m
                - Hướng: {round(degrees(heading_error))}°
            """
        elif result == TaskResult.FAILED:
            success = False
            info = "Di chuyển đến đích thất bại..."
        elif result == TaskResult.CANCELED:
            raise RuntimeError('Task bị cancel một cách bất thường')
        else:
            raise TypeError('Sai mã status trả về!')

        self.get_logger().info(info)
        with open(data_file_path, 'a') as file:
            line = \
                ','.join(map(str, (self.current_experiment_index, success,
                                   self.start_time, end_time - self.start_time,
                                   distance_error, heading_error))) \
                    + '\n'
            file.write(line)



    def get_entity_state(self, entity_name):
        req = GetEntityState.Request()
        if self.get_client.wait_for_service(timeout_sec=4.0):
            req.name = entity_name
            srv_call = self.get_client.call_async(req)
            rclpy.spin_until_future_complete(self, srv_call)
            result = srv_call.result()
            if result.success:
                return result.state
        return None

    def set_entity_state(self, entity_name, x, y, z=None, yaw=None, q=None):
        req = SetEntityState.Request()
        if self.set_client.wait_for_service(timeout_sec=4.0):
            req.state.name = entity_name
            req.state.pose.position.x = x
            req.state.pose.position.y = y
            if z is not None:
                req.state.pose.position.z = z
            if q is not None:
                req.state.pose.orientation = q
            elif yaw is not None:
                req.state.pose.orientation = quaternion_from_yaw(yaw)
            srv_call = self.set_client.call_async(req)
            rclpy.spin_until_future_complete(self, srv_call)
            result = srv_call.result()
            return result.success
        return False

# Functions --------------------------------------------------------------------

def quaternion_from_yaw(yaw):
    q = Quaternion()
    q.x, q.y, q.z, q.w = quaternion_about_axis(yaw, (0, 0, 1))
    return q

def yaw_from_pose(pose):
    q = pose.orientation
    return euler_from_quaternion((q.x, q.y, q.z, q.w))[2]

# Main thread ------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)

    eval_manager = BeetleEvaluationManager()

    while True:
        eval_manager.loop()
        sleep(0.2)

    eval_manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
