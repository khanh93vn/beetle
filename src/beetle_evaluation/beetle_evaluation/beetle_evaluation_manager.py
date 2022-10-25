from math import pi, sin, cos, hypot, degrees
from itertools import product

import rclpy

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg._quaternion import Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import quaternion_about_axis, euler_from_quaternion
from gazebo_msgs.srv import GetEntityState


# Constants --------------------------------------------------------------------
DEBUG = True

# Experiment samples
sample_distances = [2.0, 6.0]
sample_directions = [i*(pi/3) for i in range(4)]
sample_headings = [i*(pi/4) for i in range(8)]
timer_period = 1.0  # seconds

# ------------------------------------------------------------------------------
# Class BeetleEvaluationManager

class BeetleEvaluationManager(Node):

    def __init__(self):
        super().__init__('beetle_evaluation_manager')

        # Initialize gazebo clients
        self.get_entity_state_client = \
            self.create_client(GetEntityState, '/get_entity_state')
        self.get_entity_state_req = GetEntityState.Request()
        self.get_entity_state_req.name = 'beetle'
        self.get_entity_state_req.reference_frame = ''
        self.get_entity_state_res = GetEntityState.Response()
        if self.get_entity_state_client.wait_for_service(timeout_sec=3.0):
            if self.get_entity_state_res.success:
                self.current_position = self.get_entity_state_res.state.pose
            self.get_entity_state_call = \
                self.get_entity_state_client.call_async(self.get_entity_state_req)
            self.get_logger().info("Đợi lấy dữ liệu vị trí xe")
            while not self.get_entity_state_call.done():
                rclpy.spin_once(self)
            self.get_entity_state_res = self.get_entity_state_call.result()
            if self.get_entity_state_res.success:
                self.current_pose = self.get_entity_state_res.state.pose
        else:
            raise RuntimeError("Không thể lấy dữ liệu vị trí từ Gazebo")


        self._initialize_navigator()

        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if self.get_entity_state_call.done():
            self.get_entity_state_res = \
                self.get_entity_state_call.result()
            if self.get_entity_state_res.success:
                self.current_pose = self.get_entity_state_res.state.pose
        self.get_entity_state_call = \
            self.get_entity_state_client.call_async(self.get_entity_state_req)

        if self.current_experiment_index >= len(self.relative_goals):
            return

        if self.navigator.isTaskComplete():
            if self.current_experiment_index > 0:
                self.collect_results()
            self.begin_experiment(self.current_experiment_index)
            self.current_experiment_index += 1


    def _initialize_navigator(self):
        self.navigator = BasicNavigator()

        # Set initial pose
        self.get_logger().info("Đang giúp xe set trạng thái đầu")
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose = self.current_pose
        self.navigator.setInitialPose(initial_pose)

        # Generate goals
        self.relative_goals = [
            (r*cos(phi), r*sin(phi), delta_theta)
            for r, phi, delta_theta in product(
                sample_distances, sample_directions, sample_headings)]

        # Begin testing
        self.experimental_results = []
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map'
        self.current_experiment_index = 0

    def begin_experiment(self, experiment_index):
        delta_x, delta_y, delta_theta = self.relative_goals[experiment_index]
        pose = self.current_pose
        self.goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.goal_pose.pose.position.x = pose.position.x + delta_x
        self.goal_pose.pose.position.y = pose.position.y + delta_y
        self.goal_pose.pose.orientation = quaternion_from_yaw(
            yaw_from_pose(pose) + delta_theta)
        accepted = self.navigator.goToPose(self.goal_pose)
        goal_str = f"({delta_x:.2f}, {delta_y:.2f}, " \
                   f" {round(degrees(delta_theta))}°)"
        if accepted:
            info = f"Bắt đầu thử nghiệm #{self.current_experiment_index}: " \
                   f"Di chuyển đến "
        else:
            info = f"Thử nghiệm #{self.current_experiment_index} thất bại: " \
                   f"Hệ thống từ chối hoạch định đến "

        self.get_logger().info(info + goal_str)

        # TODO: add SpawnEntity or SetEntityState client

    def collect_results(self):
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Đã đến đích!")
            pose = self.current_pose
            distance_error = hypot(self.goal_pose.pose.position.y - pose.position.y,
                                   self.goal_pose.pose.position.x - pose.position.x)
            heading_error = abs(yaw_from_pose(pose) - yaw_from_pose(self.goal_pose.pose))

            self.get_logger().info(f"""
                Sai số:
                - Khoảng cách: {distance_error} m
                - Hướng: {round(degrees(heading_error))}°
            """)
            self.experimental_results.append((distance_error, heading_error))
        elif result == TaskResult.FAILED:
            self.get_logger().info("Di chuyển đến đích thất bại...")
            self.experimental_results.append(None)
        elif result == TaskResult.CANCELED:
            raise RuntimeError('Task bị cancel một cách bất thường')
        else:
            raise TypeError('Sai mã status trả về!')

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

    rclpy.spin(eval_manager)

    eval_manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
