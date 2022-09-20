from math import sin, cos, tan, atan, isnan

import rclpy
from rclpy.time import Time
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

WHEEL_BASE = 2.94
MAX_STEERING_ANGLE = 0.8

class UGVDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_robot_driver')
        self.__node.create_subscription(Twist, '/cmd_vel',
                                        self.__cmd_vel_callback, 1)

        self.reset_odometry()
        self._odometry_publisher = self.__node.create_publisher(Odometry,
                                                                "/odom", 1)
        self._tf_broadcaster = TransformBroadcaster(self.__node)

        self._last_odometry_sample_time = self.__robot.getTime()

    def __cmd_vel_callback(self, message):
        speed = message.linear.x
        if speed == 0:
            steering_angle = 0
        else:
            steering_angle = atan(WHEEL_BASE*message.angular.z/speed)
        self.__robot.setCruisingSpeed(speed*3.6)
        self.__robot.setSteeringAngle(steering_angle)

    def step(self):
        self.__robot.step()
        rclpy.spin_once(self.__node, timeout_sec=0)

        stamp = Time(seconds=self.__robot.getTime()).to_msg()

        time_diff_s = self.__robot.getTime() - self._last_odometry_sample_time

        if time_diff_s == 0.0:
            return

        # Calculate velocities
        v = self.__robot.getCurrentSpeed()/7.2
        if isnan(v):
            v = 0
        omega = v*tan(self.__robot.getSteeringAngle())/WHEEL_BASE

        # self.__node.get_logger().info(f'Last twist: {(v, omega)}')
        # self.__node.get_logger().info(f'Time diff: {time_diff_s}')
        # self.__node.get_logger().info(f'Prev angle: {self._prev_angle}')

        # Calculate position & angle
        # Fourth order Runge - Kutta
        # Reference: https://www.cs.cmu.edu/~16311/s07/labs/NXTLabs/Lab%203.html
        k00 = v * cos(self._prev_angle)
        k01 = v * sin(self._prev_angle)
        k02 = omega
        k10 = v * cos(self._prev_angle + time_diff_s * k02 / 2)
        k11 = v * sin(self._prev_angle + time_diff_s * k02 / 2)
        k12 = omega
        k20 = v * cos(self._prev_angle + time_diff_s * k12 / 2)
        k21 = v * sin(self._prev_angle + time_diff_s * k12 / 2)
        k22 = omega
        k30 = v * cos(self._prev_angle + time_diff_s * k22 / 2)
        k31 = v * sin(self._prev_angle + time_diff_s * k22 / 2)
        k32 = omega
        position = [
            self._prev_position[0] + (time_diff_s / 6) *
            (k00 + 2 * (k10 + k20) + k30),
            self._prev_position[1] + (time_diff_s / 6) *
            (k01 + 2 * (k11 + k21) + k31)
        ]
        angle = self._prev_angle + \
            (time_diff_s / 6) * (k02 + 2 * (k12 + k22) + k32)

        # Update variables
        self._prev_position = position.copy()
        self._prev_angle = angle
        self._last_odometry_sample_time = self.__robot.getTime()

        # Pack & publish odometry
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = omega
        msg.pose.pose.position.x = position[0]
        msg.pose.pose.position.y = position[1]
        msg.pose.pose.orientation.z = sin(angle / 2)
        msg.pose.pose.orientation.w = cos(angle / 2)
        self._odometry_publisher.publish(msg)

        # Pack & publish transforms
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = position[0]
        tf.transform.translation.y = position[1]
        tf.transform.translation.z = 0.0
        tf.transform.rotation.z = sin(angle / 2)
        tf.transform.rotation.w = cos(angle / 2)
        self._tf_broadcaster.sendTransform(tf)

    def reset_odometry(self):
        self._prev_position = (0.0, 0.0)
        self._prev_angle = 0.0
