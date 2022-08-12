import rclpy
from ackermann_msgs.msg import AckermannDrive


class AGVDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_robot_driver')
        self.__node.create_subscription(AckermannDrive, 'cmd_ackermann',
                                        self.__cmd_ackermann_callback, 1)

    def __cmd_ackermann_callback(self, message):
        self.__robot.setCruisingSpeed(message.speed)
        self.__robot.setSteeringAngle(message.steering_angle)

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
