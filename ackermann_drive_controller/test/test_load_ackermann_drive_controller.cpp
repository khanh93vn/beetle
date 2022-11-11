#include <gmock/gmock.h>
#include <memory>

#include "controller_manager/controller_manager.hpp"
#include "rclcpp/utilities.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

TEST(TestLoadAckermannDriveController, load_controller)
{
  rclcpp::init(0, nullptr);

  std::shared_ptr<rclcpp::Executor> executor =
    std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  controller_manager::ControllerManager cm(
    std::make_unique<hardware_interface::ResourceManager>(ros2_control_test_assets::diffbot_urdf),
    executor, "test_controller_manager");

  ASSERT_NO_THROW(
    cm.load_controller("test_ackermann_drive_controller", "ackermann_drive_controller/AckermannDriveController"));

  rclcpp::shutdown();
}
