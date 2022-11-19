#ifndef ACKERMANN_DRIVE_CONTROLLER__DIFF_DRIVE_CONTROLLER_HPP_
#define ACKERMANN_DRIVE_CONTROLLER__DIFF_DRIVE_CONTROLLER_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "ackermann_drive_controller/odometry.hpp"
#include "ackermann_drive_controller/speed_limiter.hpp"
#include "ackermann_drive_controller/visibility_control.h"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hardware_interface/handle.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "tf2_msgs/msg/tf_message.hpp"

namespace ackermann_drive_controller
{
class AckermannDriveController : public controller_interface::ControllerInterface
{
  using Twist = geometry_msgs::msg::TwistStamped;

public:
  ACKERMANN_DRIVE_CONTROLLER_PUBLIC
  AckermannDriveController();

  ACKERMANN_DRIVE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  ACKERMANN_DRIVE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  ACKERMANN_DRIVE_CONTROLLER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  ACKERMANN_DRIVE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  ACKERMANN_DRIVE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  ACKERMANN_DRIVE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  ACKERMANN_DRIVE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  ACKERMANN_DRIVE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  ACKERMANN_DRIVE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  ACKERMANN_DRIVE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  struct JointHandle
  {
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> feedback;
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> control;
  };

  const char * feedback_type() const;
  controller_interface::CallbackReturn configure_joint(
    const std::string & joint_name, const char * interface_name,
    const char * command_interface_name);

  void generate_lookup_tables();
  double rack_position_from_steering_angle(const double steering_angle);
  double steering_angle_from_rack_offset(double rack_offset);
  double approximate_rack_offset_from_curvature(const double curvature);
  double approximate_curvature_from_rack_offset(const double rack_offset);

  std::string traction_joint_name_;
  std::string steering_joint_name_;

  std::vector<JointHandle> registered_joint_handles_;
  JointHandle * registered_traction_handle_;
  JointHandle * registered_steering_handle_;

  struct WheelParams
  {
    double separation = 0.0;  // w.r.t. the midpoint of the wheel width
    double radius = 0.0;      // Assumed to be the same for both wheels
  } wheel_params_;

  struct SteerParams
  {
    double wheel_base = 0.0;
    double pivot_distance = 0.0;
    double steering_arm_1 = 0.0;
    double steering_arm_2 = 0.0;
    double rack_distance = 0.0;
    double rack_limit = 0.0;
    int lut_size = 100;

    double min_turning_radius = 0.0;
    double ackermann_angle = 0.0;
    double rack_initial_position = 0.0;
    double steering_angle_limit = 0.0;
  } steer_params_;

  std::vector<double> rack_offset_lut_;
  std::vector<double> curvature_lut_;

  struct ControlParams
  {
    double rack_kp = 1000.0;
    double rack_ki = 0.0;
    double rack_kd = 0.0;
  } ctrl_;

  struct OdometryParams
  {
    bool open_loop = false;
    bool position_feedback = true;
    bool enable_odom_tf = true;
    std::string base_frame_id = "base_link";
    std::string odom_frame_id = "odom";
    std::array<double, 6> pose_covariance_diagonal;
    std::array<double, 6> twist_covariance_diagonal;
  } odom_params_;

  Odometry odometry_;

  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>
    realtime_odometry_publisher_ = nullptr;

  std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> odometry_transform_publisher_ =
    nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>
    realtime_odometry_transform_publisher_ = nullptr;

  // Timeout to consider cmd_vel commands old
  std::chrono::milliseconds cmd_vel_timeout_{500};

  bool subscriber_is_active_ = false;
  rclcpp::Subscription<Twist>::SharedPtr velocity_command_subscriber_ = nullptr;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
    velocity_command_unstamped_subscriber_ = nullptr;

  realtime_tools::RealtimeBox<std::shared_ptr<Twist>> received_velocity_msg_ptr_{nullptr};

  std::queue<Twist> previous_commands_;  // last two commands

  // speed limiters
  SpeedLimiter limiter_linear_;
  SpeedLimiter limiter_angular_;

  bool publish_limited_velocity_ = false;
  std::shared_ptr<rclcpp::Publisher<Twist>> limited_velocity_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<Twist>> realtime_limited_velocity_publisher_ =
    nullptr;

  rclcpp::Time previous_update_timestamp_{0};

  // publish rate limiter
  double publish_rate_ = 50.0;
  rclcpp::Duration publish_period_ = rclcpp::Duration::from_nanoseconds(0);
  rclcpp::Time previous_publish_timestamp_{0};

  bool is_halted = false;
  bool use_stamped_vel_ = true;

  bool reset();
  void halt();
};
}  // namespace ackermann_drive_controller
#endif  // ACKERMANN_DRIVE_CONTROLLER__ACKERMANN_DRIVE_CONTROLLER_HPP_
