#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "ackermann_drive_controller/ackermann_drive_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace
{
constexpr auto DEFAULT_COMMAND_TOPIC = "/cmd_vel_stamped";
constexpr auto DEFAULT_COMMAND_UNSTAMPED_TOPIC = "/cmd_vel";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "/cmd_vel_out";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "/odom";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
}  // namespace

namespace ackermann_drive_controller
{
using namespace std::chrono_literals;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using hardware_interface::HW_IF_EFFORT;
using lifecycle_msgs::msg::State;

AckermannDriveController::AckermannDriveController() : controller_interface::ControllerInterface() {}

const char * AckermannDriveController::feedback_type() const
{
  return odom_params_.position_feedback ? HW_IF_POSITION : HW_IF_VELOCITY;
}

controller_interface::CallbackReturn AckermannDriveController::on_init()
{
  try
  {
    // with the lifecycle node being initialized, we can declare parameters
    auto_declare<std::string>("left_wheel_name", std::string());
    auto_declare<std::string>("right_wheel_name", std::string());
    auto_declare<std::string>("steering_joint_name_", std::string());

    auto_declare<double>("wheel_separation", wheel_params_.separation);
    auto_declare<double>("wheel_radius", wheel_params_.radius);

    auto_declare<double>("wheel_base", steer_params_.wheel_base);
    auto_declare<double>("pivot_distance", steer_params_.pivot_distance);
    auto_declare<double>("max_steering_angle", steer_params_.max_steering_angle);
    auto_declare<double>("steering_arm_1", steer_params_.steering_arm_1);
    auto_declare<double>("steering_arm_2", steer_params_.steering_arm_2);
    auto_declare<double>("rack_distance", steer_params_.rack_distance);
    auto_declare<double>("rack_limit", steer_params_.rack_limit);
    auto_declare<int>("lut_size", steer_params_.lut_size);
    auto_declare<double>("rack_kp", ctrl_.rack_kp);
    auto_declare<double>("rack_ki", ctrl_.rack_ki);
    auto_declare<double>("rack_kd", ctrl_.rack_kd);

    auto_declare<std::string>("odom_frame_id", odom_params_.odom_frame_id);
    auto_declare<std::string>("base_frame_id", odom_params_.base_frame_id);
    auto_declare<std::vector<double>>("pose_covariance_diagonal", std::vector<double>());
    auto_declare<std::vector<double>>("twist_covariance_diagonal", std::vector<double>());
    auto_declare<bool>("open_loop", odom_params_.open_loop);
    auto_declare<bool>("position_feedback", odom_params_.position_feedback);
    auto_declare<bool>("enable_odom_tf", odom_params_.enable_odom_tf);

    auto_declare<double>("cmd_vel_timeout", cmd_vel_timeout_.count() / 1000.0);
    publish_limited_velocity_ =
      auto_declare<bool>("publish_limited_velocity", publish_limited_velocity_);
    auto_declare<int>("velocity_rolling_window_size", 10);
    use_stamped_vel_ = auto_declare<bool>("use_stamped_vel", use_stamped_vel_);

    auto_declare<bool>("linear.x.has_velocity_limits", false);
    auto_declare<bool>("linear.x.has_acceleration_limits", false);
    auto_declare<bool>("linear.x.has_jerk_limits", false);
    auto_declare<double>("linear.x.max_velocity", NAN);
    auto_declare<double>("linear.x.min_velocity", NAN);
    auto_declare<double>("linear.x.max_acceleration", NAN);
    auto_declare<double>("linear.x.min_acceleration", NAN);
    auto_declare<double>("linear.x.max_jerk", NAN);
    auto_declare<double>("linear.x.min_jerk", NAN);

    auto_declare<bool>("angular.z.has_velocity_limits", false);
    auto_declare<bool>("angular.z.has_acceleration_limits", false);
    auto_declare<bool>("angular.z.has_jerk_limits", false);
    auto_declare<double>("angular.z.max_velocity", NAN);
    auto_declare<double>("angular.z.min_velocity", NAN);
    auto_declare<double>("angular.z.max_acceleration", NAN);
    auto_declare<double>("angular.z.min_acceleration", NAN);
    auto_declare<double>("angular.z.max_jerk", NAN);
    auto_declare<double>("angular.z.min_jerk", NAN);
    publish_rate_ = auto_declare<double>("publish_rate", publish_rate_);
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

InterfaceConfiguration AckermannDriveController::command_interface_configuration() const
{
  std::vector<std::string> conf_names;
  conf_names.push_back(left_wheel_name_ + "/" + HW_IF_VELOCITY);
  conf_names.push_back(right_wheel_name_ + "/" + HW_IF_VELOCITY);
  conf_names.push_back(steering_joint_name_ + "/" + HW_IF_EFFORT);
  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

InterfaceConfiguration AckermannDriveController::state_interface_configuration() const
{
  std::vector<std::string> conf_names;
  conf_names.push_back(left_wheel_name_ + "/" + feedback_type());
  conf_names.push_back(right_wheel_name_ + "/" + feedback_type());
  conf_names.push_back(steering_joint_name_ + "/" + HW_IF_POSITION);

  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::return_type AckermannDriveController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto logger = get_node()->get_logger();
  if (get_state().id() == State::PRIMARY_STATE_INACTIVE)
  {
    if (!is_halted)
    {
      halt();
      is_halted = true;
    }
    return controller_interface::return_type::OK;
  }

  std::shared_ptr<Twist> last_command_msg;
  received_velocity_msg_ptr_.get(last_command_msg);

  if (last_command_msg == nullptr)
  {
    RCLCPP_WARN(logger, "Velocity message received was a nullptr.");
    return controller_interface::return_type::ERROR;
  }

  const auto age_of_last_command = time - last_command_msg->header.stamp;
  // Brake if cmd_vel has timeout, override the stored command
  if (age_of_last_command > cmd_vel_timeout_)
  {
    last_command_msg->twist.linear.x = 0.0;
    last_command_msg->twist.angular.z = 0.0;
  }

  // command may be limited further by SpeedLimit,
  // without affecting the stored twist command
  Twist command = *last_command_msg;
  double & linear_command = command.twist.linear.x;
  double & angular_command = command.twist.angular.z;

  previous_update_timestamp_ = time;

  // Apply (possibly new) multipliers:
  const auto wheels = wheel_params_;

  const auto steer = steer_params_;

  if (odom_params_.open_loop)
  {
    odometry_.updateOpenLoop(linear_command, angular_command, time);
  }
  else
  {
    const double left_feedback =
      registered_left_wheel_handle_->feedback.get().get_value();
    const double right_feedback =
      registered_right_wheel_handle_->feedback.get().get_value();

    if (std::isnan(left_feedback) || std::isnan(right_feedback))
    {
      RCLCPP_ERROR(
        logger, "Either the left or right wheel %s is invalid", feedback_type());
      return controller_interface::return_type::ERROR;
    }

    if (odom_params_.position_feedback)
    {
      odometry_.update(left_feedback, right_feedback, time);
    }
    else
    {
      odometry_.updateFromVelocity(
        left_feedback * period.seconds(), right_feedback * period.seconds(), time);
    }
  }

  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, odometry_.getHeading());

  if (previous_publish_timestamp_ + publish_period_ < time)
  {
    previous_publish_timestamp_ += publish_period_;

    if (realtime_odometry_publisher_->trylock())
    {
      auto & odometry_message = realtime_odometry_publisher_->msg_;
      odometry_message.header.stamp = time;
      odometry_message.pose.pose.position.x = odometry_.getX();
      odometry_message.pose.pose.position.y = odometry_.getY();
      odometry_message.pose.pose.orientation.x = orientation.x();
      odometry_message.pose.pose.orientation.y = orientation.y();
      odometry_message.pose.pose.orientation.z = orientation.z();
      odometry_message.pose.pose.orientation.w = orientation.w();
      odometry_message.twist.twist.linear.x = odometry_.getLinear();
      odometry_message.twist.twist.angular.z = odometry_.getAngular();
      realtime_odometry_publisher_->unlockAndPublish();
    }

    if (odom_params_.enable_odom_tf && realtime_odometry_transform_publisher_->trylock())
    {
      auto & transform = realtime_odometry_transform_publisher_->msg_.transforms.front();
      transform.header.stamp = time;
      transform.transform.translation.x = odometry_.getX();
      transform.transform.translation.y = odometry_.getY();
      transform.transform.rotation.x = orientation.x();
      transform.transform.rotation.y = orientation.y();
      transform.transform.rotation.z = orientation.z();
      transform.transform.rotation.w = orientation.w();
      realtime_odometry_transform_publisher_->unlockAndPublish();
    }
  }

  auto & last_command = previous_commands_.back().twist;
  auto & second_to_last_command = previous_commands_.front().twist;
  limiter_linear_.limit(
    linear_command, last_command.linear.x, second_to_last_command.linear.x, period.seconds());
  limiter_angular_.limit(
    angular_command, last_command.angular.z, second_to_last_command.angular.z, period.seconds());

  previous_commands_.pop();
  previous_commands_.emplace(command);

  //    Publish limited velocity
  if (publish_limited_velocity_ && realtime_limited_velocity_publisher_->trylock())
  {
    auto & limited_velocity_command = realtime_limited_velocity_publisher_->msg_;
    limited_velocity_command.header.stamp = time;
    limited_velocity_command.twist = command.twist;
    realtime_limited_velocity_publisher_->unlockAndPublish();
  }

  // Limit angular command based on min turning radius
  const double angular_command_limit =
    fabs(linear_command) / steer.min_turning_radius;
  if (angular_command > angular_command_limit)
  {
    RCLCPP_WARN_ONCE(logger, "Maximum turning rate limit exceeded: %f", angular_command_limit);
    angular_command = angular_command_limit;
  }
  else if (angular_command < -angular_command_limit)
  {
    RCLCPP_WARN_ONCE(logger, "Minimum turning rate limit exceeded: %f", -angular_command_limit);
    angular_command = -angular_command_limit;
  }

  // Compute wheels velocities:
  double velocity_left =
    (linear_command - angular_command * wheels.separation / 2.0) / wheels.radius;
  double velocity_right =
    (linear_command + angular_command * wheels.separation / 2.0) / wheels.radius;

  // Compute steering angles
  // velocity_right *= right_steering_angle/alpha2;
  const double current_rack_pos = \
  registered_steering_handle_->feedback.get().get_value();
  static double desired_rack_pos = 0;

  if (linear_command != 0)
  {
    desired_rack_pos =
      rack_offset_from_curvature_lut(angular_command/linear_command);
  }
  const double rack_pos_error = current_rack_pos - desired_rack_pos;
  const double rack_effort = -ctrl_.rack_kp*rack_pos_error;

  // Set wheels velocities:
  registered_left_wheel_handle_->control.get().set_value(velocity_left);
  registered_right_wheel_handle_->control.get().set_value(velocity_right);

  // Set steering angles
  registered_steering_handle_->control.get().set_value(rack_effort);
  // static int cnt = 0;
  // if (cnt++ % 50 == 0)
  // {
  //   RCLCPP_INFO(logger, "Rack error: %f", rack_pos_error);
  //   RCLCPP_INFO(logger, "Rack effort: %f", rack_effort);
  // }

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn AckermannDriveController::on_configure(
  const rclcpp_lifecycle::State &)
{
  auto logger = get_node()->get_logger();

  // update parameters
  left_wheel_name_ = get_node()->get_parameter("left_wheel_name").as_string();
  right_wheel_name_ = get_node()->get_parameter("right_wheel_name").as_string();
  steering_joint_name_ = get_node()->get_parameter("steering_joint_name").as_string();

  wheel_params_.separation = get_node()->get_parameter("wheel_separation").as_double();
  wheel_params_.radius = get_node()->get_parameter("wheel_radius").as_double();

  const auto wheels = wheel_params_;

  steer_params_.wheel_base = get_node()->get_parameter("wheel_base").as_double();
  steer_params_.pivot_distance =
    get_node()->get_parameter("pivot_distance").as_double();
  steer_params_.max_steering_angle =
    get_node()->get_parameter("max_steering_angle").as_double();
  steer_params_.steering_arm_1 =
    get_node()->get_parameter("steering_arm_1").as_double();
  steer_params_.steering_arm_2 =
    get_node()->get_parameter("steering_arm_2").as_double();
  steer_params_.rack_distance =
    get_node()->get_parameter("rack_distance").as_double();
  steer_params_.rack_limit =
    get_node()->get_parameter("rack_limit").as_double();
  steer_params_.lut_size =
    get_node()->get_parameter("lut_size").as_int();


  steer_params_.ackermann_angle =
    atan(steer_params_.pivot_distance/steer_params_.wheel_base/2);
  steer_params_.min_turning_radius =
    steer_params_.wheel_base / tan(steer_params_.max_steering_angle);
  steer_params_.rack_initial_position = rack_position_from_curvature(0);
  steer_params_.curvature_limit =
    curvature_from_rack_offset(steer_params_.rack_limit);

  generate_lookup_tables();

  ctrl_.rack_kp = get_node()->get_parameter("rack_kp").as_double();
  ctrl_.rack_ki = get_node()->get_parameter("rack_ki").as_double();
  ctrl_.rack_kd = get_node()->get_parameter("rack_kd").as_double();

  odometry_.setWheelParams(wheels.separation, wheels.radius, wheels.radius);
  odometry_.setVelocityRollingWindowSize(
    get_node()->get_parameter("velocity_rolling_window_size").as_int());

  odom_params_.odom_frame_id = get_node()->get_parameter("odom_frame_id").as_string();
  odom_params_.base_frame_id = get_node()->get_parameter("base_frame_id").as_string();

  auto pose_diagonal = get_node()->get_parameter("pose_covariance_diagonal").as_double_array();
  std::copy(
    pose_diagonal.begin(), pose_diagonal.end(), odom_params_.pose_covariance_diagonal.begin());

  auto twist_diagonal = get_node()->get_parameter("twist_covariance_diagonal").as_double_array();
  std::copy(
    twist_diagonal.begin(), twist_diagonal.end(), odom_params_.twist_covariance_diagonal.begin());

  odom_params_.open_loop = get_node()->get_parameter("open_loop").as_bool();
  odom_params_.position_feedback = get_node()->get_parameter("position_feedback").as_bool();
  odom_params_.enable_odom_tf = get_node()->get_parameter("enable_odom_tf").as_bool();

  cmd_vel_timeout_ = std::chrono::milliseconds{
    static_cast<int>(get_node()->get_parameter("cmd_vel_timeout").as_double() * 1000.0)};
  publish_limited_velocity_ = get_node()->get_parameter("publish_limited_velocity").as_bool();
  use_stamped_vel_ = get_node()->get_parameter("use_stamped_vel").as_bool();

  try
  {
    limiter_linear_ = SpeedLimiter(
      get_node()->get_parameter("linear.x.has_velocity_limits").as_bool(),
      get_node()->get_parameter("linear.x.has_acceleration_limits").as_bool(),
      get_node()->get_parameter("linear.x.has_jerk_limits").as_bool(),
      get_node()->get_parameter("linear.x.min_velocity").as_double(),
      get_node()->get_parameter("linear.x.max_velocity").as_double(),
      get_node()->get_parameter("linear.x.min_acceleration").as_double(),
      get_node()->get_parameter("linear.x.max_acceleration").as_double(),
      get_node()->get_parameter("linear.x.min_jerk").as_double(),
      get_node()->get_parameter("linear.x.max_jerk").as_double());
  }
  catch (const std::runtime_error & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error configuring linear speed limiter: %s", e.what());
  }

  try
  {
    limiter_angular_ = SpeedLimiter(
      get_node()->get_parameter("angular.z.has_velocity_limits").as_bool(),
      get_node()->get_parameter("angular.z.has_acceleration_limits").as_bool(),
      get_node()->get_parameter("angular.z.has_jerk_limits").as_bool(),
      get_node()->get_parameter("angular.z.min_velocity").as_double(),
      get_node()->get_parameter("angular.z.max_velocity").as_double(),
      get_node()->get_parameter("angular.z.min_acceleration").as_double(),
      get_node()->get_parameter("angular.z.max_acceleration").as_double(),
      get_node()->get_parameter("angular.z.min_jerk").as_double(),
      get_node()->get_parameter("angular.z.max_jerk").as_double());
  }
  catch (const std::runtime_error & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error configuring angular speed limiter: %s", e.what());
  }

  if (!reset())
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  if (publish_limited_velocity_)
  {
    limited_velocity_publisher_ =
      get_node()->create_publisher<Twist>(DEFAULT_COMMAND_OUT_TOPIC, rclcpp::SystemDefaultsQoS());
    realtime_limited_velocity_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<Twist>>(limited_velocity_publisher_);
  }

  const Twist empty_twist;
  received_velocity_msg_ptr_.set(std::make_shared<Twist>(empty_twist));

  // Fill last two commands with default constructed commands
  previous_commands_.emplace(empty_twist);
  previous_commands_.emplace(empty_twist);

  // initialize command subscriber
  if (use_stamped_vel_)
  {
    velocity_command_subscriber_ = get_node()->create_subscription<Twist>(
      DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<Twist> msg) -> void
      {
        if (!subscriber_is_active_)
        {
          RCLCPP_WARN(
            get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
          return;
        }
        if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0))
        {
          RCLCPP_WARN_ONCE(
            get_node()->get_logger(),
            "Received TwistStamped with zero timestamp, setting it to current "
            "time, this message will only be shown once");
          msg->header.stamp = get_node()->get_clock()->now();
        }
        received_velocity_msg_ptr_.set(std::move(msg));
      });
  }
  else
  {
    velocity_command_unstamped_subscriber_ =
      get_node()->create_subscription<geometry_msgs::msg::Twist>(
        DEFAULT_COMMAND_UNSTAMPED_TOPIC, rclcpp::SystemDefaultsQoS(),
        [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) -> void
        {
          if (!subscriber_is_active_)
          {
            RCLCPP_WARN(
              get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
            return;
          }

          // Write fake header in the stored stamped command
          std::shared_ptr<Twist> twist_stamped;
          received_velocity_msg_ptr_.get(twist_stamped);
          twist_stamped->twist = *msg;
          twist_stamped->header.stamp = get_node()->get_clock()->now();
        });
  }

  // initialize odometry publisher and messasge
  odometry_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
    DEFAULT_ODOMETRY_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_odometry_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
      odometry_publisher_);

  auto & odometry_message = realtime_odometry_publisher_->msg_;
  odometry_message.header.frame_id = odom_params_.odom_frame_id;
  odometry_message.child_frame_id = odom_params_.base_frame_id;

  // limit the publication on the topics /odom and /tf
  publish_rate_ = get_node()->get_parameter("publish_rate").as_double();
  publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);
  previous_publish_timestamp_ = get_node()->get_clock()->now();

  // initialize odom values zeros
  odometry_message.twist =
    geometry_msgs::msg::TwistWithCovariance(rosidl_runtime_cpp::MessageInitialization::ALL);

  constexpr size_t NUM_DIMENSIONS = 6;
  for (size_t index = 0; index < 6; ++index)
  {
    // 0, 7, 14, 21, 28, 35
    const size_t diagonal_index = NUM_DIMENSIONS * index + index;
    odometry_message.pose.covariance[diagonal_index] = odom_params_.pose_covariance_diagonal[index];
    odometry_message.twist.covariance[diagonal_index] =
      odom_params_.twist_covariance_diagonal[index];
  }

  // initialize transform publisher and message
  odometry_transform_publisher_ = get_node()->create_publisher<tf2_msgs::msg::TFMessage>(
    DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_odometry_transform_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
      odometry_transform_publisher_);

  // keeping track of odom and base_link transforms only
  auto & odometry_transform_message = realtime_odometry_transform_publisher_->msg_;
  odometry_transform_message.transforms.resize(1);
  odometry_transform_message.transforms.front().header.frame_id = odom_params_.odom_frame_id;
  odometry_transform_message.transforms.front().child_frame_id = odom_params_.base_frame_id;

  previous_update_timestamp_ = get_node()->get_clock()->now();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AckermannDriveController::on_activate(
  const rclcpp_lifecycle::State &)
{
  const auto left_result =
    configure_joint(left_wheel_name_, feedback_type(), HW_IF_VELOCITY);
  const auto right_result =
    configure_joint(right_wheel_name_, feedback_type(), HW_IF_VELOCITY);
  const auto steer_result =
    configure_joint(steering_joint_name_, HW_IF_POSITION, HW_IF_EFFORT);

  if (
    left_result == controller_interface::CallbackReturn::ERROR ||
    right_result == controller_interface::CallbackReturn::ERROR ||
    steer_result == controller_interface::CallbackReturn::ERROR)
  {
    return controller_interface::CallbackReturn::ERROR;
  }
  registered_left_wheel_handle_ = &registered_joint_handles_[0];
  registered_right_wheel_handle_ = &registered_joint_handles_[1];
  registered_steering_handle_ = &registered_joint_handles_[2];

  is_halted = false;
  subscriber_is_active_ = true;

  RCLCPP_DEBUG(get_node()->get_logger(), "Subscriber and publisher are now active.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AckermannDriveController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  subscriber_is_active_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AckermannDriveController::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  received_velocity_msg_ptr_.set(std::make_shared<Twist>());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AckermannDriveController::on_error(const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

bool AckermannDriveController::reset()
{
  odometry_.resetOdometry();

  // release the old queue
  std::queue<Twist> empty;
  std::swap(previous_commands_, empty);

  registered_joint_handles_.clear();

  subscriber_is_active_ = false;
  velocity_command_subscriber_.reset();
  velocity_command_unstamped_subscriber_.reset();

  received_velocity_msg_ptr_.set(nullptr);
  is_halted = false;
  return true;
}

controller_interface::CallbackReturn AckermannDriveController::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

void AckermannDriveController::halt()
{
  for (const auto & joint_handle : registered_joint_handles_)
  {
    joint_handle.control.get().set_value(0.0);
  }
}

controller_interface::CallbackReturn AckermannDriveController::configure_joint(
  const std::string & joint_name, const char * state_interface_name,
  const char * command_interface_name)
{
  auto logger = get_node()->get_logger();
  const auto state_handle = std::find_if(
    state_interfaces_.cbegin(), state_interfaces_.cend(),
    [&joint_name, &state_interface_name](const auto & interface)
    {
      return interface.get_prefix_name() == joint_name &&
             interface.get_interface_name() == state_interface_name;
    });

  if (state_handle == state_interfaces_.cend())
  {
    RCLCPP_ERROR(logger, "Unable to obtain joint state handle for %s", joint_name.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  const auto command_handle = std::find_if(
    command_interfaces_.begin(), command_interfaces_.end(),
    [&joint_name, &command_interface_name](const auto & interface)
    {
      return interface.get_prefix_name() == joint_name &&
             interface.get_interface_name() == command_interface_name;
    });

  if (command_handle == command_interfaces_.end())
  {
    RCLCPP_ERROR(logger, "Unable to obtain joint command handle for %s", joint_name.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  registered_joint_handles_.emplace_back(
    JointHandle{std::ref(*state_handle), std::ref(*command_handle)});

  return controller_interface::CallbackReturn::SUCCESS;
}

void AckermannDriveController::generate_lookup_tables()
{
  const double rack_offset_per_step =
    2*steer_params_.rack_limit/steer_params_.lut_size;
  const double curvature_offset_per_step =
    2*steer_params_.curvature_limit/steer_params_.lut_size;

  // RCLCPP_INFO(get_node()->get_logger(), "Rack limit: %f", steer_params_.rack_limit);
  // RCLCPP_INFO(get_node()->get_logger(), "Curvature limit: %f", steer_params_.curvature_limit);
  // RCLCPP_INFO(get_node()->get_logger(), "LUT size: %d", steer_params_.lut_size);
  double rack_offset = -steer_params_.rack_limit;
  double curvature = -steer_params_.curvature_limit;
  for (int i = -steer_params_.lut_size/2; i < steer_params_.lut_size/2; i++)
  {
    rack_offset_lut_.push_back(rack_offset);
    curvature_lut_.push_back(curvature);
    // RCLCPP_INFO(get_node()->get_logger(), "Rack offset: %f", rack_offset);
    // RCLCPP_INFO(get_node()->get_logger(), "Curvature: %f", curvature);
    rack_offset += rack_offset_per_step;
    curvature += curvature_offset_per_step;
  }
}
double AckermannDriveController::rack_position_from_curvature(double curvature)
{
  double alpha, phi, v1, v2, h1, h2;

  alpha = -atan(curvature*steer_params_.wheel_base);
  phi = alpha + steer_params_.ackermann_angle;
  v1 = steer_params_.steering_arm_1 * cos(phi);
  h1 = steer_params_.steering_arm_1 * sin(phi);

  v2 = steer_params_.rack_distance - v1;
  h2 = sqrt(steer_params_.steering_arm_2*steer_params_.steering_arm_2 - v2*v2);

  return h1 + h2;
}

double AckermannDriveController::curvature_from_rack_offset(double rack_pos)
{
  const double l1 = steer_params_.steering_arm_1;
  const double l2 = steer_params_.steering_arm_2;
  const double rack_offset = rack_pos + steer_params_.rack_initial_position;

  const double l3 = hypot(rack_offset, steer_params_.rack_distance);
  const double alpha = atan2(steer_params_.rack_distance, rack_offset) +
    acos((l1*l1 + l3*l3 - l2*l2)/(2*l1*l3)) +
    steer_params_.ackermann_angle - M_PI/2;
  return -atan(alpha)/steer_params_.wheel_base;
}

double AckermannDriveController::rack_offset_from_curvature_lut(
  double curvature)
{
  const double index =
    steer_params_.lut_size*(curvature + steer_params_.curvature_limit)/steer_params_.curvature_limit/2;
  const int floored_index = floor(index);
  const double ratio = index - floored_index;
  return (1.0 - ratio) * rack_offset_lut_[index] + ratio * rack_offset_lut_[index + 1];
}

double AckermannDriveController::curvature_from_rack_offset_lut(
  double rack_offset)
{
  const double index =
    steer_params_.lut_size*(rack_offset + steer_params_.rack_limit)/steer_params_.rack_limit/2;
  const int floored_index = floor(index);
  const double ratio = index - floored_index;
  return (1.0 - ratio) * curvature_lut_[index] + ratio * curvature_lut_[index + 1];
}
}  // namespace ackermann_drive_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  ackermann_drive_controller::AckermannDriveController, controller_interface::ControllerInterface)

/*
try:
    l1, l2, d, gamma, b, L, *_ = map(float, sys.argv[3:])
except ValueError as e:
    if "not enough values to unpack" not in str(e):
        raise e
    l1 = 0.080
    l2 = 0.215
    d = 0.12365818824032602
    gamma = 0.32540155164520534
    b, L = 0.803, 1.190

def rack_pos_from_alpha(alpha, s):
    phi = alpha - s * gamma

    v1 = l1 * np.cos(phi)
    h1 = -l1 * np.sin (phi)

    v2 = d - v1
    h2 = s*np.sqrt(l2*l2 - v2*v2)

    return h1 + h2

x0 = rack_pos_from_alpha(0, 1)

def alphas_from_rack_pos(x, s=np.array([[1], [-1]])):
    x = s*x + x0

    l3 = np.hypot(x, d)
    a = np.arctan2(d, x)

    a += np.arccos((l1*l1 + l3*l3 - l2*l2)/(2*l1*l3))

    a += gamma - np.pi/2

    return a * [[1], [-1]]

def curve_radii(alpha1, alpha2):
    return -np.array([[L]]) / np.tan([alpha1, alpha2]) + [[-b/2], [b/2]]

def curvatures(alpha1, alpha2):
    return 1/curve_radii(alpha1, alpha2)


lim = 0.0325
cnt = 100
x = np.linspace(-lim, lim, cnt)
alpha1, alpha2 = alphas_from_rack_pos(x)
kappa1, kappa2 = curvatures(alpha1, alpha2)

*/
