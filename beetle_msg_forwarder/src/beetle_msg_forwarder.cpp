#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "beetle_msgs/msg/twist_lite.hpp"
#include "beetle_msgs/msg/odometry_lite.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class BeetleMsgForwarder : public rclcpp::Node
{
  public:
    BeetleMsgForwarder() : Node("msg_forwarder")
    {
      // create publishers
      odometry_publisher_ =
        this->create_publisher<nav_msgs::msg::Odometry>("odom", 2);
      cmd_vel_publisher_ =
        this->create_publisher<beetle_msgs::msg::TwistLite>("cmd_vel_lite", 2);

      // prepare publish data
      odom_.header.frame_id = std::string("odom");
      odom_.child_frame_id = std::string("base_link");

      // create callback group
      odom_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
      cmd_vel_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

      rclcpp::SubscriptionOptions options;

      // create subscribers
      options.callback_group = odom_cb_group_;
      odometry_subscription_ =
        this->create_subscription<beetle_msgs::msg::OdometryLite>(
          "odom_lite", rclcpp::SensorDataQoS(),
          std::bind(&BeetleMsgForwarder::odom_subs_callback, this, _1),
          options);

      options.callback_group = cmd_vel_cb_group_;
      cmd_vel_subscription_ =
        this->create_subscription<geometry_msgs::msg::Twist>(
          "cmd_vel", rclcpp::ServicesQoS(),
          std::bind(&BeetleMsgForwarder::cmd_vel_subs_callback, this, _1),
          options);
    }

  private:
    void odom_subs_callback( const beetle_msgs::msg::OdometryLite & odom_lite)
    {
      odom_.header.stamp = this->get_clock()->now();
      odom_.pose.pose.position.x = odom_lite.pose.x;
      odom_.pose.pose.position.y = odom_lite.pose.y;
      odom_.pose.pose.orientation.z = sin(odom_lite.pose.theta);
      odom_.pose.pose.orientation.w = cos(odom_lite.pose.theta);
      odom_.twist.twist.linear.x = odom_lite.twist.linear;
      odom_.twist.twist.angular.z = odom_lite.twist.angular;

      odometry_publisher_->publish(odom_);
    }

    void cmd_vel_subs_callback( const geometry_msgs::msg::Twist & cmd_vel)
    {
      cmd_vel_lite_.linear = cmd_vel.linear.x;
      cmd_vel_lite_.angular = cmd_vel.angular.z;

      cmd_vel_publisher_->publish(cmd_vel_lite_);
    }

    nav_msgs::msg::Odometry odom_;
    beetle_msgs::msg::TwistLite cmd_vel_lite_;

    rclcpp::CallbackGroup::SharedPtr odom_cb_group_;
    rclcpp::CallbackGroup::SharedPtr cmd_vel_cb_group_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
    rclcpp::Publisher<beetle_msgs::msg::TwistLite>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<beetle_msgs::msg::OdometryLite>::SharedPtr
      odometry_subscription_;
      rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
        cmd_vel_subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BeetleMsgForwarder>());
  rclcpp::shutdown();
  return 0;
}
