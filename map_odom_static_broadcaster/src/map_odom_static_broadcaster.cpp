#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

class MapOdomStaticBroadcaster : public rclcpp::Node
{
public:
  explicit MapOdomStaticBroadcaster()
  : Node("map_odom_static_broadcaster")
  {
    tf_static_broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    this->declare_parameter("init_pose.x", -32.0);
    this->declare_parameter("init_pose.y", 140.0);
    this->declare_parameter("init_pose.psi", M_PI/2);

    // Publish static transforms once at startup
    this->make_transforms();

  }

private:
  void make_transforms()
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map";
    t.child_frame_id = "odom";
    double init_x = this->get_parameter("init_pose.x").as_double();
    double init_y = this->get_parameter("init_pose.y").as_double();
    double init_yaw = this->get_parameter("init_pose.psi").as_double();

    t.transform.translation.x = init_x;
    t.transform.translation.y = init_y;
    t.transform.translation.z = 0;
    tf2::Quaternion q;
    q.setRPY(0, 0, init_yaw); 
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_static_broadcaster_->sendTransform(t);
  }

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  double init_x_, init_y_, init_yaw_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapOdomStaticBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
