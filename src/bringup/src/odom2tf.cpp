#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

class Odom2TF : public rclcpp::Node
{
public:
  Odom2TF() : Node("odom2tf")
  {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    // 配置 QoS 为 "Sensor Data" 模式 (Best Effort)
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.best_effort();
    qos.durability_volatile();

    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", qos, std::bind(&Odom2TF::handle_odom, this, std::placeholders::_1));
  }

private:
  void handle_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = msg->header.stamp;
    t.header.frame_id = "odom";
    t.child_frame_id = "base_footprint";

    t.transform.translation.x = msg->pose.pose.position.x;
    t.transform.translation.y = msg->pose.pose.position.y;
    t.transform.translation.z = msg->pose.pose.position.z;

    t.transform.rotation = msg->pose.pose.orientation;

    tf_broadcaster_->sendTransform(t);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odom2TF>());
  rclcpp::shutdown();
  return 0;
}