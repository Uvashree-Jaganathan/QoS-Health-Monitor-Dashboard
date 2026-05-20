#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class CameraNode : public rclcpp::Node
{
public:
  CameraNode() : Node("camera_node")
  {
    auto lidar_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    lidar_qos.reliable();

    auto camera_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    camera_qos.reliable();

    camera_qos.deadline(1200ms);
    camera_qos.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC);
    camera_qos.liveliness_lease_duration(2000ms);

    lidar_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/data_a",
      lidar_qos,
      std::bind(&CameraNode::lidar_callback, this, std::placeholders::_1));

    object_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/camera/object_status",
      camera_qos);

    heartbeat_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/node6/heartbeat",
      camera_qos);

    timer_ = this->create_wall_timer(
      500ms,
      std::bind(&CameraNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Camera node started");
  }

private:
  void lidar_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    lidar_distance_ = msg->data;
    received_lidar_ = true;

    const float safety_radius = 2.0f;

    std::string object_name;
    std::string object_status;

    if (lidar_distance_ < safety_radius)
    {
      object_status = "OBSTACLE";
      object_name = "PERSON";
    }
    else
    {
      object_status = "NO_OBSTACLE";
      object_name = "NONE";
    }

    std_msgs::msg::String object_msg;
    object_msg.data = object_status;
    object_pub_->publish(object_msg);
    object_pub_->assert_liveliness();

    RCLCPP_INFO(
      this->get_logger(),
      "Node 6: Object=%s | Status=%s",
      object_name.c_str(),
      object_status.c_str());
  }

  void timer_callback()
  {
    std_msgs::msg::String heartbeat_msg;
    heartbeat_msg.data = "camera_node alive";
    heartbeat_pub_->publish(heartbeat_msg);
    heartbeat_pub_->assert_liveliness();
  }

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr lidar_sub_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr object_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  float lidar_distance_ = 2.0f;
  bool received_lidar_ = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraNode>());
  rclcpp::shutdown();
  return 0;
}
