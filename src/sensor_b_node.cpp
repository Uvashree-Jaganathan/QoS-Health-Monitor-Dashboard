#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class SensorBNode : public rclcpp::Node
{
public:
  SensorBNode() : Node("sensor_b_node")
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.reliable();
    qos.deadline(100ms);
    qos.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC);
    qos.liveliness_lease_duration(300ms);

    speed_pub_ = this->create_publisher<std_msgs::msg::Float32>("/data_b", qos);
    heartbeat_pub_ = this->create_publisher<std_msgs::msg::String>("/node2/heartbeat", qos);

    timer_ = this->create_wall_timer(
      100ms,
      std::bind(&SensorBNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Sensor B node started: publishing normal speed and heartbeat");
  }

private:
  void timer_callback()
  {
    const float normal_speed = 0.5f;

    std_msgs::msg::Float32 speed_msg;
    speed_msg.data = normal_speed;
    speed_pub_->publish(speed_msg);

    std_msgs::msg::String heartbeat_msg;
    heartbeat_msg.data = "sensor_b_node alive";
    heartbeat_pub_->publish(heartbeat_msg);

    bool speed_ok = speed_pub_->assert_liveliness();
    bool heartbeat_ok = heartbeat_pub_->assert_liveliness();

    if (!speed_ok || !heartbeat_ok) {
      RCLCPP_WARN(this->get_logger(), "Failed to assert Node B liveliness");
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Normal speed: %.2f ",
      normal_speed);
  }

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorBNode>());
  rclcpp::shutdown();
  return 0;
}
