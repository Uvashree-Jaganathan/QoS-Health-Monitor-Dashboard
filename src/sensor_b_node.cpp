#include <algorithm>
#include <chrono>
#include <cmath>
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

    demo_started_at_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Sensor B node started: publishing adaptive speed and heartbeat");
  }

private:
  float speed_for_demo()
  {
    const double elapsed = std::fmod((this->now() - demo_started_at_).seconds(), 32.0);

    if (elapsed < 10.0) {
      return 0.58f + static_cast<float>(std::sin(elapsed * 0.45) * 0.02);
    }

    if (elapsed < 18.0) {
      const float progress = static_cast<float>((elapsed - 10.0) / 8.0);
      return 0.58f - (0.22f * progress);
    }

    if (elapsed < 23.0) {
      return 0.24f;
    }

    if (elapsed < 28.0) {
      const float progress = static_cast<float>((elapsed - 23.0) / 5.0);
      return 0.24f + (0.34f * progress);
    }

    return 0.58f + static_cast<float>(std::sin(elapsed * 0.35) * 0.015);
  }

  void timer_callback()
  {
    const float normal_speed = speed_for_demo();

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
  rclcpp::Time demo_started_at_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorBNode>());
  rclcpp::shutdown();
  return 0;
}
