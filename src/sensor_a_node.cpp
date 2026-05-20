#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class SensorANode : public rclcpp::Node
{
public:
  SensorANode() : Node("sensor_a_node")
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.reliable();
    qos.deadline(100ms);
    qos.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC);
    qos.liveliness_lease_duration(300ms);

    data_pub_ = this->create_publisher<std_msgs::msg::Float32>("/data_a", qos);
    heartbeat_pub_ = this->create_publisher<std_msgs::msg::String>("/node1/heartbeat", qos);

    timer_ = this->create_wall_timer(
      100ms,
      std::bind(&SensorANode::timer_callback, this));

    random_generator_ = std::mt19937(random_device_());
    noise_distribution_ = std::uniform_real_distribution<float>(-0.03f, 0.03f);
    demo_started_at_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Sensor A (LiDAR simulation) started");
  }

private:
  float distance_for_demo()
  {
    const double elapsed = std::fmod((this->now() - demo_started_at_).seconds(), 32.0);
    const float noise = noise_distribution_(random_generator_);

    if (elapsed < 10.0) {
      return 2.75f + static_cast<float>(std::sin(elapsed * 0.45) * 0.12) + noise;
    }

    if (elapsed < 18.0) {
      const float progress = static_cast<float>((elapsed - 10.0) / 8.0);
      return std::max(0.45f, 2.75f - (2.25f * progress) + noise);
    }

    if (elapsed < 23.0) {
      return 0.45f + static_cast<float>(std::sin(elapsed * 2.0) * 0.04) + noise;
    }

    if (elapsed < 28.0) {
      const float progress = static_cast<float>((elapsed - 23.0) / 5.0);
      return std::min(2.75f, 0.45f + (2.30f * progress) + noise);
    }

    return 2.75f + static_cast<float>(std::sin(elapsed * 0.35) * 0.10) + noise;
  }

  void timer_callback()
  {
    std_msgs::msg::Float32 data_msg;

    float distance = distance_for_demo();
    data_msg.data = distance;

    data_pub_->publish(data_msg);

    std_msgs::msg::String heartbeat_msg;
    heartbeat_msg.data = "sensor_a_node alive";
    heartbeat_pub_->publish(heartbeat_msg);

    bool data_liveliness_ok = data_pub_->assert_liveliness();
    bool heartbeat_liveliness_ok = heartbeat_pub_->assert_liveliness();

    if (!data_liveliness_ok || !heartbeat_liveliness_ok) {
      RCLCPP_WARN(this->get_logger(), "Failed to assert liveliness");
    }

    RCLCPP_INFO(this->get_logger(), "LiDAR distance: %.2f", distance);
  }

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr data_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::random_device random_device_;
  std::mt19937 random_generator_;
  std::uniform_real_distribution<float> noise_distribution_;
  rclcpp::Time demo_started_at_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorANode>());
  rclcpp::shutdown();
  return 0;
}
