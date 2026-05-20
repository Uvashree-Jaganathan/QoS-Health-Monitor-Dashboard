#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class BatteryNode : public rclcpp::Node
{
public:
  BatteryNode() : Node("battery_node")
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.reliable();
    qos.deadline(1000ms);
    qos.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC);
    qos.liveliness_lease_duration(2000ms);

    percentage_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "/battery_percentage", qos);
    status_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/battery_status", qos);
    heartbeat_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/node13/heartbeat", qos);

    timer_ = this->create_wall_timer(
      1000ms,
      std::bind(&BatteryNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Battery node started");
  }

private:
  std::string battery_status(const float percentage) const
  {
    if (percentage < 5.0f) {
      return "EMERGENCY_BATTERY";
    }

    if (percentage < 15.0f) {
      return "CRITICAL_BATTERY";
    }

    if (percentage < 30.0f) {
      return "LOW_BATTERY";
    }

    return "BATTERY_NORMAL";
  }

  float simulated_percentage() const
  {
    const int phase = (step_ / hold_steps_) % cycle_phase_count_;

    if (phase < 10) {
      return 100.0f - static_cast<float>(phase * 10);
    }

    if (phase < 14) {
      return 10.0f - static_cast<float>((phase - 10) * 2);
    }

    return 3.0f;
  }

  void timer_callback()
  {
    const float percentage = std::clamp(simulated_percentage(), 0.0f, 100.0f);
    const std::string status = battery_status(percentage);

    std_msgs::msg::Float32 percentage_msg;
    percentage_msg.data = percentage;

    std_msgs::msg::String status_msg;
    status_msg.data = status;

    std_msgs::msg::String heartbeat_msg;
    heartbeat_msg.data = "battery_node alive";

    percentage_pub_->publish(percentage_msg);
    status_pub_->publish(status_msg);
    heartbeat_pub_->publish(heartbeat_msg);

    if (!percentage_pub_->assert_liveliness() ||
        !status_pub_->assert_liveliness() ||
        !heartbeat_pub_->assert_liveliness()) {
      RCLCPP_WARN(this->get_logger(), "Failed to assert battery node liveliness");
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Battery=%.1f%% | Status=%s",
      percentage,
      status.c_str());

    step_++;
  }

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr percentage_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  int step_ = 0;
  const int hold_steps_ = 20;
  const int cycle_phase_count_ = 18;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BatteryNode>());
  rclcpp::shutdown();
  return 0;
}
