#include <chrono>
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

    // Random generator setup
    random_generator_ = std::mt19937(random_device_());
    distance_distribution_ = std::uniform_real_distribution<float>(0.3f, 3.0f);

    RCLCPP_INFO(this->get_logger(), "Sensor A (LiDAR simulation) started");
  }

private:
  void timer_callback()
  {
    std_msgs::msg::Float32 data_msg;

    // Random LiDAR distance (0.3m to 3.0m)
    float distance = distance_distribution_(random_generator_);
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
  std::uniform_real_distribution<float> distance_distribution_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorANode>());
  rclcpp::shutdown();
  return 0;
}
