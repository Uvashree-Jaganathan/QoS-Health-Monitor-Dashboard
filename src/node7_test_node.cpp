#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class Node7Test : public rclcpp::Node
{
public:
  Node7Test() : Node("node7_test_node")
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.reliable();
    qos.deadline(500ms);
    qos.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC);
    qos.liveliness_lease_duration(1000ms);

    heartbeat_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/node7/heartbeat", qos);

    timer_ = this->create_wall_timer(
      500ms,
      std::bind(&Node7Test::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Node 7 started and publishing heartbeat");
  }

private:
  void timer_callback()
  {
    std_msgs::msg::String heartbeat_msg;
    heartbeat_msg.data = "node7 alive";

    heartbeat_pub_->publish(heartbeat_msg);
    if (!heartbeat_pub_->assert_liveliness()) {
      RCLCPP_WARN(this->get_logger(), "Failed to assert Node 7 liveliness");
    }
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Node7Test>());
  rclcpp::shutdown();
  return 0;
}
