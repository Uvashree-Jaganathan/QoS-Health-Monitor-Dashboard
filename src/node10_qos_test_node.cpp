#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class Node10BadQosTest : public rclcpp::Node
{
public:
  Node10BadQosTest() : Node("node10_bad_qos_test_node")
  {
    heartbeat_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/node10/heartbeat", 10);

    timer_ = this->create_wall_timer(
      500ms,
      std::bind(&Node10BadQosTest::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Node 10 bad QoS test node started");
  }

private:
  void timer_callback()
  {
    std_msgs::msg::String heartbeat_msg;
    heartbeat_msg.data = "node10 alive";
    heartbeat_pub_->publish(heartbeat_msg);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Node10BadQosTest>());
  rclcpp::shutdown();
  return 0;
}
