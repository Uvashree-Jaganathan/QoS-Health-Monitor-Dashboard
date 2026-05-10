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

    register_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/register_node", 10);

    timer_ = this->create_wall_timer(
      500ms,
      std::bind(&Node7Test::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Node 7 started and sending registration");
  }

private:
  void timer_callback()
  {
    if (registration_count_ < 5)
    {
      std_msgs::msg::String register_msg;
      register_msg.data = "Node 7,/node7/heartbeat,500,1000,NODE7_FAILURE";
      register_pub_->publish(register_msg);

      registration_count_++;
    }

    std_msgs::msg::String heartbeat_msg;
    heartbeat_msg.data = "node7 alive";

    heartbeat_pub_->publish(heartbeat_msg);
    heartbeat_pub_->assert_liveliness();
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr register_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  int registration_count_ = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Node7Test>());
  rclcpp::shutdown();
  return 0;
}
