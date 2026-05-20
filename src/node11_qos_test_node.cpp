#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class Node11DeadlineOnly : public rclcpp::Node
{
public:
  Node11DeadlineOnly() : Node("node11_deadline_only")
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    qos.reliable();
    qos.deadline(1000ms);

    heartbeat_pub_ =
      this->create_publisher<std_msgs::msg::String>(
        "/node11/heartbeat",
        qos);

    timer_ =
      this->create_wall_timer(
        500ms,
        std::bind(&Node11DeadlineOnly::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Node 11 deadline-only started");
  }

private:
  void timer_callback()
  {
    std_msgs::msg::String heartbeat_msg;
    heartbeat_msg.data = "node11 alive";

    heartbeat_pub_->publish(heartbeat_msg);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Node11DeadlineOnly>());
  rclcpp::shutdown();
  return 0;
}
