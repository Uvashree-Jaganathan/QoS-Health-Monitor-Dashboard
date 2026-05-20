#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class Node12LivelinessOnly : public rclcpp::Node
{
public:
  Node12LivelinessOnly() : Node("node12_liveliness_only")
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    qos.reliable();

    qos.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC);

    qos.liveliness_lease_duration(1000ms);

    heartbeat_pub_ =
      this->create_publisher<std_msgs::msg::String>(
        "/node12/heartbeat",
        qos);

    register_pub_ =
      this->create_publisher<std_msgs::msg::String>(
        "/register_node",
        10);

    timer_ =
      this->create_wall_timer(
        500ms,
        std::bind(&Node12LivelinessOnly::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Node 12 liveliness-only started");
  }

private:
  void timer_callback()
  {
    if (registration_count_ < 5)
    {
      std_msgs::msg::String register_msg;

      register_msg.data =
        "Node 12,/node12/heartbeat,,1000,NODE12_LIVELINESS_FAILURE";

      register_pub_->publish(register_msg);

      registration_count_++;
    }

    std_msgs::msg::String heartbeat_msg;
    heartbeat_msg.data = "node12 alive";

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
  rclcpp::spin(std::make_shared<Node12LivelinessOnly>());
  rclcpp::shutdown();
  return 0;
}
