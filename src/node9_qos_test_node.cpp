#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


using namespace std::chrono_literals;

class Node9QosTest : public rclcpp::Node
{
public:
  Node9QosTest() : Node("node9_qos_test_node")
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.reliable();
    qos.deadline(1000ms);
    qos.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC);
    qos.liveliness_lease_duration(2000ms);

    heartbeat_pub_ =
      this->create_publisher<std_msgs::msg::String>("/node9/heartbeat", qos);

    deregister_pub_ =
      this->create_publisher<std_msgs::msg::String>("/deregister_node",10);
    shutdown_sub_ =
      this->create_subscription<std_msgs::msg::String>(
    "/node9/shutdown_request",
    10,
    std::bind(
      &Node9QosTest::shutdown_callback,
      this,
      std::placeholders::_1));
    timer_ =
      this->create_wall_timer(300ms, std::bind(&Node9QosTest::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Node 9 QoS test node started");
    
  }
  

private:
  void timer_callback()
  {
    std_msgs::msg::String heartbeat_msg;
    heartbeat_msg.data = "node9 alive";

    heartbeat_pub_->publish(heartbeat_msg);
    if (!heartbeat_pub_->assert_liveliness()) {
      RCLCPP_WARN(this->get_logger(), "Failed to assert Node 9 liveliness");
    }

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Node 9 heartbeat published with compatible QoS");
      
  }
  void shutdown_callback(
  const std_msgs::msg::String::SharedPtr)
{
  RCLCPP_WARN(
    this->get_logger(),
    "Node 9 received planned shutdown request");

  std_msgs::msg::String dereg_msg;

  dereg_msg.data = "/node9/heartbeat";

  deregister_pub_->publish(dereg_msg);

  RCLCPP_WARN(
    this->get_logger(),
    "Node 9 requesting planned deregistration");

  rclcpp::sleep_for(std::chrono::milliseconds(1000));

  rclcpp::shutdown();
}

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr deregister_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr shutdown_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Node9QosTest>());
  rclcpp::shutdown();
  return 0;
}
