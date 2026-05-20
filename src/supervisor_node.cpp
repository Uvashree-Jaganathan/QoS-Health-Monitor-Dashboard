#include <memory>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class SupervisorNode : public rclcpp::Node
{
public:
  SupervisorNode() : Node("supervisor_node")
  {
    remove_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/supervisor/remove_node",
      10,
      std::bind(&SupervisorNode::remove_callback, this, std::placeholders::_1));

    deregister_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/deregister_node",
      10);

    RCLCPP_INFO(this->get_logger(), "Supervisor node started");
  }

private:
  void remove_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::stringstream ss(msg->data);
    std::string heartbeat_topic;

    while (std::getline(ss, heartbeat_topic, ','))
    {
      if (heartbeat_topic.empty()) {
        continue;
      }

      std_msgs::msg::String dereg_msg;
      dereg_msg.data = heartbeat_topic;

      deregister_pub_->publish(dereg_msg);

      RCLCPP_WARN(
        this->get_logger(),
        "Supervisor requested removal of node with heartbeat: %s",
        heartbeat_topic.c_str());
    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr remove_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr deregister_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SupervisorNode>());
  rclcpp::shutdown();
  return 0;
}
