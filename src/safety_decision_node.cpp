#include <chrono>
#include <memory>
#include <string>
#include <sstream>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class SafetyDecisionNode : public rclcpp::Node
{
public:
  SafetyDecisionNode() : Node("safety_decision_node")
  {
    auto data_a_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    data_a_qos.reliable();
    data_a_qos.deadline(100ms);
    data_a_qos.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC);
    data_a_qos.liveliness_lease_duration(300ms);

    auto data_b_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    data_b_qos.reliable();
    data_b_qos.deadline(150ms);
    data_b_qos.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC);
    data_b_qos.liveliness_lease_duration(500ms);

    auto camera_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    camera_qos.reliable();
    camera_qos.deadline(1200ms);
    camera_qos.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC);
    camera_qos.liveliness_lease_duration(2000ms);

    auto node3_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    node3_qos.reliable();
    node3_qos.deadline(200ms);
    node3_qos.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC);
    node3_qos.liveliness_lease_duration(500ms);

    rclcpp::SubscriptionOptions data_a_options;
    data_a_options.event_callbacks.deadline_callback =
      [this](rclcpp::QOSDeadlineRequestedInfo &)
      {
        node1_ok_ = false;
      };

    data_a_options.event_callbacks.liveliness_callback =
      [this](rclcpp::QOSLivelinessChangedInfo & info)
      {
        if (info.alive_count == 0) {
          node1_ok_ = false;
        }
      };

    rclcpp::SubscriptionOptions data_b_options;
    data_b_options.event_callbacks.deadline_callback =
      [this](rclcpp::QOSDeadlineRequestedInfo &)
      {
        node2_ok_ = false;
      };

    data_b_options.event_callbacks.liveliness_callback =
      [this](rclcpp::QOSLivelinessChangedInfo & info)
      {
        if (info.alive_count == 0) {
          node2_ok_ = false;
        }
      };

    rclcpp::SubscriptionOptions camera_options;
    camera_options.event_callbacks.deadline_callback =
      [this](rclcpp::QOSDeadlineRequestedInfo &)
      {
        camera_ok_ = false;
      };

    camera_options.event_callbacks.liveliness_callback =
      [this](rclcpp::QOSLivelinessChangedInfo & info)
      {
        if (info.alive_count == 0) {
          camera_ok_ = false;
        }
      };

    data_a_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/data_a",
      data_a_qos,
      std::bind(&SafetyDecisionNode::data_a_callback, this, std::placeholders::_1),
      data_a_options);

    data_b_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/data_b",
      data_b_qos,
      std::bind(&SafetyDecisionNode::data_b_callback, this, std::placeholders::_1),
      data_b_options);

    camera_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/camera/object_status",
      camera_qos,
      std::bind(&SafetyDecisionNode::camera_callback, this, std::placeholders::_1),
      camera_options);

    status_pub_ = this->create_publisher<std_msgs::msg::String>("/system_status", node3_qos);
    speed_pub_ = this->create_publisher<std_msgs::msg::Float32>("/adjusted_speed", node3_qos);
    heartbeat_pub_ = this->create_publisher<std_msgs::msg::String>("/node3/heartbeat", node3_qos);

    timer_ = this->create_wall_timer(
      200ms,
      std::bind(&SafetyDecisionNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Safety Decision node started");
  }

private:
  void data_a_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    distance_ = msg->data;
    node1_ok_ = true;
  }

  void data_b_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    normal_speed_ = msg->data;
    node2_ok_ = true;
  }

  void camera_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    camera_status_ = msg->data;
    camera_ok_ = true;
  }

  std::string format_float(float value)
  {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(2) << value;
    return stream.str();
  }

  void timer_callback()
  {
    const float safety_radius = 2.0f;

    std_msgs::msg::String status_msg;
    std_msgs::msg::Float32 speed_msg;

    if (!node1_ok_ || !node2_ok_ || !camera_ok_)
    {
      speed_msg.data = 0.0f;
      status_msg.data = "UNSAFE_STOP";
    }
    else if (distance_ >= safety_radius)
    {
      speed_msg.data = normal_speed_;
      status_msg.data = "SAFE_NORMAL_SPEED";
    }
    else
    {
      speed_msg.data = normal_speed_ * (distance_ / safety_radius);
      status_msg.data = "UNSAFE_REDUCE_SPEED";
    }

    status_pub_->publish(status_msg);
    speed_pub_->publish(speed_msg);

    std_msgs::msg::String heartbeat_msg;
    heartbeat_msg.data = "safety_decision_node alive";
    heartbeat_pub_->publish(heartbeat_msg);
    heartbeat_pub_->assert_liveliness();

    std::string node1_output;
    std::string node2_output;
    std::string node6_output;

    if (node1_ok_)
    {
      node1_output = "Node 1: Distance=" + format_float(distance_);
    }
    else
    {
      node1_output = "Node 1: FAILED UNSAFE_SENSOR_FAILURE";
    }

    if (node2_ok_)
    {
      node2_output = "Node 2: Normal speed=" + format_float(normal_speed_);
    }
    else
    {
      node2_output = "Node 2: FAILED UNSAFE_SPEED_FAILURE";
    }

    if (camera_ok_)
    {
      node6_output = "Node 6: " + camera_status_;
    }
    else
    {
      node6_output = "Node 6: FAILED CAMERA_FAILURE";
    }

    RCLCPP_INFO(
      this->get_logger(),
      "%s | %s | %s | Node 3: Adjusted speed=%s , Status=%s",
      node1_output.c_str(),
      node2_output.c_str(),
      node6_output.c_str(),
      format_float(speed_msg.data).c_str(),
      status_msg.data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr data_a_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr data_b_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr camera_sub_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  float distance_ = 2.0f;
  float normal_speed_ = 0.5f;

  std::string camera_status_ = "NO_OBSTACLE";

  bool node1_ok_ = false;
  bool node2_ok_ = false;
  bool camera_ok_ = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyDecisionNode>());
  rclcpp::shutdown();
  return 0;
}
