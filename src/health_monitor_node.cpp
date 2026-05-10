#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "yaml-cpp/yaml.h"

using namespace std::chrono_literals;

class HealthMonitorNode : public rclcpp::Node
{
public:
  HealthMonitorNode() : Node("health_monitor_node")
  {
    health_pub_ = this->create_publisher<std_msgs::msg::String>("/health_status", 10);
    reason_pub_ = this->create_publisher<std_msgs::msg::String>("/health_reason", 10);

    register_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/register_node",
      10,
      std::bind(&HealthMonitorNode::register_node_callback, this, std::placeholders::_1));

    this->declare_parameter<std::string>("config_file", "config/health_monitor.yaml");

    std::string config_file;
    this->get_parameter("config_file", config_file);

    load_config(config_file);

    RCLCPP_INFO(this->get_logger(), "Health Monitor node started");
  }

private:
  struct MonitoredNode
  {
    std::string name;
    std::string heartbeat_topic;
    std::string failure_reason;
    int deadline_ms;
    int liveliness_ms;

    bool alive = false;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;
  };

  void load_config(const std::string & config_file)
  {
    YAML::Node config = YAML::LoadFile(config_file);

    for (const auto & node_config : config["nodes"])
    {
      std::string name = node_config["name"].as<std::string>();
      std::string heartbeat_topic = node_config["heartbeat_topic"].as<std::string>();
      std::string failure_reason = node_config["failure_reason"].as<std::string>();
      int deadline_ms = node_config["deadline_ms"].as<int>();
      int liveliness_ms = node_config["liveliness_ms"].as<int>();

      add_monitored_node(name, heartbeat_topic, deadline_ms, liveliness_ms, failure_reason);
    }
  }

  void register_node_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::stringstream ss(msg->data);
    std::string name;
    std::string heartbeat_topic;
    std::string deadline_str;
    std::string liveliness_str;
    std::string failure_reason;

    std::getline(ss, name, ',');
    std::getline(ss, heartbeat_topic, ',');
    std::getline(ss, deadline_str, ',');
    std::getline(ss, liveliness_str, ',');
    std::getline(ss, failure_reason, ',');

    if (name.empty() || heartbeat_topic.empty() || deadline_str.empty() ||
        liveliness_str.empty() || failure_reason.empty())
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Invalid registration message: %s",
        msg->data.c_str());
      return;
    }

    int deadline_ms = std::stoi(deadline_str);
    int liveliness_ms = std::stoi(liveliness_str);

    add_monitored_node(name, heartbeat_topic, deadline_ms, liveliness_ms, failure_reason);
  }

  bool is_already_monitored(const std::string & heartbeat_topic)
  {
    for (const auto & node : monitored_nodes_)
    {
      if (node->heartbeat_topic == heartbeat_topic) {
        return true;
      }
    }

    return false;
  }

  void add_monitored_node(
    const std::string & name,
    const std::string & heartbeat_topic,
    int deadline_ms,
    int liveliness_ms,
    const std::string & failure_reason)
  {
    if (is_already_monitored(heartbeat_topic))
    {
      RCLCPP_INFO(
        this->get_logger(),
        "Node already monitored: %s",
        heartbeat_topic.c_str());
      return;
    }

    auto monitored_node = std::make_shared<MonitoredNode>();

    monitored_node->name = name;
    monitored_node->heartbeat_topic = heartbeat_topic;
    monitored_node->failure_reason = failure_reason;
    monitored_node->deadline_ms = deadline_ms;
    monitored_node->liveliness_ms = liveliness_ms;

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.reliable();
    qos.deadline(std::chrono::milliseconds(deadline_ms));
    qos.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC);
    qos.liveliness_lease_duration(std::chrono::milliseconds(liveliness_ms));

    rclcpp::SubscriptionOptions options;

    options.event_callbacks.deadline_callback =
      [this, monitored_node](rclcpp::QOSDeadlineRequestedInfo &)
      {
        monitored_node->alive = false;
        publish_health_status();
      };

    options.event_callbacks.liveliness_callback =
      [this, monitored_node](rclcpp::QOSLivelinessChangedInfo & event)
      {
        monitored_node->alive = (event.alive_count > 0);
        publish_health_status();
      };

    monitored_node->subscription =
      this->create_subscription<std_msgs::msg::String>(
        heartbeat_topic,
        qos,
        [this, monitored_node](const std_msgs::msg::String::SharedPtr)
        {
          monitored_node->alive = true;
          publish_health_status();
        },
        options);

    monitored_nodes_.push_back(monitored_node);

    RCLCPP_INFO(
      this->get_logger(),
      "Monitoring %s on %s",
      name.c_str(),
      heartbeat_topic.c_str());
  }

  void publish_health_status()
  {
    std_msgs::msg::String health_msg;
    std_msgs::msg::String reason_msg;

    bool all_ok = true;
    reason_msg.data = "";

    for (const auto & node : monitored_nodes_)
    {
      if (!node->alive)
      {
        all_ok = false;

        if (!reason_msg.data.empty()) {
          reason_msg.data += ",";
        }

        reason_msg.data += node->failure_reason;
      }
    }

    if (all_ok) {
      health_msg.data = "HEALTHY";
      reason_msg.data = "NONE";
    } else {
      health_msg.data = "UNHEALTHY";
    }

    health_pub_->publish(health_msg);
    reason_pub_->publish(reason_msg);

    std::string node_states;

    for (const auto & node : monitored_nodes_)
    {
      if (!node_states.empty()) {
        node_states += " ";
      }

      node_states += node->name + "=" + (node->alive ? "OK" : "FAILED");
    }

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Health Status: %s | Reason=%s | %s",
      health_msg.data.c_str(),
      reason_msg.data.c_str(),
      node_states.c_str());
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr health_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reason_pub_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr register_sub_;

  std::vector<std::shared_ptr<MonitoredNode>> monitored_nodes_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HealthMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
