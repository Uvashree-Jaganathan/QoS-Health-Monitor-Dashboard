#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <algorithm>

#include "ament_index_cpp/get_package_share_directory.hpp"
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
      deregister_sub_ = this->create_subscription<std_msgs::msg::String>(
  	"/deregister_node",
 	 10,
     std::bind(&HealthMonitorNode::deregister_node_callback, this, std::placeholders::_1));
     
     network_status_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/network_status",
      10,
      std::bind(&HealthMonitorNode::network_status_callback, this, std::placeholders::_1));

    this->declare_parameter<std::string>("config_file", default_config_file());

    std::string config_file;
    this->get_parameter("config_file", config_file);


    load_config(config_file);

    status_timer_ = this->create_wall_timer(
    1s,
    std::bind(&HealthMonitorNode::publish_health_status, this));



    RCLCPP_INFO(this->get_logger(), "Health Monitor node started");
  }

private:
  struct MonitoredNode
  {
    std::string name;
    std::string heartbeat_topic;
    std::string failure_reason;

    int deadline_ms = 0;
    int liveliness_ms = 0;

    bool use_deadline = false;
    bool use_liveliness = false;

    bool alive = false;
    bool qos_incompatible = false;
    rclcpp::Time last_heartbeat_time;
    bool deregistered = false;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;
  };

  std::string default_config_file() const
  {
    try {
      return ament_index_cpp::get_package_share_directory("qos_health_monitor_demo") +
        "/config/health_monitor.yaml";
    } catch (const std::exception & error) {
      RCLCPP_WARN(
        this->get_logger(),
        "Could not resolve package share directory, using relative config path: %s",
        error.what());
      return "config/health_monitor.yaml";
    }
  }

  void load_config(const std::string & config_file)
  {
    YAML::Node config;

    try {
      config = YAML::LoadFile(config_file);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to load health monitor config '%s': %s",
        config_file.c_str(),
        error.what());
      return;
    }

    if (!config["nodes"] || !config["nodes"].IsSequence())
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Health monitor config '%s' must contain a nodes list",
        config_file.c_str());
      return;
    }

    for (const auto & node_config : config["nodes"])
    {
      std::string name = node_config["name"].as<std::string>();
      std::string heartbeat_topic = node_config["heartbeat_topic"].as<std::string>();
      std::string failure_reason = node_config["failure_reason"].as<std::string>();

      int deadline_ms = 0;
      int liveliness_ms = 0;

      bool use_deadline = false;
      bool use_liveliness = false;

      if (node_config["deadline_ms"])
      {
        deadline_ms = node_config["deadline_ms"].as<int>();
        use_deadline = true;
      }

      if (node_config["liveliness_ms"])
      {
        liveliness_ms = node_config["liveliness_ms"].as<int>();
        use_liveliness = true;
      }

      if (!use_deadline && !use_liveliness)
      {
        RCLCPP_WARN(
          this->get_logger(),
          "Skipping %s: at least one QoS policy is required",
          name.c_str());
        continue;
      }

      add_monitored_node(
        name,
        heartbeat_topic,
        deadline_ms,
        liveliness_ms,
        use_deadline,
        use_liveliness,
        failure_reason);
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

    if (name.empty() || heartbeat_topic.empty() || failure_reason.empty())
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Invalid registration message: %s",
        msg->data.c_str());
      return;
    }

    int deadline_ms = 0;
    int liveliness_ms = 0;

    bool use_deadline = false;
    bool use_liveliness = false;

    if (!deadline_str.empty())
    {
      deadline_ms = std::stoi(deadline_str);
      use_deadline = true;
    }

    if (!liveliness_str.empty())
    {
      liveliness_ms = std::stoi(liveliness_str);
      use_liveliness = true;
    }

    if (!use_deadline && !use_liveliness)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Node must use at least one QoS policy: %s",
        name.c_str());
      return;
    }

    add_monitored_node(
      name,
      heartbeat_topic,
      deadline_ms,
      liveliness_ms,
      use_deadline,
      use_liveliness,
      failure_reason);
  }
  
 void deregister_node_callback(const std_msgs::msg::String::SharedPtr msg)
{
  std::string heartbeat_topic = msg->data;

  auto it = std::find_if(
    monitored_nodes_.begin(),
    monitored_nodes_.end(),
    [&](const std::shared_ptr<MonitoredNode> & node)
    {
      return node->heartbeat_topic == heartbeat_topic;
    });

  if (it != monitored_nodes_.end())
  {
    (*it)->deregistered = true;
    (*it)->alive = true;

    if ((*it)->subscription) {
      (*it)->subscription.reset();
    }

    RCLCPP_WARN(
      this->get_logger(),
      "Node deregistered intentionally: %s",
      heartbeat_topic.c_str());

    publish_health_status();
  }
  else
  {
    RCLCPP_WARN(
      this->get_logger(),
      "Deregister request received but node not found: %s",
      heartbeat_topic.c_str());
  }
}

void network_status_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    network_connection_healthy_ = (msg->data != "NETWORK_UNHEALTHY");
    publish_health_status();
  }

  std::shared_ptr<MonitoredNode> find_monitored_node(const std::string & heartbeat_topic)
  {
    for (const auto & node : monitored_nodes_)
    {
      if (node->heartbeat_topic == heartbeat_topic) {
        return node;
      }
    }

    return nullptr;
  }

 
  
void add_monitored_node(
  const std::string & name,
  const std::string & heartbeat_topic,
  int deadline_ms,
  int liveliness_ms,
  bool use_deadline,
  bool use_liveliness,
  const std::string & failure_reason)
{
  auto monitored_node = find_monitored_node(heartbeat_topic);

  if (monitored_node && !monitored_node->deregistered)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Node already monitored: %s",
      heartbeat_topic.c_str());
    return;
  }

  const bool re_registering = static_cast<bool>(monitored_node);

  if (!monitored_node) {
    monitored_node = std::make_shared<MonitoredNode>();
  }

  monitored_node->name = name;
  monitored_node->heartbeat_topic = heartbeat_topic;
  monitored_node->failure_reason = failure_reason;

  monitored_node->deadline_ms = deadline_ms;
  monitored_node->liveliness_ms = liveliness_ms;

  monitored_node->use_deadline = use_deadline;
  monitored_node->use_liveliness = use_liveliness;
  monitored_node->alive = false;
  monitored_node->qos_incompatible = false;
  monitored_node->deregistered = false;

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  qos.reliable();

  if (monitored_node->use_deadline)
  {
    qos.deadline(std::chrono::milliseconds(deadline_ms));
  }

  if (monitored_node->use_liveliness)
  {
    // Keep commented only for CLI testing.
    // Re-enable when real nodes call assert_liveliness().
    qos.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC);

    qos.liveliness_lease_duration(std::chrono::milliseconds(liveliness_ms));
  }

  rclcpp::SubscriptionOptions options;
  if (monitored_node->use_deadline)
	{
  options.event_callbacks.deadline_callback =
    [this, monitored_node](rclcpp::QOSDeadlineRequestedInfo &)
    {
      if (monitored_node->deregistered) return;
      monitored_node->alive = false;

      RCLCPP_ERROR(
        this->get_logger(),
        "Deadline missed for %s",
        monitored_node->name.c_str());

      publish_health_status();
    };
   } 

   if (monitored_node->use_liveliness)
 	{
  options.event_callbacks.liveliness_callback =
    [this, monitored_node](rclcpp::QOSLivelinessChangedInfo & event)
    {
    if (monitored_node->deregistered) return;
      if (event.alive_count == 0)
      {
        monitored_node->alive = false;

        RCLCPP_ERROR(
          this->get_logger(),
          "Liveliness lost for %s",
          monitored_node->name.c_str());

        publish_health_status();
      }
    };
  }

  options.event_callbacks.incompatible_qos_callback =
    [this, monitored_node](rclcpp::QOSRequestedIncompatibleQoSInfo & event)
    {
      monitored_node->alive = false;
      monitored_node->qos_incompatible = true;

      RCLCPP_ERROR(
        this->get_logger(),
        "QoS incompatibility detected for %s on %s | last_policy_kind=%d",
        monitored_node->name.c_str(),
        monitored_node->heartbeat_topic.c_str(),
        event.last_policy_kind);

      publish_health_status();
    };

  monitored_node->subscription =
    this->create_subscription<std_msgs::msg::String>(
      heartbeat_topic,
      qos,
      [this, monitored_node](const std_msgs::msg::String::SharedPtr)
      {
        monitored_node->alive = true;
        monitored_node->qos_incompatible = false;
        monitored_node->last_heartbeat_time = this->now();

        publish_health_status();
      },
      options);

  if (!re_registering) {
    monitored_nodes_.push_back(monitored_node);
  }

  RCLCPP_INFO(
    this->get_logger(),
    "%s %s on %s",
    re_registering ? "Re-monitoring" : "Monitoring",
    name.c_str(),
    heartbeat_topic.c_str());

  publish_health_status();
}
void publish_health_status()
{
  std_msgs::msg::String health_msg;
  std_msgs::msg::String reason_msg;

  bool all_ok = true;
  reason_msg.data = "";

  for (const auto & node : monitored_nodes_)
  {
    if (node->deregistered) {
      continue;
    }

    if (!node->alive)
    {
      all_ok = false;

      if (!reason_msg.data.empty()) {
        reason_msg.data += ",";
      }

      if (node->qos_incompatible) {
        reason_msg.data += node->failure_reason + "_QOS_INCOMPATIBLE";
      } else {
        reason_msg.data += node->failure_reason;
      }
    }
  }

  if (!network_connection_healthy_)
  {
    all_ok = false;

    if (!reason_msg.data.empty()) {
      reason_msg.data += ",";
    }

    reason_msg.data += "NETWORK_CONNECTION_FAILURE";
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
    if (node->deregistered) {
      continue;
    }

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
rclcpp::TimerBase::SharedPtr status_timer_;
rclcpp::Subscription<std_msgs::msg::String>::SharedPtr register_sub_;
rclcpp::Subscription<std_msgs::msg::String>::SharedPtr deregister_sub_;

rclcpp::Subscription<std_msgs::msg::String>::SharedPtr network_status_sub_;

std::vector<std::shared_ptr<MonitoredNode>> monitored_nodes_;
  bool network_connection_healthy_ = true;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HealthMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
