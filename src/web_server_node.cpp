#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <exception>
#include <algorithm>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace
{
std::string escape_json(const std::string & input)
{
  std::ostringstream out;

  for (const char ch : input)
  {
    switch (ch)
    {
      case '"':
        out << "\\\"";
        break;
      case '\\':
        out << "\\\\";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        out << ch;
        break;
    }
  }

  return out.str();
}

std::string status_class(const std::string & value)
{
  if (value.find("UNHEALTHY") != std::string::npos ||
      value.find("UNSAFE_STOP") != std::string::npos ||
      value.find("EMERGENCY_BATTERY") != std::string::npos ||
      value.find("SAFE_LANDING") != std::string::npos ||
      value == "STOP" ||
      value.find("FAILURE") != std::string::npos) {
    return "danger";
  }

  if (value.find("HEALTHY") != std::string::npos ||
      value.find("SAFE_NORMAL") != std::string::npos ||
      value == "NORMAL") {
    return "ok";
  }

  if (value.find("REDUCE") != std::string::npos ||
      value.find("USING_") != std::string::npos ||
      value.find("RETURN_TO_BASE") != std::string::npos ||
      value.find("LOW_BATTERY") != std::string::npos ||
      value.find("CRITICAL_BATTERY") != std::string::npos ||
      value.find("BACKUP_CONNECTION") != std::string::npos ||
      value.find("DEGRADED") != std::string::npos) {
    return "warn";
  }

  return "danger";
}

std::string network_reason_value(const std::string & reason, const std::string & key)
{
  const std::string prefix = key + "=";
  const size_t start = reason.find(prefix);

  if (start == std::string::npos) {
    return "UNKNOWN";
  }

  const size_t value_start = start + prefix.size();
  const size_t value_end = reason.find(',', value_start);

  if (value_end == std::string::npos) {
    return reason.substr(value_start);
  }

  return reason.substr(value_start, value_end - value_start);
}

std::string network_state_value(const std::string & reason, const std::string & key)
{
  const std::string value = network_reason_value(reason, key);
  const size_t detail_start = value.find(':');

  if (detail_start == std::string::npos) {
    return value;
  }

  return value.substr(0, detail_start);
}

bool network_only_failure(const std::string & reason)
{
  return reason == "NETWORK_CONNECTION_FAILURE";
}
}  // namespace

class WebServerNode : public rclcpp::Node
{
public:
  WebServerNode() : Node("web_server_node")
  {
    declare_parameter<int>("port", 8080);
    port_ = get_parameter("port").as_int();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    data_a_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/data_a", qos, [this](const std_msgs::msg::Float32::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        state_.distance = msg->data;
        state_.received_distance = true;
        state_.last_data_a = now();
      });

    data_b_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/data_b", qos, [this](const std_msgs::msg::Float32::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        state_.normal_speed = msg->data;
        state_.received_speed_input = true;
        state_.last_data_b = now();
      });

    adjusted_speed_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/adjusted_speed", qos, [this](const std_msgs::msg::Float32::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        state_.adjusted_speed = msg->data;
        state_.received_adjusted_speed = true;
        state_.last_adjusted_speed = now();
      });

    system_sub_ = create_subscription<std_msgs::msg::String>(
      "/system_status", qos, [this](const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        state_.system_status = msg->data;
        state_.last_system = now();
      });

    health_sub_ = create_subscription<std_msgs::msg::String>(
      "/health_status", qos, [this](const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        state_.primary_health = msg->data;
        state_.last_primary_health = now();
      });

    health_reason_sub_ = create_subscription<std_msgs::msg::String>(
      "/health_reason", qos, [this](const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        state_.primary_reason = msg->data;
      });

    backup_health_sub_ = create_subscription<std_msgs::msg::String>(
      "/health_status_backup", qos, [this](const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        state_.backup_health = msg->data;
        state_.last_backup_health = now();
      });

    backup_reason_sub_ = create_subscription<std_msgs::msg::String>(
      "/health_reason_backup", qos, [this](const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        state_.backup_reason = msg->data;
      });

    camera_sub_ = create_subscription<std_msgs::msg::String>(
      "/camera/object_status", qos, [this](const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        state_.camera_status = msg->data;
        state_.last_camera = now();
      });

    network_status_sub_ = create_subscription<std_msgs::msg::String>(
      "/network_status", qos, [this](const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        state_.network_status = msg->data;
        state_.last_network = now();
      });

    network_reason_sub_ = create_subscription<std_msgs::msg::String>(
      "/network_reason", qos, [this](const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        state_.network_reason = msg->data;
      });

    battery_percentage_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/battery_percentage", qos, [this](const std_msgs::msg::Float32::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        state_.battery_percentage = msg->data;
        state_.received_battery = true;
        state_.last_battery = now();
      });

    battery_status_sub_ = create_subscription<std_msgs::msg::String>(
      "/battery_status", qos, [this](const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        state_.battery_status = msg->data;
        state_.last_battery = now();
      });

    register_pub_ = create_publisher<std_msgs::msg::String>("/register_node", qos);
    deregister_pub_ = create_publisher<std_msgs::msg::String>("/deregister_node", qos);

    register_sub_ = create_subscription<std_msgs::msg::String>(
      "/register_node", qos, [this](const std_msgs::msg::String::SharedPtr msg) {
        handle_register_node(msg->data);
      });

    deregister_sub_ = create_subscription<std_msgs::msg::String>(
      "/deregister_node", qos, [this](const std_msgs::msg::String::SharedPtr msg) {
        handle_deregister_node(msg->data);
      });

    add_dashboard_node("Node 1 / LiDAR", "/node1/heartbeat", 0.75);
    add_dashboard_node("Node 2 / Speed", "/node2/heartbeat", 2.0);
    add_dashboard_node("Node 3 / Safety", "/node3/heartbeat", 1.0);
    add_dashboard_node("Camera Node", "/node6/heartbeat", 1.5);
    add_dashboard_node("Network Monitor", "/node8/heartbeat", 10.0);
    add_dashboard_node("Battery Node", "/node13/heartbeat", 3.0);

    running_ = true;
    server_thread_ = std::thread(&WebServerNode::server_loop, this);

    RCLCPP_INFO(
      get_logger(),
      "Web dashboard started at http://localhost:%d",
      port_);
  }

  ~WebServerNode() override
  {
    running_ = false;

    if (server_fd_ >= 0) {
      shutdown(server_fd_, SHUT_RDWR);
      close(server_fd_);
    }

    if (server_thread_.joinable()) {
      server_thread_.join();
    }
  }

private:
  struct DashboardState
  {
    float distance = 0.0f;
    float normal_speed = 0.0f;
    float adjusted_speed = 0.0f;
    bool received_distance = false;
    bool received_speed_input = false;
    bool received_adjusted_speed = false;

    std::string system_status = "WAITING_FOR_SYSTEM_STATUS";
    std::string primary_health = "WAITING_FOR_PRIMARY_MONITOR";
    std::string primary_reason = "NONE";
    std::string backup_health = "WAITING_FOR_BACKUP_MONITOR";
    std::string backup_reason = "NONE";
    std::string camera_status = "WAITING_FOR_CAMERA";
    std::string network_status = "WAITING_FOR_NETWORK";
    std::string network_reason = "NONE";
    std::string battery_status = "WAITING_FOR_BATTERY";
    float battery_percentage = 0.0f;
    bool received_battery = false;

    rclcpp::Time last_data_a;
    rclcpp::Time last_data_b;
    rclcpp::Time last_adjusted_speed;
    rclcpp::Time last_system;
    rclcpp::Time last_primary_health;
    rclcpp::Time last_backup_health;
    rclcpp::Time last_camera;
    rclcpp::Time last_network;
    rclcpp::Time last_battery;
  };

  struct DashboardNode
  {
    std::string name;
    std::string topic;
    double timeout_seconds = 2.0;
    bool received = false;
    bool deregistered = false;
    rclcpp::Time last_heartbeat;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;
  };

  struct TestNodeControl
  {
    std::string id;
    std::string label;
    std::string registration;
    std::string topic;
  };

  double age_seconds(const rclcpp::Time & stamp) const
  {
    if (stamp.nanoseconds() == 0) {
      return -1.0;
    }

    return (now() - stamp).seconds();
  }

  std::string age_json(const rclcpp::Time & stamp) const
  {
    const double age = age_seconds(stamp);

    if (age < 0.0) {
      return "null";
    }

    std::ostringstream out;
    out << std::fixed << std::setprecision(2) << age;
    return out.str();
  }

  bool fresh(const rclcpp::Time & stamp, const double timeout_seconds) const
  {
    const double age = age_seconds(stamp);
    return age >= 0.0 && age <= timeout_seconds;
  }

  bool monitor_fresh(const rclcpp::Time & stamp) const
  {
    return fresh(stamp, monitor_timeout_seconds_);
  }

  void add_dashboard_node(
    const std::string & name,
    const std::string & topic,
    const double timeout_seconds)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    auto existing = std::find_if(
      dashboard_nodes_.begin(),
      dashboard_nodes_.end(),
      [&](const std::shared_ptr<DashboardNode> & node) {
        return node->topic == topic;
      });

    if (existing != dashboard_nodes_.end())
    {
      (*existing)->name = name;
      (*existing)->timeout_seconds = timeout_seconds;
      (*existing)->deregistered = false;
      return;
    }

    auto node = std::make_shared<DashboardNode>();
    node->name = name;
    node->topic = topic;
    node->timeout_seconds = timeout_seconds;

    node->subscription = create_subscription<std_msgs::msg::String>(
      topic,
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      [this, node](const std_msgs::msg::String::SharedPtr) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (node->deregistered) {
          return;
        }

        node->received = true;
        node->last_heartbeat = now();
      });

    dashboard_nodes_.push_back(node);
  }

  void handle_register_node(const std::string & registration)
  {
    std::stringstream ss(registration);

    std::string name;
    std::string topic;
    std::string deadline_ms;
    std::string liveliness_ms;
    std::string failure_reason;

    std::getline(ss, name, ',');
    std::getline(ss, topic, ',');
    std::getline(ss, deadline_ms, ',');
    std::getline(ss, liveliness_ms, ',');
    std::getline(ss, failure_reason, ',');

    if (name.empty() || topic.empty()) {
      RCLCPP_WARN(get_logger(), "Ignoring invalid dashboard registration: %s", registration.c_str());
      return;
    }

    if (is_test_node_topic(topic) && !is_test_node_enabled(topic)) {
      return;
    }

    double timeout_seconds = 2.0;

    try
    {
      if (!liveliness_ms.empty()) {
        timeout_seconds = std::stod(liveliness_ms) / 1000.0;
      } else if (!deadline_ms.empty()) {
        timeout_seconds = std::stod(deadline_ms) / 1000.0;
      }
    }
    catch (const std::exception &)
    {
      RCLCPP_WARN(
        get_logger(),
        "Using default dashboard timeout for invalid registration: %s",
        registration.c_str());
    }

    add_dashboard_node(name, topic, timeout_seconds);
  }

  void handle_deregister_node(const std::string & topic)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    auto existing = std::find_if(
      dashboard_nodes_.begin(),
      dashboard_nodes_.end(),
      [&](const std::shared_ptr<DashboardNode> & node) {
        return node->topic == topic;
      });

    if (existing != dashboard_nodes_.end()) {
      if (is_test_node_topic(topic)) {
        dashboard_nodes_.erase(existing);
      } else {
        (*existing)->deregistered = true;
      }
    }
  }

  std::string node_json(const DashboardNode & node) const
  {
    const bool ok = !node.deregistered && node.received &&
      fresh(node.last_heartbeat, node.timeout_seconds);
    std::ostringstream out;
    out << "{\"name\":\"" << escape_json(node.name)
        << "\",\"topic\":\"" << escape_json(node.topic)
        << "\",\"status\":\"" << (node.deregistered ? "DEREGISTERED" : (ok ? "OK" : "STALE"))
        << "\",\"age_seconds\":" << age_json(node.last_heartbeat) << "}";
    return out.str();
  }

  bool is_test_node_topic(const std::string & topic) const
  {
    return std::any_of(
      test_node_controls_.begin(),
      test_node_controls_.end(),
      [&](const TestNodeControl & node) {
        return node.topic == topic;
      });
  }

  bool is_test_node_enabled(const std::string & topic) const
  {
    std::lock_guard<std::mutex> lock(test_node_mutex_);

    return std::find(
      enabled_test_node_topics_.begin(),
      enabled_test_node_topics_.end(),
      topic) != enabled_test_node_topics_.end();
  }

  void set_test_node_enabled(const std::string & topic, const bool enabled)
  {
    std::lock_guard<std::mutex> lock(test_node_mutex_);

    auto existing = std::find(
      enabled_test_node_topics_.begin(),
      enabled_test_node_topics_.end(),
      topic);

    if (enabled && existing == enabled_test_node_topics_.end()) {
      enabled_test_node_topics_.push_back(topic);
    } else if (!enabled && existing != enabled_test_node_topics_.end()) {
      enabled_test_node_topics_.erase(existing);
    }
  }

  const TestNodeControl * find_test_node_control(const std::string & id) const
  {
    auto existing = std::find_if(
      test_node_controls_.begin(),
      test_node_controls_.end(),
      [&](const TestNodeControl & node) {
        return node.id == id;
      });

    if (existing == test_node_controls_.end()) {
      return nullptr;
    }

    return &(*existing);
  }

  void publish_register_test_node(const TestNodeControl & node)
  {
    std_msgs::msg::String msg;
    msg.data = node.registration;
    set_test_node_enabled(node.topic, true);
    register_pub_->publish(msg);
    handle_register_node(msg.data);
  }

  void publish_deregister_test_node(const TestNodeControl & node)
  {
    std_msgs::msg::String msg;
    msg.data = node.topic;
    set_test_node_enabled(node.topic, false);
    deregister_pub_->publish(msg);
    handle_deregister_node(msg.data);
  }

  bool handle_test_node_route(const std::string & path, const int client_fd)
  {
    const std::string prefix = "/api/test-node/";

    if (path.rfind(prefix, 0) != 0) {
      return false;
    }

    const std::string remainder = path.substr(prefix.size());
    const size_t slash = remainder.find('/');

    if (slash == std::string::npos) {
      send_response(client_fd, "404 Not Found", "application/json", "{\"error\":\"not_found\"}");
      return true;
    }

    const std::string id = remainder.substr(0, slash);
    const std::string action = remainder.substr(slash + 1);
    const TestNodeControl * node = find_test_node_control(id);

    if (node == nullptr) {
      send_response(client_fd, "404 Not Found", "application/json", "{\"error\":\"unknown_test_node\"}");
      return true;
    }

    if (action == "register")
    {
      publish_register_test_node(*node);
      std::ostringstream body;
      body << "{\"ok\":true,\"message\":\"" << escape_json(node->label) << " monitor enabled\"}";
      send_response(client_fd, "200 OK", "application/json", body.str());
      return true;
    }

    if (action == "deregister")
    {
      publish_deregister_test_node(*node);
      std::ostringstream body;
      body << "{\"ok\":true,\"message\":\"" << escape_json(node->label) << " monitor disabled\"}";
      send_response(client_fd, "200 OK", "application/json", body.str());
      return true;
    }

    send_response(client_fd, "404 Not Found", "application/json", "{\"error\":\"unknown_action\"}");
    return true;
  }

  std::string final_action(
    const DashboardState & state,
    const std::string & primary_health,
    const std::string & primary_reason,
    const std::string & backup_health,
    const std::string & backup_reason) const
  {
    if (primary_health != "HEALTHY" || backup_health != "HEALTHY")
    {
      if (primary_health == "UNHEALTHY" && backup_health == "UNHEALTHY" &&
          network_only_failure(primary_reason) && network_only_failure(backup_reason)) {
        return "RETURN_TO_BASE";
      }

      return "STOP";
    }

    if (state.system_status == "UNSAFE_STOP") {
      return "STOP";
    }

    if (state.system_status == "EMERGENCY_BATTERY_SAFE_LANDING") {
      return "SAFE_LANDING";
    }

    if (state.system_status.find("RETURN_TO_BASE") != std::string::npos) {
      return "RETURN_TO_BASE";
    }

    if (state.system_status.find("UNSAFE_REDUCE") != std::string::npos) {
      return "REDUCE_SPEED";
    }

    if (state.system_status.find("LOW_BATTERY_REDUCE") != std::string::npos) {
      return "REDUCE_SPEED";
    }

    if (state.system_status == "SAFE_NORMAL_SPEED") {
      return "NORMAL";
    }

    return "WAITING";
  }

  std::string state_json()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto state = state_;
    const bool primary_monitor_online = monitor_fresh(state.last_primary_health);
    const bool backup_monitor_online = monitor_fresh(state.last_backup_health);
    const std::string primary_health =
      primary_monitor_online ? state.primary_health : "MONITOR_OFFLINE";
    const std::string primary_reason =
      primary_monitor_online ? state.primary_reason : "NO_PRIMARY_HEALTH_UPDATE";
    const std::string backup_health =
      backup_monitor_online ? state.backup_health : "MONITOR_OFFLINE";
    const std::string backup_reason =
      backup_monitor_online ? state.backup_reason : "NO_BACKUP_HEALTH_UPDATE";
    const std::string action =
      final_action(state, primary_health, primary_reason, backup_health, backup_reason);

    std::ostringstream out;
    out << std::fixed << std::setprecision(2);
    out << "{";
    out << "\"system_status\":\"" << escape_json(state.system_status) << "\",";
    out << "\"system_class\":\"" << status_class(state.system_status) << "\",";
    out << "\"primary_health\":\"" << escape_json(primary_health) << "\",";
    out << "\"primary_reason\":\"" << escape_json(primary_reason) << "\",";
    out << "\"primary_class\":\"" << status_class(primary_health) << "\",";
    out << "\"backup_health\":\"" << escape_json(backup_health) << "\",";
    out << "\"backup_reason\":\"" << escape_json(backup_reason) << "\",";
    out << "\"backup_class\":\"" << status_class(backup_health) << "\",";
    out << "\"camera_status\":\"" << escape_json(state.camera_status) << "\",";
    out << "\"camera_age_seconds\":" << age_json(state.last_camera) << ",";
    out << "\"network_status\":\"" << escape_json(state.network_status) << "\",";
    out << "\"network_reason\":\"" << escape_json(state.network_reason) << "\",";
    out << "\"network_distance\":\"" <<
      escape_json(network_reason_value(state.network_reason, "DISTANCE_FROM_BASE")) << "\",";
    out << "\"network_wifi\":\"" << escape_json(network_state_value(state.network_reason, "WIFI")) << "\",";
    out << "\"network_lte\":\"" << escape_json(network_state_value(state.network_reason, "LTE")) << "\",";
    out << "\"network_starlink\":\"" << escape_json(network_state_value(state.network_reason, "STARLINK")) << "\",";
    out << "\"network_active\":\"" << escape_json(network_state_value(state.network_reason, "ACTIVE_CONNECTION")) << "\",";
    out << "\"network_class\":\"" << status_class(state.network_status) << "\",";
    out << "\"network_age_seconds\":" << age_json(state.last_network) << ",";
    out << "\"battery_status\":\"" << escape_json(state.battery_status) << "\",";
    out << "\"battery_class\":\"" << status_class(state.battery_status) << "\",";
    out << "\"battery_percentage\":" <<
      (state.received_battery ? std::to_string(state.battery_percentage) : "null") << ",";
    out << "\"battery_age_seconds\":" << age_json(state.last_battery) << ",";
    out << "\"distance\":" << (state.received_distance ? std::to_string(state.distance) : "null") << ",";
    out << "\"normal_speed\":" << (state.received_speed_input ? std::to_string(state.normal_speed) : "null") << ",";
    out << "\"adjusted_speed\":" << (state.received_adjusted_speed ? std::to_string(state.adjusted_speed) : "null") << ",";
    out << "\"final_action\":\"" << action << "\",";
    out << "\"action_class\":\"" << status_class(action) << "\",";
    out << "\"ages\":{";
    out << "\"data_a\":" << age_json(state.last_data_a) << ",";
    out << "\"data_b\":" << age_json(state.last_data_b) << ",";
    out << "\"adjusted_speed\":" << age_json(state.last_adjusted_speed) << ",";
    out << "\"system_status\":" << age_json(state.last_system) << ",";
    out << "\"primary_health\":" << age_json(state.last_primary_health) << ",";
    out << "\"backup_health\":" << age_json(state.last_backup_health);
    out << "},";
    out << "\"nodes\":[";
    for (size_t i = 0; i < dashboard_nodes_.size(); ++i)
    {
      if (i > 0) {
        out << ",";
      }

      out << node_json(*dashboard_nodes_[i]);
    }
    out << "]";
    out << "}";
    return out.str();
  }

  std::string dashboard_html() const
  {
    return R"HTML(<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>QoS Health Monitor</title>
  <style>
    :root {
      color-scheme: light;
      font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: #f4f7fb;
      color: #172033;
    }
    * { box-sizing: border-box; }
    body { margin: 0; min-height: 100vh; }
    header {
      background: #14213d;
      color: #ffffff;
      padding: 18px 28px;
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 16px;
    }
    h1 { font-size: 22px; margin: 0; font-weight: 700; }
    .connection { font-size: 13px; color: #c8d3e6; }
    main { padding: 24px; max-width: 1180px; margin: 0 auto; }
    .grid { display: grid; grid-template-columns: repeat(4, minmax(0, 1fr)); gap: 16px; }
    .panel {
      background: #ffffff;
      border: 1px solid #dce4ef;
      border-radius: 8px;
      padding: 16px;
      min-width: 0;
      box-shadow: 0 1px 2px rgba(20, 33, 61, 0.06);
    }
    .panel h2 {
      margin: 0 0 12px;
      font-size: 13px;
      color: #5c6f8a;
      font-weight: 700;
      text-transform: uppercase;
      letter-spacing: 0;
    }
    .value { font-size: 24px; font-weight: 800; line-height: 1.15; overflow-wrap: anywhere; }
    .subvalue { margin-top: 8px; color: #60708a; font-size: 14px; overflow-wrap: anywhere; }
    .ok { color: #13795b; }
    .warn { color: #a15c00; }
    .danger { color: #bd2636; }
    .wide { grid-column: span 2; }
    table { width: 100%; border-collapse: collapse; font-size: 14px; }
    th, td { padding: 10px 8px; border-bottom: 1px solid #e7edf5; text-align: left; }
    th { color: #5c6f8a; font-size: 12px; text-transform: uppercase; letter-spacing: 0; }
    .metric-row {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 12px;
    }
    .metric {
      background: #f7f9fc;
      border: 1px solid #e4ebf4;
      border-radius: 6px;
      padding: 12px;
    }
    .network-row { margin-top: 12px; }
    .metric strong { display: block; font-size: 12px; color: #5c6f8a; margin-bottom: 5px; }
    .metric span { font-size: 21px; font-weight: 800; }
    .speed-bar {
      width: 100%;
      height: 12px;
      margin-top: 10px;
      overflow: hidden;
      border-radius: 999px;
      background: #dfe7f2;
      border: 1px solid #c9d5e5;
    }
    .speed-fill {
      width: 0%;
      height: 100%;
      border-radius: inherit;
      background: linear-gradient(90deg, #1b998b, #f2c14e, #e84855);
      transition: width 220ms ease;
    }
    .controls { display: flex; flex-wrap: wrap; gap: 10px; }
    .test-node-controls { display: grid; gap: 10px; }
    .test-node-control {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 12px;
      padding: 10px 0;
      border-bottom: 1px solid #e7edf5;
    }
    .test-node-control:last-child { border-bottom: 0; }
    .test-node-control span { font-weight: 700; }
    button {
      border: 1px solid #b9c6d8;
      border-radius: 6px;
      background: #ffffff;
      color: #172033;
      cursor: pointer;
      font: inherit;
      font-size: 14px;
      font-weight: 700;
      padding: 9px 12px;
    }
    button:hover { background: #edf3fa; }
    @media (max-width: 900px) {
      header { align-items: flex-start; flex-direction: column; padding: 16px; }
      main { padding: 16px; }
      .grid { grid-template-columns: 1fr; }
      .wide { grid-column: span 1; }
      .metric-row { grid-template-columns: 1fr; }
      .test-node-control { align-items: flex-start; flex-direction: column; }
    }
  </style>
</head>
<body>
  <header>
    <h1>QoS Health Monitor Dashboard</h1>
    <div id="connection" class="connection">Connecting to ROS2 web server...</div>
  </header>
  <main>
    <section class="grid">
      <article class="panel">
        <h2>System Status</h2>
        <div id="systemStatus" class="value">--</div>
        <div id="systemAge" class="subvalue">Age: --</div>
      </article>
      <article class="panel">
        <h2>Primary Monitor</h2>
        <div id="primaryHealth" class="value">--</div>
        <div id="primaryReason" class="subvalue">Reason: --</div>
        <div id="primaryAge" class="subvalue">Age: --</div>
      </article>
      <article class="panel">
        <h2>Backup Monitor</h2>
        <div id="backupHealth" class="value">--</div>
        <div id="backupReason" class="subvalue">Reason: --</div>
        <div id="backupAge" class="subvalue">Age: --</div>
      </article>
      <article class="panel wide">
        <h2>Live Inputs</h2>
        <div class="metric-row">
          <div class="metric"><strong>LiDAR Distance</strong><span id="distance">--</span></div>
          <div class="metric">
            <strong>Normal Speed</strong>
            <span id="normalSpeed">--</span>
            <div class="speed-bar" aria-label="Normal speed bar">
              <div id="normalSpeedFill" class="speed-fill"></div>
            </div>
          </div>
          <div class="metric">
            <strong>Adjusted Speed</strong>
            <span id="adjustedSpeed">--</span>
            <div class="speed-bar" aria-label="Adjusted speed bar">
              <div id="adjustedSpeedFill" class="speed-fill"></div>
            </div>
          </div>
          <div class="metric"><strong>Camera</strong><span id="cameraStatus">--</span></div>
        </div>
      </article>
      <article class="panel wide">
        <h2>Battery</h2>
        <div id="batteryStatus" class="value">--</div>
        <div id="batteryPercent" class="subvalue">Charge: --</div>
        <div id="batteryAge" class="subvalue">Age: --</div>
        <div class="speed-bar" aria-label="Battery percentage bar">
          <div id="batteryFill" class="speed-fill"></div>
        </div>
      </article>
      <article class="panel wide">
        <h2>Network</h2>
        <div id="networkStatus" class="value">--</div>
        <div id="networkDistance" class="subvalue">Distance from base: --</div>
        <div id="networkActive" class="subvalue">Active connection: --</div>
        <div id="networkAge" class="subvalue">Age: --</div>
        <div class="metric-row network-row">
          <div class="metric"><strong>WiFi</strong><span id="networkWifi">--</span></div>
          <div class="metric"><strong>LTE</strong><span id="networkLte">--</span></div>
          <div class="metric"><strong>Starlink</strong><span id="networkStarlink">--</span></div>
        </div>
      </article>
      <article class="panel wide">
        <h2>Test Node Monitors</h2>
        <div class="test-node-controls">
          <div class="test-node-control">
            <span>Node 7</span>
            <div class="controls">
              <button type="button" data-node-id="node7" data-action="register">Enable</button>
              <button type="button" data-node-id="node7" data-action="deregister">Disable</button>
            </div>
          </div>
          <div class="test-node-control">
            <span>Node 9</span>
            <div class="controls">
              <button type="button" data-node-id="node9" data-action="register">Enable</button>
              <button type="button" data-node-id="node9" data-action="deregister">Disable</button>
            </div>
          </div>
          <div class="test-node-control">
            <span>Node 10</span>
            <div class="controls">
              <button type="button" data-node-id="node10" data-action="register">Enable</button>
              <button type="button" data-node-id="node10" data-action="deregister">Disable</button>
            </div>
          </div>
          <div class="test-node-control">
            <span>Node 11</span>
            <div class="controls">
              <button type="button" data-node-id="node11" data-action="register">Enable</button>
              <button type="button" data-node-id="node11" data-action="deregister">Disable</button>
            </div>
          </div>
          <div class="test-node-control">
            <span>Node 12</span>
            <div class="controls">
              <button type="button" data-node-id="node12" data-action="register">Enable</button>
              <button type="button" data-node-id="node12" data-action="deregister">Disable</button>
            </div>
          </div>
        </div>
        <div id="nodeControlStatus" class="subvalue">--</div>
      </article>
      <article class="panel wide">
        <h2>Heartbeat State</h2>
        <table>
          <thead><tr><th>Node</th><th>Topic</th><th>Status</th><th>Age</th></tr></thead>
          <tbody id="nodeRows"></tbody>
        </table>
      </article>
    </section>
  </main>
  <script>
    const fields = {
      connection: document.getElementById('connection'),
      systemStatus: document.getElementById('systemStatus'),
      systemAge: document.getElementById('systemAge'),
      primaryHealth: document.getElementById('primaryHealth'),
      primaryReason: document.getElementById('primaryReason'),
      primaryAge: document.getElementById('primaryAge'),
      backupHealth: document.getElementById('backupHealth'),
      backupReason: document.getElementById('backupReason'),
      backupAge: document.getElementById('backupAge'),
      distance: document.getElementById('distance'),
      normalSpeed: document.getElementById('normalSpeed'),
      normalSpeedFill: document.getElementById('normalSpeedFill'),
      adjustedSpeed: document.getElementById('adjustedSpeed'),
      adjustedSpeedFill: document.getElementById('adjustedSpeedFill'),
      cameraStatus: document.getElementById('cameraStatus'),
      batteryStatus: document.getElementById('batteryStatus'),
      batteryPercent: document.getElementById('batteryPercent'),
      batteryAge: document.getElementById('batteryAge'),
      batteryFill: document.getElementById('batteryFill'),
      networkStatus: document.getElementById('networkStatus'),
      networkDistance: document.getElementById('networkDistance'),
      networkActive: document.getElementById('networkActive'),
      networkAge: document.getElementById('networkAge'),
      networkWifi: document.getElementById('networkWifi'),
      networkLte: document.getElementById('networkLte'),
      networkStarlink: document.getElementById('networkStarlink'),
      nodeControlStatus: document.getElementById('nodeControlStatus'),
      nodeRows: document.getElementById('nodeRows')
    };

    function fmt(value, suffix = '') {
      return value === null || value === undefined ? '--' : `${Number(value).toFixed(2)}${suffix}`;
    }

    function setClass(el, cls) {
      el.className = `value ${cls}`;
    }

    function setSpeedBar(el, value) {
      const maxSpeed = 0.7;
      const safeValue = value === null || value === undefined ? 0 : Number(value);
      const percent = Math.max(0, Math.min(100, (safeValue / maxSpeed) * 100));
      el.style.width = `${percent}%`;
    }

    function render(data) {
      fields.connection.textContent = `Live at ${new Date().toLocaleTimeString()}`;

      fields.systemStatus.textContent = data.system_status;
      setClass(fields.systemStatus, data.system_class);
      fields.systemAge.textContent = `Age: ${fmt(data.ages.system_status, ' s')}`;

      fields.primaryHealth.textContent = data.primary_health;
      setClass(fields.primaryHealth, data.primary_class);
      fields.primaryReason.textContent = `Reason: ${data.primary_reason}`;
      fields.primaryAge.textContent = `Age: ${fmt(data.ages.primary_health, ' s')}`;

      fields.backupHealth.textContent = data.backup_health;
      setClass(fields.backupHealth, data.backup_class);
      fields.backupReason.textContent = `Reason: ${data.backup_reason}`;
      fields.backupAge.textContent = `Age: ${fmt(data.ages.backup_health, ' s')}`;

      fields.distance.textContent = fmt(data.distance, ' m');
      fields.normalSpeed.textContent = fmt(data.normal_speed, ' m/s');
      setSpeedBar(fields.normalSpeedFill, data.normal_speed);
      fields.adjustedSpeed.textContent = fmt(data.adjusted_speed, ' m/s');
      setSpeedBar(fields.adjustedSpeedFill, data.adjusted_speed);
      fields.cameraStatus.textContent = data.camera_status;
      fields.batteryStatus.textContent = data.battery_status;
      setClass(fields.batteryStatus, data.battery_class);
      fields.batteryPercent.textContent = `Charge: ${fmt(data.battery_percentage, '%')}`;
      fields.batteryAge.textContent = `Age: ${fmt(data.battery_age_seconds, ' s')}`;
      fields.batteryFill.style.width = `${Math.max(0, Math.min(100, Number(data.battery_percentage || 0)))}%`;
      fields.networkStatus.textContent = data.network_status;
      setClass(fields.networkStatus, data.network_class);
      fields.networkDistance.textContent = `Distance from base: ${data.network_distance}`;
      fields.networkActive.textContent = `Active connection: ${data.network_active}`;
      fields.networkAge.textContent = `Age: ${fmt(data.network_age_seconds, ' s')}`;
      fields.networkWifi.textContent = data.network_wifi;
      fields.networkLte.textContent = data.network_lte;
      fields.networkStarlink.textContent = data.network_starlink;

      fields.nodeRows.innerHTML = data.nodes.map((node) => {
        const cls = node.status === 'OK' ? 'ok' : (node.status === 'DEREGISTERED' ? 'warn' : 'danger');
        return `<tr><td>${node.name}</td><td>${node.topic}</td><td class="${cls}">${node.status}</td><td>${fmt(node.age_seconds, ' s')}</td></tr>`;
      }).join('');
    }

    function connect() {
      const stream = new EventSource('/events');
      stream.onmessage = (event) => render(JSON.parse(event.data));
      stream.onopen = () => { fields.connection.textContent = 'Connected'; };
      stream.onerror = () => {
        fields.connection.textContent = 'Connection lost, retrying...';
      };
    }

    function sendNodeControl(path, label) {
      fields.nodeControlStatus.textContent = `${label}...`;
      fetch(path)
        .then((response) => response.json())
        .then((data) => {
          fields.nodeControlStatus.textContent = data.message;
        })
        .catch(() => {
          fields.nodeControlStatus.textContent = 'Command failed';
        });
    }

    document.querySelectorAll('[data-node-id][data-action]').forEach((button) => {
      button.addEventListener('click', () => {
        const nodeId = button.dataset.nodeId;
        const action = button.dataset.action;
        const label = `${action === 'register' ? 'Enabling' : 'Disabling'} ${button.closest('.test-node-control').querySelector('span').textContent} monitor`;
        sendNodeControl(`/api/test-node/${nodeId}/${action}`, label);
      });
    });

    fetch('/api/status').then((response) => response.json()).then(render).catch(() => {});
    connect();
  </script>
</body>
</html>)HTML";
  }

  bool send_all(const int client_fd, const std::string & data) const
  {
    size_t sent = 0;

    while (sent < data.size())
    {
      const ssize_t result = send(client_fd, data.data() + sent, data.size() - sent, MSG_NOSIGNAL);

      if (result <= 0) {
        return false;
      }

      sent += static_cast<size_t>(result);
    }

    return true;
  }

  void send_response(
    const int client_fd,
    const std::string & status,
    const std::string & content_type,
    const std::string & body) const
  {
    std::ostringstream response;
    response << "HTTP/1.1 " << status << "\r\n";
    response << "Content-Type: " << content_type << "\r\n";
    response << "Content-Length: " << body.size() << "\r\n";
    response << "Connection: close\r\n";
    response << "Cache-Control: no-store\r\n\r\n";
    response << body;
    send_all(client_fd, response.str());
  }

  void handle_events(const int client_fd)
  {
    const std::string headers =
      "HTTP/1.1 200 OK\r\n"
      "Content-Type: text/event-stream\r\n"
      "Cache-Control: no-cache\r\n"
      "Connection: keep-alive\r\n"
      "Access-Control-Allow-Origin: *\r\n\r\n";

    if (!send_all(client_fd, headers)) {
      return;
    }

    while (running_)
    {
      const std::string event = "data: " + state_json() + "\n\n";

      if (!send_all(client_fd, event)) {
        break;
      }

      std::this_thread::sleep_for(500ms);
    }
  }

  void handle_client(const int client_fd)
  {
    char buffer[2048];
    const ssize_t received = recv(client_fd, buffer, sizeof(buffer) - 1, 0);

    if (received <= 0) {
      close(client_fd);
      return;
    }

    buffer[received] = '\0';
    std::istringstream request(buffer);
    std::string method;
    std::string path;
    request >> method >> path;

    if (method != "GET")
    {
      send_response(client_fd, "405 Method Not Allowed", "application/json", "{\"error\":\"method_not_allowed\"}");
    }
    else if (path == "/" || path == "/index.html")
    {
      send_response(client_fd, "200 OK", "text/html; charset=utf-8", dashboard_html());
    }
    else if (path == "/api/status")
    {
      send_response(client_fd, "200 OK", "application/json", state_json());
    }
    else if (handle_test_node_route(path, client_fd))
    {
    }
    else if (path == "/api/node7/register")
    {
      const TestNodeControl * node = find_test_node_control("node7");
      if (node != nullptr) {
        publish_register_test_node(*node);
      }
      send_response(
        client_fd,
        "200 OK",
        "application/json",
        "{\"ok\":true,\"message\":\"Node 7 monitor enabled\"}");
    }
    else if (path == "/api/node7/deregister")
    {
      const TestNodeControl * node = find_test_node_control("node7");
      if (node != nullptr) {
        publish_deregister_test_node(*node);
      }
      send_response(
        client_fd,
        "200 OK",
        "application/json",
        "{\"ok\":true,\"message\":\"Node 7 monitor disabled\"}");
    }
    else if (path == "/events")
    {
      handle_events(client_fd);
    }
    else
    {
      send_response(client_fd, "404 Not Found", "application/json", "{\"error\":\"not_found\"}");
    }

    close(client_fd);
  }

  void server_loop()
  {
    server_fd_ = socket(AF_INET, SOCK_STREAM, 0);

    if (server_fd_ < 0)
    {
      RCLCPP_ERROR(get_logger(), "Failed to create HTTP socket: %s", std::strerror(errno));
      return;
    }

    int opt = 1;
    setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(static_cast<uint16_t>(port_));

    if (bind(server_fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0)
    {
      RCLCPP_ERROR(get_logger(), "Failed to bind HTTP server on port %d: %s", port_, std::strerror(errno));
      close(server_fd_);
      server_fd_ = -1;
      return;
    }

    if (listen(server_fd_, 16) < 0)
    {
      RCLCPP_ERROR(get_logger(), "Failed to listen on port %d: %s", port_, std::strerror(errno));
      close(server_fd_);
      server_fd_ = -1;
      return;
    }

    while (running_)
    {
      sockaddr_in client_address{};
      socklen_t client_length = sizeof(client_address);
      const int client_fd = accept(
        server_fd_,
        reinterpret_cast<sockaddr *>(&client_address),
        &client_length);

      if (client_fd < 0)
      {
        if (running_) {
          RCLCPP_WARN(get_logger(), "HTTP accept failed: %s", std::strerror(errno));
        }
        continue;
      }

      std::thread(&WebServerNode::handle_client, this, client_fd).detach();
    }
  }

  int port_ = 8080;
  int server_fd_ = -1;
  const double monitor_timeout_seconds_ = 3.0;
  std::atomic<bool> running_{false};
  std::thread server_thread_;

  std::mutex mutex_;
  mutable std::mutex test_node_mutex_;
  DashboardState state_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr data_a_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr data_b_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr adjusted_speed_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr system_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr health_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr health_reason_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr backup_health_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr backup_reason_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr camera_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr network_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr network_reason_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_percentage_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr battery_status_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr register_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr deregister_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr register_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr deregister_sub_;
  std::vector<std::shared_ptr<DashboardNode>> dashboard_nodes_;
  std::vector<std::string> enabled_test_node_topics_;
  const std::vector<TestNodeControl> test_node_controls_ = {
    {"node7", "Node 7", "Node 7,/node7/heartbeat,500,1000,NODE7_FAILURE", "/node7/heartbeat"},
    {"node9", "Node 9", "Node 9,/node9/heartbeat,1000,2000,NODE9_QOS_TEST_FAILURE", "/node9/heartbeat"},
    {"node10", "Node 10", "Node 10,/node10/heartbeat,500,1000,NODE10_BAD_QOS_FAILURE", "/node10/heartbeat"},
    {"node11", "Node 11", "Node 11,/node11/heartbeat,1000,,NODE11_DEADLINE_FAILURE", "/node11/heartbeat"},
    {"node12", "Node 12", "Node 12,/node12/heartbeat,,2000,NODE12_LIVELINESS_FAILURE", "/node12/heartbeat"},
  };
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WebServerNode>());
  rclcpp::shutdown();
  return 0;
}
