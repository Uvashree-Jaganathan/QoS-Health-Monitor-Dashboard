#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class NetworkMonitorNode : public rclcpp::Node
{
public:
  NetworkMonitorNode() : Node("network_monitor_node")
  {
    network_status_pub_ =
      this->create_publisher<std_msgs::msg::String>("/network_status", 10);

    network_reason_pub_ =
      this->create_publisher<std_msgs::msg::String>("/network_reason", 10);

    auto heartbeat_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    heartbeat_qos.reliable();
    heartbeat_qos.deadline(5000ms);
    heartbeat_qos.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC);
    heartbeat_qos.liveliness_lease_duration(10000ms);

    heartbeat_pub_ =
      this->create_publisher<std_msgs::msg::String>("/node8/heartbeat", heartbeat_qos);

    timer_ = this->create_wall_timer(
      1000ms,
      std::bind(&NetworkMonitorNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Network Monitor node started");
  }

private:
  struct NetworkState
  {
    std::string wifi;
    std::string lte;
    std::string starlink;
    std::string active;
    std::string status;
  };

  void timer_callback()
  {
    NetworkState state = get_fake_network_state();
    apply_network_loss_grace_period(state);

    std_msgs::msg::String status_msg;
    std_msgs::msg::String reason_msg;
    std_msgs::msg::String heartbeat_msg;

    status_msg.data = state.status;

    reason_msg.data =
      "WIFI=" + state.wifi +
      ",LTE=" + state.lte +
      ",STARLINK=" + state.starlink +
      ",ACTIVE_CONNECTION=" + state.active;

    heartbeat_msg.data = "network_monitor_node alive";

    network_status_pub_->publish(status_msg);
    network_reason_pub_->publish(reason_msg);
    heartbeat_pub_->publish(heartbeat_msg);

    if (!heartbeat_pub_->assert_liveliness()) {
      RCLCPP_WARN(this->get_logger(), "Failed to assert network monitor liveliness");
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Network=%s | %s",
      status_msg.data.c_str(),
      reason_msg.data.c_str());

    step_++;
  }

  NetworkState get_fake_network_state()
  {
    int phase = step_ % (connected_phase_count_ + 1);

    if (phase < connected_phase_count_ && phase % 3 == 0)
    {
      return {
        "CONNECTED:Lab_WiFi:SIGNAL_82:BAND_5GHz",
        "AVAILABLE:SIGNAL_70",
        "AVAILABLE:SIGNAL_65",
        "WIFI",
        "NETWORK_HEALTHY"
      };
    }

    if (phase < connected_phase_count_ && phase % 3 == 1)
    {
      return {
        "KILLED",
        "CONNECTED:SIGNAL_70",
        "AVAILABLE:SIGNAL_65",
        "LTE",
        "NETWORK_DEGRADED"
      };
    }

    if (phase < connected_phase_count_)
    {
      return {
        "KILLED",
        "KILLED",
        "CONNECTED:SIGNAL_65",
        "STARLINK",
        "NETWORK_DEGRADED"
      };
    }

    return {
      "KILLED",
      "KILLED",
      "KILLED",
      "NONE",
      "NETWORK_DEGRADED"
    };
  }

  void apply_network_loss_grace_period(NetworkState & state)
  {
    if (state.active != "NONE") {
      network_loss_active_ = false;
      return;
    }

    if (!network_loss_active_) {
      network_loss_active_ = true;
      network_loss_started_at_ = this->now();
    }

    if ((this->now() - network_loss_started_at_).seconds() >= network_loss_grace_seconds_) {
      state.status = "NETWORK_UNHEALTHY";
    } else {
      state.status = "NETWORK_DEGRADED";
    }
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr network_status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr network_reason_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  int step_ = 0;
  bool network_loss_active_ = false;
  rclcpp::Time network_loss_started_at_;
  const double network_loss_grace_seconds_ = 0.0;
  const int connected_phase_count_ = 8;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NetworkMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
