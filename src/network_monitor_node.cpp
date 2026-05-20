#include <chrono>
#include <iomanip>
#include <memory>
#include <sstream>
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
    double distance_m = 0.0;
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
      "DISTANCE_FROM_BASE=" + format_distance(state.distance_m) +
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
    const double distance_m = simulated_distance_from_base();

    if (distance_m <= wifi_range_m_)
    {
      return {
        "CONNECTED:Factory_AP_5G:RSSI_-52dBm:BAND_5GHz:LATENCY_18ms:LOSS_0.2%",
        "STANDBY:RSRP_-84dBm:SINR_19dB:LATENCY_48ms:LOSS_0.5%",
        "STANDBY:SNR_10.1dB:LATENCY_82ms:LOSS_0.8%",
        "WIFI",
        "NETWORK_HEALTHY",
        distance_m
      };
    }

    if (distance_m <= lte_range_m_)
    {
      return {
        "FAILED:OUT_OF_RANGE:LAST_RSSI_-83dBm",
        "CONNECTED:RSRP_-94dBm:SINR_12dB:LATENCY_72ms:LOSS_1.5%",
        "STANDBY:SNR_8.4dB:LATENCY_102ms:LOSS_1.1%",
        "LTE",
        "BACKUP_CONNECTION",
        distance_m
      };
    }

    if (distance_m <= starlink_range_m_)
    {
      return {
        "FAILED:OUT_OF_RANGE:LAST_RSSI_-90dBm",
        "FAILED:NO_SERVICE:RSRP_-121dBm:SINR_1dB",
        "CONNECTED:SNR_7.6dB:LATENCY_126ms:LOSS_2.1%",
        "STARLINK",
        "BACKUP_CONNECTION",
        distance_m
      };
    }

    return {
      "FAILED:OUT_OF_RANGE:LAST_RSSI_-94dBm",
      "FAILED:NO_SERVICE:RSRP_-126dBm",
      "FAILED:SATELLITE_OBSTRUCTED:SNR_1.4dB",
      "NONE",
      "RETURN_TO_BASE",
      distance_m
    };
  }

  double simulated_distance_from_base() const
  {
    const int phase = step_ % cycle_phase_count_;

    if (phase < 12) {
      return interpolate(phase, 0, 11, 10.0, 75.0);
    }

    if (phase < 28) {
      return interpolate(phase, 12, 27, 120.0, 2800.0);
    }

    if (phase < 52) {
      return interpolate(phase, 28, 51, 3500.0, 19000.0);
    }

    if (phase < 60) {
      return interpolate(phase, 52, 59, 21000.0, 24000.0);
    }

    if (phase < 84) {
      return interpolate(phase, 60, 83, 19000.0, 3500.0);
    }

    if (phase < 100) {
      return interpolate(phase, 84, 99, 2800.0, 120.0);
    }

    return interpolate(phase, 100, 111, 75.0, 10.0);
  }

  double interpolate(
    const int phase,
    const int start_phase,
    const int end_phase,
    const double start_distance,
    const double end_distance) const
  {
    const double span = static_cast<double>(end_phase - start_phase);
    if (span <= 0.0) {
      return end_distance;
    }

    const double progress = static_cast<double>(phase - start_phase) / span;
    return start_distance + ((end_distance - start_distance) * progress);
  }

  std::string format_distance(const double distance_m) const
  {
    std::ostringstream out;
    out << std::fixed << std::setprecision(0) << distance_m << "m,";
    return out.str();
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
      state.status = "RETURN_TO_BASE";
    } else {
      state.status = "BACKUP_CONNECTION";
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
  const double wifi_range_m_ = 80.0;
  const double lte_range_m_ = 3000.0;
  const double starlink_range_m_ = 20000.0;
  const int cycle_phase_count_ = 112;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NetworkMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
