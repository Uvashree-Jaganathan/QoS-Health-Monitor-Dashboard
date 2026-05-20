#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class StatusDisplayNode : public rclcpp::Node
{
public:
  StatusDisplayNode() : Node("status_display_node")
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.reliable();

    health_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/health_status", qos,
      std::bind(&StatusDisplayNode::health_callback, this, std::placeholders::_1));

    reason_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/health_reason", qos,
      std::bind(&StatusDisplayNode::reason_callback, this, std::placeholders::_1));

    backup_health_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/health_status_backup", qos,
      std::bind(&StatusDisplayNode::backup_health_callback, this, std::placeholders::_1));

    backup_reason_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/health_reason_backup", qos,
      std::bind(&StatusDisplayNode::backup_reason_callback, this, std::placeholders::_1));

    system_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/system_status", qos,
      std::bind(&StatusDisplayNode::system_callback, this, std::placeholders::_1));

    speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/adjusted_speed", qos,
      std::bind(&StatusDisplayNode::speed_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      500ms,
      std::bind(&StatusDisplayNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Status Display node started");
  }

private:
  void health_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    health_status_ = msg->data;
    last_health_time_ = this->now();
    received_health_ = true;
  }

  void reason_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    health_reason_ = msg->data;
  }

  void backup_health_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    backup_health_status_ = msg->data;
    last_backup_health_time_ = this->now();
    received_backup_health_ = true;
  }

  void backup_reason_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    backup_health_reason_ = msg->data;
  }

  void system_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    system_status_ = msg->data;
    received_system_ = true;
  }

  void speed_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    adjusted_speed_ = msg->data;
    received_speed_ = true;
  }

  std::string combine_reasons(const std::string & reason1, const std::string & reason2)
  {
    if (reason1 == "NONE" && reason2 == "NONE") {
      return "NONE";
    }

    if (reason1 == "NONE") {
      return reason2;
    }

    if (reason2 == "NONE") {
      return reason1;
    }

    if (reason1 == reason2) {
      return reason1;
    }

    return reason1 + "," + reason2;
  }

  bool is_network_only_failure(const std::string & reason)
  {
    return reason == "NETWORK_CONNECTION_FAILURE";
  }

  void timer_callback()
  {
    if (!received_system_ || !received_speed_) return;

    auto now = this->now();

    bool primary_failed = false;
    bool backup_failed = false;

    if (!received_health_) {
      primary_failed = true;
    } else if ((now - last_health_time_).seconds() > 2.0) {
      primary_failed = true;
    }

    if (!received_backup_health_) {
      backup_failed = true;
    } else if ((now - last_backup_health_time_).seconds() > 2.0) {
      backup_failed = true;
    }

    if (primary_failed && backup_failed)
    {
      RCLCPP_ERROR(this->get_logger(),
        "\033[31mRED | Health=UNHEALTHY | System=UNSAFE_STOP | Speed=0.00 | Action=STOP | Reason=ALL_HEALTH_MONITORS_FAILED\033[0m");
    }
    else if (primary_failed)
    {
      RCLCPP_WARN(this->get_logger(),
        "\033[33mYELLOW | Health=%s | System=%s | Speed=%.2f | Action=USING_BACKUP_MONITOR | Reason=PRIMARY_HEALTH_MONITOR_FAILURE\033[0m",
        backup_health_status_.c_str(),
        system_status_.c_str(),
        adjusted_speed_);

      if (backup_health_status_ == "UNHEALTHY")
      {
        RCLCPP_ERROR(this->get_logger(),
          "\033[31mRED | Health=UNHEALTHY | System=UNSAFE_STOP | Speed=0.00 | Action=STOP | Reason=%s\033[0m",
          backup_health_reason_.c_str());
      }
    }
    else if (backup_failed)
    {
      RCLCPP_WARN(this->get_logger(),
        "\033[33mYELLOW | Health=%s | System=%s | Speed=%.2f | Action=USING_PRIMARY_MONITOR | Reason=BACKUP_HEALTH_MONITOR_FAILURE\033[0m",
        health_status_.c_str(),
        system_status_.c_str(),
        adjusted_speed_);

      if (health_status_ == "UNHEALTHY")
      {
        RCLCPP_ERROR(this->get_logger(),
          "\033[31mRED | Health=UNHEALTHY | System=UNSAFE_STOP | Speed=0.00 | Action=STOP | Reason=%s\033[0m",
          health_reason_.c_str());
      }
    }
    else if (health_status_ == "UNHEALTHY" || backup_health_status_ == "UNHEALTHY")
    {
      std::string final_reason = combine_reasons(health_reason_, backup_health_reason_);

      if (is_network_only_failure(final_reason))
      {
        RCLCPP_WARN(this->get_logger(),
          "\033[33mYELLOW | Health=UNHEALTHY | System=%s | Speed=%.2f | Action=RETURN_TO_BASE | Reason=%s\033[0m",
          system_status_.c_str(),
          adjusted_speed_,
          final_reason.c_str());
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(),
          "\033[31mRED | Health=UNHEALTHY | System=UNSAFE_STOP | Speed=0.00 | Action=STOP | Reason=%s\033[0m",
          final_reason.c_str());
      }
    }
    else if (system_status_ == "EMERGENCY_BATTERY_SAFE_LANDING")
    {
      RCLCPP_ERROR(this->get_logger(),
        "\033[31mRED | Health=HEALTHY | System=%s | Speed=0.00 | Action=SAFE_LANDING | Reason=EMERGENCY_BATTERY\033[0m",
        system_status_.c_str());
    }
    else if (system_status_ == "NETWORK_RETURN_TO_BASE")
    {
      RCLCPP_WARN(this->get_logger(),
        "\033[33mYELLOW | Health=HEALTHY | System=%s | Speed=%.2f | Action=RETURN_TO_BASE | Reason=NETWORK_CONNECTION_FAILURE\033[0m",
        system_status_.c_str(),
        adjusted_speed_);
    }
    else if (system_status_.find("RETURN_TO_BASE") != std::string::npos)
    {
      RCLCPP_WARN(this->get_logger(),
        "\033[33mYELLOW | Health=HEALTHY | System=%s | Speed=%.2f | Action=RETURN_TO_BASE | Reason=CRITICAL_BATTERY\033[0m",
        system_status_.c_str(),
        adjusted_speed_);
    }
    else if (system_status_.find("LOW_BATTERY_REDUCE") != std::string::npos)
    {
      RCLCPP_WARN(this->get_logger(),
        "\033[33mYELLOW | Health=HEALTHY | System=%s | Speed=%.2f | Action=REDUCE_SPEED | Reason=LOW_BATTERY\033[0m",
        system_status_.c_str(),
        adjusted_speed_);
    }
    else if (system_status_ == "UNSAFE_STOP")
    {
      RCLCPP_ERROR(this->get_logger(),
        "\033[31mRED | Health=HEALTHY | System=UNSAFE_STOP | Speed=0.00 | Action=STOP | Reason=MISSING_REQUIRED_INPUT\033[0m");
    }
    else if (system_status_.find("UNSAFE") != std::string::npos)
    {
      RCLCPP_WARN(this->get_logger(),
        "\033[33mYELLOW | Health=HEALTHY | System=%s | Speed=%.2f | Action=REDUCE_SPEED | Monitors=PRIMARY_OK,BACKUP_OK\033[0m",
        system_status_.c_str(),
        adjusted_speed_);
    }
    else
    {
      RCLCPP_INFO(this->get_logger(),
        "\033[32mGREEN | Health=HEALTHY | System=%s | Speed=%.2f | Action=NORMAL | Monitors=PRIMARY_OK,BACKUP_OK\033[0m",
        system_status_.c_str(),
        adjusted_speed_);
    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr health_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr reason_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr backup_health_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr backup_reason_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr system_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::string health_status_ = "UNHEALTHY";
  std::string health_reason_ = "NONE";
  std::string backup_health_status_ = "UNHEALTHY";
  std::string backup_health_reason_ = "NONE";
  std::string system_status_ = "UNSAFE_STOP";
  float adjusted_speed_ = 0.0f;

  bool received_health_ = false;
  bool received_backup_health_ = false;
  bool received_system_ = false;
  bool received_speed_ = false;

  rclcpp::Time last_health_time_;
  rclcpp::Time last_backup_health_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StatusDisplayNode>());
  rclcpp::shutdown();
  return 0;
}
