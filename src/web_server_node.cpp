#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

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
  if (value.find("HEALTHY") != std::string::npos ||
      value.find("SAFE_NORMAL") != std::string::npos ||
      value == "NORMAL") {
    return "ok";
  }

  if (value.find("REDUCE") != std::string::npos ||
      value.find("USING_") != std::string::npos) {
    return "warn";
  }

  return "danger";
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

    node1_heartbeat_sub_ = make_heartbeat_sub("/node1/heartbeat", 0);
    node2_heartbeat_sub_ = make_heartbeat_sub("/node2/heartbeat", 1);
    node3_heartbeat_sub_ = make_heartbeat_sub("/node3/heartbeat", 2);
    node6_heartbeat_sub_ = make_heartbeat_sub("/node6/heartbeat", 3);

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

    rclcpp::Time last_data_a;
    rclcpp::Time last_data_b;
    rclcpp::Time last_adjusted_speed;
    rclcpp::Time last_system;
    rclcpp::Time last_primary_health;
    rclcpp::Time last_backup_health;
    rclcpp::Time last_camera;
    rclcpp::Time last_heartbeat[4];
    bool received_heartbeat[4] = {false, false, false, false};
  };

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr make_heartbeat_sub(
    const std::string & topic,
    const size_t index)
  {
    return create_subscription<std_msgs::msg::String>(
      topic,
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      [this, index](const std_msgs::msg::String::SharedPtr) {
        std::lock_guard<std::mutex> lock(mutex_);
        state_.received_heartbeat[index] = true;
        state_.last_heartbeat[index] = now();
      });
  }

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

  std::string node_json(
    const std::string & name,
    const std::string & topic,
    const rclcpp::Time & heartbeat,
    const bool received,
    const double timeout_seconds) const
  {
    const bool ok = received && fresh(heartbeat, timeout_seconds);
    std::ostringstream out;
    out << "{\"name\":\"" << escape_json(name)
        << "\",\"topic\":\"" << escape_json(topic)
        << "\",\"status\":\"" << (ok ? "OK" : "STALE")
        << "\",\"age_seconds\":" << age_json(heartbeat) << "}";
    return out.str();
  }

  std::string final_action(const DashboardState & state) const
  {
    if (state.primary_health == "UNHEALTHY" || state.backup_health == "UNHEALTHY" ||
        state.system_status == "UNSAFE_STOP") {
      return "STOP";
    }

    if (state.system_status.find("UNSAFE_REDUCE") != std::string::npos) {
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
    const std::string action = final_action(state);

    std::ostringstream out;
    out << std::fixed << std::setprecision(2);
    out << "{";
    out << "\"system_status\":\"" << escape_json(state.system_status) << "\",";
    out << "\"system_class\":\"" << status_class(state.system_status) << "\",";
    out << "\"primary_health\":\"" << escape_json(state.primary_health) << "\",";
    out << "\"primary_reason\":\"" << escape_json(state.primary_reason) << "\",";
    out << "\"primary_class\":\"" << status_class(state.primary_health) << "\",";
    out << "\"backup_health\":\"" << escape_json(state.backup_health) << "\",";
    out << "\"backup_reason\":\"" << escape_json(state.backup_reason) << "\",";
    out << "\"backup_class\":\"" << status_class(state.backup_health) << "\",";
    out << "\"camera_status\":\"" << escape_json(state.camera_status) << "\",";
    out << "\"camera_age_seconds\":" << age_json(state.last_camera) << ",";
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
    out << node_json("Node 1 / LiDAR", "/node1/heartbeat", state.last_heartbeat[0], state.received_heartbeat[0], 0.75) << ",";
    out << node_json("Node 2 / Speed", "/node2/heartbeat", state.last_heartbeat[1], state.received_heartbeat[1], 2.0) << ",";
    out << node_json("Node 3 / Safety", "/node3/heartbeat", state.last_heartbeat[2], state.received_heartbeat[2], 1.0) << ",";
    out << node_json("Camera Node", "/node6/heartbeat", state.last_heartbeat[3], state.received_heartbeat[3], 1.5);
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
    .metric strong { display: block; font-size: 12px; color: #5c6f8a; margin-bottom: 5px; }
    .metric span { font-size: 21px; font-weight: 800; }
    @media (max-width: 900px) {
      header { align-items: flex-start; flex-direction: column; padding: 16px; }
      main { padding: 16px; }
      .grid { grid-template-columns: 1fr; }
      .wide { grid-column: span 1; }
      .metric-row { grid-template-columns: 1fr; }
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
        <h2>Final Action</h2>
        <div id="finalAction" class="value">WAITING</div>
        <div id="adjustedSpeed" class="subvalue">Adjusted speed: --</div>
      </article>
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
          <div class="metric"><strong>Normal Speed</strong><span id="normalSpeed">--</span></div>
          <div class="metric"><strong>Camera</strong><span id="cameraStatus">--</span></div>
        </div>
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
      finalAction: document.getElementById('finalAction'),
      adjustedSpeed: document.getElementById('adjustedSpeed'),
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
      cameraStatus: document.getElementById('cameraStatus'),
      nodeRows: document.getElementById('nodeRows')
    };

    function fmt(value, suffix = '') {
      return value === null || value === undefined ? '--' : `${Number(value).toFixed(2)}${suffix}`;
    }

    function setClass(el, cls) {
      el.className = `value ${cls}`;
    }

    function render(data) {
      fields.connection.textContent = `Live at ${new Date().toLocaleTimeString()}`;
      fields.finalAction.textContent = data.final_action;
      setClass(fields.finalAction, data.action_class);
      fields.adjustedSpeed.textContent = `Adjusted speed: ${fmt(data.adjusted_speed, ' m/s')}`;

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
      fields.cameraStatus.textContent = data.camera_status;

      fields.nodeRows.innerHTML = data.nodes.map((node) => {
        const cls = node.status === 'OK' ? 'ok' : 'danger';
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
  std::atomic<bool> running_{false};
  std::thread server_thread_;

  std::mutex mutex_;
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
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr node1_heartbeat_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr node2_heartbeat_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr node3_heartbeat_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr node6_heartbeat_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WebServerNode>());
  rclcpp::shutdown();
  return 0;
}
