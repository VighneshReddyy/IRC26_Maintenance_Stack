// relay.cpp
#include <iostream>
#include <string>
#include <thread>
#include <atomic>
#include <chrono>
#include <mutex>
#include <cstring>
#include <arpa/inet.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

static constexpr const char* LISTEN_IP = "10.0.0.25";
static constexpr int LISTEN_PORT = 5010;

static constexpr const char* ESP_IP = "10.0.0.7";
static constexpr int ESP_PORT = 5005;

class RelayNode : public rclcpp::Node
{
public:
  RelayNode()
  : Node("relay_node_udp_bridge")
  {
    RCLCPP_INFO(this->get_logger(), "Relay node starting...");

    // ---------------- ROS Subscriptions ----------------
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&RelayNode::cmdVelCallback, this, std::placeholders::_1));

    arm_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/arm_vel", 10,
      std::bind(&RelayNode::armCallback, this, std::placeholders::_1));

    mode_cmd_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/autonomous_mode_cmd", 10,
      std::bind(&RelayNode::modeCmdCallback, this, std::placeholders::_1));

    mode_state_pub_ = this->create_publisher<std_msgs::msg::Bool>("/autonomous_mode_state", 10);

    mode_timer_ = this->create_wall_timer(200ms, std::bind(&RelayNode::publishModeState, this));

    // ---------------- Autonomous toggle service ----------------
    toggle_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/toggle_autonomous",
      std::bind(&RelayNode::toggleAutonomousService, this, std::placeholders::_1, std::placeholders::_2)
    );

    RCLCPP_INFO(this->get_logger(), "Service /toggle_autonomous ready.");

    // ---------------- Start UDP Threads ----------------
    listener_thread_ = std::thread(&RelayNode::udpListenerThread, this);
    sender_thread_   = std::thread(&RelayNode::udpSenderThread, this);

    RCLCPP_INFO(this->get_logger(), "Relay node initialized.");
  }

  ~RelayNode()
  {
    running_.store(false);

    if (listener_thread_.joinable()) listener_thread_.join();
    if (sender_thread_.joinable())   sender_thread_.join();
  }

private:

  // ---------------- SERVICE: Toggle Autonomous ----------------
  void toggleAutonomousService(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    bool old_mode = autonomous_mode_.load();
    bool new_mode = !old_mode;

    autonomous_mode_.store(new_mode);

    RCLCPP_WARN(this->get_logger(),
      "Mode toggled → %s",
      new_mode ? "AUTONOMOUS (Z1)" : "MANUAL (Z0)");

    response->success = true;
    response->message = new_mode ? "Autonomous ON" : "Autonomous OFF";
  }


  // ---------------- ROS CALLBACKS ----------------
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    int left  = static_cast<int>(msg->linear.x  * 100.0);
    int right = static_cast<int>(msg->angular.z * 100.0);

    std::lock_guard<std::mutex> lock(mtx_);
    last_motor_packet_ = "L" + std::to_string(left) + "R" + std::to_string(right) + "E";
  }

  void armCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    last_arm_packet_ = msg->data;
  }

  void modeCmdCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    bool old_mode = autonomous_mode_.load();
    bool new_mode = msg->data;

    autonomous_mode_.store(new_mode);

    if (old_mode != new_mode)
      RCLCPP_WARN(this->get_logger(), "Mode changed → %s",
                  new_mode ? "AUTONOMOUS (Z1)" : "MANUAL (Z0)");
  }

  void publishModeState()
  {
    auto m = std::make_shared<std_msgs::msg::Bool>();
    m->data = autonomous_mode_.load();
    mode_state_pub_->publish(*m);
  }

  // ---------------- UDP LISTENER ----------------
  void udpListenerThread()
  {
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create UDP listener socket");
      return;
    }

    sockaddr_in servaddr{};
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = inet_addr(LISTEN_IP);
    servaddr.sin_port = htons(LISTEN_PORT);

    if (bind(sockfd, reinterpret_cast<sockaddr*>(&servaddr), sizeof(servaddr)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to bind UDP listener to %s:%d", LISTEN_IP, LISTEN_PORT);
      close(sockfd);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Listening for manual data on %s:%d", LISTEN_IP, LISTEN_PORT);

    constexpr size_t BUF_SZ = 1500;
    char buffer[BUF_SZ];
    sockaddr_in cliaddr{};
    socklen_t addrlen = sizeof(cliaddr);

    while (rclcpp::ok() && running_.load()) {
      memset(buffer, 0, BUF_SZ);
      ssize_t n = recvfrom(sockfd, buffer, BUF_SZ-1, 0,
                           reinterpret_cast<sockaddr*>(&cliaddr), &addrlen);

      if (n > 0) {
        std::string s(buffer, static_cast<size_t>(n));
        {
          std::lock_guard<std::mutex> lock(mtx_);
          last_manual_packet_ = s;
        }

        if (!autonomous_mode_.load()) {
          RCLCPP_INFO(this->get_logger(), "MANUAL | %s", s.c_str());
        }
      }
    }

    close(sockfd);
  }

  // ---------------- UDP SENDER ----------------
  void udpSenderThread()
  {
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create UDP sender socket");
      return;
    }

    sockaddr_in espAddr{};
    espAddr.sin_family = AF_INET;
    espAddr.sin_port = htons(ESP_PORT);

    if (inet_pton(AF_INET, ESP_IP, &espAddr.sin_addr) != 1) {
      RCLCPP_ERROR(this->get_logger(), "Invalid ESP IP: %s", ESP_IP);
      close(sockfd);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "UDP sender configured to %s:%d", ESP_IP, ESP_PORT);

    while (rclcpp::ok() && running_.load()) {
      std::string packet_to_send;

      if (!autonomous_mode_.load()) {
        std::lock_guard<std::mutex> lock(mtx_);
        packet_to_send = last_manual_packet_;
      } else {
        std::lock_guard<std::mutex> lock(mtx_);
        if (last_arm_packet_.empty())
          packet_to_send = last_motor_packet_;
        else
          packet_to_send = last_arm_packet_ + last_motor_packet_;
      }

      if (!packet_to_send.empty()) {
        ssize_t sent = sendto(sockfd, packet_to_send.c_str(), packet_to_send.size(), 0,
                              reinterpret_cast<const sockaddr*>(&espAddr), sizeof(espAddr));

        if (sent < 0) {
          RCLCPP_ERROR(this->get_logger(), "Failed to send UDP packet to ESP");
        } else if (autonomous_mode_.load()) {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "%s", packet_to_send.c_str());
        }
      }

      std::this_thread::sleep_for(50ms);
    }

    close(sockfd);
  }

  // ---------------- MEMBERS ----------------
  std::atomic<bool> running_{true};
  std::atomic<bool> autonomous_mode_{false};

  std::thread listener_thread_;
  std::thread sender_thread_;

  std::mutex mtx_;
  std::string last_manual_packet_ = "M0X0Y0P0Q0A0S0J0DE";
  std::string last_motor_packet_  = "L0R0E";
  std::string last_arm_packet_    = "";

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr arm_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mode_cmd_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mode_state_pub_;
  rclcpp::TimerBase::SharedPtr mode_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr toggle_srv_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RelayNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
