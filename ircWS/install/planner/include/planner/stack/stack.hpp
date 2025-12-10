#pragma once
#include <chrono>
#include <cmath>
#include <string>
#include <iostream>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "custom_msgs/msg/imu_data.hpp"
#include "custom_msgs/msg/marker_tag.hpp"

namespace rover_stack {

class StackNode : public rclcpp::Node {
public:
    StackNode() : Node("stack_node") {
        kMaxLinearVel_ = 0.6;
        kMaxAngularVel_ = 0.5;
        time_offset_ = 3.0;

        stack_state_ = StackState::kManualState;
        search_state_ = SearchPatternState::kMoveForward;
        search_init_ = false;
        delivery_sent_ = false;
        marker_locked_ = false;

        yaw_ = 0.0;
        dest_heading_ = 0.0;
        offset_counter_ = 0;
        search_skew_dir_ = 1;

        vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        arm_pub_ = create_publisher<std_msgs::msg::String>("/arm_vel", 10);

        autonomous_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/autonomous_mode_state", 10,
            std::bind(&StackNode::autonomousCallback, this, std::placeholders::_1));

        imu_sub_ = create_subscription<custom_msgs::msg::ImuData>(
            "/imu_data", 10,
            std::bind(&StackNode::imuCallback, this, std::placeholders::_1));

        marker_sub_ = create_subscription<custom_msgs::msg::MarkerTag>(
            "/marker_detect", 10,
            std::bind(&StackNode::markerCallback, this, std::placeholders::_1));

        gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
            "/fix", 10,
            std::bind(&StackNode::gpsCallback, this, std::placeholders::_1));

        toggle_client_ = create_client<std_srvs::srv::Trigger>("/toggle_autonomous");

        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&StackNode::timerCallback, this));

        RCLCPP_INFO(get_logger(), "Stack initialized.");
    }

private:
    enum class StackState {
        kManualState,
        kNavigationModeSelect,
        kSearchPattern,
        kConeFollowing,
        kObstacleAvoidance,
        kObjectDelivery
    };

    enum class SearchPatternState {
        kMoveForward,
        kTurnRight,
        kTurnLeft,
        kOffsetTurn,
        kResetHeading
    };

    struct MarkerInfo {
        bool is_found = false;
        int id = 0;
        double x = 0.0;
        double y = 0.0;
    };

    void getNavigationInput() {
        std::cout << "Navigation Mode:\n";
        std::cout << "0 = GPS\n1 = Cone Mode\nEnter mode: " << std::flush;
        std::cin >> nav_mode_;

        if (nav_mode_ == 1) {
            std::cout << "Cone Mapping:\n1 = Orange\n2 = Red\n3 = Blue\n4 = Green\n5 = Yellow\nEnter cone number: " << std::flush;
            std::cin >> cone_id_target_;

            std::cout << "Search Skew:\n1 = Left\n2 = Right\nEnter option: " << std::flush;
            int opt; std::cin >> opt;
            search_skew_dir_ = (opt == 2) ? -1 : 1;
        }

        search_init_ = false;
        search_state_ = SearchPatternState::kMoveForward;
        stack_state_ = StackState::kSearchPattern;
        marker_locked_ = false;
        marker_.is_found = false;
        marker_.id = 0;

        RCLCPP_INFO(get_logger(), "Navigation input accepted.");
    }

    void timerCallback() {
        updateMarkerStability();

        switch (stack_state_) {
            case StackState::kManualState: handleManual(); break;
            case StackState::kSearchPattern: handleSearch(); break;
            case StackState::kConeFollowing: handleConeFollow(); break;
            case StackState::kObstacleAvoidance: handleObstacle(); break;
            case StackState::kObjectDelivery: handleDelivery(); break;
            case StackState::kNavigationModeSelect: break;
        }
    }

    void handleManual() {
        geometry_msgs::msg::Twist stop;
        publishCmdVel(stop);
    }

    void handleSearch() {
        if (!search_init_) {
            delivery_sent_ = false;
            end_time_ = now() + rclcpp::Duration::from_seconds(time_offset_);
            offset_counter_ = 0;
            search_init_ = true;
            search_state_ = SearchPatternState::kMoveForward;
            marker_locked_ = false;
            marker_.is_found = false;
            marker_.id = 0;
            RCLCPP_INFO(get_logger(), "Search: Pattern initialized.");
        }

        if (markerDetected(cone_id_target_)) {
            RCLCPP_INFO(get_logger(), "Search: Marker detected. Switching to cone follow.");
            stack_state_ = StackState::kConeFollowing;
            search_init_ = false;
            return;
        }

        geometry_msgs::msg::Twist cmd;

        switch (search_state_) {

            case SearchPatternState::kMoveForward: {
                double remaining = (end_time_ - now()).seconds();

                if (now() < end_time_) {
                    cmd.linear.x = kMaxLinearVel_;
                    RCLCPP_INFO_THROTTLE(
                        get_logger(), *get_clock(), 1500,
                        "Search: Moving forward (%.1f sec remaining).",
                        remaining);
                } else {
                    dest_heading_ = normalize(yaw_ - 90.0);
                    search_state_ = SearchPatternState::kTurnRight;
                    RCLCPP_INFO(get_logger(),
                        "Search: Forward segment complete. Turning right to %.1f°.",
                        dest_heading_);
                }
            } break;

            case SearchPatternState::kTurnRight: {
                double err = normalize(dest_heading_ - yaw_);
                if (!aligned(dest_heading_)) {
                    cmd.angular.z = kMaxAngularVel_;
                    RCLCPP_INFO_THROTTLE(
                        get_logger(), *get_clock(), 1200,
                        "Search: Turning right (error %.1f°).", err);
                } else {
                    dest_heading_ = normalize(yaw_ + 180.0);
                    search_state_ = SearchPatternState::kTurnLeft;
                    RCLCPP_INFO(get_logger(),
                        "Search: Right turn aligned. Turning left to %.1f°.",
                        dest_heading_);
                }
            } break;

            case SearchPatternState::kTurnLeft: {
                double err = normalize(dest_heading_ - yaw_);
                if (!aligned(dest_heading_)) {
                    cmd.angular.z = -kMaxAngularVel_;
                    RCLCPP_INFO_THROTTLE(
                        get_logger(), *get_clock(), 1200,
                        "Search: Turning left (error %.1f°).", err);
                } else {
                    dest_heading_ = normalize(yaw_ + 15.0 * search_skew_dir_);
                    search_state_ = SearchPatternState::kOffsetTurn;
                    RCLCPP_INFO(get_logger(),
                        "Search: Left turn aligned. Applying skew offset to %.1f°.",
                        dest_heading_);
                }
            } break;

            case SearchPatternState::kOffsetTurn: {
                double err = normalize(dest_heading_ - yaw_);
                if (std::abs(err) > 5.0) {
                    cmd.angular.z =
                        (err > 0.0 ? kMaxAngularVel_ : -kMaxAngularVel_) * 0.6;

                    RCLCPP_INFO_THROTTLE(
                        get_logger(), *get_clock(), 1200,
                        "Search: Offset turn (target %.1f°, error %.1f°).",
                        dest_heading_, err);
                } else {
                    offset_counter_++;
                    if (offset_counter_ >= 4) {
                        time_offset_ += 2.0;
                        offset_counter_ = 0;
                        RCLCPP_INFO(get_logger(),
                            "Search: Expanding pattern. Forward time increased to %.1f sec.",
                            time_offset_);
                    }

                    search_state_ = SearchPatternState::kResetHeading;
                    RCLCPP_INFO(get_logger(),
                        "Search: Offset turn complete. Resetting heading.");
                }
            } break;

            case SearchPatternState::kResetHeading:
                end_time_ = now() + rclcpp::Duration::from_seconds(time_offset_);
                search_state_ = SearchPatternState::kMoveForward;
                RCLCPP_INFO(get_logger(),
                    "Search: Heading reset. New forward cycle (%.1f sec).",
                    time_offset_);
                break;
        }

        publishCmdVel(cmd);
    }

    void handleConeFollow() {
        if (!marker_locked_) {
            RCLCPP_WARN(get_logger(), "Cone follow: Marker lock lost. Returning to search.");
            stack_state_ = StackState::kSearchPattern;
            search_init_ = false;
            return;
        }

        double dist = std::hypot(marker_.x, marker_.y);
        if (dist <= 2.0) {
            geometry_msgs::msg::Twist stop;
            publishCmdVel(stop);
            delivery_sent_ = false;
            RCLCPP_INFO(get_logger(), "Cone follow: Target reached. Switching to delivery.");
            stack_state_ = StackState::kObjectDelivery;
            return;
        }

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = std::min(dist * 0.2, kMaxLinearVel_);
        cmd.angular.z = std::clamp(marker_.y * 1.5, -kMaxAngularVel_, kMaxAngularVel_);
        publishCmdVel(cmd);
    }

    void handleObstacle() { }

    void handleDelivery() {
        if (!markerDetected(cone_id_target_)) {
            RCLCPP_WARN(get_logger(), "Delivery: Marker lost. Returning to search.");
            stack_state_ = StackState::kSearchPattern;
            search_init_ = false;
            marker_locked_ = false;
            marker_.is_found = false;
            marker_.id = 0;
            return;
        }

        if (!delivery_sent_) {
            std_msgs::msg::String msg;
            msg.data = "P0Q0M0A0";
            arm_pub_->publish(msg);
            delivery_sent_ = true;
            delivery_timer_ = now() + rclcpp::Duration::from_seconds(2.0);
            RCLCPP_INFO(get_logger(), "Delivery: Packet sent. Waiting.");
            return;
        }

        if (now() < delivery_timer_) return;

        if (toggle_client_->wait_for_service(std::chrono::seconds(2))) {
            auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
            toggle_client_->async_send_request(req);
        }

        delivery_sent_ = false;
        marker_locked_ = false;
        marker_.is_found = false;
        marker_.id = 0;
        stack_state_ = StackState::kManualState;
        RCLCPP_INFO(get_logger(), "Delivery: Complete. Switching to manual.");
    }

    rclcpp::Time now() { return get_clock()->now(); }

    void publishCmdVel(const geometry_msgs::msg::Twist &cmd) {
        vel_pub_->publish(cmd);
    }

    bool aligned(double t) {
        return std::abs(normalize(t - yaw_)) <= 5.0;
    }

    bool markerDetected(int id) {
        if (marker_locked_) return true;
        return raw_marker_.is_found && raw_marker_.id == id;
    }

    double normalize(double a) {
        while (a > 180.0) a -= 360.0;
        while (a < -180.0) a += 360.0;
        return a;
    }

    void autonomousCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        bool was_auto = (stack_state_ != StackState::kManualState);

        if (!msg->data) {
            stack_state_ = StackState::kManualState;
            marker_locked_ = false;
            marker_.is_found = false;
            marker_.id = 0;
            if (was_auto) RCLCPP_INFO(get_logger(), "Autonomous disabled.");
            return;
        }

        if (stack_state_ == StackState::kManualState) {
            marker_locked_ = false;
            marker_.is_found = false;
            marker_.id = 0;
            RCLCPP_INFO(get_logger(), "Autonomous enabled.");
            getNavigationInput();
        }
    }

    void imuCallback(const custom_msgs::msg::ImuData::SharedPtr msg) {
        static double a = 0.2;
        yaw_ = a * msg->orientation.z + (1.0 - a) * yaw_;
    }

    void markerCallback(const custom_msgs::msg::MarkerTag::SharedPtr msg) {
        raw_marker_.is_found = msg->is_found;
        raw_marker_.id = msg->id;
        raw_marker_.x = msg->x;
        raw_marker_.y = msg->y;
    }

    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr) { }

    void updateMarkerStability() {
        if (!marker_locked_) {
            if (raw_marker_.is_found && raw_marker_.id == cone_id_target_) {
                marker_locked_ = true;
                marker_ = raw_marker_;
            }
        } else {
            if (raw_marker_.is_found) {
                if (raw_marker_.id == cone_id_target_) {
                    marker_ = raw_marker_;
                } else {
                    marker_locked_ = false;
                    marker_.is_found = false;
                    marker_.id = 0;
                    if (stack_state_ == StackState::kConeFollowing ||
                        stack_state_ == StackState::kObjectDelivery) {
                        RCLCPP_WARN(get_logger(),
                            "Marker changed. Returning to search.");
                        stack_state_ = StackState::kSearchPattern;
                        search_init_ = false;
                    }
                }
            }
        }
    }

    MarkerInfo marker_;
    MarkerInfo raw_marker_;

    StackState stack_state_;
    SearchPatternState search_state_;

    bool search_init_;
    bool delivery_sent_;
    bool marker_locked_;

    int nav_mode_;
    int cone_id_target_;
    int offset_counter_;
    int search_skew_dir_;

    double yaw_;
    double dest_heading_;
    double kMaxLinearVel_;
    double kMaxAngularVel_;
    double time_offset_;

    rclcpp::Time end_time_;
    rclcpp::Time delivery_timer_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr autonomous_sub_;
    rclcpp::Subscription<custom_msgs::msg::ImuData>::SharedPtr imu_sub_;
    rclcpp::Subscription<custom_msgs::msg::MarkerTag>::SharedPtr marker_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr arm_pub_;

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr toggle_client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rover_stack
