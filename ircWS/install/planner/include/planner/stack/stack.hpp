#pragma once

#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <mutex>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "custom_msgs/msg/marker_tag.hpp"
#include "custom_msgs/msg/imu_data.hpp"

using namespace std::chrono_literals;

namespace rover_stack {

// ============================================================================
// SEARCH PATTERN ENGINE
// ============================================================================

class SearchPattern {
public:
    enum class State { kMoveForward, kTurnRight, kTurnLeft, kOffsetTurn, kResetHeading };

    struct MarkerInfo {
        bool cone_found = false;
        int id = -1;
        float x = 0;   // dist
        float y = 0;   // offset
    };

    explicit SearchPattern(rclcpp::Node* n)
        : node_(n)
    {
        vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        kMaxLinearVel_ = 0.6;
        kMaxAngularVel_ = 0.6;
        offsetAngular_ = 0.3;
        time_offset_ = 3.0;
        yaw_ = 0;
        dest_heading_ = 0;
        offset_counter_ = 0;
        search_skew_dir_ = 1;
        search_init_ = false;
        search_state_ = State::kMoveForward;
        target_id_ = -1;
    }

    void setMarker(const MarkerInfo &m) { marker_ = m; }
    void setYaw(double y) { yaw_ = y; }
    void setTargetId(int id) { target_id_ = id; }
    bool targetFound() const { return marker_.cone_found && marker_.id == target_id_; }

    void run() { handle(); }

private:
    void handle() {
        if (!search_init_) {
            search_init_ = true;
            offset_counter_ = 0;
            end_time_ = now() + rclcpp::Duration::from_seconds(time_offset_);
            search_state_ = State::kMoveForward;
            RCLCPP_INFO(node_->get_logger(), "[SEARCH] Init target_id=%d yaw=%.2f", target_id_, yaw_);
        }

        if (targetFound()) {
            RCLCPP_INFO(node_->get_logger(),
                "[SEARCH] ConeFound id=%d dist=%.2f offset=%.2f", marker_.id, marker_.x, marker_.y);

            geometry_msgs::msg::Twist stop;
            vel_pub_->publish(stop);
            return;
        }

        geometry_msgs::msg::Twist cmd;

        switch (search_state_) {
            case State::kMoveForward: {
                double t = (end_time_ - now()).seconds();
                RCLCPP_INFO(node_->get_logger(),
                    "[SEARCH] MoveForward vx=%.2f time_left=%.2f yaw=%.2f",
                    kMaxLinearVel_, t, yaw_);

                if (now() < end_time_) cmd.linear.x = kMaxLinearVel_;
                else {
                    dest_heading_ = norm(yaw_ - 90);
                    RCLCPP_INFO(node_->get_logger(), "[SEARCH] → TurnRight target=%.2f", dest_heading_);
                    search_state_ = State::kTurnRight;
                }
            } break;

            case State::kTurnRight: {
                double err = norm(dest_heading_ - yaw_);
                RCLCPP_INFO(node_->get_logger(),
                    "[SEARCH] TurnRight yaw=%.2f target=%.2f err=%.2f", yaw_, dest_heading_, err);

                if (fabs(err) > 5) cmd.angular.z = -kMaxAngularVel_;
                else {
                    dest_heading_ = norm(yaw_ + 180);
                    RCLCPP_INFO(node_->get_logger(), "[SEARCH] → TurnLeft target=%.2f", dest_heading_);
                    search_state_ = State::kTurnLeft;
                }
            } break;

            case State::kTurnLeft: {
                double err = norm(dest_heading_ - yaw_);
                RCLCPP_INFO(node_->get_logger(),
                    "[SEARCH] TurnLeft yaw=%.2f target=%.2f err=%.2f", yaw_, dest_heading_, err);

                if (fabs(err) > 5) cmd.angular.z = kMaxAngularVel_;
                else {
                    dest_heading_ = norm(yaw_ + 15);
                    RCLCPP_INFO(node_->get_logger(), "[SEARCH] → OffsetTurn target=%.2f", dest_heading_);
                    search_state_ = State::kOffsetTurn;
                }
            } break;

            case State::kOffsetTurn: {
                double err = norm(dest_heading_ - yaw_);
                RCLCPP_INFO(node_->get_logger(),
                    "[SEARCH] OffsetTurn yaw=%.2f target=%.2f err=%.2f", yaw_, dest_heading_, err);

                if (fabs(err) > 5) cmd.angular.z = (err > 0 ? offsetAngular_ : -offsetAngular_);
                else {
                    offset_counter_++;
                    if (offset_counter_ >= 4) time_offset_ += 2;
                    RCLCPP_INFO(node_->get_logger(), "[SEARCH] → ResetHeading next_time=%.2f", time_offset_);
                    search_state_ = State::kResetHeading;
                }
            } break;

            case State::kResetHeading: {
                RCLCPP_INFO(node_->get_logger(), "[SEARCH] ResetHeading");
                end_time_ = now() + rclcpp::Duration::from_seconds(time_offset_);
                search_state_ = State::kMoveForward;
            } break;
        }

        vel_pub_->publish(cmd);
    }

    double norm(double v) {
        while (v > 180) v -= 360;
        while (v < -180) v += 360;
        return v;
    }

    rclcpp::Time now() { return node_->get_clock()->now(); }

    rclcpp::Node *node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

    MarkerInfo marker_;
    int target_id_;
    int search_skew_dir_;

    double yaw_, dest_heading_;
    double kMaxLinearVel_, kMaxAngularVel_, offsetAngular_;
    double time_offset_;
    int offset_counter_;
    bool search_init_;
    State search_state_;
    rclcpp::Time end_time_;
};

// ============================================================================
// STACK NODE (MAIN STATE MACHINE + FOLLOWING FIXES)
// ============================================================================

class StackNode : public rclcpp::Node {
public:
    enum class State {
        kManualState,
        kNavigationModeSelect,
        kSearchPattern,
        kConeFollowing,
        kObstacleAvoidance,
        kObjectDelivery
    };

    StackNode()
        : Node("stack_node")
    {
        cmd_pub_  = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        arm_pub_  = create_publisher<std_msgs::msg::String>("/arm_vel", 10);

        imu_sub_ = create_subscription<custom_msgs::msg::ImuData>(
            "/imu_data", 10, std::bind(&StackNode::imuCB, this, std::placeholders::_1));

        marker_sub_ = create_subscription<custom_msgs::msg::MarkerTag>(
            "/marker_detect", 10, std::bind(&StackNode::markerCB, this, std::placeholders::_1));

        gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
            "/fix", 10, std::bind(&StackNode::gpsCB, this, std::placeholders::_1));

        auto_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/autonomous_mode_state", 10, std::bind(&StackNode::autoCB, this, std::placeholders::_1));

        obs_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/obstacles", 10, std::bind(&StackNode::obsCB, this, std::placeholders::_1));

        toggle_client_ = create_client<std_srvs::srv::Trigger>("/toggle_autonomous");

        search_ = std::make_shared<SearchPattern>(this);

        // initial states
        state_ = State::kManualState;
        prev_state_ = State::kManualState;

        cone_found_ = false;
        cone_id_ = -1;
        cone_x_ = 100;
        cone_y_ = 100;

        filtered_cone_x_ = 100;
        filtered_cone_y_ = 100;

        imu_yaw_ = 0;
        autonomous_ = false;
        obstacle_ = false;
        target_id_ = -1;

        kMaxLin_ = 0.6;
        kMaxAng_ = 0.6;

        // FOLLOW FIX PARAMETERS
        follow_vx_max_ = 0.25;
        follow_vx_min_ = 0.05;
        follow_vx_close_cut_ = 2.0;
        ang_gain_ = 0.35;
        ang_max_clamp_ = 0.35;
        offset_filter_alpha_ = 0.7;

        cone_locked_ = false;
        last_valid_cone_time_ = this->get_clock()->now();
        cone_lost_timeout_ = 2.0;

        timer_ = create_wall_timer(20ms, std::bind(&StackNode::run, this));
    }

private:
    // ----------------------------------------------------------------------
    // CALLBACKS
    // ----------------------------------------------------------------------

    void imuCB(const custom_msgs::msg::ImuData::SharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_);
        imu_yaw_ = m->orientation.z;
    }

    void markerCB(const custom_msgs::msg::MarkerTag::SharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_);

        cone_found_ = m->is_found;
        cone_id_ = m->id;
        cone_x_ = m->x;
        cone_y_ = m->y;

        // filtering
        filtered_cone_x_ = offset_filter_alpha_ * filtered_cone_x_ + (1 - offset_filter_alpha_) * cone_x_;
        filtered_cone_y_ = offset_filter_alpha_ * filtered_cone_y_ + (1 - offset_filter_alpha_) * cone_y_;

        if (state_ == State::kSearchPattern) {
            SearchPattern::MarkerInfo mi;
            mi.cone_found = cone_found_;
            mi.id = cone_id_;
            mi.x = cone_x_;
            mi.y = cone_y_;
            search_->setMarker(mi);
            search_->setYaw(imu_yaw_);
        }

        if (cone_found_ && cone_id_ == target_id_) {
            last_valid_cone_time_ = this->get_clock()->now();
            cone_locked_ = true;

            if (state_ == State::kSearchPattern) {
                RCLCPP_INFO(get_logger(),
                    "[CALLBACK][FOLLOW] Correct cone detected id=%d dist=%.2f → ConeFollowing",
                    cone_id_, cone_x_);
                state_ = State::kConeFollowing;
            }
        }
    }

    void gpsCB(const sensor_msgs::msg::NavSatFix::SharedPtr) {}

    void autoCB(const std_msgs::msg::Bool::SharedPtr m) {
        autonomous_ = m->data;

        if (autonomous_) {
            if (state_ == State::kManualState) {
                RCLCPP_INFO(get_logger(), "[SYSTEM] Autonomous ON → NavigationModeSelect");
                state_ = State::kNavigationModeSelect;
            }
        } else {
            RCLCPP_INFO(get_logger(), "[SYSTEM] Autonomous OFF → Manual");
            geometry_msgs::msg::Twist stop;
            cmd_pub_->publish(stop);
            state_ = State::kManualState;
        }
    }

    void obsCB(const std_msgs::msg::Bool::SharedPtr m) {
        obstacle_ = m->data;
        if (obstacle_) {
            prev_state_ = state_;
            state_ = State::kObstacleAvoidance;
        } else if (state_ == State::kObstacleAvoidance) {
            state_ = prev_state_;
        }
    }

    // ----------------------------------------------------------------------
    // MAIN STATE MACHINE
    // ----------------------------------------------------------------------

    void run() {
        if (state_ == State::kManualState) {
            geometry_msgs::msg::Twist stop;
            cmd_pub_->publish(stop);
            return;
        }

        switch (state_) {
            case State::kNavigationModeSelect: navSelect(); break;
            case State::kSearchPattern:        searchRun(); break;
            case State::kConeFollowing:        followCone(); break;
            case State::kObstacleAvoidance:    avoid(); break;
            case State::kObjectDelivery:       deliver(); break;
            default: break;
        }
    }

    // ----------------------------------------------------------------------
    // STATE: NAV SELECT
    // ----------------------------------------------------------------------

    void navSelect() {
        int mode;
        std::cout << "Enter 0 for GPS, 1 for Cone: ";
        std::cin >> mode;

        if (mode == 1) {
            int color;
            std::cout << "Color ID (1-5): ";
            std::cin >> color;

            target_id_ = color;
            search_->setTargetId(color);

            RCLCPP_INFO(get_logger(), "[NAV] Cone mode selected target_id=%d", target_id_);
            state_ = State::kSearchPattern;
            return;
        }

        RCLCPP_WARN(get_logger(), "[NAV] GPS not implemented");
    }

    // ----------------------------------------------------------------------
    // STATE: SEARCHING
    // ----------------------------------------------------------------------

    void searchRun() {
        search_->setYaw(imu_yaw_);
        search_->run();
    }

    // ----------------------------------------------------------------------
    // STATE: FOLLOWING  (FULLY FIXED)
    // ----------------------------------------------------------------------

    void followCone() {
        std::lock_guard<std::mutex> lk(mtx_);

        auto now = this->get_clock()->now();
        double dt = (now - last_valid_cone_time_).seconds();

        // long-term lost
        if (dt > cone_lost_timeout_) {
            RCLCPP_WARN(get_logger(),
                "[FOLLOW] Cone lost for %.2f sec → SearchPattern", dt);

            cone_locked_ = false;
            state_ = State::kSearchPattern;
            return;
        }

        geometry_msgs::msg::Twist v;

        // ---- LINEAR CONTROL ----
        if (filtered_cone_x_ >= follow_vx_close_cut_)
            v.linear.x = follow_vx_max_;
        else
            v.linear.x = std::max(follow_vx_min_, follow_vx_max_ * (filtered_cone_x_ / follow_vx_close_cut_));

        // ---- ANGULAR CONTROL (SMOOTHED, CLAMPED, NO OSCILLATION) ----
        double wz = filtered_cone_y_ * ang_gain_;
        if (wz > ang_max_clamp_)  wz = ang_max_clamp_;
        if (wz < -ang_max_clamp_) wz = -ang_max_clamp_;
        v.angular.z = wz;

        RCLCPP_INFO(get_logger(),
            "[FOLLOW] vx=%.2f wz=%.2f dist=%.2f offset=%.2f lock=%.2f",
            v.linear.x, v.angular.z, filtered_cone_x_, filtered_cone_y_, dt);

        cmd_pub_->publish(v);

        // target reached
        if (filtered_cone_x_ <= 2.0) {
            RCLCPP_INFO(get_logger(), "[FOLLOW] TargetReached → DELIVERY");
            geometry_msgs::msg::Twist stop;
            cmd_pub_->publish(stop);
            state_ = State::kObjectDelivery;
        }
    }

    // ----------------------------------------------------------------------
    // OTHER STATES
    // ----------------------------------------------------------------------

    void avoid() {
        geometry_msgs::msg::Twist stop;
        cmd_pub_->publish(stop);
    }

    void deliver() {
        if (cone_id_ != target_id_) {
            RCLCPP_WARN(get_logger(), "[DELIVERY] Cone mismatch → SearchPattern");
            state_ = State::kSearchPattern;
            return;
        }

        std_msgs::msg::String m;
        m.data = "P10Q20A30";
        arm_pub_->publish(m);

        if (toggle_client_->service_is_ready()) {
            auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
            toggle_client_->async_send_request(req);
        }

        cone_found_ = false;
        cone_id_ = -1;
        cone_x_ = 100;
        cone_y_ = 100;

        target_id_ = -1;
        cone_locked_ = false;

        state_ = State::kManualState;
    }

    // ----------------------------------------------------------------------
    // MEMBERS
    // ----------------------------------------------------------------------

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr arm_pub_;

    rclcpp::Subscription<custom_msgs::msg::ImuData>::SharedPtr imu_sub_;
    rclcpp::Subscription<custom_msgs::msg::MarkerTag>::SharedPtr marker_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr auto_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obs_sub_;

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr toggle_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<SearchPattern> search_;

    std::mutex mtx_;

    State state_;
    State prev_state_;

    bool cone_found_;
    int cone_id_;
    float cone_x_;
    float cone_y_;

    // filtered values
    double filtered_cone_x_;
    double filtered_cone_y_;

    double imu_yaw_;
    bool autonomous_;
    bool obstacle_;

    int target_id_;

    double obj_lin_[2];
    double obj_ang_[2];
    double kMaxLin_;
    double kMaxAng_;

    // follow state
    bool cone_locked_;
    rclcpp::Time last_valid_cone_time_;
    double cone_lost_timeout_;

    // follow control tuning
    double follow_vx_max_;
    double follow_vx_min_;
    double follow_vx_close_cut_;
    double ang_gain_;
    double ang_max_clamp_;
    double offset_filter_alpha_;
};

} // namespace rover_stack
