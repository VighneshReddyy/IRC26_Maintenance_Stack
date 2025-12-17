#ifndef IRC_PLANNER_H_
#define IRC_PLANNER_H_

#include <rclcpp/rclcpp.hpp>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <string>
#include <iterator>
#include <algorithm>
#include <map>
#include <functional> 
#include <mutex>
#include <chrono>
#include <memory>


#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "custom_msgs/msg/marker_tag.hpp"
#include "custom_msgs/msg/imu_data.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

namespace planner
{
    // Static Variable Definitions

    // Rover Dimensions and Physical Properties

    const double kRoverLength = 1.50;
    const double kRoverBreadth = 1.25;

    // Rover Speed & Obstacle constraints and parameters

    const double kMaxLinearVel = 0.6;
    const double kMinLinearVel = 0;

    const double kMaxAngularVel = 0.85;
    const double kMinAngularVel = 0;

    const double kMaxObsThreshold = 3.0;
    const double kMinObsThreshold = 0.5;
    const double kMinYObsThreshold = 0;

    const double kMaxXObsDistThreshold = 2;
    const double kMinXObsDistThreshold = 1;
    const double kMaxYObjDistThreshold = 2;
    const double kMinYObjDistThreshold = 0;

    const double kStopVel = 0;

    // Goal Distance Threshold

    const double kDistanceThreshold = 2;

    // Enum for Rover state
    // Class defines different states of the rover during the misson including:
    // 1. Manual Mode
    // 2. Selecct Navigation Mode (0/1 : GPS/Cone)
    // 3. Search Pattern
    // 4. Object Following
    // 5. Obstacle Avoidance
    // 6. Object Delivery

    enum State
    {
        kManualState,
        kNavigationModeSelect,
        kCoordinateFollowing,
        kSearchPattern,
        kConeFollowing,
        kObstacleAvoidance,
        kObjectDelivery
    };


    enum SearchPatternType
    {
        kMoveForward,
        kTurnRight,
        kTurnLeft,
        kOffsetTurn,
        kResetHeading
    };

    // Struct for Latitude and Longitude

    typedef struct Coordinates
    {
        double latitude;
        double longitude;
        
    } Coordinates;

  class SensorCallback : public rclcpp::Node
{
public:SensorCallback(): Node("planner_node"),
current_orientation(0.0),
dest_orientation(0.0),
curr_time(0.0),
temp_time(0.0),
curr_location{0.0, 0.0},
nav_mode(-1),
nav_selected(false),
cone_id_selected(false),
goal_location{0.0, 0.0},
gps_goal_set(false),
cone_detect(false),
gps_goal_reached(false),
cone_goal_reached(false),
obstacle_detect(false),
rover_state(false),
cloud(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>()),
last_rover_state(false),
CurrState(kManualState),
PrevState(kManualState)


{
        declare_parameter("imu_topic", "/imu_data");
        declare_parameter("gps_topic", "/fix");
        declare_parameter("cone_topic", "/marker_detect");
        declare_parameter("point_cloud_topic", "/local_grid_obstacle");
        declare_parameter("cmd_vel_topic", "/cmd_vel");
        declare_parameter("arm_topic", "/arm_vel");
        declare_parameter("state_topic", "/autonomous_mode_state");
        declare_parameter("target_cone_id", 1);

        const auto imu_topic   = get_parameter("imu_topic").as_string();
        const auto gps_topic   = get_parameter("gps_topic").as_string();
        const auto cone_topic  = get_parameter("cone_topic").as_string();
        const auto cloud_topic = get_parameter("point_cloud_topic").as_string();
        const auto cmd_vel     = get_parameter("cmd_vel_topic").as_string();
        const auto arm_topic   = get_parameter("arm_topic").as_string();
        const auto state_topic = get_parameter("state_topic").as_string();
        target_cone_id_ = get_parameter("target_cone_id").as_int();

        // Initializing Publishers

        vel_pub   = create_publisher<geometry_msgs::msg::Twist>(cmd_vel, 10);
        arm_pub   = create_publisher<std_msgs::msg::String>(arm_topic, 10);

        // Initializing Subscribers

        imu_sub_ = create_subscription<custom_msgs::msg::ImuData>(
            imu_topic, 10,
            std::bind(&SensorCallback::imuCallback, this, std::placeholders::_1));

        gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
            gps_topic, 10,
            std::bind(&SensorCallback::gpsCallback, this, std::placeholders::_1));

        cone_sub_ = create_subscription<custom_msgs::msg::MarkerTag>(
            cone_topic, 10,
            std::bind(&SensorCallback::coneCallback, this, std::placeholders::_1));

        subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic, 10,
            std::bind(&SensorCallback::pclCallback, this, std::placeholders::_1));

        auto_sub_ = create_subscription<std_msgs::msg::Bool>(
            state_topic, 10,
            std::bind(&SensorCallback::stateCallback, this, std::placeholders::_1));

        // Initializing Services

        toggle_client_ = create_client<std_srvs::srv::Trigger>("/toggle_autonomous");

        // timer for stackRun()
        stack_timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&SensorCallback::stackRun, this));

        //  Formation of Equations

        obs_avoid_linear  = straightLineEquation(
            kMinObsThreshold, kStopVel,
            kMaxObsThreshold, kMaxLinearVel);

        obs_avoid_angular = straightLineEquation(
            kRoverBreadth / 2, kStopVel,
            kMinYObsThreshold, kMaxAngularVel);

        obj_follow_linear = straightLineEquation(
            kMaxXObsDistThreshold, kMaxLinearVel,
            kMinXObsDistThreshold, kStopVel);

        obj_follow_angular = straightLineEquation(
            kMinYObjDistThreshold, kStopVel,
            kMaxYObjDistThreshold, kMaxAngularVel);
    }

private:
    // ROS Variables
       
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr arm_pub;

    // Subscribers
    rclcpp::Subscription<custom_msgs::msg::ImuData>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<custom_msgs::msg::MarkerTag>::SharedPtr cone_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr auto_sub_;

    // Service Client
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr toggle_client_;

    rclcpp::TimerBase::SharedPtr stack_timer_;

    // Equation Constants

    std::vector<double> obs_avoid_linear;
    std::vector<double> obs_avoid_angular;
    std::vector<double> obj_follow_linear;
    std::vector<double> obj_follow_angular;

    // Velocity publisher variable

    geometry_msgs::msg::Twist velocity;

    // Sensor Data Variables

    double cone_x = 100, cone_y = 100, obs_x = 100, obs_y = 100;
    double current_orientation, dest_orientation;
    double curr_time, temp_time;
    Coordinates curr_location;
    int target_cone_id_;
    int nav_mode = -1;        // 0 = GPS, 1 = Cone
    bool nav_selected = false;
    bool cone_id_selected = false;
    Coordinates goal_location;
    bool gps_goal_set = false;

    // Truthtable select lines

    bool cone_detect = false;
    bool gps_goal_reached = false;
    bool cone_goal_reached = false;

    bool obstacle_detect = false;
    bool rover_state = false;

    int search_pattern_function = 0;
    bool search_pattern_flag = false;
    double initial_heading = 0.0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    
    std::mutex state_mutex_;


    bool last_rover_state = false;
    rclcpp::Time last_gps_time_;


    // Rover state variables

    State CurrState;
    State PrevState;
    SearchPatternType FollowPattern = kMoveForward;

    // Returns a enum of rover state implementing the turth table below:
    // +========+=======+============+=======+===================+
    // | Arrow  | Cone  | Obstacle   | Goal  | State             |
    // +========+=======+============+=======+===================+
    // | 0      | 0     | 0          | 0     | Search Pattern    |
    // | 1      | 0     | 0          | 0     | Arrow Following   |
    // | 0/1    | 1     | 0          | 0     | Cone Following    |
    // | 0/1    | 0/1   | 1          | 0     | Obstacle Avoiding |
    // | 0/1    | 0/1   | 0/1        | 1     | Data Analysis     |
    // +--------+-------+------------+-------+-------------------+

    void RoverStateClassifier();

    // IMU Callback
    void imuCallback(const custom_msgs::msg::ImuData::SharedPtr imu_msg);

    // GPS Callback
    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr fix_);

    // PCL Callback
    void pclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);

    // Cone Callback
    void coneCallback(const custom_msgs::msg::MarkerTag::SharedPtr cone);

    // State Callback
    void stateCallback(const std_msgs::msg::Bool::SharedPtr state);

    void stackRun();
    void objectDelivery();
    void navigationModeSelect();
    void publishVel(const geometry_msgs::msg::Twist& msg);


    // Calling Search Pattern following function from controller class
    void callSearchPattern(SearchPatternType search_pattern_type);

    // Function to publish velocity for Obstacle Avoidance
    void obstacleAvoidance();

    // Function to publish velocity to Follow Objects
    void objectFollowing();

    // Function to publish velocity to follow GPS Coordinates
    void coordinateFollowing();

    // Set Search Pattern Flag
    void setSearchPatternFlag(bool a);

    // Get Search Pattern Flag
    bool getSearchPatternFlag();

    void hardStop();
    void disableAutonomous();



    std::vector<double> straightLineEquation(double x1, double y1, double x2, double y2);

    // Setters & Getters
    void setConeStatus(bool status);
    void setGPSStatus(bool status);
    void setGoalStatus();
    State getState(bool current);
    bool getObstacleStatus();
    bool getGPSStatus();
    SearchPatternType getSearchPatternType();
    void setSearchPatternType(SearchPatternType patternType);

    // Call State Classifier function
    void callStateClassifier();

    // Returns the distance between two coordniates on the Earth
    double haversine(Coordinates curr, Coordinates dest);

    // Returns the bearing of the rover between two coordinates on the Earth
    double coordinateBearing();

    // Returns the least angle to travel between two IMU angles
    double bearing(double target, double current);

    void obstacleClassifier();

    // Setting goal coordiantes for the task

    double normalize360(double angle);
};


    // Function Definitons for State Classifier Class

   void SensorCallback::RoverStateClassifier()
{
    // Navigation mode selection is exclusive
    if (CurrState == kNavigationModeSelect)
        return;

    // Highest priority: delivery
    if (cone_goal_reached)
    {
        CurrState = kObjectDelivery;
        return;
    }

    // Obstacle handling with recovery
    if (obstacle_detect)
    {
        if (CurrState != kObstacleAvoidance)
            PrevState = CurrState;

        CurrState = kObstacleAvoidance;
        return;
    }

    // Obstacle cleared → return to previous state
    if (CurrState == kObstacleAvoidance && !obstacle_detect)
    {
        CurrState = PrevState;
        return;
    }

    // Cone navigation
    if (nav_mode == 1 && cone_detect)
    {
        CurrState = kConeFollowing;
        return;
    }

    // GPS navigation
    if (nav_mode == 0 && gps_goal_set && !gps_goal_reached)
    {
        CurrState = kCoordinateFollowing;
        return;
    }

    // Default fallback
    CurrState = kSearchPattern;
}





    // Function Definitions for Sensor Interpreter Class

    void SensorCallback::setConeStatus(bool status)
    {
        cone_detect = status;
    }

    void SensorCallback::setGoalStatus()
{
    static rclcpp::Time close_since;

    if (nav_mode != 1 || !cone_detect || cone_goal_reached)
    {
        close_since = rclcpp::Time(0);
        return;
    }

    if (cone_x <= kDistanceThreshold)
    {
        if (close_since.nanoseconds() == 0)
            close_since = this->get_clock()->now();

        if ((this->get_clock()->now() - close_since).seconds() > 0.5)
        {
            cone_goal_reached = true;

            RCLCPP_INFO(
                this->get_logger(),
                "[CONE] Goal reached — entering delivery state");
        }
    }
    else
    {
        close_since = rclcpp::Time(0);
    }
}






    State SensorCallback::getState(bool curr)
    {
        if (curr == true)
            return CurrState;
        else
            return PrevState;
    }
    bool SensorCallback::getObstacleStatus()
    {
        return obstacle_detect;
    }

    SearchPatternType SensorCallback::getSearchPatternType()
    {
        return FollowPattern;
    }
    void SensorCallback::setSearchPatternType(SearchPatternType state)
    {
        FollowPattern = state;
    }

    void SensorCallback::callStateClassifier()
    {
        RoverStateClassifier();
    }

    double SensorCallback::haversine(Coordinates curr, Coordinates dest)
    {
        double dLat = dest.latitude - curr.latitude;
        double dLon = dest.longitude - curr.longitude;

        double a = sin(dLat/2)*sin(dLat/2) +
                cos(curr.latitude)*cos(dest.latitude) *
                sin(dLon/2)*sin(dLon/2);

        double c = 2 * atan2(sqrt(a), sqrt(1-a));
        return 6371000.0 * c;
    }

void SensorCallback::obstacleClassifier()
{
    obstacle_detect = false;
    obs_x = kMaxObsThreshold + 1.0;
    obs_y = 0.0;

    if (!cloud || cloud->points.empty())
        return;

    const float x_min = 0.5;
    const float x_max = kMaxObsThreshold;
    const float y_min = -0.4;
    const float y_max =  0.4;

    float closest_x = x_max;

    for (const auto& point : cloud->points)
    {
        if (point.x < x_min || point.x > x_max)
            continue;
        if (point.y < y_min || point.y > y_max)
            continue;

        if (cone_detect)
        {
            if (std::abs(point.x - cone_x) < 0.3 &&
                std::abs(point.y - cone_y) < 0.3)
                continue;
        }

        if (point.x < closest_x)
        {
            closest_x = point.x;
            obs_x = point.x;
            obs_y = point.y;
            obstacle_detect = true;
        }
    }
}




    // Function Definitions for Sensor Callback Class

    void SensorCallback::imuCallback(const custom_msgs::msg::ImuData::SharedPtr imu_msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_orientation = normalize360(imu_msg->orientation.z);
    }




    void SensorCallback::gpsCallback(
    const sensor_msgs::msg::NavSatFix::SharedPtr fix_)
    {
        if (fix_->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX)
            return;

        curr_location.latitude  = fix_->latitude  * M_PI / 180.0;
        curr_location.longitude = fix_->longitude * M_PI / 180.0;

        last_gps_time_ = this->get_clock()->now();
    }


    void SensorCallback::pclCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{
    if (!rover_state)
        return;

    std::lock_guard<std::mutex> lock(state_mutex_);

    pcl::PointCloud<pcl::PointXYZ>::Ptr temp(
        new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*cloud_msg, *temp);
    cloud.swap(temp);
    obstacleClassifier();
}





void SensorCallback::coneCallback(
    const custom_msgs::msg::MarkerTag::SharedPtr cone)
{
    if (!rover_state)
        return;

    std::lock_guard<std::mutex> lock(state_mutex_);

    if (cone->is_found && cone->id == target_cone_id_)
    {
        cone_detect = true;
        cone_x = cone->x;
        cone_y = cone->y;
    }
    else
    {
        cone_detect = false;
        cone_x = 100.0;
        cone_y = 100.0;
    }
}




void SensorCallback::stateCallback(
    const std_msgs::msg::Bool::SharedPtr state)
{

    // ---------- NO CHANGE ----------
    if (state->data == last_rover_state)
        return;

    last_rover_state = state->data;
    rover_state      = state->data;

    // ======================================================
    // MANUAL MODE
    // ======================================================
    if (!rover_state)
    {
        CurrState = kManualState;
        PrevState = kManualState;

        // Reset task-level goals (Step-9 fix)
        cone_goal_reached = false;
        gps_goal_reached  = false;

        cone_detect     = false;
        obstacle_detect = false;

        nav_mode        = -1;
        nav_selected    = false;
        gps_goal_set    = false;
        cone_id_selected = false;

        FollowPattern = kMoveForward;

        velocity.linear.x  = 0.0;
        velocity.angular.z = 0.0;
        publishVel(velocity);

        RCLCPP_WARN(this->get_logger(), "[MODE] MANUAL MODE");
        return;
    }

    // ======================================================
    // AUTONOMOUS MODE ENABLED
    // ======================================================
    RCLCPP_INFO(this->get_logger(), "[MODE] AUTONOMOUS MODE");

    // Reset navigation selection
    nav_mode         = -1;
    nav_selected     = false;
    gps_goal_set     = false;
    cone_id_selected = false;

    // Reset task-level goals (VERY IMPORTANT)
    cone_goal_reached = false;
    gps_goal_reached  = false;

    initial_heading = current_orientation;
    FollowPattern   = kMoveForward;

    CurrState = kNavigationModeSelect;
    PrevState = kNavigationModeSelect;
}



   void SensorCallback::stackRun()
{
    if (!rover_state)
        return;

    if (CurrState == kNavigationModeSelect)
    {
        navigationModeSelect();
        return;
    }

    setGoalStatus();
    callStateClassifier();

    switch (CurrState)
    {
        case kSearchPattern:
            callSearchPattern(FollowPattern);
            break;

        case kConeFollowing:
            objectFollowing();
            break;

        case kObstacleAvoidance:
            obstacleAvoidance();
            break;

        case kCoordinateFollowing:
            coordinateFollowing();
            break;

        case kObjectDelivery:
            objectDelivery();
            break;

        default:
            publishVel(geometry_msgs::msg::Twist());
            break;
    }
}



    // Function Definitons for Equation Generator Class
    inline void SensorCallback::publishVel(const geometry_msgs::msg::Twist& msg)
    {

        geometry_msgs::msg::Twist cmd = msg;
        cmd.linear.x  = std::clamp(cmd.linear.x,  0.0, kMaxLinearVel);
        cmd.angular.z = std::clamp(cmd.angular.z, -kMaxAngularVel, kMaxAngularVel);

        vel_pub->publish(cmd);
    }


   
    std::vector<double> SensorCallback::straightLineEquation(double x1, double y1,double x2, double y2)
{
    std::vector<double> eq(2);
    if (std::abs(x2 - x1) < 1e-6)
    {
        eq[0] = 0.0;
        eq[1] = y1;
        return eq;
    }

    double m = (y2 - y1) / (x2 - x1);
    double c = y1 - m * x1;

    eq[0] = m;
    eq[1] = c;
    return eq;
}


    // Function Definitons for Search Pattern Class
void SensorCallback::callSearchPattern(SearchPatternType /*unused*/)
{
    static bool search_init = false;
    static rclcpp::Time end_time;
    static double dest_heading = 0.0;
    static int offset_counter = 0;
    static double time_offset = 3.0;

    geometry_msgs::msg::Twist cmd;
    auto now = this->get_clock()->now();

    double yaw;
    bool obstacle;
    bool cone;

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        yaw = current_orientation;
        obstacle = obstacle_detect;
        cone = cone_detect;
    }

    // ---------- EXIT / RESET CONDITIONS ----------
    if (CurrState != kSearchPattern)
    {
        search_init = false;
        return;
    }

    if (obstacle || cone)
    {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "[SEARCH] Interrupted (obstacle=%d cone=%d)",
            obstacle, cone);

        search_init = false;
        publishVel(geometry_msgs::msg::Twist());
        return;
    }

    // ---------- ENTRY INITIALIZATION ----------
    if (!search_init)
    {
        search_init = true;
        offset_counter = 0;
        time_offset = 3.0;

        end_time = now + rclcpp::Duration::from_seconds(time_offset);
        FollowPattern = kMoveForward;

        RCLCPP_INFO(
            this->get_logger(),
            "[SEARCH] Init | yaw=%.2f | forward_time=%.1f",
            yaw, time_offset);
        return;
    }

    // ---------- FSM ----------
    switch (FollowPattern)
    {
        case kMoveForward:
        {
            if (now < end_time)
            {
                cmd.linear.x = kMaxLinearVel;
            }
            else
            {
                dest_heading = normalize360(yaw - 90.0);
                FollowPattern = kTurnRight;

                RCLCPP_INFO(
                    this->get_logger(),
                    "[SEARCH] -> TURN_RIGHT | target=%.2f",
                    dest_heading);
            }
            break;
        }

        case kTurnRight:
        {
            double err = bearing(dest_heading, yaw);

            RCLCPP_DEBUG_THROTTLE(
                this->get_logger(), *this->get_clock(), 500,
                "[SEARCH] TURN_RIGHT | err=%.2f",
                err);

            if (std::abs(err) > 5.0)
            {
                cmd.angular.z = std::clamp(
                    err * 0.02,
                    -kMaxAngularVel,
                     kMaxAngularVel);
            }
            else
            {
                dest_heading = normalize360(yaw + 180.0);
                FollowPattern = kTurnLeft;

                RCLCPP_INFO(
                    this->get_logger(),
                    "[SEARCH] -> TURN_LEFT | target=%.2f",
                    dest_heading);
            }
            break;
        }

        case kTurnLeft:
        {
            double err = bearing(dest_heading, yaw);

            RCLCPP_DEBUG_THROTTLE(
                this->get_logger(), *this->get_clock(), 500,
                "[SEARCH] TURN_LEFT | err=%.2f",
                err);

            if (std::abs(err) > 5.0)
            {
                cmd.angular.z = std::clamp(
                    err * 0.02,
                    -kMaxAngularVel,
                     kMaxAngularVel);
            }
            else
            {
                dest_heading = normalize360(yaw + 15.0);
                FollowPattern = kOffsetTurn;

                RCLCPP_INFO(
                    this->get_logger(),
                    "[SEARCH] -> OFFSET_TURN | target=%.2f",
                    dest_heading);
            }
            break;
        }

        case kOffsetTurn:
        {
            double err = bearing(dest_heading, yaw);

            RCLCPP_DEBUG_THROTTLE(
                this->get_logger(), *this->get_clock(), 500,
                "[SEARCH] OFFSET_TURN | err=%.2f count=%d",
                err, offset_counter);

            if (std::abs(err) > 5.0)
            {
                cmd.angular.z = std::clamp(err * 0.02, -0.4, 0.4);
            }
            else
            {
                offset_counter++;
                time_offset = std::min(3.0 + offset_counter * 2.0, 7.0);
                FollowPattern = kResetHeading;

                RCLCPP_INFO(
                    this->get_logger(),
                    "[SEARCH] -> RESET | next_forward=%.1f",
                    time_offset);
            }
            break;
        }

        case kResetHeading:
        {
            end_time = now + rclcpp::Duration::from_seconds(time_offset);
            FollowPattern = kMoveForward;

            RCLCPP_INFO(
                this->get_logger(),
                "[SEARCH] Reset complete | forward_time=%.1f",
                time_offset);
            break;
        }
    }

    publishVel(cmd);
}


void SensorCallback::obstacleAvoidance()
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    velocity.linear.x =
        std::clamp(obs_avoid_linear[0] * obs_x + obs_avoid_linear[1],
                   0.0, kMaxLinearVel);

    if (std::abs(obs_y) < 0.05)
    {
        velocity.angular.z = kMaxAngularVel;
    }
    else if (obs_y > 0)
    {
        velocity.angular.z = -kMaxAngularVel;
    }
    else
    {
        velocity.angular.z = kMaxAngularVel;
    }

    publishVel(velocity);
}


   
void SensorCallback::objectFollowing()
{
    static rclcpp::Time last_seen;
    static bool was_tracking = false;

    auto now = this->get_clock()->now();

    // ---------- CONE VISIBILITY ----------
    if (cone_detect)
    {
        last_seen = now;

        if (!was_tracking)
        {
            RCLCPP_INFO(this->get_logger(),
                "[CONE] Tracking started | x=%.2f y=%.2f",
                cone_x, cone_y);
            was_tracking = true;
        }
    }

    // ---------- CONE LOST TIMEOUT ----------
    if ((now - last_seen).seconds() > 1.0)
    {
        if (was_tracking)
        {
            RCLCPP_WARN(this->get_logger(),
                "[CONE] Lost for %.2f s → returning to SEARCH",
                (now - last_seen).seconds());
        }

        was_tracking = false;
        CurrState = kSearchPattern;
        return;
    }

    // ---------- LINEAR VELOCITY ----------
    velocity.linear.x =
        std::clamp(obj_follow_linear[0] * cone_x + obj_follow_linear[1],
                   0.0, kMaxLinearVel);

    // ---------- ANGULAR VELOCITY ----------
    if (std::abs(cone_y) < 0.02)
    {
        velocity.angular.z = 0.0;
    }
    else if (cone_y > 0)
    {
        // Cone on LEFT → turn LEFT (positive yaw)
        velocity.angular.z = std::min(
            obj_follow_angular[0] * std::abs(cone_y) + obj_follow_angular[1],
            kMaxAngularVel);
    }
    else
    {
        // Cone on RIGHT → turn RIGHT (negative yaw)
        velocity.angular.z = -std::min(
            obj_follow_angular[0] * std::abs(cone_y) + obj_follow_angular[1],
            kMaxAngularVel);
    }

    // ---------- DEBUG TRACKING ----------
    RCLCPP_DEBUG_THROTTLE(
        this->get_logger(), *this->get_clock(), 300,
        "[CONE] x=%.2f y=%.2f | lin=%.2f ang=%.2f",
        cone_x, cone_y,
        velocity.linear.x, velocity.angular.z);

    publishVel(velocity);
}




   void SensorCallback::navigationModeSelect()
{
    if (!nav_selected)
    {
        std::cout << "\nSelect Navigation Mode:\n";
        std::cout << "0 -> GPS\n";
        std::cout << "1 -> Cone\n";
        std::cout << "Enter choice: ";
        std::cin >> nav_mode;

        if (nav_mode == 0)
        {
            std::cout << "[INPUT] GPS MODE selected\n";
            nav_selected = true;
            return;
        }
        else if (nav_mode == 1)
        {
            std::cout << "[INPUT] CONE MODE selected\n";
            nav_selected = true;
            return;
        }
        else
        {
            std::cout << "Invalid input\n";
            nav_mode = -1;
            return;
        }
    }

    if (nav_mode == 1 && !cone_id_selected)
    {
        std::cout << "Enter target cone ID: ";
        std::cin >> target_cone_id_;
        cone_id_selected = true;

        CurrState = kSearchPattern;
        PrevState = kSearchPattern;
        return;
    }

    if (nav_mode == 0 && !gps_goal_set)
    {
        std::cout << "Enter goal latitude (deg): ";
        std::cin >> goal_location.latitude;
        std::cout << "Enter goal longitude (deg): ";
        std::cin >> goal_location.longitude;

        goal_location.latitude  *= M_PI / 180;
        goal_location.longitude *= M_PI / 180;

        gps_goal_set = true;
        CurrState = kCoordinateFollowing;
        PrevState = kCoordinateFollowing;
        return;
    }
}


void SensorCallback::objectDelivery()
{
    static bool delivery_started = false;
    static rclcpp::Time start_time;

    if (!delivery_started)
    {
        delivery_started = true;
        start_time = this->get_clock()->now();

        std_msgs::msg::String msg;
        msg.data = "DROP";
        arm_pub->publish(msg);

        RCLCPP_WARN(this->get_logger(),
            "[DELIVERY] Drop command issued");
        return;
    }

    if ((this->get_clock()->now() - start_time).seconds() < 5.0)
        return;

    std_msgs::msg::String msg;
    msg.data = "STOP";
    arm_pub->publish(msg);

    hardStop();

    // Correct system-level mode switch
    disableAutonomous();

    delivery_started = false;

    RCLCPP_WARN(this->get_logger(),
        "[MISSION] Delivery complete — switching to MANUAL MODE");
}


double SensorCallback::coordinateBearing()
{
    double y = sin(goal_location.longitude - curr_location.longitude) * cos(goal_location.latitude);
    double x = cos(curr_location.latitude) * sin(goal_location.latitude) -
               sin(curr_location.latitude) * cos(goal_location.latitude) *
               cos(goal_location.longitude - curr_location.longitude);

    double brng = atan2(y, x) * 180.0 / M_PI;
    return normalize360(brng);
}


double SensorCallback::bearing(double target, double current)
{
    target = normalize360(target);
    current = normalize360(current);

    double diff = target - current;

    if (diff > 180.0) diff -= 360.0;
    if (diff < -180.0) diff += 360.0;

    return diff;   // range [-180, +180]
}



void SensorCallback::coordinateFollowing()
{
    auto now = this->get_clock()->now();

    if ((now - last_gps_time_).seconds() > 1.5)
    {
        publishVel(geometry_msgs::msg::Twist());
        return;
    }

    double dist = haversine(curr_location, goal_location);

    if (dist <= kDistanceThreshold)
    {
        gps_goal_reached = true;
        hardStop();
        rover_state = false;
        CurrState = kManualState;
        PrevState = kManualState;
        return;
    }

    double yaw;
    {
        yaw = current_orientation;
    }

    double err = bearing(coordinateBearing(), yaw);

    velocity.linear.x =std::clamp(kMaxLinearVel * (dist / 5.0),0.15, kMaxLinearVel);
    velocity.angular.z =std::clamp(err * 0.02,-kMaxAngularVel,kMaxAngularVel);
    publishVel(velocity);
}


void SensorCallback::hardStop()
{
    geometry_msgs::msg::Twist stop;
    stop.linear.x  = 0.0;
    stop.linear.y  = 0.0;
    stop.linear.z  = 0.0;
    stop.angular.x = 0.0;
    stop.angular.y = 0.0;
    stop.angular.z = 0.0;

    for (int i = 0; i < 5; ++i)
    {
        vel_pub->publish(stop);
        rclcpp::sleep_for(std::chrono::milliseconds(20));
    }
}

inline double SensorCallback::normalize360(double angle)
{
    while (angle < 0.0)
        angle += 360.0;
    while (angle >= 360.0)
        angle -= 360.0;
    return angle;
}

inline bool SensorCallback::getGPSStatus()
{
    return gps_goal_reached;
}

inline void SensorCallback::setGPSStatus(bool status)
{
    gps_goal_reached = status;
}

void SensorCallback::disableAutonomous()
{
    if (!toggle_client_->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_ERROR(this->get_logger(),
            "[MODE] toggle_autonomous service not available");
        return;
    }

    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();

    toggle_client_->async_send_request(
        req,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
        {
            auto res = future.get();
            if (res->success)
            {
                RCLCPP_WARN(this->get_logger(),
                    "[MODE] Autonomous DISABLED via service");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(),
                    "[MODE] Failed to disable autonomy: %s",
                    res->message.c_str());
            }
        });
}


} // namespace planner

#endif
