#ifndef IRC_PLANNER_H_
#define IRC_PLANNER_H_

#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <vector>
#include <string>
#include <mutex>
#include <memory>

#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "custom_msgs/msg/marker_tag.hpp"
#include "custom_msgs/msg/imu_data.hpp"
#include "custom_msgs/msg/planner_status.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace planner
{
const double kRoverLength  = 1.50;
const double kRoverBreadth = 1.25;

const double kMaxLinearVel  = 1.0;   
const double kMinLinearVel  = 0.0;
const double kMaxAngularVel = 1.5;   

// Obstacle thresholds 
const double kMaxObsThreshold = 1.0;
const double kMinObsThreshold = 0.5;
const double kMinYObsThreshold = 0.3;

// Object (cone) following thresholds 
const double kMaxXObjThreshold = 0.9;   
const double kMinXObjThreshold = 0.25;  

const double kMaxYObjThreshold = 0.4;  
const double kMinYObjThreshold = 0.08;  
const double kStopVel = 0.0;
const double kDistanceThreshold = 1.0;


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
    kFaceForward,
    kMoveForward,
    kTurnRight,
    kTurnLeft,
    kOffsetTurn,
};

enum SearchSkew
{
    kNoSkew    = -1,
    kLeftSkew  = 0,
    kRightSkew = 1
};

struct Coordinates
{
    double latitude;
    double longitude;
};

class SensorCallback : public rclcpp::Node
{
public:
    SensorCallback();

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr arm_pub;
    rclcpp::Publisher<custom_msgs::msg::PlannerStatus>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr deliver_pub_;

    rclcpp::Subscription<custom_msgs::msg::ImuData>::SharedPtr imu_sub_;
    rclcpp::Subscription<custom_msgs::msg::ImuData>::SharedPtr external_imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<custom_msgs::msg::MarkerTag>::SharedPtr cone_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr auto_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr delivered_sub_;

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr toggle_client_;
    rclcpp::TimerBase::SharedPtr stack_timer_;

    State CurrState;
    State PrevState;
    SearchPatternType FollowPattern;

    bool gps_goal_set;
    bool cone_detect;
    bool gps_goal_reached;
    bool cone_goal_reached;
    bool obstacle_detect;
    bool rover_state;
    bool last_rover_state;
    bool gps_aligned_;
    bool delivery_requested_;
    bool delivery_done_;

    rclcpp::Time delivery_start_time_;
    rclcpp::Time last_cone_time_;
    rclcpp::Time last_gps_time_;
    rclcpp::Time nowHere;
    rclcpp::Time nextHere;

    int nav_mode;
    int target_cone_id_;
    bool nav_select_done_;

    double current_orientation;
    double cone_x;
    double cone_y;
    double obs_x;
    double obs_y;

    bool search_init_;
    bool search_timing_;
    bool search_ref_set_;
    bool search_aligned_;
    bool spot_turn_back_;

    rclcpp::Time search_end_time_;
    double search_base_heading_;
    double search_offset_deg_;
    double search_forward_time_;
    SearchSkew search_skew;

    Coordinates curr_location;
    Coordinates goal_location;

    double locked_bearing_deg_;
    bool bearing_locked_;
    double search_origin_heading_;
    bool skew_cycle_;


    double zed_yaw;
    double bno_yaw;

    std::vector<double> obs_avoid_linear;
    std::vector<double> obs_avoid_angular;
    std::vector<double> obj_follow_linear;
    std::vector<double> obj_follow_angular;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

    std::mutex state_mutex_;

    void stackRun();
    void RoverStateClassifier();

    void imuCallback(const custom_msgs::msg::ImuData::SharedPtr msg);
    void externalImuCallback(const custom_msgs::msg::ImuData::SharedPtr msg);
    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr fix);
    void pclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);
    void coneCallback(const custom_msgs::msg::MarkerTag::SharedPtr cone);
    void stateCallback(const std_msgs::msg::Bool::SharedPtr state);
    void deliveredCallback(const std_msgs::msg::Bool::SharedPtr msg);

    void coordinateFollowing();
    void obstacleAvoidance();
    void objectFollowing();
    void callSearchPattern();
    void objectDelivery();
    void navigationModeSelect();

    void publishVel(const geometry_msgs::msg::Twist& msg);
    void hardStop();
    void disableAutonomous();
    void obstacleClassifier();
    void setGoalStatus();
    void setSearchSkew(int skew);
    void resetSearchPattern();
    bool isConeFresh();

    std::vector<double> straightLineEquation(double x1, double y1, double x2, double y2);

    double haversine(Coordinates curr, Coordinates dest);
    double gpsBearing(Coordinates curr, Coordinates dest);
    double gpsAngleFix(double angle);
    double headingError(double target, double current);
    double normalize360(double angle);
};

} // namespace planner

#endif
