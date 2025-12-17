
#include "rclcpp/rclcpp.hpp"
#include "stack/irc_planner.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<planner::SensorCallback>());
    rclcpp::shutdown();
    return 0;
}
