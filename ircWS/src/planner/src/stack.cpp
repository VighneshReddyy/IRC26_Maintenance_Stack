#include "rclcpp/rclcpp.hpp"
#include "stack/stack.hpp"

using namespace rover_stack;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StackNode>());
    rclcpp::shutdown();
    return 0;
}
