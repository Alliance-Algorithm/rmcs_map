#include "ros2/node.hpp"
#include <rclcpp/executors.hpp>

int main(int argc, const char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DecisionInterfaceNode>());
    rclcpp::shutdown();
}