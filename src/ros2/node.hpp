#include "map/process.hpp"
#include "ros2/param.hpp"

#include <decision_interface/msg/detail/game_status__struct.hpp>
#include <geometry_msgs/msg/detail/vector3__struct.hpp>
#include <nav_msgs/msg/detail/occupancy_grid__struct.hpp>
#include <std_msgs/msg/detail/int32__struct.hpp>

#include <rclcpp/node.hpp>

#include <memory>

class DecisionInterfaceNode : public rclcpp::Node {
public:
    DecisionInterfaceNode()
        : Node("decision_interface", rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true))
    {
        param::load(*this);
    }

private:
    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>> grid_map_publisher;
    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>> cost_map_publisher;
    std::shared_ptr<rclcpp::Publisher<decision_interface::msg::GameStatus>> status_publisher;

    std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Pose2D>> velocity_subscription;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Int32>> rotation_subscription;
    std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Vector3>> gimbal_subscription;

    std::shared_ptr<Process> process_;
};