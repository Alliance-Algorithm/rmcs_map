#include "ros2/node.hpp"
#include "ros2/convert.hpp"
#include "ros2/param.hpp"

#include <pcl/common/transforms.h>
#include <rclcpp/logging.hpp>

#include <memory>
#include <string>

DecisionInterfaceNode::DecisionInterfaceNode()
    : Node(param::get<std::string>("name.node"))
{
    auto info = "Topic Names\n"
              + param::get<std::string>("name.grid") + "\n"
              + param::get<std::string>("name.cost") + "\n"
              + param::get<std::string>("name.status") + "\n"
              + param::get<std::string>("name.transformed_map") + "\n"
              + param::get<std::string>("name.control.velocity") + "\n"
              + param::get<std::string>("name.control.rotation") + "\n"
              + param::get<std::string>("name.control.gimbal");

    RCLCPP_INFO(get_logger(), "%s", info.c_str());

    grid_map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(param::get<std::string>("name.grid"), 10);
    cost_map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(param::get<std::string>("name.cost"), 10);
    status_publisher_   = this->create_publisher<decision_interface::msg::GameStatus>(param::get<std::string>("name.status"), 10);
    cloud_publisher_    = this->create_publisher<sensor_msgs::msg::PointCloud2>(param::get<std::string>("name.transformed_map"), 10);

    velocity_subscription_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
        param::get<std::string>("name.control.velocity"), 10, [this](const std::unique_ptr<geometry_msgs::msg::Pose2D>& msg) {
            velocity_subscription_callback(msg);
        });
    rotation_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
        param::get<std::string>("name.control.rotation"), 10, [this](const std::unique_ptr<std_msgs::msg::Int32>& msg) {
            rotation_subscription_callback(msg);
        });
    gimbal_subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>(
        param::get<std::string>("name.control.gimbal"), 10, [this](const std::unique_ptr<geometry_msgs::msg::Vector3>& msg) {
            gimbal_subscription_callback(msg);
        });
    livox_subscription_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        param::get<std::string>("name.lidar"), 10, [this](const std::unique_ptr<livox_ros_driver2::msg::CustomMsg>& msg) {
            livox_subscription_callback(msg);
        });
}

void DecisionInterfaceNode::velocity_subscription_callback(const std::unique_ptr<geometry_msgs::msg::Pose2D>& msg) { }
void DecisionInterfaceNode::rotation_subscription_callback(const std::unique_ptr<std_msgs::msg::Int32>& msg) { }
void DecisionInterfaceNode::gimbal_subscription_callback(const std::unique_ptr<geometry_msgs::msg::Vector3>& msg) { }

void DecisionInterfaceNode::livox_subscription_callback(const std::unique_ptr<livox_ros_driver2::msg::CustomMsg>& msg)
{
    // make standard pointcloud
    auto pointcloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    ros2::convert::livox_to_pcl(msg->points, *pointcloud);

    // transform pointcloud
    auto q = Eigen::AngleAxisd { std::numbers::pi, Eigen::Vector3d::UnitY() };
    auto t = Eigen::Translation3d { 0, 0, -0.6 };
    pcl::transformPointCloud(*pointcloud, *pointcloud, Eigen::Affine3d { q * t });

    // make publishable pointcloud
    auto pointcloud2 = std::make_shared<sensor_msgs::msg::PointCloud2>();
    ros2::convert::pcl_to_pc2(*pointcloud, *pointcloud2);

    pointcloud2->header.stamp    = msg->header.stamp;
    pointcloud2->header.frame_id = param::get<std::string>("name.frame.map");

    // auto grid_map = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    // ros2::convert::node_to_grid_map(*process_->generate_node_map(pointcloud), *grid_map);

    // grid_map->header.frame_id = param::name::map_link_name;
    // grid_map->header.stamp    = msg->header.stamp;

    cloud_publisher_->publish(*pointcloud2);
}
