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
    auto info =
        "Topic Names\n"
        + param::get<std::string>("name.grid") + "\n"
        + param::get<std::string>("name.cost") + "\n"
        + param::get<std::string>("name.status") + "\n"
        + param::get<std::string>("name.transformed_map") + "\n"
        + param::get<std::string>("name.control.velocity") + "\n"
        + param::get<std::string>("name.control.rotation") + "\n"
        + param::get<std::string>("name.control.gimbal");
    RCLCPP_INFO(get_logger(), "%s", info.c_str());

    grid_map_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(param::get<std::string>("name.grid"), 10);
    cost_map_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(param::get<std::string>("name.cost"), 10);
    status_publisher_   = create_publisher<decision_interface::msg::GameStatus>(param::get<std::string>("name.status"), 10);
    cloud_publisher_    = create_publisher<sensor_msgs::msg::PointCloud2>(param::get<std::string>("name.transformed_map"), 10);

    velocity_subscription_ = create_subscription<geometry_msgs::msg::Pose2D>(
        param::get<std::string>("name.control.velocity"), 10, [this](const std::unique_ptr<geometry_msgs::msg::Pose2D>& msg) {
            velocity_subscription_callback(msg);
        });
    rotation_subscription_ = create_subscription<std_msgs::msg::Int32>(
        param::get<std::string>("name.control.rotation"), 10, [this](const std::unique_ptr<std_msgs::msg::Int32>& msg) {
            rotation_subscription_callback(msg);
        });
    gimbal_subscription_ = create_subscription<geometry_msgs::msg::Vector3>(
        param::get<std::string>("name.control.gimbal"), 10, [this](const std::unique_ptr<geometry_msgs::msg::Vector3>& msg) {
            gimbal_subscription_callback(msg);
        });

    auto pointcloud_type = param::get<std::string>("switch.pointcloud_type");
    if (pointcloud_type == "livox") {
        livox_subscription_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
            param::get<std::string>("name.lidar"), 10, [this](const std::unique_ptr<livox_ros_driver2::msg::CustomMsg>& msg) {
                livox_subscription_callback(msg);
            });
    } else if (pointcloud_type == "pointcloud2") {
        pointcloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            param::get<std::string>("name.lidar"), 10, [this](const std::unique_ptr<sensor_msgs::msg::PointCloud2>& msg) {
                pointcloud2_subscription_callback(msg);
            });
    }

    process_ = std::make_shared<Process>();

    process_->grid_width_    = param::get<float>("grid.grid_width");
    process_->resolution_    = param::get<float>("grid.resolution");
    process_->lidar_blind_   = param::get<float>("grid.lidar_blind");
    process_->height_wight_  = param::get<float>("grid.height_wight");
    process_->ground_height_ = param::get<float>("grid.ground_height");
    process_->grid_number_   = static_cast<int>(param::get<float>("grid.grid_width") / param::get<float>("grid.resolution"));
}

void DecisionInterfaceNode::velocity_subscription_callback(const std::unique_ptr<geometry_msgs::msg::Pose2D>& msg) { }
void DecisionInterfaceNode::rotation_subscription_callback(const std::unique_ptr<std_msgs::msg::Int32>& msg) { }
void DecisionInterfaceNode::gimbal_subscription_callback(const std::unique_ptr<geometry_msgs::msg::Vector3>& msg) { }

void DecisionInterfaceNode::pointcloud_process(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& pointcloud, const std_msgs::msg::Header& header)
{
    static auto publish_transformed_cloud = param::get<bool>("switch.publish_transformed_cloud");
    static auto map_frame_id              = param::get<std::string>("name.frame.map");

    // transform pointcloud
    auto q = Eigen::AngleAxisd { std::numbers::pi, Eigen::Vector3d::UnitY() };
    auto t = Eigen::Translation3d { 0, 0, -0.6 };
    pcl::transformPointCloud(*pointcloud, *pointcloud, Eigen::Affine3d { q * t });

    // make publishable pointcloud
    auto pointcloud2 = std::make_shared<sensor_msgs::msg::PointCloud2>();
    ros2::convert::pcl_to_pc2(*pointcloud, *pointcloud2);

    pointcloud2->header.frame_id = map_frame_id;
    pointcloud2->header.stamp    = header.stamp;

    if (publish_transformed_cloud)
        cloud_publisher_->publish(*pointcloud2);

    // generate grid map
    auto grid_map = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    auto node_map = process_->generate_node_map(pointcloud);
    ros2::convert::node_to_grid_map(*node_map, *grid_map);

    grid_map->header.frame_id = map_frame_id;
    grid_map->header.stamp    = header.stamp;
    grid_map->info.height     = process_->grid_number_;
    grid_map->info.width      = process_->grid_number_;
    grid_map->info.resolution = process_->resolution_;

    grid_map->info.origin.position.x = -process_->grid_width_ / 2.0;
    grid_map->info.origin.position.y = -process_->grid_width_ / 2.0;

    grid_map_publisher_->publish(*grid_map);
}

void DecisionInterfaceNode::livox_subscription_callback(const std::unique_ptr<livox_ros_driver2::msg::CustomMsg>& msg)
{
    // make standard pointcloud
    auto pointcloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    ros2::convert::livox_to_pcl(msg->points, *pointcloud);

    // process
    pointcloud_process(pointcloud, msg->header);
}

void DecisionInterfaceNode::pointcloud2_subscription_callback(const std::unique_ptr<sensor_msgs::msg::PointCloud2>& msg)
{
    // make standard pointcloud
    auto pointcloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    ros2::convert::pc2_to_pcl(*msg, *pointcloud);

    // process
    pointcloud_process(pointcloud, msg->header);
}
