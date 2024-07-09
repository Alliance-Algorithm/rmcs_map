#include "ros2/node.hpp"
#include "ros2/convert.hpp"
#include "ros2/param.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/common/transforms.h>
#include <rclcpp/logging.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>

#include <memory>
#include <string>

MapNode::MapNode()
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
    RCLCPP_INFO(this->get_logger(), "%s", info.c_str());

    grid_map_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(param::get<std::string>("name.grid"), 10);
    cost_map_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(param::get<std::string>("name.cost"), 10);
    status_publisher_   = create_publisher<rmcs_map::msg::GameStatus>(param::get<std::string>("name.status"), 10);
    cloud_publisher_    = create_publisher<sensor_msgs::msg::PointCloud2>(param::get<std::string>("name.transformed_map"), 10);

    auto pointcloud_type = param::get<std::string>("switch.pointcloud_type");
    RCLCPP_INFO(this->get_logger(), "pointcloud type: %s", pointcloud_type.c_str());
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

    static_transform_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    publish_static_transform();
}

void MapNode::publish_static_transform()
{
    auto transform_stamp = geometry_msgs::msg::TransformStamped();

    auto quaternion = Eigen::Quaterniond {
        Eigen::AngleAxisd { std::numbers::pi, Eigen::Vector3d::UnitY() }
    };

    transform_stamp.transform.rotation.w = quaternion.w();
    transform_stamp.transform.rotation.x = quaternion.x();
    transform_stamp.transform.rotation.y = quaternion.y();
    transform_stamp.transform.rotation.z = quaternion.z();

    transform_stamp.transform.translation.z = 0.6;

    transform_stamp.header.stamp    = this->get_clock()->now();
    transform_stamp.header.frame_id = param::get<std::string>("name.frame.map");
    transform_stamp.child_frame_id  = param::get<std::string>("name.frame.lidar");
    static_transform_broadcaster_->sendTransform(transform_stamp);
}

void MapNode::publish_status(const rmcs_map::msg::GameStatus& status)
{
    status_publisher_->publish(status);
}

void MapNode::pointcloud_process(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& pointcloud, const std_msgs::msg::Header& header)
{
    static auto publish_transformed_cloud = param::get<bool>("switch.publish_transformed_cloud");
    static auto map_frame_id              = param::get<std::string>("name.frame.map");

    auto q = Eigen::AngleAxisd { std::numbers::pi, Eigen::Vector3d::UnitY() };
    auto t = Eigen::Translation3d { 0, 0, -0.6 };

    pcl::transformPointCloud(*pointcloud, *pointcloud, Eigen::Affine3d { q * t });

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

    // make publishable pointcloud
    auto pointcloud2 = std::make_shared<sensor_msgs::msg::PointCloud2>();
    ros2::convert::pcl_to_pc2(*pointcloud, *pointcloud2);

    pointcloud2->header.frame_id = map_frame_id;
    pointcloud2->header.stamp    = header.stamp;

    if (publish_transformed_cloud)
        cloud_publisher_->publish(*pointcloud2);
}

void MapNode::livox_subscription_callback(const std::unique_ptr<livox_ros_driver2::msg::CustomMsg>& msg)
{
    // make standard pointcloud
    auto pointcloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    ros2::convert::livox_to_pcl(msg->points, *pointcloud);

    // process
    pointcloud_process(pointcloud, msg->header);
}

void MapNode::pointcloud2_subscription_callback(const std::unique_ptr<sensor_msgs::msg::PointCloud2>& msg)
{
    // make standard pointcloud
    auto pointcloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    ros2::convert::pc2_to_pcl(*msg, *pointcloud);

    // process
    pointcloud_process(pointcloud, msg->header);
}