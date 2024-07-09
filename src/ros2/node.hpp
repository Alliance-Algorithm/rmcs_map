#pragma once

#include "map/process.hpp"

#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rmcs_map/msg/game_status.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/int32.hpp>

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <memory>

class MapNode : public rclcpp::Node {
public:
    MapNode();

    void publish_status(const rmcs_map::msg::GameStatus&);

private:
    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>> grid_map_publisher_;
    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>> cost_map_publisher_;
    std::shared_ptr<rclcpp::Publisher<rmcs_map::msg::GameStatus>> status_publisher_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> cloud_publisher_;

    std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Pose2D>> velocity_subscription_;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Int32>> rotation_subscription_;
    std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Vector3>> gimbal_subscription_;

    std::shared_ptr<rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>> livox_subscription_;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>> pointcloud_subscription_;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_transform_broadcaster_;

    std::shared_ptr<Process> process_;

private:
    void publish_static_transform();
    void pointcloud_process(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& pointcloud, const std_msgs::msg::Header& header);
    void livox_subscription_callback(const std::unique_ptr<livox_ros_driver2::msg::CustomMsg>& msg);
    void pointcloud2_subscription_callback(const std::unique_ptr<sensor_msgs::msg::PointCloud2>& msg);
};