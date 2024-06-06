#include "map/process.hpp"

#include <decision_interface/msg/game_status.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/int32.hpp>

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include <memory>

class DecisionInterfaceNode : public rclcpp::Node {
public:
    DecisionInterfaceNode();

private:
    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>> grid_map_publisher_;
    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>> cost_map_publisher_;
    std::shared_ptr<rclcpp::Publisher<decision_interface::msg::GameStatus>> status_publisher_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> cloud_publisher_;

    std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Pose2D>> velocity_subscription_;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Int32>> rotation_subscription_;
    std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Vector3>> gimbal_subscription_;

    std::shared_ptr<rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>> livox_subscription_;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>> pointcloud_subscription_;

    std::shared_ptr<Process> process_;

private:
    virtual void velocity_subscription_callback(const std::unique_ptr<geometry_msgs::msg::Pose2D>& msg);
    virtual void rotation_subscription_callback(const std::unique_ptr<std_msgs::msg::Int32>& msg);
    virtual void gimbal_subscription_callback(const std::unique_ptr<geometry_msgs::msg::Vector3>& msg);
    void pointcloud_process(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& pointcloud, const std_msgs::msg::Header& header);
    void livox_subscription_callback(const std::unique_ptr<livox_ros_driver2::msg::CustomMsg>& msg);
    void pointcloud2_subscription_callback(const std::unique_ptr<sensor_msgs::msg::PointCloud2>& msg);
};