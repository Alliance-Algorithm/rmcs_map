#pragma once

#include "node.hpp"

#include <Eigen/Eigen>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class Process {
public:
    Process();

    void info(const bool& flag) noexcept { enable_info_ = flag; }
    void info(const std::string& string) const;

    static std::unique_ptr<type::NodeMap> generate_grid_map(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& pointcloud);
    static std::unique_ptr<type::NodeMap> generate_cost_map(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& pointcloud);

private:
    bool enable_info_ = false;
};
