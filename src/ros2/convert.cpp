#include "convert.hpp"
#include "../map/node.hpp"
#include "../ros2/param.hpp"

#include <pcl_conversions/pcl_conversions.h>

void ros2::convert::livox_to_pcl(
    const std::vector<livox_ros_driver2::msg::CustomPoint>& livox,
    pcl::PointCloud<pcl::PointXYZ>& pcl)
{
    for (const auto point : livox) {
        pcl.points.emplace_back(point.x, point.y, point.z);
    }
}

void ros2::convert::pc2_to_pcl(const sensor_msgs::msg::PointCloud2& pc2, pcl::PointCloud<pcl::PointXYZ>& pcl)
{
    pcl::fromROSMsg(pc2, pcl);
}

void ros2::convert::node_to_grid_map(type::NodeMap& node_map, nav_msgs::msg::OccupancyGrid& occupancy_map)
{
    occupancy_map.info.width             = param::process::grid_number;
    occupancy_map.info.height            = param::process::grid_number;
    occupancy_map.info.resolution        = float(param::process::resolution);
    occupancy_map.info.origin.position.x = -param::process::grid_width / 2;
    occupancy_map.info.origin.position.y = -param::process::grid_width / 2;
    occupancy_map.data                   = std::vector<int8_t>(param::process::grid_number * param::process::grid_number);

    for (const auto& node : *node_map)
        occupancy_map.data[node.x + node.y * param::process::grid_number] = node.value;
};
