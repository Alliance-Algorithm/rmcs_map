#include "map/process.hpp"
#include "map/filter.hpp"
#include "map/node.hpp"
#include "ros2/param.hpp"

#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <rclcpp/logging.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <type_traits>

template <typename RangeType, typename ValueType>
bool in_range(const ValueType& val, const RangeType& min, const RangeType& max)
{
    return (val > min && val < max);
}

Process::Process() = default;

void Process::info(const std::string& string) const
{
    if (enable_info_)
        RCLCPP_INFO(rclcpp::get_logger(param::name::node_name), "%s", string.c_str());
}

std::unique_ptr<type::NodeMap> Process::generate_grid_map(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& pointcloud)
{
    using PointType = std::remove_cvref<decltype((*pointcloud)[0])>::type;

    auto filter = pcl::PassThrough<PointType> {};
    filter.setInputCloud(pointcloud);
    filter.setFilterFieldName("x");
    filter.setFilterLimits(-float(param::process::grid_width / 2.0), float(param::process::grid_width / 2.0));
    filter.filter(*pointcloud);
    filter.setFilterFieldName("y");
    filter.setFilterLimits(-float(param::process::grid_width / 2.0), float(param::process::grid_width / 2.0));
    filter.filter(*pointcloud);
    filter.setFilterFieldName("z");
    filter.setFilterLimits(float(param::process::ground_height), 2233);
    filter.filter(*pointcloud);

    auto map = std::make_unique<type::NodeMap>(param::process::grid_number, param::process::grid_number);

    for (const auto& point : *pointcloud) {
        auto x = int(point.x + (param::process::grid_width / 2.0) / param::process::resolution);
        auto y = int(point.y + (param::process::grid_width / 2.0) / param::process::resolution);

        if (!in_range(x, size_t(0), size_t(param::process::grid_number)) || !in_range(y, size_t(0), size_t(param::process::grid_number)))
            continue;

        auto& node = (*map)(x, y);
        node.value = std::max(node.value, int8_t(point.z * param::process::z_weight));
    }

    filter::handle(*map);

    for (auto& node : **map) {
        if (node.value > param::process::ground_height) {
            node.type  = type::NodeType::BLOCK;
            node.value = -1;
        } else {
            node.type  = type::NodeType::AVAILABLE;
            node.value = 0;
        }
    }

    return map;
}

std::unique_ptr<type::NodeMap> Process::generate_cost_map(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& pointcloud)
{
    return nullptr;
}