#include "map/process.hpp"
#include "map/filter.hpp"
#include "map/node.hpp"

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

Process::Process()
{
    info("map process has been constructed");
};

void Process::info(const std::string& string) const
{
    static auto logger = rclcpp::get_logger("map_process");

    if (enable_info_)
        RCLCPP_INFO(logger, "%s", string.c_str());
}

std::unique_ptr<type::NodeMap> Process::generate_node_map(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& pointcloud)
{
    using PointType = std::remove_cvref<decltype((*pointcloud)[0])>::type;

    auto filter = pcl::PassThrough<PointType> {};
    filter.setInputCloud(pointcloud);
    filter.setFilterFieldName("x");
    filter.setFilterLimits(-float(grid_width_ / 2.0), float(grid_width_ / 2.0));
    filter.filter(*pointcloud);
    filter.setFilterFieldName("y");
    filter.setFilterLimits(-float(grid_width_ / 2.0), float(grid_width_ / 2.0));
    filter.filter(*pointcloud);
    filter.setFilterFieldName("z");
    filter.setFilterLimits(float(ground_height_), 2233);
    filter.filter(*pointcloud);

    auto map = std::make_unique<type::NodeMap>(grid_number_, grid_number_);

    for (const auto& point : *pointcloud) {
        auto x = int((point.x + (grid_width_ / 2.0)) / resolution_);
        auto y = int((point.y + (grid_width_ / 2.0)) / resolution_);

        if (!in_range(x, size_t(0), size_t(grid_number_)) || !in_range(y, size_t(0), size_t(grid_number_)))
            continue;

        auto& node  = (*map)(x, y);
        node.height = std::max(node.height, point.z);
    }

    filter::handle(*map);

    // for (auto& node : **map) {
    //     if (node.value > 0) {
    //         node.type  = type::NodeType::BLOCK;
    //         node.value = -1;
    //     } else {
    //         node.type  = type::NodeType::AVAILABLE;
    //         node.value = 0;
    //     }
    // }

    return map;
}

std::unique_ptr<type::NodeMap> Process::generate_cost_map(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& pointcloud)
{
    return nullptr;
}