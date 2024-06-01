#pragma once

#include <decision_interface/msg/game_status.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/int32.hpp>

#include <rclcpp/node.hpp>

namespace param {
void load(rclcpp::Node& node);

namespace name {
    // public
    inline std::string node_name;
    inline std::string grid_topic_name;
    inline std::string cost_topic_name;
    inline std::string status_topic_name;
    // subscribe
    inline std::string velocity_topic_name;
    inline std::string rotation_topic_name;
    inline std::string gimbal_topic_name;
} // namespace name

namespace process {
    // generate
    inline bool cost_map;
    // lidar
    inline int livox_frames;
    // grid config
    inline double resolution;
    inline double grid_width;
    inline size_t grid_number;
    inline double lidar_blind;
    inline int expand_size;
    inline int discrete_point;
    inline double z_weight;
    // transform
    inline double transform_translation_x;
    inline double transform_translation_y;
    inline double transform_translation_z;
    inline double transform_quaternion_x;
    inline double transform_quaternion_y;
    inline double transform_quaternion_z;
    inline double transform_quaternion_w;
    // filter
    inline double ground_height;
} // namespace process
}; // namespace param
