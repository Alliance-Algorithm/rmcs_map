#include "param.hpp"

void param::load(rclcpp::Node& node)
{
    using namespace param::process;
    using namespace param::name;

    // names
    node.get_parameter("names.node", node_name);
    node.get_parameter("names.map.grid", grid_topic_name);
    node.get_parameter("names.map.cost", cost_topic_name);
    node.get_parameter("names.status", status_topic_name);
    node.get_parameter("names.control.velocity", velocity_topic_name);
    node.get_parameter("names.control.rotation", rotation_topic_name);
    node.get_parameter("names.control.gimbal", gimbal_topic_name);
    // process config
    node.get_parameter("generate.cost_map", cost_map);
    node.get_parameter("lidar.livox_frames", livox_frames);
    node.get_parameter("transform.translation.x", transform_translation_x);
    node.get_parameter("transform.translation.y", transform_translation_y);
    node.get_parameter("transform.translation.z", transform_translation_z);
    node.get_parameter("transform.quaternion.x", transform_quaternion_x);
    node.get_parameter("transform.quaternion.y", transform_quaternion_y);
    node.get_parameter("transform.quaternion.z", transform_quaternion_z);
    node.get_parameter("grid.resolution", resolution);
    node.get_parameter("grid.grid_width", grid_width);
    node.get_parameter("grid.lidar_blind", lidar_blind);
    node.get_parameter("grid.z_wight", z_weight);
    node.get_parameter("filter.ground_height", ground_height);
};