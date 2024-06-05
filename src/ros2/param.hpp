#pragma once

#include <rclcpp/node.hpp>
#include <string>

namespace param {

template <typename T>
decltype(auto) get(const std::string& name)
{
    // static node for lazy constructing
    static auto node = rclcpp::Node {
        "param_server",
        rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true)
    };

    auto param = T {};
    node.get_parameter<T>(name, param);
    return param;
}

} // namespace param