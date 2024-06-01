#include "filter.hpp"
#include "../ros2/param.hpp"
#include "node.hpp"

#include <opencv2/opencv.hpp>

void filter::handle(type::NodeMap& data)
{
    auto mat = cv::Mat(int(param::process::grid_number), int(param::process::grid_number), CV_8UC1);

    for (const auto node : *data)
        mat.at<int8_t>(node.x, node.y) = node.value;

    auto element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3), cv::Point(-1, -1));
    cv::dilate(mat, mat, element, cv::Point(-1, -1), 6);

    element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(-1, -1));
    cv::morphologyEx(mat, mat, cv::MORPH_CLOSE, element, cv::Point(-1, -1), 1);

    for (auto& node : *data)
        node.value = mat.at<int8_t>(node.x, node.y);
}
