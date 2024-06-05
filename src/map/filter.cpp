#include "filter.hpp"
#include "node.hpp"

#include <opencv2/opencv.hpp>

void filter::handle(type::NodeMap& node_map)
{
    auto mat = cv::Mat(int(node_map.width()), int(node_map.length()), CV_8UC1);

    for (const auto node : *node_map)
        mat.at<int8_t>(node.x, node.y) = node.value;

    auto element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3), cv::Point(-1, -1));
    cv::dilate(mat, mat, element, cv::Point(-1, -1), 6);

    element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(-1, -1));
    cv::morphologyEx(mat, mat, cv::MORPH_CLOSE, element, cv::Point(-1, -1), 1);

    for (auto& node : *node_map)
        node.value = mat.at<int8_t>(node.x, node.y);
}
