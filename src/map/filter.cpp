#include "map/filter.hpp"
#include "map/node.hpp"
#include "ros2/param.hpp"

#include <opencv2/opencv.hpp>

void filter::handle(type::NodeMap& node_map)
{
    auto mat = cv::Mat(
        static_cast<int>(node_map.width()),
        static_cast<int>(node_map.length()),
        CV_8UC1);
    auto element = cv::Mat();

    for (const auto node : *node_map)
        mat.at<int8_t>(node.x, node.y) = node.value;

    element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(6, 6), cv::Point(-1, -1));
    cv::morphologyEx(mat, mat, cv::MORPH_CLOSE, element, cv::Point(-1, -1), 1);

    element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3), cv::Point(-1, -1));
    cv::dilate(mat, mat, element, cv::Point(-1, -1), 6);

    cv::blur(mat, mat, cv::Size(3, 3));

    element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5), cv::Point(-1, -1));
    cv::morphologyEx(mat, mat, cv::MORPH_GRADIENT, element, cv::Point(-1, -1), 1);

    static const auto threshold = param::get<int>("filter.threshold");
    mat.forEach<int8_t>([](int8_t& pixel, const int* position) {
        pixel = pixel > threshold ? -1 : 0;
    });

    for (auto& node : *node_map) {
        node.value = mat.at<int8_t>(node.x, node.y);
    }
}
