#include "map/filter.hpp"
#include "map/node.hpp"
#include "ros2/param.hpp"
#include <fstream>

#include <opencv2/opencv.hpp>

static inline void make_gradient_map(cv::Mat& origin)
{
    static auto mat_gradient_x = origin.clone();
    static auto mat_gradient_y = origin.clone();

    static const auto rows_start = 0;
    static const auto rows_mid   = origin.cols / 2;
    static const auto rows_end   = origin.cols;

    static const auto cols_start = 0;
    static const auto cols_mid   = origin.rows / 2;
    static const auto cols_end   = origin.rows;

    auto left_up    = origin(cv::Range(rows_start, rows_mid), cv::Range(cols_start, cols_mid));
    auto left_down  = origin(cv::Range(rows_mid, rows_end), cv::Range(cols_start, cols_mid));
    auto right_up   = origin(cv::Range(rows_start, rows_mid), cv::Range(cols_mid, cols_end));
    auto right_down = origin(cv::Range(rows_mid, rows_end), cv::Range(cols_mid, cols_end));

    auto left_up_x    = mat_gradient_x(cv::Range(rows_start, rows_mid), cv::Range(cols_start, cols_mid));
    auto left_down_x  = mat_gradient_x(cv::Range(rows_mid, rows_end), cv::Range(cols_start, cols_mid));
    auto right_up_x   = mat_gradient_x(cv::Range(rows_start, rows_mid), cv::Range(cols_mid, cols_end));
    auto right_down_x = mat_gradient_x(cv::Range(rows_mid, rows_end), cv::Range(cols_mid, cols_end));

    auto left_up_y    = mat_gradient_y(cv::Range(rows_start, rows_mid), cv::Range(cols_start, cols_mid));
    auto left_down_y  = mat_gradient_y(cv::Range(rows_mid, rows_end), cv::Range(cols_start, cols_mid));
    auto right_up_y   = mat_gradient_y(cv::Range(rows_start, rows_mid), cv::Range(cols_mid, cols_end));
    auto right_down_y = mat_gradient_y(cv::Range(rows_mid, rows_end), cv::Range(cols_mid, cols_end));

    cv::Sobel(~left_up, left_up_x, CV_8U, 1, 0);
    cv::Sobel(~left_down, left_down_x, CV_8U, 1, 0);
    cv::Sobel(right_up, right_up_x, CV_8U, 1, 0);
    cv::Sobel(right_down, right_down_x, CV_8U, 1, 0);

    cv::Sobel(~left_up, left_up_y, CV_8U, 0, 1);
    cv::Sobel(~right_up, right_up_y, CV_8U, 0, 1);
    cv::Sobel(left_down, left_down_y, CV_8U, 0, 1);
    cv::Sobel(right_down, right_down_y, CV_8U, 0, 1);

    cv::addWeighted(mat_gradient_x, 0.5, mat_gradient_y, 0.5, 0, origin);
}

void filter::handle(type::NodeMap& node_map)
{
    auto mat = cv::Mat(static_cast<int>(node_map.width()), static_cast<int>(node_map.length()), CV_8UC1);

    auto element = cv::Mat();

    for (const auto node : *node_map)
        mat.at<int8_t>(node.x, node.y) = node.value;

    // dilate
    element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3), cv::Point(-1, -1));
    cv::dilate(mat, mat, element, cv::Point(-1, -1), 2);

    // close
    element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(6, 6), cv::Point(-1, -1));
    cv::morphologyEx(mat, mat, cv::MORPH_CLOSE, element, cv::Point(-1, -1), 2);

    // gradient
    make_gradient_map(mat);

    // dilate
    element = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::dilate(mat, mat, element);

    // range
    static const auto threshold = param::get<int>("filter.threshold");
    mat.forEach<uint8_t>([](uint8_t& pixel, const int* position) {
        pixel = static_cast<uint8_t>(pixel < threshold ? 0 : pixel);
        pixel = static_cast<uint8_t>(pixel > 100 ? 100 : pixel);
    });

    for (auto& node : *node_map) {
        node.value = mat.at<int8_t>(node.x, node.y);
    }
}
