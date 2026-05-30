/**
 * @file ring.cpp
 * @brief 圆环检测实现
 */

#include "find_line_lib/ring.h"
#include "find_line_lib/get_start_point.h"

#include <tuple>
#include <vector>
#include <algorithm>
#include <cmath>
#include <opencv2/opencv.hpp>

namespace find_line_lib {

// ==================== 静态缓冲区 ====================
// 用于存储扫描过程中的中间结果，避免重复分配内存
static std::vector<std::tuple<Point, Point>> line_length_buffer;
static std::vector<std::tuple<Point, Point>> line_length_points_buffer;
static std::tuple<Point, Point> ring_result_buffer;

// ==================== 发现圆环 ====================
std::tuple<Point, Point>* discover_ring(const uint8_t* bin_img, int width, int height, int split) {
    int h = height;
    int w = width;

    // 计算中间区域的范围
    float split_height = static_cast<float>(h) / split;
    int start_row = static_cast<int>(split_height * (split / 2));
    int end_row = static_cast<int>(split_height * (split / 2 + split % 2));

    int min_width = w;
    Point min_width_point(0, 0);
    Point min_width_point2(0, 0);

    // 在中间区域扫描每一行，找最窄的左右边界
    for (int row = start_row; row < end_row; ++row) {
        // 从图像中心 (w/2, row) 开始水平扫描
        Point start_pt(w / 2, row);
        auto result = get_start_point(bin_img, w, h,
            &start_pt,
            1, w - 1, 1, h - 1, "horizontal");

        if (result != nullptr) {
            Point lp, rp;
            std::tie(lp, rp) = *result;
            // 如果这行比当前最窄的还窄，更新结果
            if (rp.x - lp.x < min_width) {
                min_width = rp.x - lp.x;
                min_width_point = lp;
                min_width_point2 = rp;
            }
        }
    }

    // 如果没找到任何边界点，返回空
    if (min_width_point.x == 0 && min_width_point.y == 0) {
        return nullptr;
    }

    ring_result_buffer = std::make_tuple(min_width_point, min_width_point2);
    return &ring_result_buffer;
}

// ==================== 准备入环 ====================
std::tuple<Point, Point>* prepare_enter_ring(const uint8_t* bin_img, int width, int height,
                                              StatusSwitcher* ss,
                                              const std::tuple<Point, Point>* start_point,
                                              int jump_threshold,
                                              int scan_start_height,
                                              int scan_row_step,
                                              int center_distance_threshold) {
    int h = height;
    int w = width;

    // 清空缓冲区
    line_length_buffer.clear();
    line_length_points_buffer.clear();

    // 从上往下扫描，记录每行的左右边界和宽度
    for (int row = scan_start_height; row < h - 1; row += scan_row_step) {
        Point start_pt(w / 2, row);
        auto result = get_start_point(bin_img, w, h,
            &start_pt,
            1, w - 1, 1, h - 1, "horizontal");

        if (result != nullptr) {
            Point lp, rp;
            std::tie(lp, rp) = *result;
            line_length_buffer.push_back(std::make_tuple(lp, rp));
            line_length_points_buffer.push_back(*result);
        }
    }

    // 检测宽度跳变
    for (size_t i = 1; i < line_length_buffer.size(); ++i) {
        Point lp_prev, rp_prev;
        std::tie(lp_prev, rp_prev) = line_length_buffer[i - 1];

        Point lp_curr, rp_curr;
        std::tie(lp_curr, rp_curr) = line_length_buffer[i];

        int prev_width = rp_prev.x - lp_prev.x;
        int curr_width = rp_curr.x - lp_curr.x;

        // 检测到宽度突然增加
        if (curr_width - prev_width > jump_threshold) {
            // 计算前后两行的中心点
            Point prev_center((lp_prev.x + rp_prev.x) / 2, (lp_prev.y + rp_prev.y) / 2);
            Point curr_center((lp_curr.x + rp_curr.x) / 2, (lp_curr.y + rp_curr.y) / 2);

#ifdef SMTC2GO_DEBUG
            LOG_DEBUG("线长变化：%d", curr_width - prev_width);
            LOG_DEBUG("突变前中点: (%d, %d), 突变后中点: (%d, %d)",
                      prev_center.x, prev_center.y, curr_center.x, curr_center.y);
#endif

            // 根据中心点偏移方向判断左右
            if (prev_center.x - curr_center.x > center_distance_threshold) {
                // 中心点左移，说明车在向右转，入口在左边（左环）
                if (ss->ring_type == RingType::Left) {
                    return &line_length_points_buffer[i - 1];
                } else {
#ifdef SMTC2GO_DEBUG
                    LOG_WARN("实际检测到的圆坏类型是左，不是%d", static_cast<int>(ss->ring_type));
#endif
                    return &line_length_points_buffer[i - 1];
                }
            } else if (prev_center.x - curr_center.x < -center_distance_threshold) {
                // 中心点右移，说明车在向左转，入口在右边（右环）
                if (ss->ring_type == RingType::Right) {
                    return &line_length_points_buffer[i - 1];
                } else {
#ifdef SMTC2GO_DEBUG
                    LOG_WARN("实际检测到的圆坏类型是右，不是%d", static_cast<int>(ss->ring_type));
#endif
                    return &line_length_points_buffer[i - 1];
                }
            } else {
#ifdef SMTC2GO_DEBUG
                LOG_DEBUG("突变过小，可能是十字");
#endif
                return nullptr;
            }
        }
    }

#ifdef SMTC2GO_DEBUG
    LOG_DEBUG("线长变化不满足跳变阈值");
#endif
    return nullptr;
}

// ==================== 准备出环 ====================
std::tuple<Point, Point>* prepare_exit_ring(const uint8_t* bin_img, int width, int height,
                                             StatusSwitcher* ss,
                                             const std::tuple<Point, Point>* start_point,
                                             int jump_threshold,
                                             int scan_stop_height,
                                             int scan_row_step,
                                             int center_distance_threshold) {
    int h = height;
    int w = width;

    line_length_buffer.clear();
    line_length_points_buffer.clear();

    // 从下往上扫描（row 递减，scan_row_step 为负数）
    for (int row = h - 1; row > scan_stop_height; row += scan_row_step) {
        Point start_pt(w / 2, row);
        auto result = get_start_point(bin_img, w, h,
            &start_pt,
            1, w - 1, 1, h - 1, "horizontal");

        if (result != nullptr) {
            Point lp, rp;
            std::tie(lp, rp) = *result;
            line_length_buffer.push_back(std::make_tuple(lp, rp));
            line_length_points_buffer.push_back(*result);
        }
    }

    // 检测宽度跳变
    for (size_t i = 1; i < line_length_buffer.size(); ++i) {
        Point lp_prev, rp_prev;
        std::tie(lp_prev, rp_prev) = line_length_buffer[i - 1];

        Point lp_curr, rp_curr;
        std::tie(lp_curr, rp_curr) = line_length_buffer[i];

        int prev_width = rp_prev.x - lp_prev.x;
        int curr_width = rp_curr.x - lp_curr.x;

        if (curr_width - prev_width > jump_threshold) {
            Point prev_center((lp_prev.x + rp_prev.x) / 2, (lp_prev.y + rp_prev.y) / 2);
            Point curr_center((lp_curr.x + rp_curr.x) / 2, (lp_curr.y + rp_curr.y) / 2);

#ifdef SMTC2GO_DEBUG
            LOG_DEBUG("线长变化：%d", curr_width - prev_width);
            LOG_DEBUG("突变前中点: (%d, %d), 突变后中点: (%d, %d)",
                      prev_center.x, prev_center.y, curr_center.x, curr_center.y);

            cv::Mat debug_img(height, width, CV_8UC1, const_cast<uint8_t*>(bin_img));
            cv::line(debug_img, cv::Point(prev_center.x, prev_center.y),
                     cv::Point(curr_center.x, curr_center.y), cv::Scalar(0), 2);
#ifdef SMTC2GO_DEBUG_IMSHOW
            cv::imshow("debug", debug_img);
            cv::waitKey(0);
#endif
#endif

            // 根据中心点偏移方向判断出口位置
            if (prev_center.x - curr_center.x > center_distance_threshold) {
                // 左环：返回 i 的右点 和 start_point 的左点
                if (ss->ring_type == RingType::Left) {
                    return &line_length_points_buffer[i];
                } else {
#ifdef SMTC2GO_DEBUG
                    LOG_WARN("实际检测到的圆坏类型是右，不是%d", static_cast<int>(ss->ring_type));
#endif
                    return &line_length_points_buffer[i];
                }
            } else if (prev_center.x - curr_center.x < -center_distance_threshold) {
                // 右环：返回 i 的左点 和 start_point 的右点
                if (ss->ring_type == RingType::Right) {
                    return &line_length_points_buffer[i];
                } else {
#ifdef SMTC2GO_DEBUG
                    LOG_WARN("实际检测到的圆坏类型是左，不是%d", static_cast<int>(ss->ring_type));
#endif
                    return &line_length_points_buffer[i];
                }
            } else {
#ifdef SMTC2GO_DEBUG
                LOG_DEBUG("突变过小，可能是十字");
#endif
                return nullptr;
            }
        }
    }

    return nullptr;
}

}