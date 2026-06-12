#include "find_line_lib/common.h"
#include "find_line_lib/calculate_wheel_speeds.h"
#include "find_line_lib/skeleton.h"
#include "find_line_lib/get_start_point.h"

#include <opencv2/opencv.hpp>

#include <cmath>
#include <tuple>
#include <cstring>
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>

#ifdef SMTC2GO_DEBUG
#include <filesystem>
#include <string>
#endif

namespace find_line_lib {

// ==================== 匿名命名空间：辅助函数 ====================
namespace
{

#ifdef SMTC2GO_DEBUG
const char* ring_status_name(RingStatus status) {
    return status == RingStatus::NotFound ? "未发现" :
           status == RingStatus::Discovered ? "已发现" :
           status == RingStatus::PrepareEnter ? "准备入环" :
           status == RingStatus::PrepareExit ? "准备出环" :
           status == RingStatus::AboutToExit ? "即将出环" : "出环中";
}

const char* ring_type_name(RingType type) {
    return type == RingType::None ? "无" :
           type == RingType::Left ? "左" : "右";
}

void create_debug_directories() {
    std::filesystem::create_directories("debug_frames/未发现");
    std::filesystem::create_directories("debug_frames/发现圆坏");
    std::filesystem::create_directories("debug_frames/已发现");
    std::filesystem::create_directories("debug_frames/准备入环");
    std::filesystem::create_directories("debug_frames/准备出环");
    std::filesystem::create_directories("debug_frames/即将出环");
    std::filesystem::create_directories("debug_frames/出环中");
}

void save_debug_images(const char* status, int frame_number,
                       const cv::Mat& original, const cv::Mat& skeleton,
                       const cv::Mat& result, int width, int height) {
    char path[512];
    snprintf(path, sizeof(path), "debug_frames/%s/frame_%04d_原图.png", status, frame_number);
    cv::imwrite(path, original);
    snprintf(path, sizeof(path), "debug_frames/%s/frame_%04d_骨架.png", status, frame_number);
    cv::imwrite(path, skeleton);
    snprintf(path, sizeof(path), "debug_frames/%s/frame_%04d_结果.png", status, frame_number);
    cv::imwrite(path, result);
}
#endif
bool choose_legacy_target_endpoint(const SkeletonAnalysisResult& skel_result,
                                   RingStatus status,
                                   RingType type,
                                   int width, int height,
                                   cv::Point& target_pt) {
  
    if (skel_result.endpoint_points.empty()) return false;

    // ==========================================================
    // 1. 【核心拦截】：准备入环阶段 (PrepareEnter) -> 彻底防抖、防提前转
    // ==========================================================
    if (status == RingStatus::PrepareEnter) {
        double max_weight = -1.0;

        if (type == RingType::Left) {
            // 左环岛：追求 x 越小越好。(width - x) 越大。
            for (const auto& p : skel_result.endpoint_points) {
                int x = std::get<0>(p);
                int y = std::get<1>(p);
                
                double dx = static_cast<double>(width - x);
                // 【平方级非线性拉力】+ 压低纵向干扰
                double weight = (dx * dx) + 0.05 * (height - y); 
                
                if (weight > max_weight) { 
                    max_weight = weight; 
                    target_pt = cv::Point(x, y); 
                }
            }
        } else {
            // 右环岛：追求 x 越大越好。
            for (const auto& p : skel_result.endpoint_points) {
                int x = std::get<0>(p);
                int y = std::get<1>(p);
                
                double dx = static_cast<double>(x);
                // 【平方级非线性拉力】
                double weight = (dx * dx) + 0.05 * (height - y);
                
                if (weight > max_weight) { 
                    max_weight = weight; 
                    target_pt = cv::Point(x, y); 
                }
            }
        }
        return max_weight >= 0.0;
    }

    // ==========================================================
    // 2. 准备出环阶段 (PrepareExit)
    // ==========================================================
    if (status == RingStatus::PrepareExit) {
        double w_x = 0.7;
        double w_y = 0.8;
        double max_weight = -1.0;

        if (type == RingType::Left) {
            for (const auto& p : skel_result.endpoint_points) {
                int x = std::get<0>(p);
                int y = std::get<1>(p);
                double weight = w_x * (width - x) + w_y * (height - y);
                if (weight > max_weight) { 
                    max_weight = weight; 
                    target_pt = cv::Point(x, y); 
                }
            }
        } else {
            for (const auto& p : skel_result.endpoint_points) {
                int x = std::get<0>(p);
                int y = std::get<1>(p);
                double weight = w_x * x + w_y * (height - y);
                if (weight > max_weight) { 
                    max_weight = weight; 
                    target_pt = cv::Point(x, y); 
                }
            }
        }
        return max_weight >= 0.0;
    }
    // ==========================================================
    // 3. 常规巡线期 (NotFound / Discovered / Exiting) -> 寻最远点
    // ==========================================================
    int min_y = height;
    bool found = false;
    for (const auto& p : skel_result.endpoint_points) {
        int x = std::get<0>(p);
        int y = std::get<1>(p);
        if (y < min_y) { 
            min_y = y; 
            target_pt = cv::Point(x, y); 
            found = true;
        }
    }
    return found;
}

bool nearest_skeleton_point(const uint8_t* skeleton, int width, int height,
                            cv::Point seed, cv::Point& nearest) {
    int best_dist = std::numeric_limits<int>::max();
    bool found = false;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (skeleton[y * width + x] == 0) continue;
            int dx = x - seed.x;
            int dy = y - seed.y;
            int dist = dx * dx + dy * dy;
            if (dist < best_dist) { best_dist = dist; nearest = cv::Point(x, y); found = true; }
        }
    }
    return found;
}

bool find_skeleton_path(const uint8_t* skeleton, int width, int height,
                        cv::Point start, cv::Point target,
                        std::vector<cv::Point>& path) {
    path.clear();
    if (start.x < 0 || start.x >= width || start.y < 0 || start.y >= height ||
        target.x < 0 || target.x >= width || target.y < 0 || target.y >= height)
        return false;
    if (skeleton[start.y * width + start.x] == 0 || skeleton[target.y * width + target.x] == 0)
        return false;

    const int total = width * height;
    std::vector<int> parent(total, -1);
    std::vector<uint8_t> visited(total, 0);
    std::queue<int> q;

    int start_idx = start.y * width + start.x;
    int target_idx = target.y * width + target.x;
    visited[start_idx] = 1;
    q.push(start_idx);

    const int dirs[8][2] = {{-1,-1},{0,-1},{1,-1},{-1,0},{1,0},{-1,1},{0,1},{1,1}};

    while (!q.empty()) {
        int idx = q.front(); q.pop();
        if (idx == target_idx) break;
        int x = idx % width;
        int y = idx / width;
        for (auto& dir : dirs) {
            int nx = x + dir[0], ny = y + dir[1];
            if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
            int nidx = ny * width + nx;
            if (visited[nidx] || skeleton[nidx] == 0) continue;
            visited[nidx] = 1;
            parent[nidx] = idx;
            q.push(nidx);
        }
    }

    if (!visited[target_idx]) return false;
    for (int idx = target_idx; idx != -1; idx = parent[idx]) {
        path.push_back(cv::Point(idx % width, idx / width));
        if (idx == start_idx) break;
    }
    std::reverse(path.begin(), path.end());
    return !path.empty() && path.front() == start && path.back() == target;
}

double segment_length(cv::Point a, cv::Point b) {
    int dx = a.x - b.x, dy = a.y - b.y;
    return std::sqrt(static_cast<double>(dx * dx + dy * dy));
}

double polyline_length(const std::vector<cv::Point>& path) {
    double length = 0.0;
    for (size_t i = 1; i < path.size(); ++i)
        length += segment_length(path[i - 1], path[i]);
    return length;
}

cv::Point point_at_distance(const std::vector<cv::Point>& path, double distance) {
    if (path.empty()) return cv::Point(0, 0);
    if (path.size() == 1 || distance <= 0.0) return path.front();
    double walked = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
        double seg_len = segment_length(path[i - 1], path[i]);
        if (walked + seg_len >= distance) {
            double t = seg_len > 0.0 ? (distance - walked) / seg_len : 0.0;
            int x = static_cast<int>(std::round(path[i-1].x + (path[i].x - path[i-1].x) * t));
            int y = static_cast<int>(std::round(path[i-1].y + (path[i].y - path[i-1].y) * t));
            return cv::Point(x, y);
        }
        walked += seg_len;
    }
    return path.back();
}

bool select_folded_target_point(const uint8_t* skeleton, int width, int height,
                                const SkeletonAnalysisResult& skel_result,
                                RingStatus status, RingType type,
                                cv::Point& legacy_target, cv::Point& folded_target) {
    if (!choose_legacy_target_endpoint(skel_result, status, type, width, height, legacy_target))
        return false;

    folded_target = legacy_target;

    // 长度越长，比例越小；长度越短，比例越大
    //路程越长，给的控制比例就越小（越稳健）；路程越短，给的控制比例就越大（越灵敏）。
    auto length_to_ratio = [](double total_length) -> double {
        constexpr double base_ratio = 0.6;
        constexpr double length_scale = 100.0;
        return base_ratio * length_scale / (length_scale + total_length);
    };

    bool has_left = skel_result.left_start_x > 0 || skel_result.left_start_y > 0;
    bool has_right = skel_result.right_start_x > 0 || skel_result.right_start_y > 0;
    if (!has_left || !has_right) return true;

    cv::Point left_start(skel_result.left_start_x, skel_result.left_start_y);
    cv::Point right_start(skel_result.right_start_x, skel_result.right_start_y);
    cv::Point start_center((left_start.x + right_start.x) / 2,
                           (left_start.y + right_start.y) / 2);

    cv::Point left_skel, right_skel;
    std::vector<cv::Point> left_path, right_path;
    if (nearest_skeleton_point(skeleton, width, height, left_start, left_skel) &&
        nearest_skeleton_point(skeleton, width, height, right_start, right_skel) &&
        find_skeleton_path(skeleton, width, height, left_skel, legacy_target, left_path) &&
        find_skeleton_path(skeleton, width, height, right_skel, legacy_target, right_path)) {
        int li = static_cast<int>(left_path.size()) - 1;
        int ri = static_cast<int>(right_path.size()) - 1;
        while (li >= 0 && ri >= 0 && left_path[li] == right_path[ri]) { --li; --ri; }
        int merge_index = li + 1;
        int common_len = static_cast<int>(left_path.size()) - merge_index;
        if (merge_index >= 0 && merge_index < static_cast<int>(left_path.size()) && common_len >= 3) {
            std::vector<cv::Point> virtual_path;
            virtual_path.push_back(start_center);
            for (size_t i = static_cast<size_t>(merge_index); i < left_path.size(); ++i)
                virtual_path.push_back(left_path[i]);
            double total_length = polyline_length(virtual_path);
            folded_target = point_at_distance(virtual_path, total_length * length_to_ratio(total_length));
            return true;
        }
    }

    cv::Point center_skel;
    std::vector<cv::Point> center_path;
    if (nearest_skeleton_point(skeleton, width, height, start_center, center_skel) &&
        find_skeleton_path(skeleton, width, height, center_skel, legacy_target, center_path)) {
        center_path.insert(center_path.begin(), start_center);
        double total_length = polyline_length(center_path);
        folded_target = point_at_distance(center_path, total_length * 0.5);
    }
    return true;
}

} // anonymous namespace


/**
 * 
 * - 轨1 (line_size == 2): 单点前瞻模式。基于横向偏差控制，引入内外侧独立增益，
 * 使车辆在环岛或单边阶段能强行向外侧拉开距离，防止切内线咬尾。
 * - 轨2 (line_size >  2): 全局巡线模式。基于最小二乘法拟合赛道斜率，通过 tanh 非线性平滑输出。
 * - 防抖: 引入多级自适应低通滤波 (alpha) 与死区趋势保留，彻底消除回正抽搐与大弯跳点抖动。
 * * @param line             赛道点数组（2个点时为当前点与前瞻点；大于2个点时为全局骨架点）
 * @param line_size        点的数量
 * @param base_speed       基础期望车速
 * @param max_gain_ratio   最大速度增益比例（用于常规巡线限制）
 * @return std::tuple<float, float> [左轮速度, 右轮速度]
 * * @note 【调参黄金法则】
 * 1. 进弯迟钝/推头：调大外轮增速系数 `k_p_outer` (默认 0.003)
 * 2. 弯内吃内线/咬单边：调大内轮减速系数 `k_p_inner` (默认 0.010)
 * 3. 进环岛大弯高频乱抖：调小大弯滤波系数 `alpha` (默认 0.4 -> 调至 0.3 或 0.25)
 * 4. 直道细微扭动：调大死区 `DEAD_ZONE` (默认 2)
 */


std::tuple<float, float> calculate_wheel_speeds(const Point* line, int line_size,
                                                 float base_speed,
                                                 float max_gain_ratio) {
   // 静态变量用于低通滤波
    static float last_left_speed = base_speed;
    static float last_right_speed = base_speed;

    float left_speed = base_speed;
    float right_speed = base_speed;

    // 当传入2个点（起点和前瞻点）时，纯粹基于横向偏差控制（更适合状态机控制）
    if (line_size == 2) {
        int target_x = line[1].x; // 目标点 folded_target 的 X 坐标
        int img_center_x = 80;    // 屏幕中心
        
        // 【优化】：为了让你离单边远一点，加入方向补偿偏移（根据环岛状态调整或在此微调）
        int error_x = target_x - img_center_x; 
        int abs_error = std::abs(error_x);

        float target_left = base_speed;
        float target_right = base_speed;

        const int DEAD_ZONE = 2; // 死区

        if (abs_error > DEAD_ZONE) {
            float effective_error = static_cast<float>(abs_error - DEAD_ZONE);

            // 动态计算【降速/增速系数】
            float k_p_inner = 0.010f; // 稍微加大内侧减速灵敏度，让车身往外撤，拉大与单边距离
            float inner_factor = 1.0f - (effective_error * k_p_inner);
            if (inner_factor < 0.2f) inner_factor = 0.2f;

            float k_p_outer = 0.003f;
            float outer_factor = 1.0f + (effective_error * k_p_outer);
            if (outer_factor > 1.4f) outer_factor = 1.4f;

            // 分方向计算目标轮速
            if (error_x > 0) { // 右转
                target_right = base_speed * inner_factor;
                target_left  = base_speed * outer_factor;
            } else {           // 左转
                target_left  = base_speed * inner_factor;
                target_right = base_speed * outer_factor;
            }
            } else {
        // === 【优化：真正的平缓回归】 ===
        // 不要直接等于 base_speed，而是向 base_speed 靠拢，保留 50% 上一帧的转向趋势
        target_left  = 0.5f * base_speed + 0.5f * last_left_speed;
        target_right = 0.5f * base_speed + 0.5f * last_right_speed;
    }

        // 自适应动态低通滤波
        float alpha = 0.3f; 
        if (abs_error <= DEAD_ZONE) {
            alpha = 0.7f; // 回正时响应快一点，但不要强制 1.0 冲刷掉历史
        } else if (abs_error <= 15) {
            alpha = 0.6f; 
        } else {
            alpha = 0.4f; // 进环岛大弯时大滤波，对抗图像抖动
        }

        left_speed  = alpha * target_left  + (1.0f - alpha) * last_left_speed;
        right_speed = alpha * target_right + (1.0f - alpha) * last_right_speed;

        last_left_speed  = left_speed;
        last_right_speed = right_speed;

    } else {
        // 如果点数大于2（走多点最小二乘拟合），采用前半段的斜率模型
        float sum_x = 0.0f, sum_y = 0.0f, sum_yy = 0.0f, sum_xy = 0.0f;
        for (int i = 0; i < line_size; ++i) {
            sum_x += static_cast<float>(line[i].x);
            sum_y += static_cast<float>(line[i].y);
            sum_yy += static_cast<float>(line[i].y) * line[i].y;
            sum_xy += static_cast<float>(line[i].x) * line[i].y;
        }
        float denom = static_cast<float>(line_size) * sum_yy - sum_y * sum_y;
        float slope = 0.0f;
        if (std::abs(denom) > 1e-5f) {
            slope = (static_cast<float>(line_size) * sum_xy - sum_y * sum_x) / denom;
        }
        float max_gain = base_speed * max_gain_ratio;
        if (slope > 5.0f) slope = 5.0f;
        if (slope < -5.0f) slope = -5.0f;

        float gain = max_gain * std::tanh(0.1f * slope);
        left_speed = base_speed + gain;
        right_speed = base_speed - gain;
    }

    return std::make_tuple(left_speed, right_speed);
}

std::tuple<float, float> calculate_wheel_speeds(const cv::Mat &image,
                                                float base_speed,
                                                float max_gain_ratio,
                                                cv::Mat &debug_image) {
  // ==================== 帧处理 ====================
  TRACE_SCOPE("整帧处理");

    // static 状态机变量
    static RingStatus s_ring_status = RingStatus::NotFound;
    static RingType s_ring_type = RingType::None;
    static int s_ring_detect_count = 0;
    static int s_ring_disappear_count = 0;
    static int s_single_path_count = 0;
    static int s_dual_path_count = 0;
    static int s_frame_number = 0;
#ifdef SMTC2GO_DEBUG
#ifdef SMTC2GO_DEBUG_IMSHOW
    static bool s_debug_dirs_created = false;
    static bool s_windows_created = false;
    if (!s_debug_dirs_created) { create_debug_directories(); s_debug_dirs_created = true; }
    if (!s_windows_created) {
        cv::namedWindow("Skeleton", cv::WINDOW_NORMAL);
        cv::resizeWindow("Skeleton", 400, 300);
        cv::namedWindow("Result", cv::WINDOW_NORMAL);
        cv::resizeWindow("Result", 400, 300);
        s_windows_created = true;
    }
#endif // SMTC2GO_DEBUG_IMSHOW
#endif
#ifdef SMTC2GO_DEBUG_TRACE_PERFORMANCE
    static PerfTraceAccumulator s_perf_acc;
    static int s_perf_frame_count = 0;
#endif
    ++s_frame_number;
    int frame_number = s_frame_number;
    constexpr int confirm_threshold = 15;
    constexpr int img_width = 160;
    constexpr int img_height = 120;

    // 图像预处理
    cv::Mat resized;
    cv::Mat dilated;
    {
    TRACE_SCOPE("图像预处理");
    cv::resize(image, resized, cv::Size(img_width, img_height));

    cv::Mat gray;
    cv::cvtColor(resized, gray, cv::COLOR_BGR2GRAY);

    cv::Mat blurred;
    cv::GaussianBlur(gray, blurred, cv::Size(3, 3), 0);

    cv::Mat binary;
    cv::threshold(blurred, binary, 127, 255, cv::THRESH_BINARY);

    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::Mat eroded;
    cv::erode(binary, eroded, element, cv::Point(-1, -1), 2);
    cv::dilate(eroded, dilated, element, cv::Point(-1, -1), 1);
    } // preprocess

    // 轮廓检测
    std::vector<std::vector<cv::Point>> contours;
    {
    TRACE_SCOPE("轮廓检测");
    cv::findContours(dilated, contours, cv::noArray(), cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    }

#ifdef SMTC2GO_DEBUG
    cv::Mat result_img = resized.clone();
    cv::Mat skeleton_img = cv::Mat::zeros(img_height, img_width, CV_8UC1);
#endif
    int best_contour_index = -1;
    int min_area = img_width * img_height * 10 / 100;

    // get_start_point 获取 L/R 起点
    uint8_t full_binary[img_height * img_width];
    for (int y = 0; y < img_height; ++y)
        for (int x = 0; x < img_width; ++x)
            full_binary[y * img_width + x] = dilated.at<uint8_t>(y, x);

    int ref_center_x = img_width / 2;
    Point start_pt(img_width / 2, img_height - 1);
    auto start_result = get_start_point(
        full_binary, img_width, img_height,
        &start_pt,
        1, img_width - 1, 1, img_height - 1, "horizontal");
    if (start_result != nullptr) {
        auto& left_pt = std::get<0>(*start_result);
        auto& right_pt = std::get<1>(*start_result);
        ref_center_x = (left_pt.x + right_pt.x) / 2;
    }

    // 筛选最底部居中轮廓
    int best_dist = img_width;
    for (size_t i = 0; i < contours.size(); ++i) {
        double area = cv::contourArea(contours[i]);
        if (area < min_area) continue;
        cv::Rect bounding = cv::boundingRect(contours[i]);
        int bottom_y = bounding.y + bounding.height;
        if (bottom_y < img_height - 2) continue;
        int contour_center_x = bounding.x + bounding.width / 2;
        int dist = std::abs(contour_center_x - ref_center_x);
        if (dist < best_dist) { best_dist = dist; best_contour_index = static_cast<int>(i); }
    }

    if (best_contour_index >= 0) {
#ifdef SMTC2GO_DEBUG
        cv::drawContours(result_img, contours, best_contour_index, cv::Scalar(0, 255, 0), 2);
#endif

        // 骨架分析
        uint8_t skeleton_result[img_height * img_width];
        {
        TRACE_SCOPE("骨架提取");

        cv::Mat road_mask = cv::Mat::zeros(img_height, img_width, CV_8UC1);
        cv::drawContours(road_mask, contours, best_contour_index, cv::Scalar(255), -1);

        cv::Mat road_area;
        cv::bitwise_and(dilated, dilated, road_area, road_mask);

        uint8_t road_binary[img_height * img_width];
        for (int y = 0; y < img_height; ++y)
            for (int x = 0; x < img_width; ++x)
                road_binary[y * img_width + x] = road_area.at<uint8_t>(y, x);

        skeletonize(road_binary, skeleton_result, img_width, img_height);
        } // skeleton

#ifdef SMTC2GO_DEBUG
        skeleton_img = cv::Mat(img_height, img_width, CV_8UC1, skeleton_result).clone();
#endif

        uint8_t binary_for_start[img_height * img_width];
        for (int y = 0; y < img_height; ++y)
            for (int x = 0; x < img_width; ++x)
                binary_for_start[y * img_width + x] = dilated.at<uint8_t>(y, x);

        auto skel_result = [&]() {
            TRACE_SCOPE("骨架分析");
            return analyze_skeleton(skeleton_result, img_width, img_height, binary_for_start);
        }();

#ifdef SMTC2GO_DEBUG
        // 绘制特征点
        cv::Scalar blue(255, 0, 0), red(0, 0, 255), green(0, 255, 0);
        cv::Scalar cyan(255, 255, 0), purple(255, 0, 255), yellow(0, 255, 255);

        for (auto& p : skel_result.endpoint_points)
            cv::circle(result_img, cv::Point(std::get<0>(p), std::get<1>(p)), 3, blue, -1);
        for (auto& p : skel_result.bifurcation_points)
            cv::circle(result_img, cv::Point(std::get<0>(p), std::get<1>(p)), 5, red, -1);

        if (skel_result.left_start_x > 0 || skel_result.left_start_y > 0) {
            cv::circle(result_img, cv::Point(skel_result.left_start_x, skel_result.left_start_y), 4, cyan, -1);
            cv::putText(result_img, "L",
                cv::Point(skel_result.left_start_x - 5, skel_result.left_start_y - 8),
                cv::FONT_HERSHEY_SIMPLEX, 0.3, cyan, 1);
        }
        if (skel_result.right_start_x > 0 || skel_result.right_start_y > 0) {
            cv::circle(result_img, cv::Point(skel_result.right_start_x, skel_result.right_start_y), 4, cyan, -1);
            cv::putText(result_img, "R",
                cv::Point(skel_result.right_start_x - 5, skel_result.right_start_y - 8),
                cv::FONT_HERSHEY_SIMPLEX, 0.3, cyan, 1);
        }
#endif

        auto ring_result = [&]() {
            TRACE_SCOPE("圆环检测");
            return detect_ring(skeleton_result, img_width, img_height);
        }();

#ifdef SMTC2GO_DEBUG
        if (ring_result.has_ring) {
            cv::Scalar green(0, 255, 0);
            cv::circle(result_img, cv::Point(ring_result.ring_center_x, 60), 20, green, 2);
            std::string ring_text = "RING " + std::string(1, ring_result.ring_side);
            cv::putText(result_img, ring_text,
                cv::Point(ring_result.ring_center_x - 20, 50),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, green, 1);
        }
#endif

        cv::Point legacy_target, folded_target;
        float left_speed = base_speed;
        float right_speed = base_speed;
        {
        TRACE_SCOPE("目标点选择");
        if (select_folded_target_point(skeleton_result, img_width, img_height, skel_result,
                                       s_ring_status, s_ring_type, legacy_target, folded_target)) {
#ifdef SMTC2GO_DEBUG
            cv::Scalar purple(255, 0, 255), yellow(0, 255, 255);
            cv::circle(result_img, legacy_target, 5, purple, -1);
            cv::circle(result_img, folded_target, 5, yellow, -1);
#endif

            cv::Point start_center(
                (skel_result.left_start_x + skel_result.right_start_x) / 2,
                (skel_result.left_start_y + skel_result.right_start_y) / 2
            );
            Point line[2] = {
                Point(start_center.x, start_center.y),
                Point(folded_target.x, folded_target.y)
            };
            // auto [rs, ls] = calculate_wheel_speeds(line, 2, base_speed,
            // max_gain_ratio);
            auto error = (img_width / 2) - folded_target.x;
            left_speed = base_speed + (error * max_gain_ratio);
            right_speed = base_speed - (error * max_gain_ratio);

#ifdef SMTC2GO_DEBUG
            char speed_text[32];
            snprintf(speed_text, sizeof(speed_text), "l:%.1f|r:%.1f", left_speed, right_speed);
            cv::putText(result_img, speed_text, cv::Point(folded_target.x + 4, folded_target.y - 4),
                cv::FONT_HERSHEY_SIMPLEX, 0.3, yellow, 1);
#endif
        }
        } // select_target

        bool has_ring = ring_result.has_ring;
        int branch_count = skel_result.branch_count;
        int endpoint_count = skel_result.endpoint_count;

        // 状态机转移
        switch (s_ring_status) {
            case RingStatus::NotFound:
                if (has_ring) {
                    s_ring_detect_count++;
                    s_ring_disappear_count = 0;
                    if (s_ring_detect_count >= confirm_threshold) {
                        s_ring_status = RingStatus::Discovered;
                        s_ring_type = (ring_result.ring_side == 'l') ? RingType::Left : RingType::Right;
                        s_ring_detect_count = 0;
                        LOG_INFO("[帧 %d] 未发现 -> 已发现 (%s圆环)", frame_number,
                                 s_ring_type == RingType::Left ? "左" : "右");
                    }
                } else { s_ring_detect_count = 0; }
                break;
            case RingStatus::Discovered:
                if (!has_ring) {
                    s_ring_disappear_count++;
                    s_ring_detect_count = 0;
                    if (s_ring_disappear_count >= confirm_threshold) {
                        s_ring_status = RingStatus::PrepareEnter;
                        s_ring_disappear_count = 0;
                        LOG_INFO("[帧 %d] 已发现 -> 准备入环", frame_number);
                    }
                } else {
                    s_ring_disappear_count = 0;
                    s_ring_type = (ring_result.ring_side == 'l') ? RingType::Left : RingType::Right;
                }
                break;
            case RingStatus::PrepareEnter:
              if (branch_count == 1 && endpoint_count == 2) {
                s_single_path_count++;
                s_dual_path_count = 0;
                if (s_single_path_count >= confirm_threshold) {
                  s_ring_status = RingStatus::PrepareExit;
                  s_single_path_count = 0;
                  LOG_INFO("[帧 %d] 准备入环 -> 准备出环", frame_number);
                }
                } else { s_single_path_count = 0; }
                break;
            case RingStatus::PrepareExit:
              if (branch_count > 1 && endpoint_count > 2) {
                s_dual_path_count++;
                s_single_path_count = 0;
                if (s_dual_path_count >= confirm_threshold) {
                  s_ring_status = RingStatus::AboutToExit;
                  s_dual_path_count = 0;
                  LOG_INFO("[帧 %d] 准备出环 -> 即将出环", frame_number);
                }
              } else {
                s_dual_path_count = 0;
              }
                break;
            case RingStatus::AboutToExit:
              if (branch_count >= 2 && endpoint_count >= 3) {
                s_single_path_count++;
                s_dual_path_count = 0;
                if (s_single_path_count >= confirm_threshold) {
                  s_ring_status = RingStatus::Exiting;
                  s_ring_type = RingType::None;
                  s_single_path_count = 0;
                  LOG_INFO("[帧 %d] 即将出环 -> 出环中", frame_number);
                }
                } else { s_single_path_count = 0; }
                break;
            case RingStatus::Exiting:
                if (endpoint_count > 1) {
                    s_single_path_count++; s_dual_path_count = 0;
                    if (s_single_path_count >= confirm_threshold) {
                        s_ring_status = RingStatus::NotFound;
                        s_ring_type = RingType::None;
                        s_single_path_count = 0;
                        LOG_INFO("[帧 %d] 出环中 -> 未发现", frame_number);
                    }
                } else { s_single_path_count = 0; }
                break;
        }

#ifdef SMTC2GO_DEBUG
        if (frame_number % 20 == 0) {
            LOG_DEBUG("[帧 %d] 状态=%s 圆坏类型=%s 分支=%d 端点=%d 有环=%d 环位置=%d",
                      frame_number, ring_status_name(s_ring_status), ring_type_name(s_ring_type),
                      branch_count, endpoint_count, has_ring ? 1 : 0, ring_result.ring_center_x);
        }
        if (frame_number % 10 == 0) {
#ifdef SMTC2GO_DEBUG_SAVE_IMAGES
          save_debug_images(ring_status_name(s_ring_status), frame_number,
                              resized, skeleton_img, result_img, img_width, img_height);
#endif
                            }

#ifdef SMTC2GO_DEBUG_IMSHOW
        cv::Mat result_display, skeleton_display;
        cv::resize(result_img, result_display, cv::Size(400, 300));
        cv::resize(skeleton_img, skeleton_display, cv::Size(400, 300));
        cv::imshow("Result", result_display);
        cv::imshow("Skeleton", skeleton_display);
#endif
#endif

#ifdef SMTC2GO_DEBUG_TRACE_PERFORMANCE
        if (s_frame_number % 100 == 0) {
            s_perf_acc.report();
        }
#endif

#ifdef SMTC2GO_DEBUG
        // 把带紫色 legacy_target / 黄色 folded_target 标注的 result_img
        // 复制给调用方，便于网页/文件/显示等二次渲染。
        if (debug_image.empty()) {
          debug_image = result_img.clone();
        } else {
          result_img.copyTo(debug_image);
        }
#endif

        return std::make_tuple(left_speed, right_speed);

    } else {
      LOG_WARN("[帧 %d] 未找到合适的轮廓，丢线了", frame_number);

#ifdef SMTC2GO_DEBUG
      // 没有轮廓时，至少把原图缩放后的结果交给调用方
      if (debug_image.empty()) {
        debug_image = resized.clone();
      } else {
        resized.copyTo(debug_image);
      }
#endif
      return std::make_tuple(0, 0);
    }
}

} // namespace find_line_lib
