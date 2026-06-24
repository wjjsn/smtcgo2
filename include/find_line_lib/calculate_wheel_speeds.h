#pragma once

#include "common.h"
#include <tuple>
#include <opencv2/core/mat.hpp>

namespace find_line_lib
{

// ==================== 轮速计算 ====================
std::tuple<float, float> calculate_wheel_speeds(const Point *line,
						int line_size,
						float base_speed = 50.0f,
						float max_gain_ratio = 0.8f);

// 带调试图像输出的重载：
//   debug_image 在库编译时启用了 SMTC2GO_DEBUG 的情况下会填充为带
//   骨架/端点/分叉点/紫色 legacy_target/黄色 folded_target 等标注的 BGR 图；
//   未启用 SMTC2GO_DEBUG 时不会填充内容。
//   debug_image 可以传 cv::Mat() 表示不关心。
std::tuple<float, float> calculate_wheel_speeds(const cv::Mat &image,
						float base_speed,
						float max_gain_ratio,
						cv::Mat &debug_image);

inline std::tuple<float, float>
calculate_wheel_speeds(const cv::Mat &image, float base_speed = 50.0f,
		       float max_gain_ratio = 0.8f)
{
	cv::Mat unused;
	return calculate_wheel_speeds(image, base_speed, max_gain_ratio,
				      unused);
}
}
