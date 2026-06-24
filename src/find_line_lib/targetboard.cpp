/**
 * @file targetboard.cpp
 * @brief 目标板检测实现
 */

#include "find_line_lib/targetboard.h"

#include <tuple>
#include <vector>
#include <algorithm>
#include <opencv2/imgproc.hpp>

namespace find_line_lib
{

// ==================== 创建红色掩码 ====================
// 红色在 HSV 空间中分布在两端（0-15 和 150-180），需要两个范围
cv::Mat create_red_mask(const cv::Mat &hsv, const cv::Scalar &lower_red1,
			const cv::Scalar &upper_red1,
			const cv::Scalar &lower_red2,
			const cv::Scalar &upper_red2)
{
	cv::Mat mask1, mask2;
	// cv::inRange 需要输出参数
	cv::inRange(hsv, lower_red1, upper_red1, mask1);
	cv::inRange(hsv, lower_red2, upper_red2, mask2);
	cv::Mat result;
	cv::bitwise_or(mask1, mask2, result);
	return result;
}

// ==================== 形态学操作 ====================
// 对应 Python: apply_morphology(mask, kernel_size=3)
//
// Python 逻辑：
//   kernel = np.ones((kernel_size, kernel_size), np.uint8)
//   mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # 闭运算：填洞
//   mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)   # 开运算：去噪
cv::Mat apply_morphology(const cv::Mat &mask, int kernel_size)
{
	cv::Mat kernel = cv::getStructuringElement(
		cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
	cv::Mat result = mask.clone();
	// 闭运算：先膨胀后腐蚀，填充白色区域内部的小黑洞
	cv::morphologyEx(result, result, cv::MORPH_CLOSE, kernel);
	// 开运算：先腐蚀后膨胀，去除白色区域边缘的细小噪点
	cv::morphologyEx(result, result, cv::MORPH_OPEN, kernel);
	return result;
}

// ==================== 筛选中心轮廓 ====================
std::vector<std::tuple<std::vector<cv::Point>, double, int, int, int, int> >
filter_center_contours(const std::vector<std::vector<cv::Point> > &contours,
		       int img_width, float center_ratio, double min_area,
		       int min_width, int min_height)
{
	// 计算中心区域范围（图像中央的 center_ratio 范围）
	float center_range_min = (0.5f - center_ratio / 2.0f) * img_width;
	float center_range_max = (0.5f + center_ratio / 2.0f) * img_width;

	std::vector<
		std::tuple<std::vector<cv::Point>, double, int, int, int, int> >
		filtered;

	// 遍历所有轮廓，筛选符合条件的
	for (const auto &cnt : contours) {
		double area = cv::contourArea(cnt); // 计算轮廓面积
		cv::Rect bbox = cv::boundingRect(cnt); // 计算边界框
		int cnt_center_x = bbox.x + bbox.width / 2; // 边界框中心 X 坐标

		// 筛选条件：中心在中心区域内，且面积和尺寸足够大
		if (center_range_min < cnt_center_x &&
		    cnt_center_x < center_range_max && area > min_area &&
		    bbox.width > min_width && bbox.height > min_height) {
			filtered.emplace_back(cnt, area, bbox.x, bbox.y,
					      bbox.width, bbox.height);
		}
	}

	// 按面积降序排序
	std::sort(filtered.begin(), filtered.end(),
		  [](const auto &a, const auto &b) {
			  return std::get<1>(a) > std::get<1>(b);
		  });

	return filtered;
}

// ==================== 找四个角点 ====================
std::tuple<cv::Point, cv::Point, cv::Point, cv::Point>
find_corner_points(const std::vector<cv::Point> &points)
{
	int n = static_cast<int>(points.size());
	if (n == 0) {
		return std::make_tuple(cv::Point(0, 0), cv::Point(0, 0),
				       cv::Point(0, 0), cv::Point(0, 0));
	}

	// 初始化：假设第一个点是所有极值点
	int min_add_idx = 0;
	int max_add_idx = 0;
	int min_diff_idx = 0;
	int max_diff_idx = 0;

	int min_add = points[0].x + points[0].y;
	int max_add = points[0].x + points[0].y;
	int max_diff = points[0].x - points[0].y;
	int min_diff = points[0].x - points[0].y;

	// 遍历所有点，找 x+y 最小/最大，x-y 最大/最小
	for (int i = 1; i < n; ++i) {
		int sum_val = points[i].x + points[i].y; // x + y
		int diff_val = points[i].x - points[i].y; // x - y

		if (sum_val < min_add) {
			min_add = sum_val;
			min_add_idx = i;
		}
		if (sum_val > max_add) {
			max_add = sum_val;
			max_add_idx = i;
		}
		if (diff_val > max_diff) {
			max_diff = diff_val;
			max_diff_idx = i;
		}
		if (diff_val < min_diff) {
			min_diff = diff_val;
			min_diff_idx = i;
		}
	}

	// 返回四个角点：tl, tr, br, bl
	cv::Point tl = points[min_add_idx]; // 左上：x+y 最小
	cv::Point br = points[max_add_idx]; // 右下：x+y 最大
	cv::Point tr = points[max_diff_idx]; // 右上：x-y 最大
	cv::Point bl = points[min_diff_idx]; // 左下：x-y 最小

	return std::make_tuple(tl, tr, br, bl);
}

// ==================== 绘制角点 ====================
// 对应 Python: draw_corner_points(result, corners, color, radius, thickness)
//
// Python 逻辑：
//   for point in corners:
//       cv2.circle(result, (px, py), radius, color, thickness)
void draw_corner_points(
	cv::Mat &result,
	const std::tuple<cv::Point, cv::Point, cv::Point, cv::Point> &corners,
	const cv::Scalar &color, int radius, int thickness)
{
	cv::Point tl, tr, br, bl;
	std::tie(tl, tr, br, bl) = corners;

	cv::circle(result, tl, radius, color, thickness);
	cv::circle(result, tr, radius, color, thickness);
	cv::circle(result, br, radius, color, thickness);
	cv::circle(result, bl, radius, color, thickness);
}

// ==================== 检测目标板 ====================
std::tuple<cv::Point, cv::Point, cv::Point, cv::Point> *
check_targetboard(const cv::Mat &rgb_img, StatusSwitcher *ss)
{
	static std::tuple<cv::Point, cv::Point, cv::Point, cv::Point>
		result_buffer;

	// BGR 转 HSV
	cv::Mat hsv;
	cv::cvtColor(rgb_img, hsv, cv::COLOR_BGR2HSV);

	// 创建红色掩码并进行形态学处理
	cv::Mat mask = create_red_mask(hsv);
	mask = apply_morphology(mask);

	// 找轮廓
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL,
			 cv::CHAIN_APPROX_SIMPLE);

	if (contours.empty()) {
		return nullptr;
	}

	// 筛选中心区域的轮廓
	auto center_contours = filter_center_contours(contours, rgb_img.cols);

	if (center_contours.empty()) {
		return nullptr;
	}

	// 取面积最大的轮廓，找四个角点
	const auto &best_cnt = std::get<0>(center_contours[0]);
	auto corners = find_corner_points(best_cnt);
	result_buffer = corners;
	return &result_buffer;
}

// ==================== 根据识别状态补线 ====================
std::tuple<Point, Point> *supplement_line_by_status(
	StatusSwitcher *ss, const std::tuple<Point, Point> *start_point,
	const std::tuple<cv::Point, cv::Point, cv::Point, cv::Point>
		*targetboard_points)
{
	// 不需要补线的状态：未发现、直行
	if (ss->model_status == ModelRecognitionStatus::NotFound ||
	    ss->model_status == ModelRecognitionStatus::Straight) {
		return nullptr;
	}

	if (start_point == nullptr || targetboard_points == nullptr) {
		return nullptr;
	}

	static std::tuple<Point, Point> line_buffer;

	// 解析四个角点
	cv::Point tl, tr, br, bl;
	std::tie(tl, tr, br, bl) = *targetboard_points;

	// 解析左右边界起点
	Point left_point = std::get<0>(*start_point);
	Point right_point = std::get<1>(*start_point);

	if (ss->model_status == ModelRecognitionStatus::Left) {
		// 左岔路：用右起点连左下角
		int bl_x = bl.x;
		int bl_y = bl.y;
		line_buffer = std::make_tuple(right_point, Point(bl_x, bl_y));
	} else if (ss->model_status == ModelRecognitionStatus::Right) {
		// 右岔路：用左起点连右下角
		int br_x = br.x;
		int br_y = br.y;
		line_buffer = std::make_tuple(left_point, Point(br_x, br_y));
	} else {
		return nullptr;
	}

	return &line_buffer;
}

}