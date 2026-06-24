/**
 * @file check_roadblock.cpp
 * @brief 路障检测实现
 */

#include "find_line_lib/check_roadblock.h"

#include <tuple>
#include <algorithm>
#include <opencv2/opencv.hpp>

namespace find_line_lib
{

// 静态缓冲区，与 Python 的返回值方式类似
static std::tuple<Point, Point> roadblock_result_buffer;

// ==================== 检查路障 ====================
// 对应 Python: 检查路障(binary_img, start_point, 阈值)
//
// 完整 Python 逻辑：
//   inverted_img = cv2.bitwise_not(binary_img)
//   num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(inverted_img, connectivity=8)
//
//   # 遍历所有连通域，找最下方且有效的
//   for i in range(1, num_labels):
//       bx, by, bw, bh, area = stats[i]
//       if area < 100: continue
//       if bx == 0 or (bx+bw) == w: continue  # 过滤贴边的
//       bottom_y = by + bh
//       if bottom_y <= max(y_left, y_right) and bottom_y > max_bottom_y:
//           max_bottom_y = bottom_y; target_label = i
//
//   # 获取目标路障的像素坐标
//   pixel_indices = np.where(labels == target_label)
//   y_coords, x_coords = pixel_indices
//
//   # 情况A：路障在左边，挨左边界太近
//   dist_to_left = bx - x_left
//   if 0 <= dist_to_left <= 阈值:
//       idx_se = np.argmax(x_coords + y_coords)  # 找 x+y 最大的点（右下角）
//       return (左起点, (x_coords[idx_se], y_coords[idx_se]))
//
//   # 情况B：路障在右边，挨右边界太近
//   dist_to_right = x_right - (bx+bw)
//   if 0 <= dist_to_right <= 阈值:
//       idx_sw = np.argmax(y_coords - x_coords)  # 找 y-x 最大的点（左下角）
//       return (右起点, (x_coords[idx_sw], y_coords[idx_sw]))
//
// C++ 实现完全一致
std::tuple<Point, Point> *
check_roadblock(const uint8_t *binary_img, int width, int height,
		const std::tuple<Point, Point> *start_point, int threshold)
{
	int h = height;
	int w = width;

	// 解析起始点（与 Python 的 x_left, y_left, x_right, y_right 一致）
	Point left_start_point = std::get<0>(*start_point); // 原变量名: 左起点
	Point right_start_point = std::get<1>(*start_point); // 原变量名: 右起点
	int x_left = left_start_point.x;
	int y_left = left_start_point.y;
	int x_right = right_start_point.x;
	int y_right = right_start_point.y;

	// ==================== 1. 图像反转 ====================
	// Python: inverted_img = cv2.bitwise_not(binary_img)
	// 将黑色路障变成白色连通域，方便后续连通域分析
	cv::Mat inverted_img;
	cv::bitwise_not(cv::Mat(h, w, CV_8UC1,
				const_cast<uint8_t *>(binary_img)),
			inverted_img);

	cv::Mat labels, stats, centroids;
	int num_labels = cv::connectedComponentsWithStats(inverted_img, labels,
							  stats, centroids, 8);

	// ==================== 3. 遍历连通域找目标路障 ====================
	int target_label = -1;
	int max_bottom_y = -1;

	for (int i = 1; i < num_labels; ++i) {
		// 获取连通域的统计信息（与 Python 的 stats[i] 一致）
		int bx = stats.at<int>(i, cv::CC_STAT_LEFT);
		int by = stats.at<int>(i, cv::CC_STAT_TOP);
		int bw = stats.at<int>(i, cv::CC_STAT_WIDTH);
		int bh = stats.at<int>(i, cv::CC_STAT_HEIGHT);
		int area = stats.at<int>(i, cv::CC_STAT_AREA);

		// 过滤噪点（面积太小的不要）
		if (area < 100) {
			continue;
		}

		// 过滤掉贴着图像最左和最右两侧的"赛道外全黑背景"
		// 与 Python 的 if bx == 0 or (bx + bw) == w: continue 一致
		if (bx == 0 || (bx + bw) == w) {
			continue;
		}

		// 计算边界框底部 Y 坐标
		int bottom_y = by + bh;

		// 筛选：在起点下方，且是最下方的
		// Python: if bottom_y <= max(y_left, y_right) and bottom_y > max_bottom_y:
		if (bottom_y <= std::max(y_left, y_right) &&
		    bottom_y > max_bottom_y) {
			max_bottom_y = bottom_y;
			target_label = i;
		}
	}

	// 没找到任何有效的路障连通域
	if (target_label == -1) {
		return nullptr;
	}

	// ==================== 4. 获取目标路障的信息 ====================
	int bx = stats.at<int>(target_label, cv::CC_STAT_LEFT);
	int by = stats.at<int>(target_label, cv::CC_STAT_TOP);
	int bw = stats.at<int>(target_label, cv::CC_STAT_WIDTH);
	int bh = stats.at<int>(target_label, cv::CC_STAT_HEIGHT);

	// 获取目标连通域的所有像素坐标（与 Python 的 np.where(labels == target_label) 对应）
	cv::Mat mask = (labels == target_label);
	std::vector<cv::Point> points;
	cv::findNonZero(mask, points);

	if (points.empty()) {
		return nullptr;
	}

	// ==================== 5. 情况A：路障在左边 ====================
	// Python: dist_to_left = bx - x_left; if 0 <= dist_to_left <= 阈值:
	int dist_to_left = bx - x_left;
	if (0 <= dist_to_left && dist_to_left <= threshold) {
		// Python: idx_se = np.argmax(x_coords + y_coords)
		// 找 x+y 最大的点，即右下角点
		int max_x_at_max_y = points[0].x;
		int max_y = points[0].y;
		int max_sum = points[0].x + points[0].y;
		for (const auto &pt : points) {
			int sum = pt.x + pt.y;
			if (sum > max_sum) {
				max_sum = sum;
				max_x_at_max_y = pt.x;
				max_y = pt.y;
			}
		}

		Point block_bottom_right_point(max_x_at_max_y,
					       max_y); // 原变量名: 黑块右下角点

		// 障碍物中心必须在图像下半部分（与 Python 一致）
		if (block_bottom_right_point.y < h / 2) {
			return nullptr;
		}

		// 返回避障引导线：左起点 -> 黑块右下角点
		roadblock_result_buffer = std::make_tuple(
			left_start_point, block_bottom_right_point);
		return &roadblock_result_buffer;
	}

	// ==================== 6. 情况B：路障在右边 ====================
	// Python: dist_to_right = x_right - (bx + bw); if 0 <= dist_to_right <= 阈值:
	int bx_right = bx + bw;
	int dist_to_right = x_right - bx_right;
	if (0 <= dist_to_right && dist_to_right <= threshold) {
		// Python: idx_sw = np.argmax(y_coords - x_coords)
		// 找 y-x 最大的点，即左下角点
		int min_x_at_max_y = points[0].x;
		int max_y = points[0].y;
		int max_y_minus_x = points[0].y - points[0].x;
		for (const auto &pt : points) {
			int y_minus_x = pt.y - pt.x;
			if (y_minus_x > max_y_minus_x) {
				max_y_minus_x = y_minus_x;
				min_x_at_max_y = pt.x;
				max_y = pt.y;
			}
		}

		Point block_bottom_left_point(min_x_at_max_y,
					      max_y); // 原变量名: 黑块左下角点

		if (block_bottom_left_point.y < h / 2) {
			return nullptr;
		}

		// 返回避障引导线：右起点 -> 黑块左下角点
		roadblock_result_buffer = std::make_tuple(
			right_start_point, block_bottom_left_point);
		return &roadblock_result_buffer;
	}

	// 路障不在阈值范围内，不触发避障
	return nullptr;
}

}
