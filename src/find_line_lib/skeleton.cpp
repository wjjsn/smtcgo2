#include "find_line_lib/skeleton.h"
#include "find_line_lib/common.h"
#include "find_line_lib/get_start_point.h"

#include <cstring>
#include <vector>
#include <cmath>
#include <algorithm>
#include <opencv2/opencv.hpp>

namespace find_line_lib
{

using namespace find_line_lib;

void skeletonize(const uint8_t *binary_img, uint8_t *skeleton, int width,
		 int height)
{
	static const uint8_t lut[256] = {
		0, 0, 0, 1, 0, 0, 1, 3, 0, 0, 3, 1, 1, 0, 1, 3, 0, 0, 0, 0,
		0, 0, 0, 0, 2, 0, 2, 0, 3, 0, 3, 3, 0, 0, 0, 0, 0, 0, 0, 0,
		3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0,
		3, 0, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0,
		0, 0, 0, 0, 2, 0, 0, 0, 2, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0,
		3, 0, 0, 0, 3, 0, 2, 0, 0, 0, 3, 1, 0, 0, 1, 3, 0, 0, 0, 0,
		0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
		3, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 3, 1, 3, 0, 0, 1, 3,
		0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 2, 3, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
		3, 3, 0, 1, 0, 0, 0, 0, 2, 2, 0, 0, 2, 0, 0, 0
	};

	int padded_w = width + 2;
	int padded_h = height + 2;
	int padded_size = padded_w * padded_h;
	int img_size = width * height;

	std::vector<uint8_t> current(img_size, 0);
	std::vector<uint8_t> padded(padded_size, 0);
	std::vector<uint8_t> cleaned(padded_size, 0);

	for (int i = 0; i < img_size; ++i) {
		current[i] = binary_img[i] > 127 ? 1 : 0;
	}

	bool pixel_removed = true;
	while (pixel_removed) {
		pixel_removed = false;

		std::fill(padded.begin(), padded.end(), 0);
		for (int y = 0; y < height; ++y) {
			for (int x = 0; x < width; ++x) {
				padded[(y + 1) * padded_w + (x + 1)] =
					current[y * width + x];
			}
		}

		for (int pass_num = 0; pass_num < 2; ++pass_num) {
			bool first_pass = (pass_num == 0);
			cleaned = padded;

			for (int row = 1; row < height + 1; ++row) {
				for (int col = 1; col < width + 1; ++col) {
					int idx = row * padded_w + col;
					if (padded[idx] == 0) {
						continue;
					}

					int neighbors =
						padded[idx - padded_w - 1] +
						2 * padded[idx - padded_w] +
						4 * padded[idx - padded_w + 1] +
						8 * padded[idx + 1] +
						16 * padded[idx + padded_w + 1] +
						32 * padded[idx + padded_w] +
						64 * padded[idx + padded_w - 1] +
						128 * padded[idx - 1];

					uint8_t lut_val = lut[neighbors];
					if (lut_val == 0) {
						continue;
					}

					if (lut_val == 3 ||
					    (lut_val == 1 && first_pass) ||
					    (lut_val == 2 && !first_pass)) {
						cleaned[idx] = 0;
						pixel_removed = true;
					}
				}
			}

			padded = cleaned;
		}

		for (int y = 0; y < height; ++y) {
			for (int x = 0; x < width; ++x) {
				current[y * width + x] =
					padded[(y + 1) * padded_w + (x + 1)];
			}
		}
	}

	for (int y = 0; y < height; ++y) {
		for (int x = 0; x < width; ++x) {
			skeleton[y * width + x] = current[y * width + x] * 255;
		}
	}
}

// ==================== 骨架分析 ====================
// 在骨架上找分叉点和端点
SkeletonAnalysisResult analyze_skeleton(const uint8_t *skeleton, int width,
					int height, const uint8_t *binary_img)
{
	SkeletonAnalysisResult result;
	result.bifurcation_count = 0;
	result.endpoint_count = 0;
	result.branch_count = 0;
	result.left_start_x = 0;
	result.left_start_y = 0;
	result.right_start_x = 0;
	result.right_start_y = 0;

	// 8个邻居偏移量
	const int neighbors[8][2] = { { -1, -1 }, { -1, 0 }, { -1, 1 },
				      { 0, 1 },	  { 1, 1 },  { 1, 0 },
				      { 1, -1 },  { 0, -1 } };

	int bifurcation_count = 0;
	int endpoint_count = 0;

	// 遍历骨架上的每个白点（全图遍历，包括边界）
	for (int y = 0; y < height; ++y) {
		for (int x = 0; x < width; ++x) {
			if (skeleton[y * width + x] == 0)
				continue;

			// 数8个邻居中有多少个白点（越界邻居不计入）
			int neighbor_count = 0;
			for (int i = 0; i < 8; ++i) {
				int ny = y + neighbors[i][0];
				int nx = x + neighbors[i][1];
				if (ny >= 0 && ny < height && nx >= 0 &&
				    nx < width) {
					if (skeleton[ny * width + nx] > 0)
						++neighbor_count;
				}
			}

			// 邻居>=3是分叉点，=1是端点
			if (neighbor_count >= 3) {
				result.bifurcation_points.push_back(
					std::make_tuple(x, y));
				++bifurcation_count;
			} else if (neighbor_count == 1) {
				result.endpoint_points.push_back(
					std::make_tuple(x, y));
				++endpoint_count;
			}
		}
	}

	auto start_points = get_start_point(binary_img, width, height);
	if (start_points != nullptr) {
		Point left_start, right_start;
		std::tie(left_start, right_start) = *start_points;
		result.left_start_x = left_start.x;
		result.left_start_y = left_start.y;
		result.right_start_x = right_start.x;
		result.right_start_y = right_start.y;
	}

	// 过滤掉离左右起始点过近的端点（从15降到8，保留更多端点）
	int distance_threshold = 8;
	std::vector<std::tuple<int, int> > filtered_endpoints;
	for (auto &ep : result.endpoint_points) {
		int ex = std::get<0>(ep);
		int ey = std::get<1>(ep);
		bool keep = true;

		if (result.left_start_x > 0 || result.left_start_y > 0) {
			int dist = static_cast<int>(
				std::sqrt((ex - result.left_start_x) *
						  (ex - result.left_start_x) +
					  (ey - result.left_start_y) *
						  (ey - result.left_start_y)));
			if (dist < distance_threshold)
				keep = false;
		}
		if (result.right_start_x > 0 || result.right_start_y > 0) {
			int dist = static_cast<int>(
				std::sqrt((ex - result.right_start_x) *
						  (ex - result.right_start_x) +
					  (ey - result.right_start_y) *
						  (ey - result.right_start_y)));
			if (dist < distance_threshold)
				keep = false;
		}

		if (keep)
			filtered_endpoints.push_back(ep);
	}

	result.endpoint_points = filtered_endpoints;
	endpoint_count = static_cast<int>(result.endpoint_points.size());

	result.bifurcation_count = bifurcation_count;
	result.endpoint_count = endpoint_count;

	// 分支数量估算：
	// 端点数/2（每条分支有2个端点）
	if (endpoint_count == 0) {
		result.branch_count = 0;
	} else if (bifurcation_count == 0) {
		result.branch_count = 1;
	} else {
		result.branch_count = (endpoint_count + 1) / 2;
	}

	return result;
}

// ==================== 圆环检测 ====================
// 通过膨胀骨架形成轮廓，然后用 RETR_TREE 层级关系找内孔
RingDetectionResult detect_ring(const uint8_t *skeleton, int width, int height,
				double min_area_ratio, int dilate_kernel_size,
				int dilate_iterations)
{
	RingDetectionResult result;
	result.has_ring = false;
	result.ring_center_x = 0;
	result.ring_side = 'n';
	result.ring_area = 0;

	cv::Mat skeleton_mat(height, width, CV_8UC1,
			     const_cast<uint8_t *>(skeleton));
	cv::Mat thick;
	cv::Mat kernel =
		cv::Mat::ones(dilate_kernel_size, dilate_kernel_size, CV_8UC1);
	cv::dilate(skeleton_mat, thick, kernel, cv::Point(-1, -1),
		   dilate_iterations);

	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(thick, contours, hierarchy, cv::RETR_TREE,
			 cv::CHAIN_APPROX_SIMPLE);

	if (hierarchy.empty()) {
		return result;
	}

	double min_area = static_cast<double>(width) * height * min_area_ratio;
	int center_x = width / 2;

	for (size_t i = 0; i < contours.size(); ++i) {
		if (hierarchy[i][3] == -1) {
			continue;
		}

		double hole_area = cv::contourArea(contours[i]);
		if (hole_area > min_area) {
			cv::Rect bbox = cv::boundingRect(contours[i]);
			int ring_center_x = bbox.x + bbox.width / 2;
			result.has_ring = true;
			result.ring_center_x = ring_center_x;
			result.ring_area = static_cast<int>(hole_area);
			result.ring_side = (ring_center_x < center_x) ? 'l' :
									'r';
			break;
		}
	}

	return result;
}

}
