#pragma once
#include "opencv2/opencv.hpp"
namespace find_line_lib
{
struct start_point {
	cv::Point left;
	cv::Point right;
};
class tools {
    public:
	auto find_best_contour(cv::Mat &bin_img,
			       std::vector<std::vector<cv::Point> > &contours)
	{
		int img_width_ = bin_img.cols;
		int img_height_ = bin_img.rows;
		int ref_center_x = bin_img.cols / 2;
		int ref_center_y = bin_img.rows;
		bool has_ref_point = false;
		start_point sp;
		auto start_result = get_start_point(bin_img, sp);
		if (start_result) {
			ref_center_x = (sp.left.x + sp.right.x) / 2;
			ref_center_y = (sp.left.y + sp.right.y) / 2;
			has_ref_point = true;
		}

		// 筛选最底部居中轮廓
		int best_dist = img_width_;
		int best_contour_index = -1;
		for (size_t i = 0; i < contours.size(); ++i) {
			double area = cv::contourArea(contours[i]);
			cv::Rect bounding = cv::boundingRect(contours[i]);
			int bottom_y = bounding.y + bounding.height;
			if (bottom_y < img_height_ - 2)
				continue;
			// 参考点必须在轮廓内
			if (has_ref_point) {
				cv::Point ref_pt(ref_center_x, ref_center_y);
				if (cv::pointPolygonTest(contours[i], ref_pt,
							 false) < 0)
					continue;
			}
			int contour_center_x = bounding.x + bounding.width / 2;
			int dist = std::abs(contour_center_x - ref_center_x);
			if (dist < best_dist) {
				best_dist = dist;
				best_contour_index = static_cast<int>(i);
			}
		}
		return best_contour_index;
	}
	static auto find_center_line(cv::Mat &bin_img, int start_width = -1,
				     int start_height = -1)
	{
		std::vector<cv::Point> center_line;

		if (start_width < 0)
			start_width = bin_img.cols / 2;
		if (start_height < 0)
			start_height = bin_img.rows;

		int current_w = start_width;

		// 高度索引从 start_height - 1 开始向上遍历 (避免越界)
		for (int h = start_height - 1; h >= 0; --h) {
			// 获取当前行的首地址，提升访问效率（假设图像为单通道 CV_8UC1）
			uchar *row_ptr = bin_img.ptr<uchar>(h);

			int found_white_w = -1;

			// 1. 向左向右找白点 (值为 255)
			if (row_ptr[current_w] == 255) {
				found_white_w = current_w; // 当前点就是白点
			} else {
				// 当前点是黑的，以 current_w 为中心向两边扩散寻找
				int offset = 1;
				while (current_w - offset >= 0 ||
				       current_w + offset < bin_img.cols) {
					// 向左找
					if (current_w - offset >= 0 &&
					    row_ptr[current_w - offset] ==
						    255) {
						found_white_w =
							current_w - offset;
						break;
					}
					// 向右找
					if (current_w + offset < bin_img.cols &&
					    row_ptr[current_w + offset] ==
						    255) {
						found_white_w =
							current_w + offset;
						break;
					}
					offset++;
				}
			}

			// 2. 如果一整行找完都没有白的
			if (found_white_w == -1) {
				if (center_line.empty()) {
					// 还没找到线段的起点，向上一行继续找
					continue;
				} else {
					// 已经开始记录中心线，但下一行全黑，说明线断了，结束循环
					break;
				}
			}

			// 3. 找出这段白线的左右边界，以求中点
			int left_edge = found_white_w;
			int right_edge = found_white_w;

			// 沿找到的白点向左扩展直到黑点
			while (left_edge > 0 && row_ptr[left_edge - 1] == 255) {
				left_edge--;
			}
			// 沿找到的白点向右扩展直到黑点
			while (right_edge < bin_img.cols - 1 &&
			       row_ptr[right_edge + 1] == 255) {
				right_edge++;
			}

			// 计算这段白线的中点
			current_w = (left_edge + right_edge) / 2;

			// 保存中点，并作为下一行的起点（循环内自动复用 current_w）
			center_line.emplace_back(current_w, h);
		}

		return center_line;
	}
	static bool get_start_point(cv::Mat &bin_img, start_point &sp,
				    int start_width = -1, int start_height = -1)
	{
		if (start_width < 0)
			start_width = bin_img.cols / 2;
		if (start_height < 0)
			start_height = bin_img.rows;

		int current_w = start_width;

		// 高度索引从 start_height - 1 开始向上遍历 (避免越界)
		for (int h = start_height - 1; h >= 0; --h) {
			// 获取当前行的首地址，提升访问效率（假设图像为单通道 CV_8UC1）
			uchar *row_ptr = bin_img.ptr<uchar>(h);

			int found_white_w = -1;

			// 1. 向左向右找白点 (值为 255)
			if (row_ptr[current_w] == 255) {
				found_white_w = current_w; // 当前点就是白点
			} else {
				// 当前点是黑的，以 current_w 为中心向两边扩散寻找
				int offset = 1;
				while (current_w - offset >= 0 ||
				       current_w + offset < bin_img.cols) {
					// 向左找
					if (current_w - offset >= 0 &&
					    row_ptr[current_w - offset] ==
						    255) {
						found_white_w =
							current_w - offset;
						break;
					}
					// 向右找
					if (current_w + offset < bin_img.cols &&
					    row_ptr[current_w + offset] ==
						    255) {
						found_white_w =
							current_w + offset;
						break;
					}
					offset++;
				}
			}

			// 2. 如果一整行找完都没有白的
			if (found_white_w == -1) {
				continue; // 向上一行继续找
			}

			// 3. 找出这段白线的左右边界，以求中点
			int left_edge = found_white_w;
			int right_edge = found_white_w;

			// 沿找到的白点向左扩展直到黑点
			while (left_edge > 0 && row_ptr[left_edge - 1] == 255) {
				left_edge--;
			}
			// 沿找到的白点向右扩展直到黑点
			while (right_edge < bin_img.cols - 1 &&
			       row_ptr[right_edge + 1] == 255) {
				right_edge++;
			}

			// 计算这段白线的中点
			current_w = (left_edge + right_edge) / 2;

			sp.left = cv::Point(left_edge, h);
			sp.right = cv::Point(right_edge, h);
			return true; // 找到起始点，返回 true
		}
		return false;
	}
	auto find_concave_point(
		std::vector<cv::Point> &approx,
		std::vector<cv::Point> &output_concavePoints,
		std::vector<cv::Point> &output_concavePoints_will_not_be_used,
		int angle_threshold = 150)
	{
		// std::vector<cv::Point> approx;
		// cv::approxPolyDP(contour, approx, approxPolyDP_epsilon,
		// 		 true);

		std::vector<int> hull; // 计算凸点
		cv::convexHull(approx, hull, false, false);
		if (!hull.empty() && hull.size() != approx.size()) {
			std::vector<cv::Vec4i> defects;
			cv::convexityDefects(approx, hull, defects);
			for (const auto &defect : defects) {
				// cv::Vec4i 的结构：
				// [0] = 缺陷起始点的索引
				// [1] = 缺陷结束点的索引
				// [2] = 缺陷最深点（凹点）的索引
				// [3] = 凹点到凸包外边的近似距离（固定扩大了256倍，实际距离需除以
				// 256.0）

				int depth = defect[3] / 256;

				// 过滤掉微小的噪声起伏，只有当凹陷深度大于一定阈值时才认为是凹点
				if (depth > 5) {
					int farthestIdx = defect
						[2]; // 凹点在原 contour 中的索引
					// 计算前一个点、当前点、后一个点组成的角度
					int n = static_cast<int>(approx.size());
					cv::Point prev =
						approx[(farthestIdx - 1 + n) %
						       n];
					cv::Point curr = approx[farthestIdx];
					cv::Point next =
						approx[(farthestIdx + 1) % n];
					cv::Point v1 = prev - curr;
					cv::Point v2 = next - curr;
					double dot = v1.x * v2.x + v1.y * v2.y;
					double len1 = std::sqrt(v1.x * v1.x +
								v1.y * v1.y);
					double len2 = std::sqrt(v2.x * v2.x +
								v2.y * v2.y);
					double cos_angle = dot / (len1 * len2);
					cos_angle = std::clamp(cos_angle, -1.0,
							       1.0);
					double angle = std::acos(cos_angle) *
						       180.0 / CV_PI;
					//过滤掉角度过大的点，避免误判
					if (angle > angle_threshold) {
						output_concavePoints_will_not_be_used
							.push_back(curr);
					} else {
						output_concavePoints.push_back(
							curr);
					}
				}
			}
			return true; // 找到凹点，返回 true
		} else {
			return false; // 没有凹点
		}
	}
};
}