#pragma once
#include "opencv2/opencv.hpp"
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <vector>
namespace find_line_lib
{

struct start_point {
	cv::Point left;
	cv::Point right;
};

class road {
	/*
    默认从底部的中间开始找
    向左向右找白点，如果一整行找完都没有白的，则向上一行继续找
    如果找的白的了，则找出这段白线的中点，作为下一行的起点
    如果下一行是全黑，则结束，否则循环
    */
    public:
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
};
}