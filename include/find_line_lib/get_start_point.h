#pragma once

#include "common.h"
#include <tuple>

namespace find_line_lib
{

// ==================== 起点查找 ====================
// 在二值图像中查找左右边界起点（或上下边界起点）
// 用于跟踪算法的起始点搜索
//
// 参数说明：
//   bin_img: 输入的二值图像（0=黑，255=白）
//   width, height: 图像宽高
//   start_point: 起始搜索点，默认从底部中间开始
//   left_limit/right_limit: 左右边界限制（默认1和width-1）
//   upper_limit/lower_limit: 上下边界限制
//   direction: 搜索方向，"horizontal"横向（找左右边界）
//              其他值则纵向（找上下边界）
//
// 返回值：
//   找到返回左右/上下两个点的指针，没找到返回nullptr

[[deprecated("请用class imgproc代替")]]
std::tuple<Point, Point> *
get_start_point(const uint8_t *bin_img, int width, int height,
		const Point *start_point = nullptr, int left_limit = 1,
		int right_limit = -1, int upper_limit = 1, int lower_limit = -1,
		const char *direction = "horizontal");
}