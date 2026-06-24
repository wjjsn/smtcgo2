/**
 * @file check_roadblock.h
 * @brief 路障检测函数声明
 *
 * 本模块负责在二值图像中检测路障（障碍物），并返回避障引导连线
 */

#pragma once

#include "common.h"
#include <tuple>

namespace find_line_lib
{

/**
 * @brief 检查二值图像中的路障，返回避障引导连线
 *
 * 算法步骤：
 *   1. 图像反转（黑变白），将路障变成连通域
 *   2. 使用 cv::connectedComponentsWithStats 找所有连通域
 *   3. 筛选：面积>100，不贴边，在起点下方
 *   4. 取最下方的连通域作为目标路障
 *   5. 计算到左右边界的距离，判断在左边还是右边
 *   6. 返回对应的避障引导线
 * @param binary_img 二值图像 (0=黑/道路外/路障, 255=白/道路)
 * @param width 图像宽度
 * @param height 图像高度
 * @param start_point 起始点坐标 (左起点, 右起点)
 * @param threshold 距离阈值，路障边缘与对应边界起点横向距离<=此值时触发避障
 * @return 左侧路障返回 (左起点, 黑块右下角点)，右侧路障返回 (右起点, 黑块左下角点)，无路障返回 nullptr
 */
std::tuple<Point, Point> *
check_roadblock(const uint8_t *binary_img, int width, int height,
		const std::tuple<Point, Point> *start_point, int threshold);

}