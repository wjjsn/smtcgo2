/**
 * @file ring.h
 * @brief 圆环检测相关函数声明
 *
 * 本模块负责智能车圆环检测的三个阶段：
 *   1. discover_ring: 在图像中间区域找最窄线（发现圆环入口）
 *   2. prepare_enter_ring: 扫描下半部分，检测宽度突变（准备进入圆环）
 *   3. prepare_exit_ring: 从下往上扫描，检测宽度突变（准备驶出圆环）
 *
 * 圆环状态机：
 *   NotFound -> Discovered -> PrepareEnter -> PrepareExit -> AboutToExit -> Exiting
 */

#pragma once

#include "common.h"
#include <tuple>

namespace find_line_lib
{

/**
 * @brief 发现圆环：在图像中间区域找左右边界最窄的地方
 *
 * 算法原理：
 *   把图像垂直分成 split 份，只看中间那一份
 *   在中间区域逐行扫描，找左右边界距离最短的那一行
 *   宽度最窄的位置就是圆环入口（因为圆环会压缩赛道宽度）
 */
std::tuple<Point, Point> *discover_ring(const uint8_t *bin_img, int width,
					int height, int split = 9);

/**
 * @brief 准备进入圆环：扫描图像下半部分，检测宽度跳变
 *
 * 算法原理：
 *   从 scan_start_height 开始往下扫描
 *   每隔 scan_row_step 行分析一次左右边界宽度
 *   当检测到宽度突然增加（curr_width - prev_width > jump_threshold）时
 *   说明车正在接近圆环入口（圆环半径大，赛道变宽）
 *   再根据中心点偏移方向判断是左环还是右环
 */
std::tuple<Point, Point> *
prepare_enter_ring(const uint8_t *bin_img, int width, int height,
		   StatusSwitcher *ss,
		   const std::tuple<Point, Point> *start_point,
		   int jump_threshold = 55, int scan_start_height = 50,
		   int scan_row_step = 8, int center_distance_threshold = 20);

/**
 * @brief 准备驶出圆环：从下往上扫描，检测宽度变化
 *
 * 算法原理：
 *   从图像底部往上扫描（scan_row_step 为负数）
 *   当检测到宽度突然增加时，说明正在接近圆环出口
 *   根据中心点偏移方向判断出口在左还是右
 *   （与 prepare_enter_ring 方向相反，但检测逻辑相似）
 *
 * 调试功能：
 *   检测到跳变时会显示 debug 窗口，画出中点连线，等待按键继续
 */
std::tuple<Point, Point> *
prepare_exit_ring(const uint8_t *bin_img, int width, int height,
		  StatusSwitcher *ss,
		  const std::tuple<Point, Point> *start_point,
		  int jump_threshold = 40, int scan_stop_height = 70,
		  int scan_row_step = -8, int center_distance_threshold = 20);

}