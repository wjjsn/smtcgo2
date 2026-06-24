/**
 * @file targetboard.h
 * @brief 目标板检测相关函数声明
 *
 * 本模块负责检测赛道上的目标板（红色标志物）并根据状态补线
 */

#pragma once

#include "common.h"
#include <tuple>
#include <vector>
#include <opencv2/opencv.hpp>

namespace find_line_lib
{

/**
 * @brief 根据 HSV 颜色空间创建红色区域掩码
 *
 * 红色在 HSV 空间中分布在两端（0-15 和 150-180），需要两个范围
 */
cv::Mat
create_red_mask(const cv::Mat &hsv,
		const cv::Scalar &lower_red1 = cv::Scalar(0, 100, 100),
		const cv::Scalar &upper_red1 = cv::Scalar(15, 255, 255),
		const cv::Scalar &lower_red2 = cv::Scalar(150, 100, 100),
		const cv::Scalar &upper_red2 = cv::Scalar(180, 255, 255));

/**
 * @brief 对掩码图像进行形态学操作
 *
 * 操作流程：
 *   1. MORPH_CLOSE: 先膨胀后腐蚀，填充白色区域内部的小黑洞
 *   2. MORPH_OPEN: 先腐蚀后膨胀，去除白色区域边缘的细小噪点
 */
cv::Mat apply_morphology(const cv::Mat &mask, int kernel_size = 3);

/**
 * @brief 筛选位于图像中心区域的红色轮廓
 *
 * 筛选条件：
 *   - 轮廓中心 X 坐标位于图像中央的 center_ratio 范围内
 *   - 轮廓面积大于 min_area
 *   - 轮廓宽高大于指定的最小值
 */
std::vector<std::tuple<std::vector<cv::Point>, double, int, int, int, int> >
filter_center_contours(const std::vector<std::vector<cv::Point> > &contours,
		       int img_width, float center_ratio = 0.5,
		       double min_area = 1000, int min_width = 40,
		       int min_height = 20);

/**
 * @brief 从轮廓点集中找出四个角点（左上、右上、右下、左下）
 *
 * 算法原理：
 *   - 左上角 (tl): x + y 的值最小
 *   - 右下角 (br): x + y 的值最大
 *   - 右上角 (tr): x - y 的值最大
 *   - 左下角 (bl): x - y 的值最小
 */
std::tuple<cv::Point, cv::Point, cv::Point, cv::Point>
find_corner_points(const std::vector<cv::Point> &points);

/**
 * @brief 在图像上绘制角点（圆点）
 */
void draw_corner_points(
	cv::Mat &result,
	const std::tuple<cv::Point, cv::Point, cv::Point, cv::Point> &corners,
	const cv::Scalar &color = cv::Scalar(0, 255, 0), int radius = 3,
	int thickness = -1);

/**
 * @brief 检测目标板（红色标志物）
 *
 * @param rgb_img RGB 彩色图像
 * @param ss 状态切换器（当前未使用，为保持接口一致而保留）
 * @return 目标板的四个角点 (tl, tr, br, bl)，检测失败返回 nullptr
 */
std::tuple<cv::Point, cv::Point, cv::Point, cv::Point> *
check_targetboard(const cv::Mat &rgb_img, StatusSwitcher *ss);

/**
 * @brief 根据识别状态计算补线的两个端点
 *
 * 补线逻辑：
 *   - 识别为左岔路：用右边界起点连接目标板左下角 (bl)
 *   - 识别为右岔路：用左边界起点连接目标板右下角 (br)
 *   - 其他状态（未发现/直行）：不需要补线
 */
std::tuple<Point, Point> *supplement_line_by_status(
	StatusSwitcher *ss, const std::tuple<Point, Point> *start_point,
	const std::tuple<cv::Point, cv::Point, cv::Point, cv::Point>
		*targetboard_points);

}