/**
 * @file test_get_start_point.cpp
 * @brief 测试 find_line_lib::get_start_point() 函数 - 找边界起点
 *
 * 本测试程序用于验证在二值图像中找到赛道左右边界起点的功能。
 * 算法从图像中间位置出发，向左和向右搜索白色像素的边界点。
 *
 * 算法原理：
 *   1. 从图像中心行的指定位置开始
 *   2. 向左搜索白色像素的右边缘（黑白跳变点）
 *   3. 向右搜索白色像素的左边缘
 *   4. 返回左右两个边界起点坐标
 *
 * 使用方法：
 *   ./app_get_start_point.exe
 *   （需要 png/roadblock_left.jpg 存在）
 *
 * 预期输出：
 *   打印左右边界起点的坐标，并在图像上用圆点标记
 */

#include <opencv2/opencv.hpp>
#include "find_line_lib/get_start_point.h"
#include "find_line_lib/common.h"

/**
 * @brief 主函数
 * @return 程序退出码，0表示正常退出，1表示加载图像失败
 *
 * 测试流程：
 *   1. 加载测试图像 png/roadblock_left.jpg
 *   2. 将图像缩放到 640x480 大小
 *   3. 转换为灰度图
 *   4. 进行二值化处理（阈值127）
 *   5. 调用 get_start_point 找左右边界起点
 *   6. 在原图上标记找到的起点（红色=左边界起点，蓝色=右边界起点）
 *   7. 显示结果图像，等待按键退出
 */
int main()
{
	// 加载测试图像
	cv::Mat img = cv::imread("png/roadblock_left.jpg");
	if (img.empty()) {
		std::cout << "Failed to load image" << std::endl;
		return 1;
	}

	// 将图像缩放到 640x480（标准测试分辨率）
	cv::resize(img, img, cv::Size(640, 480));

	// 转换为灰度图
	cv::Mat gray;
	cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

	// 二值化：灰度值>127的变为255（白色），否则变为0（黑色）
	cv::Mat bin_img;
	cv::threshold(gray, bin_img, 127, 255, cv::THRESH_BINARY);

	// 获取图像尺寸
	int h = bin_img.rows;
	int w = bin_img.cols;

	// 调用 get_start_point 找边界起点
	// 参数说明：
	//   bin_img.data: 二值图像数据指针
	//   w, h: 图像宽高
	//   Point(w/2, h/2): 搜索起点（图像中心）
	//   1, w-1: 搜索范围的左右边界
	//   1, h-1: 搜索范围的上下边界
	//   "horizontal": 水平方向搜索模式
	find_line_lib::Point start_pt(w / 2, h / 2);
	auto result = find_line_lib::get_start_point(bin_img.data, w, h,
						     &start_pt, 1, w - 1, 1,
						     h - 1, "horizontal");

	// 检查是否成功找到边界起点
	if (result != nullptr) {
		// 提取左右边界起点坐标
		auto start_point_l = std::get<0>(*result);
		auto start_point_r = std::get<1>(*result);

		// 在图像上标记左边界起点（红色圆点，半径3，填充）
		cv::circle(img, cv::Point(start_point_l.x, start_point_l.y), 3,
			   cv::Scalar(0, 0, 255), -1);
		// 在图像上标记右边界起点（蓝色圆点，半径3，填充）
		cv::circle(img, cv::Point(start_point_r.x, start_point_r.y), 3,
			   cv::Scalar(255, 0, 0), -1);

		// 打印左右边界起点坐标到控制台
		std::cout << "Left: (" << start_point_l.x << ", "
			  << start_point_l.y << ")" << std::endl;
		std::cout << "Right: (" << start_point_r.x << ", "
			  << start_point_r.y << ")" << std::endl;
	} else {
		// 未能找到有效的边界起点
		std::cout << "Failed to get start point" << std::endl;
	}

	// 显示标记了起点的图像
	cv::imshow("img", img);
	// 等待按键，0表示无限等待
	cv::waitKey(0);
	// 关闭所有OpenCV创建的窗口
	cv::destroyAllWindows();
	return 0;
}