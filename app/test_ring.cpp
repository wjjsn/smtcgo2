/**
 * @file test_ring.cpp
 * @brief 测试圆环检测相关功能 - 发现圆环、准备入环、准备出环
 *
 * 本测试程序全面测试圆环状态机的三个阶段：
 *   1. discover_ring: 发现圆环入口
 *   2. prepare_enter_ring: 准备进入圆环
 *   3. prepare_exit_ring: 准备驶出圆环
 *
 * 圆环状态机说明：
 *   - NotFound: 未发现圆环
 *   - Discovered: 发现圆环
 *   - PrepareEnter: 准备进入
 *   - PrepareExit: 准备驶出
 *   - AboutToExit: 即将驶出
 *   - Exiting: 正在驶出
 *
 * 使用方法：
 *   ./app_ring.exe
 *   （需要 png/ 目录下有相应测试图片）
 *
 * 测试图片：
 *   - png/found_left_ring.jpg, png/found_right_ring.jpg: 发现圆环测试
 *   - png/enter_left_ring_0.jpg 等: 准备入环测试
 *   - png/exit_left_ring_1.jpg 等: 准备出环测试
 */

#include <opencv2/opencv.hpp>
#include "find_line_lib/get_start_point.h"
#include "find_line_lib/ring.h"
#include "find_line_lib/common.h"
#include "find_line_lib/status_switcher.h"

/**
 * @brief 主函数
 * @return 程序退出码，始终为0
 *
 * 测试分为三个部分：
 *
 * 【第一部分】测试发现圆环 (discover_ring)
 *   遍历左右圆环的测试图片
 *   调用 discover_ring 检测圆环入口点
 *   在图像上标记左右入口点并显示
 *
 * 【第二部分】测试准备入环 (prepare_enter_ring)
 *   遍历入环测试图片
 *   首先调用 get_start_point 获取搜索起点
 *   然后调用 prepare_enter_ring 计算入环路径
 *   在图像上标记入环点并显示
 *
 * 【第三部分】测试准备出环 (prepare_exit_ring)
 *   遍历出环测试图片
 *   同样先获取搜索起点
 *   调用 prepare_exit_ring 计算出环路径
 *   参数包含角度和距离补偿值
 *   在图像上标记出环点并显示
 */
int main()
{
	// ==================== 第一部分：测试发现圆环 ====================
	std::cout << "测试发现圆环" << std::endl;

	// 定义左右圆环的测试图片路径
	std::vector<std::string> images = { "png/found_left_ring.jpg",
					    "png/found_right_ring.jpg" };

	// 遍历每张图片进行测试
	for (const auto &img_path : images) {
		// 加载RGB图像
		cv::Mat rgb_img = cv::imread(img_path);
		if (rgb_img.empty()) {
			std::cout << "Failed to load: " << img_path
				  << std::endl;
			continue;
		}

		// 转换为灰度图
		cv::Mat gray;
		cv::cvtColor(rgb_img, gray, cv::COLOR_BGR2GRAY);

		// 二值化处理
		cv::Mat bin_img;
		cv::threshold(gray, bin_img, 127, 255, cv::THRESH_BINARY);

		// 调用 discover_ring 检测圆环
		// 参数：二值图像数据、宽度、高度、搜索领域大小
		auto result = find_line_lib::discover_ring(
			bin_img.data, bin_img.cols, bin_img.rows, 9);

		if (result == nullptr) {
			std::cout << "发现圆环失败: " << img_path << std::endl;
			continue;
		}

		// 获取返回的左右入口点坐标
		auto lp = std::get<0>(*result);
		auto rp = std::get<1>(*result);

		// 在图像上标记左入口点（红色圆点）
		cv::circle(rgb_img, cv::Point(lp.x, lp.y), 3,
			   cv::Scalar(0, 0, 255), -1);
		// 在图像上标记右入口点（蓝色圆点）
		cv::circle(rgb_img, cv::Point(rp.x, rp.y), 3,
			   cv::Scalar(255, 0, 0), -1);

		// 打印检测结果
		std::cout << "发现圆环: (" << lp.x << "," << lp.y << ") -> ("
			  << rp.x << "," << rp.y << ")" << std::endl;

		// 显示带标记的图像
		cv::imshow("rgb_img", rgb_img);
		cv::waitKey(0);
	}

	// ==================== 第二部分：测试准备入环 ====================
	std::cout << "\n测试准备入环" << std::endl;

	// 定义准备入环阶段的测试图片
	std::vector<std::string> enter_images = { "png/enter_left_ring_0.jpg",
						  "png/enter_left_ring_1.jpg",
						  "png/enter_right_ring_0.jpg",
						  "png/enter_right_ring_1.jpg",
						  "png/sz-1.jpg",
						  "png/sz-2.jpg" };

	for (const auto &img_path : enter_images) {
		// 加载图像
		cv::Mat rgb_img = cv::imread(img_path);
		if (rgb_img.empty()) {
			std::cout << "Failed to load: " << img_path
				  << std::endl;
			continue;
		}

		// 图像预处理：灰度化 + 二值化
		cv::Mat gray;
		cv::cvtColor(rgb_img, gray, cv::COLOR_BGR2GRAY);
		cv::Mat bin_img;
		cv::threshold(gray, bin_img, 127, 255, cv::THRESH_BINARY);

		int h = bin_img.rows;
		int w = bin_img.cols;

		// 首先获取搜索起点
		// 从图像底部中心向上搜索
		find_line_lib::Point start_pt(w / 2, h - 1);
		auto start_result = find_line_lib::get_start_point(
			bin_img.data, w, h, &start_pt, 1, w - 1, 1, h - 1,
			"horizontal");
		if (start_result == nullptr) {
			std::cout << "获取起点失败: " << img_path << std::endl;
			continue;
		}

		// 创建状态切换器（虽然入环阶段可能不需要ring_type）
		find_line_lib::StatusSwitcher ss;

		// 调用 prepare_enter_ring 准备入环
		// 参数：二值图数据、宽高、圆环类型、搜索起点
		auto result = find_line_lib::prepare_enter_ring(
			bin_img.data, w, h, &ss, start_result);
		if (result == nullptr) {
			std::cout << "准备入环失败: " << img_path << std::endl;
			// 失败时也显示图像，方便调试
			cv::imshow("bin_img", bin_img);
			cv::imshow("rgb_img", rgb_img);
			cv::waitKey(0);
			continue;
		}

		// 获取入环路径点
		auto lp = std::get<0>(*result);
		auto rp = std::get<1>(*result);

		// 标记并显示
		cv::circle(rgb_img, cv::Point(lp.x, lp.y), 3,
			   cv::Scalar(0, 0, 255), -1);
		cv::circle(rgb_img, cv::Point(rp.x, rp.y), 3,
			   cv::Scalar(255, 0, 0), -1);

		std::cout << "准备入环成功: (" << lp.x << "," << lp.y
			  << ") -> (" << rp.x << "," << rp.y << ")"
			  << std::endl;

		cv::imshow("bin_img", bin_img);
		cv::imshow("rgb_img", rgb_img);
		cv::waitKey(0);
	}

	// ==================== 第三部分：测试准备出环 ====================
	std::cout << "\n测试准备出环" << std::endl;

	// 定义准备出环阶段的测试图片
	std::vector<std::string> exit_images = { "png/exit_left_ring_1.jpg",
						 "png/exit_right_ring_1.jpg",
						 "png/sz-1.jpg",
						 "png/sz-2.jpg" };

	for (const auto &img_path : exit_images) {
		cv::Mat rgb_img = cv::imread(img_path);
		if (rgb_img.empty()) {
			std::cout << "Failed to load: " << img_path
				  << std::endl;
			continue;
		}

		// 同样的图像预处理
		cv::Mat gray;
		cv::cvtColor(rgb_img, gray, cv::COLOR_BGR2GRAY);
		cv::Mat bin_img;
		cv::threshold(gray, bin_img, 127, 255, cv::THRESH_BINARY);

		int h = bin_img.rows;
		int w = bin_img.cols;

		// 获取搜索起点
		find_line_lib::Point start_pt(w / 2, h - 1);
		auto start_result = find_line_lib::get_start_point(
			bin_img.data, w, h, &start_pt, 1, w - 1, 1, h - 1,
			"horizontal");
		if (start_result == nullptr) {
			std::cout << "获取起点失败: " << img_path << std::endl;
			continue;
		}

		// 创建状态切换器
		find_line_lib::StatusSwitcher ss;
		ss.ring_type = find_line_lib::RingType::Left;

		// 调用 prepare_exit_ring 准备出环
		// 参数比其他函数多，包含角度和距离的补偿值
		// 这些补偿值可能用于校准出环路径的预测
		auto result = find_line_lib::prepare_exit_ring(
			bin_img.data, w, h, &ss, start_result, 37, 70, -9, 17);
		if (result == nullptr) {
			std::cout << "准备出环失败: " << img_path << std::endl;
			cv::imshow("bin_img", bin_img);
			cv::imshow("rgb_img", rgb_img);
			cv::waitKey(0);
			continue;
		}

		auto lp = std::get<0>(*result);
		auto rp = std::get<1>(*result);

		cv::circle(rgb_img, cv::Point(lp.x, lp.y), 3,
			   cv::Scalar(0, 0, 255), -1);
		cv::circle(rgb_img, cv::Point(rp.x, rp.y), 3,
			   cv::Scalar(255, 0, 0), -1);

		std::cout << "准备出环成功: (" << lp.x << "," << lp.y
			  << ") -> (" << rp.x << "," << rp.y << ")"
			  << std::endl;

		cv::imshow("bin_img", bin_img);
		cv::imshow("rgb_img", rgb_img);
		cv::waitKey(0);
	}

	// 关闭所有OpenCV窗口
	cv::destroyAllWindows();
	return 0;
}