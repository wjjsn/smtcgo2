/**
 * @file test_check_targetboard.cpp
 * @brief 测试目标板检测功能 - check_targetboard 和 supplement_line_by_status
 *
 * 本测试程序验证目标板（TargetBoard）检测以及根据状态补线的功能。
 * 目标板是智能车比赛中的标志物，用于识别赛道类型和位置。
 *
 * 功能说明：
 *   1. check_targetboard: 在图像中检测目标板的位置
 *   2. supplement_line_by_status: 根据当前赛道类型（左右直岔），
 *      在检测到的目标板处补全赛道线
 *
 * 赛道类型 (ModelRecognitionStatus):
 *   - NotFound: 未识别到
 *   - Left: 左边岔路
 *   - Straight: 直道
 *   - Right: 右边岔路
 *
 * 使用方法：
 *   ./app_check_targetboard.exe
 *   （需要 png/targetboard.png 存在）
 *
 * 输出说明：
 *   依次显示四种赛道状态下的补线结果窗口
 */

#include <opencv2/opencv.hpp>
#include <iostream>
#include "find_line_lib/targetboard.h"
#include "find_line_lib/status_switcher.h"
#include "find_line_lib/get_start_point.h"
#include "find_line_lib/common.h"

/**
 * @brief 主函数
 * @return 程序退出码，0表示正常，1表示加载图片或检测失败
 *
 * 处理流程：
 *   1. 加载目标板测试图片 png/targetboard.png
 *   2. 调用 check_targetboard 检测目标板位置
 *   3. 获取赛道边界起点（用于补线计算）
 *   4. 遍历四种赛道状态（NotFound, Left, Straight, Right）
 *   5. 对每种状态调用 supplement_line_by_status 获取补线点
 *   6. 在图像上画出补线并显示
 */
int main()
{
	// 调试输出：标记程序开始执行
	std::cout << "1" << std::endl;

	// 加载目标板测试图片
	cv::Mat img = cv::imread("png/targetboard.png");
	if (img.empty()) {
		std::cout << "Failed to load image" << std::endl;
		return 1;
	}

	// 调试输出：标记图片加载完成
	std::cout << "2" << std::endl;

	// 创建状态切换器，用于管理当前赛道状态
	find_line_lib::StatusSwitcher ss;

	// 调试输出：标记状态切换器创建完成
	std::cout << "3" << std::endl;

	// 调用 check_targetboard 检测目标板
	// 参数：图像数据、宽度、高度、当前赛道识别状态
	// 返回：目标板的两个对角点坐标（如果有）
	auto targetboard_result = find_line_lib::check_targetboard(img, &ss);

	// 调试输出：标记目标板检测完成
	std::cout << "4" << std::endl;

	// 检查目标板检测是否成功
	if (targetboard_result == nullptr) {
		std::cout << "检查目标板失败" << std::endl;
		return 1;
	}

	// 调试输出：标记目标板检测成功
	std::cout << "5" << std::endl;

	// 图像预处理：灰度化 + 二值化
	cv::Mat gray;
	cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
	cv::Mat bin_img;
	cv::threshold(gray, bin_img, 127, 255, cv::THRESH_BINARY);

	int h = bin_img.rows;
	int w = bin_img.cols;

	// 获取赛道边界起点，用于后续补线计算
	// 从图像底部中心向上搜索
	find_line_lib::Point start_pt(w / 2, h - 1);
	auto start_result =
		find_line_lib::get_start_point(bin_img.data, w, h, &start_pt, 1,
					       w - 1, 1, h - 1, "horizontal");

	// 遍历所有可能的赛道状态
	for (int status = 0;
	     status <
	     static_cast<int>(find_line_lib::ModelRecognitionStatus::Right) + 1;
	     status++) {
		// 将整数状态转换为枚举类型
		ss.model_status =
			static_cast<find_line_lib::ModelRecognitionStatus>(
				status);

		// 创建一份图像用于显示（不污染原图）
		cv::Mat show_img = img.clone();

		// 根据当前状态调用 supplement_line_by_status 获取补线点
		// 这个函数根据赛道类型（左、右、直岔）来决定如何补线
		auto line_pts = find_line_lib::supplement_line_by_status(
			&ss, start_result, targetboard_result);

		// 检查是否需要补线
		if (line_pts != nullptr) {
			// 成功获取补线点
			auto p1 = std::get<0>(*line_pts);
			auto p2 = std::get<1>(*line_pts);

			// 在图像上画一条绿线（2像素宽）连接两个补线点
			cv::line(show_img, cv::Point(p1.x, p1.y),
				 cv::Point(p2.x, p2.y), cv::Scalar(0, 255, 0),
				 2);

			// 打印补线结果到控制台
			std::cout << "状态[" << status << "]: 成功获取补线点 ("
				  << p1.x << "," << p1.y << ") -> (" << p2.x
				  << "," << p2.y << ")" << std::endl;
		} else {
			// 该状态下无需补线（例如直道可能不需要额外补线）
			std::cout << "状态[" << status << "]: 无需补线"
				  << std::endl;
		}

		// 创建以状态编号命名的窗口，显示补线结果
		// 每个状态对应一个独立的窗口，方便对比
		cv::imshow(std::to_string(status), show_img);
		// 等待按键，每个状态按一次键继续
		cv::waitKey(0);
	}

	// 关闭所有OpenCV窗口
	cv::destroyAllWindows();
	return 0;
}