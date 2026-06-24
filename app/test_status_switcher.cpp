/**
 * @file test_status_switcher.cpp
 * @brief 测试状态切换器 - RingStatus 圆环状态机
 *
 * 本测试程序演示如何使用圆环状态机（RingStatus）来处理视频流中的圆环检测。
 * 状态机是智能车比赛中常用的设计模式，用于管理复杂的状态转换逻辑。
 *
 * 圆环状态机状态说明：
 *   - NotFound: 初始状态，未发现圆环
 *   - Discovered: 刚发现圆环，需要确认
 *   - PrepareEnter: 准备进入圆环
 *   - PrepareExit: 准备驶出圆环
 *   - AboutToExit: 即将驶出圆环
 *   - Exiting: 正在驶出圆环
 *
 * 状态转换条件：
 *   NotFound -> Discovered: 连续检测到圆环超过阈值次数
 *   Discovered -> PrepareEnter: 圆环消失超过阈值次数
 *   PrepareEnter -> PrepareExit: 检测到单分支单端点
 *   PrepareExit -> AboutToExit: 检测到双端点
 *   AboutToExit -> Exiting: 再次检测到单分支单端点
 *   Exiting -> NotFound: 检测到多端点
 *
 * 使用方法：
 *   ./app_status_switcher.exe
 *   （需要 png/left_ring.mp4 视频文件存在）
 *
 * 演示说明：
 *   程序跳转到第3000帧开始演示，因为前面的帧可能没有圆环
 */

#include <opencv2/opencv.hpp>
#include "find_line_lib/status_switcher.h"
#include "find_line_lib/ring.h"
#include "find_line_lib/get_start_point.h"
#include "find_line_lib/common.h"

/**
 * @brief 主函数
 * @return 程序退出码，0表示正常，1表示视频打开失败
 *
 * 处理流程：
 *   1. 打开视频文件 png/left_ring.mp4
 *   2. 跳转到第3000帧开始处理（演示用）
 *   3. 循环读取视频帧：
 *      a. 预处理：缩放、灰度化、二值化
 *      b. 根据当前状态执行相应检测：
 *         - PrepareEnter: 调用 prepare_enter_ring
 *         - PrepareExit: 调用 prepare_exit_ring
 *      c. 在图像上标记检测结果
 *      d. 显示彩色图和二值图
 *      e. 检测状态转换条件并更新状态
 *   4. 视频结束或用户按键退出
 */
int main()
{
	// 创建状态切换器实例，管理圆环状态
	find_line_lib::StatusSwitcher ss;

	// 打开测试视频
	cv::VideoCapture cap("png/left_ring.mp4");

	// 手动设置状态为 PrepareExit（用于测试出环逻辑）
	// 实际使用时从未发现状态开始
	ss.ring_status = find_line_lib::RingStatus::PrepareExit;
	ss.ring_type = find_line_lib::RingType::Left;

	// 检查视频是否成功打开
	if (!cap.isOpened()) {
		std::cout << "Failed to open video" << std::endl;
		return 1;
	}

	// 帧计数器
	int frame_count = 0;

	// 出环计数，用于判断是否满足出环条件
	int exit_ring_count = 0;

	// 创建并设置彩色图显示窗口
	cv::namedWindow("color_img", cv::WINDOW_NORMAL);
	cv::resizeWindow("color_img", 400, 300);

	// 创建并设置二值图显示窗口
	cv::namedWindow("bin_img", cv::WINDOW_NORMAL);
	cv::resizeWindow("bin_img", 400, 300);

	// 主循环：逐帧处理视频
	while (true) {
		cv::Mat frame;

		// 读取一帧
		if (!cap.read(frame)) {
			// 视频结束或读取错误，退出循环
			break;
		}

		// 帧计数增加
		frame_count++;

		// 跳转到第3000帧开始演示
		// 这是因为视频前面的帧可能没有圆环，我们从有圆环的地方开始测试
		if (frame_count < 3000) {
			continue;
		}

		// 图像预处理
		cv::Mat img;
		// 缩放到 160x120（智能车视觉常用分辨率）
		cv::resize(frame, img, cv::Size(160, 120));

		// 灰度化
		cv::Mat gray;
		cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

		// 二值化
		cv::Mat bin_img;
		cv::threshold(gray, bin_img, 127, 255, cv::THRESH_BINARY);

		// 获取图像尺寸
		int h = bin_img.rows;
		int w = bin_img.cols;

		// 根据当前状态执行相应的处理
		switch (ss.ring_status) {
		case find_line_lib::RingStatus::NotFound:
			// 未发现状态：不做任何处理，等待检测到圆环
			break;

		case find_line_lib::RingStatus::Discovered:
			// 已发现状态：保持等待，确认圆环存在
			break;

		case find_line_lib::RingStatus::PrepareEnter: {
			// 准备入环状态：调用 prepare_enter_ring 检测入环点
			// 首先获取搜索起点
			find_line_lib::Point start_pt(w / 2, h - 1);
			auto start_result = find_line_lib::get_start_point(
				bin_img.data, w, h, &start_pt, 1, w - 1, 1,
				h - 1, "horizontal");
			if (start_result == nullptr) {
				break;
			}

			// 调用 prepare_enter_ring 获取入环路径点
			// 参数中的 30, 10 可能是方向和距离补偿
			auto result = find_line_lib::prepare_enter_ring(
				bin_img.data, w, h, &ss, start_result, 30, 10);

			if (result == nullptr) {
				// 返回空说明可能已经不在入环阶段，切换到出环状态
				ss.ring_status =
					find_line_lib::RingStatus::PrepareExit;
				std::cout << "切换状态到: "
					  << static_cast<int>(ss.ring_status)
					  << std::endl;
			} else {
				// 成功获取入环点，标记到图像上
				auto lp = std::get<0>(*result);
				auto rp = std::get<1>(*result);
				// 红色圆点标记左入环点
				cv::circle(img, cv::Point(lp.x, lp.y), 3,
					   cv::Scalar(0, 0, 255), -1);
				// 蓝色圆点标记右入环点
				cv::circle(img, cv::Point(rp.x, rp.y), 3,
					   cv::Scalar(255, 0, 0), -1);
			}
			break;
		}

		case find_line_lib::RingStatus::PrepareExit: {
			// 准备出环状态：调用 prepare_exit_ring 检测出环点
			find_line_lib::Point start_pt(w / 2, h - 1);
			auto start_result = find_line_lib::get_start_point(
				bin_img.data, w, h, &start_pt, 1, w - 1, 1,
				h - 1, "horizontal");
			if (start_result == nullptr) {
				break;
			}

			// 调用 prepare_exit_ring 获取出环路径点
			// 参数包含多个补偿值：角度补偿、距离补偿等
			auto result = find_line_lib::prepare_exit_ring(
				bin_img.data, w, h, &ss, start_result, 30, 10,
				-9, 10);

			if (result == nullptr) {
				// 返回空说明没有检测到出环路径，重置计数器
				exit_ring_count = 0;
			} else {
				// 成功获取出环点，标记到图像上
				auto lp = std::get<0>(*result);
				auto rp = std::get<1>(*result);
				cv::circle(img, cv::Point(lp.x, lp.y), 3,
					   cv::Scalar(0, 0, 255), -1);
				cv::circle(img, cv::Point(rp.x, rp.y), 3,
					   cv::Scalar(255, 0, 0), -1);

				// 成功检测到出环点，计数增加
				exit_ring_count++;

				// 连续检测到10次出环点，切换到 Exiting 状态
				if (exit_ring_count >= 10) {
					ss.ring_status = find_line_lib::
						RingStatus::Exiting;
					std::cout << "切换状态到: "
						  << static_cast<int>(
							     ss.ring_status)
						  << std::endl;
				}
			}
			break;
		}

		case find_line_lib::RingStatus::Exiting:
			// 正在出环状态：不做特殊处理
			break;

		default:
			// 未知状态：不应该发生
			break;
		}

		// 显示处理后的彩色图
		cv::imshow("color_img", img);
		// 显示二值图
		cv::imshow("bin_img", bin_img);

		// 等待1毫秒，检查是否有按键输入
		// 如果按下 q 键则退出程序
		char key = cv::waitKey(1) & 0xFF;
		if (key == 'q')
			break;
	}

	// 打印视频处理统计信息
	std::cout << "视频结束，总帧数: " << frame_count << std::endl;

	// 关闭所有OpenCV创建的窗口
	cv::destroyAllWindows();
	return 0;
}