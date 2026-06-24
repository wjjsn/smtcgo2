/**
 * @file test_check_roadblock.cpp
 * @brief 测试路障检测功能 - check_roadblock
 *
 * 本测试程序验证在图像中检测路障（Roadblock）的能力。
 * 路障是赛道中的一种障碍物，需要被检测出来以便进行路径规划。
 *
 * 算法原理：
 *   1. 加载测试图片并预处理（缩放、灰度化、二值化）
 *   2. 调用 get_start_point 找到赛道边界起点
 *   3. 调用 check_roadblock 在二值图上检测路障位置
 *   4. 如果检测到路障，在图像上画一条红线标记
 *
 * 使用方法：
 *   ./app_check_roadblock.exe
 *   （自动遍历 png/*.jpg 所有图片）
 *
 * 输出说明：
 *   对于每张图片：
 *     - 显示原图
 *     - 如果检测到路障，画出标记线
 *     - 等待按键继续下一张
 */

#include <opencv2/opencv.hpp>
#include "find_line_lib/check_roadblock.h"
#include "find_line_lib/get_start_point.h"
#include "find_line_lib/common.h"

/**
 * @brief 主函数
 * @return 程序退出码，始终为0
 *
 * 处理流程：
 *   1. 使用 cv::glob 遍历 png/ 目录下所有 .jpg 文件
 *   2. 对每张图片按顺序进行处理：
 *      a. 加载并缩放到 160x120（智能车摄像头常见分辨率）
 *      b. 灰度化 + 二值化（阈值127）
 *      c. 获取赛道边界起点
 *      d. 调用 check_roadblock 检测路障
 *      e. 如果检测成功，在原图上画出两点之间的连线
 *      f. 显示结果并等待按键
 */
int main()
{
	// 用于存储所有找到的图片路径
	std::vector<std::string> image_files;

	// 遍历 png/ 目录下所有 .jpg 文件
	// cv::glob 会递归搜索，但这里只搜索根目录
	cv::glob("png/*.jpg", image_files);

	// 对文件列表排序，保证处理顺序的一致性
	std::sort(image_files.begin(), image_files.end());

	// 遍历每张图片进行处理
	for (const auto &img_file : image_files) {
		// 加载图片
		cv::Mat img = cv::imread(img_file);
		if (img.empty()) {
			// 加载失败则跳过（可能是权限问题或文件损坏）
			continue;
		}

		// 缩放到 160x120，智能车视觉常用分辨率
		cv::resize(img, img, cv::Size(160, 120));

		// 转换为灰度图
		cv::Mat gray;
		cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

		// 二值化处理
		// 灰度值 > 127 的像素变为 255（白色），表示赛道
		// 灰度值 <= 127 的像素变为 0（黑色），表示背景
		cv::Mat bin_img;
		cv::threshold(gray, bin_img, 127, 255, cv::THRESH_BINARY);

		// 获取图像尺寸
		int h = bin_img.rows;
		int w = bin_img.cols;

		// 首先调用 get_start_point 获取搜索起点
		// 从图像底部中心 (w/2, h-1) 向上搜索赛道边界
		// 搜索范围限制在 x: [1, w-1], y: [1, h-1]
		find_line_lib::Point start_pt(w / 2, h - 1);
		auto start_result = find_line_lib::get_start_point(
			bin_img.data, w, h, &start_pt, 1, w - 1, 1, h - 1,
			"horizontal");

		// 如果无法获取起点，说明图像中可能没有赛道，跳过
		if (start_result == nullptr) {
			continue;
		}

		// 调用 check_roadblock 检测路障
		// 参数：
		//   bin_img.data: 二值图像数据
		//   w, h: 图像宽高
		//   start_result: 赛道边界起点
		//   60: 检测阈值，可能表示路障的最小尺寸
		auto result = find_line_lib::check_roadblock(bin_img.data, w, h,
							     start_result, 60);

		// 检查是否检测到路障
		if (result == nullptr) {
			// 未检测到路障，直接显示原图
			cv::imshow("result", img);
			cv::waitKey(0);
			continue;
		}

		// 成功检测到路障，获取两个标记点
		auto p1 = std::get<0>(*result);
		auto p2 = std::get<1>(*result);

		// 在图像上画一条红线连接两个点，线宽为2像素
		// 这条线表示检测到的路障位置
		cv::line(img, cv::Point(p1.x, p1.y), cv::Point(p2.x, p2.y),
			 cv::Scalar(0, 0, 255), 2);

		// 显示标记后的图像
		cv::imshow("result", img);
		// 等待按键，1表示等待1毫秒即可继续
		cv::waitKey(0);
	}

	// 关闭所有OpenCV创建的窗口
	cv::destroyAllWindows();
	return 0;
}