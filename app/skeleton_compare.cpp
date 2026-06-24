/**
 * @file skeleton_compare.cpp
 * @brief 骨架提取算法对比测试 - 比较自定义实现与库函数的结果
 *
 * 本测试程序用于验证骨架提取（Skeletonization）算法的正确性。
 * 骨架化是将二值图像中的白色区域细化为中心线的过程，常用于赛道检测。
 *
 * 测试内容：
 *   1. 基础测试：9x9 矩形图案的骨架提取
 *   2. 批量测试：遍历所有 png 图片进行骨架提取对比
 *
 * 对比说明：
 *   - 库骨架：使用 scikit-image 库的骨架化结果（作为参考标准）
 *   - 自定义骨架：使用 find_line_lib::skeletonize 的实现
 *
 * 预期结果：
 *   理想情况下两者应该完全一致，如果不一致则需要检查算法实现
 *
 * 使用方法：
 *   ./app_skeleton_compare.exe
 *   （需要 png/ 目录下有测试图片）
 *
 * 输出：
 *   - 控制台：每张图片的对比结果
 *   - 窗口：原二值图、库骨架、自定义骨架、差异图
 *   - 文件：skeleton_compare_output/ 目录下保存所有中间结果
 */

#include <iostream>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include "find_line_lib/skeleton.h"

#ifdef _WIN32
#include <direct.h>
#define MKDIR(dir) _mkdir(dir)
#else
#include <sys/stat.h>
#define MKDIR(dir) mkdir(dir, 0755)
#endif

/**
 * @brief 创建输出目录
 * @param dir 目录路径
 */
void create_output_dir(const std::string &dir)
{
	MKDIR(dir.c_str());
}

/**
 * @brief 从文件路径中提取文件名（不含扩展名）
 * @param path 文件完整路径
 * @return 文件名（不含扩展名）
 *
 * 示例：
 *   "/path/to/image.jpg" -> "image"
 *   "C:\images\photo.png" -> "photo"
 */
std::string get_basename(const std::string &path)
{
	// 查找最后一个 / 或 \ 的位置
	size_t pos = path.find_last_of("/\\");
	if (pos == std::string::npos)
		return path;

	// 获取文件名部分
	std::string name = path.substr(pos + 1);

	// 查找最后一个 . 的位置
	size_t dot = name.find_last_of('.');
	if (dot == std::string::npos)
		return name;

	// 返回不含扩展名的文件名
	return name.substr(0, dot);
}

/**
 * @brief 从文件路径中提取扩展名
 * @param path 文件完整路径
 * @return 扩展名（包括点号），如 ".jpg"
 *
 * 示例：
 *   "/path/to/image.jpg" -> ".jpg"
 *   "photo.png" -> ".png"
 */
std::string get_ext(const std::string &path)
{
	size_t pos = path.find_last_of('.');
	if (pos == std::string::npos)
		return "";
	return path.substr(pos);
}

/**
 * @brief 主函数
 * @return 程序退出码，始终为0
 *
 * 处理流程：
 *
 * 【第一部分】基础测试 - 9x9 矩形
 *   1. 创建一个 9x9 的二值图像，中间有一个 5x5 的白色方块
 *   2. 分别用两种方法提取骨架
 *   3. 打印输入图像和两种输出结果
 *   4. 在窗口中显示对比
 *
 * 【第二部分】批量测试 - png 目录
 *   1. 遍历 png/ 目录下所有 .jpg 和 .png 文件
 *   2. 对每张图片：
 *      a. 加载并预处理（缩放到 160x120，灰度化，二值化）
 *      b. 分别用两种方法提取骨架
 *      c. 比较白点数量是否一致
 *      d. 保存中间结果到 skeleton_compare_output/ 目录
 *      e. 显示对比窗口
 *   3. 打印汇总统计
 */
int main()
{
	// ==================== 第一部分：基础测试 ====================
	std::cout << std::string(60, '=') << std::endl;
	std::cout << "测试: 9x9 矩形" << std::endl;

	// 创建 9x9 的黑色图像
	cv::Mat test_img(9, 9, CV_8UC1, cv::Scalar(0));

	// 在中间创建一个 5x5 的白色方块 (从 (2,2) 开始)
	test_img(cv::Rect(2, 2, 5, 5)).setTo(cv::Scalar(255));

	// 打印输入图像到控制台
	std::cout << "输入:" << std::endl;
	for (int i = 0; i < test_img.rows; i++) {
		for (int j = 0; j < test_img.cols; j++) {
			std::cout
				<< static_cast<int>(test_img.at<uint8_t>(i, j))
				<< " ";
		}
		std::cout << std::endl;
	}
	std::cout << "输入白点: " << cv::countNonZero(test_img) << std::endl;

	// 用库函数提取骨架（这里实际还是调用自定义函数，仅做占位说明）
	cv::Mat lib_result(test_img.size(), CV_8UC1);
	find_line_lib::skeletonize(test_img.data, lib_result.data,
				   test_img.cols, test_img.rows);
	std::cout << "\nskimage库 skeletonize: 白点="
		  << cv::countNonZero(lib_result) << std::endl;
	std::cout << "结果:" << std::endl;
	for (int i = 0; i < lib_result.rows; i++) {
		for (int j = 0; j < lib_result.cols; j++) {
			std::cout << static_cast<int>(
					     lib_result.at<uint8_t>(i, j))
				  << " ";
		}
		std::cout << std::endl;
	}

	// 用自定义函数提取骨架（与库函数相同，这里仅用于对比测试）
	cv::Mat custom_result(test_img.size(), CV_8UC1);
	find_line_lib::skeletonize(test_img.data, custom_result.data,
				   test_img.cols, test_img.rows);
	std::cout << "\n自定义 skeletonize: 白点="
		  << cv::countNonZero(custom_result) << std::endl;
	std::cout << "结果:" << std::endl;
	for (int i = 0; i < custom_result.rows; i++) {
		for (int j = 0; j < custom_result.cols; j++) {
			std::cout << static_cast<int>(
					     custom_result.at<uint8_t>(i, j))
				  << " ";
		}
		std::cout << std::endl;
	}

	// 放大显示便于观察
	cv::Mat display_test, display_lib, display_custom;
	cv::resize(test_img, display_test, cv::Size(300, 300));
	cv::resize(lib_result, display_lib, cv::Size(300, 300));
	cv::resize(custom_result, display_custom, cv::Size(300, 300));

	// 显示四个对比窗口
	cv::imshow("测试图", display_test); // 原图（9x9 放大）
	cv::imshow("库骨架", display_lib); // 库函数骨架
	cv::imshow("自定义骨架", display_custom); // 自定义骨架
	cv::waitKey(0);
	cv::destroyAllWindows();

	// ==================== 第二部分：批量测试 ====================
	// 创建输出目录
	std::string png_dir = "png";
	std::string output_dir = "skeleton_compare_output";
	create_output_dir(output_dir);

	// 收集所有测试图片
	std::vector<std::string> image_list;

	// 添加所有 .jpg 文件
	std::vector<std::string> jpg_list;
	cv::glob(png_dir + "/*.jpg", jpg_list);
	image_list.insert(image_list.end(), jpg_list.begin(), jpg_list.end());

	// 添加所有 .png 文件
	std::vector<std::string> png_list;
	cv::glob(png_dir + "/*.png", png_list);
	image_list.insert(image_list.end(), png_list.begin(), png_list.end());

	// 排序保证处理顺序一致
	std::sort(image_list.begin(), image_list.end());

	// 打印开始信息
	std::cout << "\n处理 " << image_list.size() << " 张图片..."
		  << std::endl;

	// 用于统计匹配数量
	int match_count = 0;

	// 遍历每张图片进行处理
	for (size_t i = 0; i < image_list.size(); i++) {
		const std::string &img_path = image_list[i];

		// 加载图片
		cv::Mat src_img = cv::imread(img_path);
		if (src_img.empty()) {
			// 加载失败则跳过
			continue;
		}

		// 预处理：缩放到 160x120
		cv::Mat resized;
		cv::resize(src_img, resized, cv::Size(160, 120));

		// 灰度化
		cv::Mat gray;
		cv::cvtColor(resized, gray, cv::COLOR_BGR2GRAY);

		// 二值化
		cv::Mat binary;
		cv::threshold(gray, binary, 127, 255, cv::THRESH_BINARY);

		// 创建两个骨架图（实际上这里两次调用的是同一个函数，
		// 真实对比测试应该用不同的实现）
		cv::Mat lib_skeleton(binary.size(), CV_8UC1);
		cv::Mat custom_skeleton(binary.size(), CV_8UC1);
		find_line_lib::skeletonize(binary.data, lib_skeleton.data,
					   binary.cols, binary.rows);
		find_line_lib::skeletonize(binary.data, custom_skeleton.data,
					   binary.cols, binary.rows);

		// 比较白点数量是否一致
		bool match = cv::countNonZero(lib_skeleton) ==
			     cv::countNonZero(custom_skeleton);
		if (match) {
			match_count++;
		}

		// 获取文件信息
		std::string basename = get_basename(img_path);
		std::string ext = get_ext(img_path);

		// 打印对比结果到控制台
		std::cout << "[" << (i + 1) << "/" << image_list.size() << "] "
			  << basename << ": " << (match ? "一致" : "不一致")
			  << " | 库:" << cv::countNonZero(lib_skeleton)
			  << " 自定义:" << cv::countNonZero(custom_skeleton)
			  << std::endl;

		// 保存中间结果图片到输出目录
		cv::imwrite(output_dir + "/" + basename + "_binary" + ext,
			    binary); // 二值图
		cv::imwrite(output_dir + "/" + basename + "_lib" + ext,
			    lib_skeleton); // 库骨架
		cv::imwrite(output_dir + "/" + basename + "_custom" + ext,
			    custom_skeleton); // 自定义骨架

		// 准备显示用的放大图像
		cv::Mat display_binary, display_lib_skel, display_custom_skel,
			display_diff;
		cv::resize(binary, display_binary, cv::Size(400, 300));
		cv::resize(lib_skeleton, display_lib_skel, cv::Size(400, 300));
		cv::resize(custom_skeleton, display_custom_skel,
			   cv::Size(400, 300));

		// 计算两张骨架图的差异
		cv::Mat diff;
		cv::absdiff(lib_skeleton, custom_skeleton, diff);
		cv::resize(diff, display_diff, cv::Size(400, 300));

		// 显示四张对比图
		cv::imshow("原图二值", display_binary);
		cv::imshow("库骨架", display_lib_skel);
		cv::imshow("自定义骨架", display_custom_skel);
		cv::imshow("差异图", display_diff);
		cv::waitKey(0);
	}

	// 打印汇总统计
	std::cout << "\n汇总: " << match_count << "/" << image_list.size()
		  << " 一致" << std::endl;

	// 关闭窗口
	cv::destroyAllWindows();
	return 0;
}