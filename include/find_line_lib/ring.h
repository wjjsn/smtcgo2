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
#include "find_line_lib/get_start_point.h"
#include <algorithm>
#include <filesystem>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <string>
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

constexpr int img_width = 160;
constexpr int img_height = 120;
class ring {
	RingStatus ring_status_;
	RingType ring_type_;
	int frame_number_ = 0;

	bool
	try_discover_ring(const uint8_t *bin_img,
			  const std::vector<cv::Point_<int> > &best_contour,
			  int width, int height, int split)
	{
		return false;
	}

    public:
	ring()
	{
#ifdef SMTC2GO_DEBUG
#ifdef SMTC2GO_DEBUG_IMSHOW
		cv::namedWindow("Result", cv::WINDOW_NORMAL);
		cv::resizeWindow("Result", 400, 300);
#endif // SMTC2GO_DEBUG_IMSHOW
#endif
#ifdef SMTC2GO_DEBUG
#ifdef SMTC2GO_DEBUG_SAVE_IMAGES
		std::filesystem::create_directories("debug_frames/未发现");
		std::filesystem::create_directories("debug_frames/发现圆坏");
		std::filesystem::create_directories("debug_frames/已发现");
		std::filesystem::create_directories("debug_frames/准备入环");
		std::filesystem::create_directories("debug_frames/准备出环");
		std::filesystem::create_directories("debug_frames/即将出环");
		std::filesystem::create_directories("debug_frames/出环中");
#endif // SMTC2GO_DEBUG_SAVE_IMAGES
#endif
		ring_status_ = RingStatus::NotFound;
		ring_type_ = RingType::None;
	}
	/*接收一个彩色图片，返回一个补好线了的图片*/
	cv::Mat check_ring(cv::Mat &raw_image)
	{
		TRACE_SCOPE("整帧处理");
		++frame_number_;

		// 图像预处理
		cv::Mat resized;
		cv::Mat dilated;
		{
			TRACE_SCOPE("图像预处理");
			cv::resize(raw_image, resized,
				   cv::Size(img_width, img_height));

			cv::Mat gray;
			cv::cvtColor(resized, gray, cv::COLOR_BGR2GRAY);

			cv::Mat blurred;
			cv::GaussianBlur(gray, blurred, cv::Size(3, 3), 0);

			cv::Mat binary;
			cv::threshold(blurred, binary, 127, 255,
				      cv::THRESH_BINARY);

			cv::Mat element = cv::getStructuringElement(
				cv::MORPH_RECT, cv::Size(3, 3));
			cv::Mat eroded;
			cv::erode(binary, eroded, element, cv::Point(-1, -1),
				  2);
			cv::dilate(eroded, dilated, element, cv::Point(-1, -1),
				   1);
		} // preprocess

		// 轮廓检测
		std::vector<std::vector<cv::Point> > contours;
		{
			TRACE_SCOPE("检测所有轮廓");
			cv::findContours(dilated, contours, cv::noArray(),
					 cv::RETR_TREE,
					 cv::CHAIN_APPROX_SIMPLE);
		}
		cv::Mat result_img = resized.clone();
		int best_contour_index = -1;
		{
			TRACE_SCOPE("找到对的轮廓");
			int min_area = img_width * img_height * 10 /
				       100; // get_start_point 获取 L/R 起点
			uint8_t full_binary[img_height * img_width];
			for (int y = 0; y < img_height; ++y)
				for (int x = 0; x < img_width; ++x)
					full_binary[y * img_width + x] =
						dilated.at<uint8_t>(y, x);

			int ref_center_x = img_width / 2;
			Point start_pt(img_width / 2, img_height - 1);
			auto start_result = get_start_point(
				full_binary, img_width, img_height, &start_pt,
				1, img_width - 1, 1, img_height - 1,
				"horizontal");
			if (start_result != nullptr) {
				auto &left_pt = std::get<0>(*start_result);
				auto &right_pt = std::get<1>(*start_result);
				ref_center_x = (left_pt.x + right_pt.x) / 2;
			}

			// 筛选最底部居中轮廓
			int best_dist = img_width;
			for (size_t i = 0; i < contours.size(); ++i) {
				double area = cv::contourArea(contours[i]);
				if (area < min_area)
					continue;
				cv::Rect bounding =
					cv::boundingRect(contours[i]);
				int bottom_y = bounding.y + bounding.height;
				if (bottom_y < img_height - 2)
					continue;
				int contour_center_x =
					bounding.x + bounding.width / 2;
				int dist = std::abs(contour_center_x -
						    ref_center_x);
				if (dist < best_dist) {
					best_dist = dist;
					best_contour_index =
						static_cast<int>(i);
				}
			}
		}

		std::vector<cv::Point> concavePoints;
		std::vector<cv::Point> concavePoints_will_not_be_used;
		{
			TRACE_SCOPE("找到凹点");
			if (best_contour_index >= 0) {
				std::vector<cv::Point> approx;
				cv::approxPolyDP(
					contours.at(best_contour_index), approx,
					2, true); //

				std::vector<int> hull; // 计算凸点
				cv::convexHull(approx, hull, false, false);

				if (hull.size() == approx.size()) {
					// 纯凸出的形状
#ifdef SMTC2GO_DEBUG
					LOG_DEBUG(
						"原%d个点，化简后%d个点",
						contours.at(best_contour_index)
							.size(),
						approx.size());
					cv::drawContours(
						result_img,
						std::vector<
							std::vector<cv::Point> >{
							approx },
						0, cv::Scalar(0, 255, 0), 2);
					for (auto i = 0; i < approx.size();
					     ++i) {
						cv::putText(
							result_img,
							std::to_string(i),
							approx.at(i),
							cv::FONT_HERSHEY_SIMPLEX,
							0.4,
							cv::Scalar(0, 0, 255));
					}
#endif
				} else if (!hull.empty()) {
					std::vector<cv::Vec4i> defects;
					cv::convexityDefects(approx, hull,
							     defects);
					for (const auto &defect : defects) {
						// cv::Vec4i 的结构：
						// [0] = 缺陷起始点的索引
						// [1] = 缺陷结束点的索引
						// [2] = 缺陷最深点（凹点）的索引
						// [3] = 凹点到凸包外边的近似距离（固定扩大了256倍，实际距离需除以
						// 256.0）

						int depth = defect[3] / 256;

						// 过滤掉微小的噪声起伏，只有当凹陷深度大于一定阈值时才认为是凹点
						if (depth > 5) {
							int farthestIdx = defect
								[2]; // 凹点在原 contour 中的索引
							// 计算前一个点、当前点、后一个点组成的角度
							int n = static_cast<int>(
								approx.size());
							cv::Point prev = approx
								[(farthestIdx -
								  1 + n) %
								 n];
							cv::Point curr = approx
								[farthestIdx];
							cv::Point next = approx
								[(farthestIdx +
								  1) %
								 n];
							cv::Point v1 =
								prev - curr;
							cv::Point v2 =
								next - curr;
							double dot =
								v1.x * v2.x +
								v1.y * v2.y;
							double len1 = std::sqrt(
								v1.x * v1.x +
								v1.y * v1.y);
							double len2 = std::sqrt(
								v2.x * v2.x +
								v2.y * v2.y);
							double cos_angle =
								dot /
								(len1 * len2);
							cos_angle = std::clamp(
								cos_angle, -1.0,
								1.0);
							double angle =
								std::acos(
									cos_angle) *
								180.0 / CV_PI;
								//过滤掉角度过大的点，避免误判
							if (angle > 150.0) {
								concavePoints_will_not_be_used
									.push_back(
										curr);
							} else {
								concavePoints.push_back(
									curr);
							}
						}
					}
#ifdef SMTC2GO_DEBUG
					// LOG_DEBUG(
					// 	"原%d个点，化简后%d个点",
					// 	contours.at(best_contour_index)
					// 		.size(),
					// 	approx.size());
					cv::drawContours(
						result_img,
						std::vector<
							std::vector<cv::Point> >{
							approx },
						0, cv::Scalar(0, 255, 0), 2);
					for (auto point :
					     concavePoints // 这个点就是要找的角点
					) {
						cv::circle(result_img, point, 3,
							   cv::Scalar(0xA5,
								      0x2A,
								      0x2A),
							   1);
					}
					for (auto point :
					     concavePoints_will_not_be_used // 这是被过滤掉的角点
					) {
						cv::circle(result_img, point, 3,
							   cv::Scalar(0xA5,
								      0x00,
								      0xFF),
							   1);
					}
					for (auto i = 0; i < approx.size();
					     ++i) {
						cv::putText(
							result_img,
							std::to_string(i),
							approx.at(i),
							cv::FONT_HERSHEY_SIMPLEX,
							0.4,
							cv::Scalar(0, 0, 255));
					}

#endif
				}
			}
		}

		// 准备平面二值数组，供 get_start_point 使用
		uint8_t full_binary[img_height * img_width];
		for (int y = 0; y < img_height; ++y)
			for (int x = 0; x < img_width; ++x)
				full_binary[y * img_width + x] =
					dilated.at<uint8_t>(y, x);

		// 辅助 lambda：判断凹点位置
		auto is_at_bottom = [](cv::Point p) { // 是下面的
			return p.y > img_height * 2 / 3;
		};
		auto is_at_left = [](cv::Point p) { return p.x < img_width / 2; }; // 是左边的
		auto is_at_right = [](cv::Point p) { return p.x > img_width / 2; }; // 是右边的
		auto find_nearest_top_left = [](cv::Point a, cv::Point b) { // 找最近左上角点
			return (a.y + a.x) < (b.y + b.x);
		};
		auto find_nearest_top_right = [](cv::Point a, cv::Point b) { // 找最近右上角点
			return (a.y + (img_width - a.x)) < (b.y + (img_width - b.x));
		};
		auto find_topmost = [](cv::Point a, cv::Point b) { // 找最上方点
			return a.y < b.y;
		};

		switch (ring_status_) {
		case RingStatus::NotFound: {
			/*
      以左圆环为例
      1. 左下角有一个凹点
      2. 较小的圆环时，检测到的轮廓区域内部会包括一大坨二值化后为黑色的
      3.
      较大的圆环时，圆环导致轮廓凹进去的部分会检测到一个凹点。这个点和圆环检测到的最窄的点是重合的
      */
			if (!(concavePoints.empty())) {
				for (auto corner : concavePoints) { // 角点
					if (is_at_bottom(corner)) {
						// auto value = discover_ring(
						// 	full_binary, img_width,
						// 	img_height);
						if (!concavePoints_will_not_be_used.empty()) {
							if (is_at_left(
								    corner)) {
#ifdef SMTC2GO_DEBUG
							LOG_DEBUG(
								"发现左圆环，凹点(%d,%d)",
								corner.x, corner.y);
#endif
							ring_status_ = RingStatus::
								Discovered;
							ring_type_ =
								RingType::Left;
							} else if (is_at_right(
									   corner)) {
#ifdef SMTC2GO_DEBUG
							LOG_DEBUG(
								"发现右圆环，凹点(%d,%d)",
								corner.x, corner.y);
#endif
							ring_status_ = RingStatus::
								Discovered;
							ring_type_ =
								RingType::Right;
							} else {
								ring_status_ = RingStatus::
									NotFound;
								ring_type_ =
									RingType::None;
							}
						}
					}
				}
			}
		} break;
		case RingStatus::Discovered: {
			/*
			以左圆环为例
			1. 用get_start_point函数获取起始点
			2. 找到最靠近左上的凹点（优先选择更靠近上方的）
			3. 连接右起始点和凹点
			*/

			if (!(concavePoints.empty())) {
				// 获取左右起始点
				Point sp(img_width / 2, img_height - 1);
				auto start_result = get_start_point(
					full_binary, img_width, img_height, &sp,
					1, img_width - 1, 1, img_height - 1,
					"horizontal");

				if (start_result != nullptr) {
					auto &left_pt =
						std::get<0>(*start_result);
					auto &right_pt =
						std::get<1>(*start_result);

					// 找到最靠近上方的凹点
					auto nearest = std::min_element(
						concavePoints.begin(),
						concavePoints.end(),
						find_topmost);

					if (nearest != concavePoints.end()) {
					if (ring_type_ == RingType::Left) {
						// 左环：连接右起始点和凹点
						cv::line(result_img,
							 cv::Point(right_pt.x,
								   right_pt.y),
							 *nearest,
							 cv::Scalar(0, 0, 255),
							 2);
#ifdef SMTC2GO_DEBUG
						LOG_DEBUG(
							"Discovered: 右起点(%d,%d) -> 凹点(%d,%d)",
							right_pt.x, right_pt.y,
							nearest->x, nearest->y);
#endif
					} else if (ring_type_ == RingType::Right) {
						// 右环：连接左起始点和凹点
						cv::line(result_img,
							 cv::Point(left_pt.x,
								   left_pt.y),
							 *nearest,
							 cv::Scalar(0, 0, 255),
							 2);
#ifdef SMTC2GO_DEBUG
						LOG_DEBUG(
							"Discovered: 左起点(%d,%d) -> 凹点(%d,%d)",
							left_pt.x, left_pt.y,
							nearest->x, nearest->y);
#endif
					}
				}
				}
			} else {
				ring_status_ = RingStatus::PrepareExit;
			}
		} break;

			// case RingStatus::PrepareEnter: {
			// };

		case RingStatus::PrepareExit: {
			if (!(concavePoints.empty())) {
				ring_status_ = RingStatus::AboutToExit;
			}
		} break;

		case RingStatus::AboutToExit: {
			if (!(concavePoints.empty())) {
				auto best_contour =
					contours.at(best_contour_index);

				if (!best_contour.empty()) {
					if (ring_type_ == RingType::Left) {
						// 左环：找轮廓上最靠近左上的点
						auto top_left = std::min_element(
							best_contour.begin(),
							best_contour.end(),
							find_nearest_top_left);

						// 找最靠近左上的凹点
						auto nearest_concave = std::min_element(
							concavePoints.begin(),
							concavePoints.end(),
							find_nearest_top_left);

						// 连接左上角点和凹点
						cv::line(result_img, *top_left,
							 *nearest_concave,
							 cv::Scalar(0, 0, 255), 2);
#ifdef SMTC2GO_DEBUG
						LOG_DEBUG(
							"AboutToExit: 左上角(%d,%d) -> 凹点(%d,%d)",
							top_left->x, top_left->y,
							nearest_concave->x,
							nearest_concave->y);
#endif
					} else if (ring_type_ == RingType::Right) {
						// 右环：找轮廓上最靠近右上的点
						auto top_right = std::min_element(
							best_contour.begin(),
							best_contour.end(),
							find_nearest_top_right);

						// 找最靠近右上的凹点
						auto nearest_concave = std::min_element(
							concavePoints.begin(),
							concavePoints.end(),
							find_nearest_top_right);

						// 连接右上角点和凹点
						cv::line(result_img, *top_right,
							 *nearest_concave,
							 cv::Scalar(0, 0, 255), 2);
#ifdef SMTC2GO_DEBUG
						LOG_DEBUG(
							"AboutToExit: 右上角(%d,%d) -> 凹点(%d,%d)",
							top_right->x, top_right->y,
							nearest_concave->x,
							nearest_concave->y);
#endif
					}
				}
			} else {
				ring_status_ = RingStatus::Exiting;
			}
		} break;

		case RingStatus::Exiting: {
			/*
			以左圆环为例
			1. 无凹点时，画一条斜率为-1的线经过轮廓质心
			2. 有凹点时，在左边中下方找到凹点则用左起始点连接
			*/
			if (concavePoints.empty()) {
				auto best_contour =
					contours.at(best_contour_index);
				if (!best_contour.empty()) {
					cv::Rect bbox =
						cv::boundingRect(best_contour);
					// 计算轮廓质心
					cv::Moments m =
						cv::moments(best_contour);
					int cx =
						static_cast<int>(m.m10 / m.m00);
					int cy =
						static_cast<int>(m.m01 / m.m00);

					// 画一条指定斜率的线穿过质心
					float slope = (ring_type_ == RingType::Left)
						      ? 0.5f : -0.5f;
					int half = std::max(bbox.width,
							    bbox.height);
					// 方向向量 (1, slope)，归一化后乘以 half/2
					double len =
						std::sqrt(1.0 + slope * slope);
					int dx = static_cast<int>(
						half / 2.0 / len);
					int dy = static_cast<int>(
						half / 2.0 * slope / len);
					cv::line(result_img,
						 cv::Point(cx - dx, cy - dy),
						 cv::Point(cx + dx, cy + dy),
						 cv::Scalar(0, 0, 255), 2);
					cv::circle(result_img,
						   cv::Point(cx, cy), 1,
						   cv::Scalar(255, 0, 0));
#ifdef SMTC2GO_DEBUG
					LOG_DEBUG(
						"Exiting: 画对角线穿过中心(%d,%d) 斜率%.1f",
						cx, cy, slope);
#endif
				}
			} else {
				// 根据环类型在对应侧中下方找凹点
				bool is_left_ring = (ring_type_ == RingType::Left);
				auto it = std::find_if(
					concavePoints.begin(),
					concavePoints.end(), [&](cv::Point p) {
						return (is_left_ring ? is_at_left(p) : is_at_right(p)) &&
						       p.y > img_height / 4;
					});

				if (it != concavePoints.end()) {
					// 用对应侧起始点连接
					Point sp(img_width / 2, img_height - 1);
					auto start_result = get_start_point(
						full_binary, img_width,
						img_height, &sp, 1,
						img_width - 1, 1,
						img_height - 1, "horizontal");

					if (start_result != nullptr) {
						auto &side_pt = is_left_ring
							? std::get<0>(*start_result)
							: std::get<1>(*start_result);
						cv::line(result_img,
							 cv::Point(side_pt.x,
								   side_pt.y),
							 *it,
							 cv::Scalar(0, 0, 255),
							 2);
						if ((it->y) >
						    img_height * 4 / 5) {
							LOG_INFO(
								"已出环: 凹点(%d,%d)",
								it->x, it->y);
							ring_status_ =
								RingStatus::
									NotFound;
							break;
						}
#ifdef SMTC2GO_DEBUG
						LOG_DEBUG(
							"Exiting: %s起点(%d,%d) -> 凹点(%d,%d)",
							is_left_ring ? "左" : "右",
							side_pt.x, side_pt.y,
							it->x, it->y);
#endif
					}
				} else {
					ring_status_ = RingStatus::NotFound;
				}
			}
		} break;
		}
#ifdef SMTC2GO_DEBUG_IMSHOW
		cv::Mat result_display;
		cv::resize(result_img, result_display, cv::Size(400, 300));
		cv::imshow("Result", result_display);
#endif
		return result_img;
	}
};
} // namespace find_line_lib