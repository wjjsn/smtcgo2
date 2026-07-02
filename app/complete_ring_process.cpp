#include <opencv2/opencv.hpp>

#include <iostream>
#include <vector>
#include <algorithm>

#include "find_line_lib/calculate_wheel_speeds.h"
#include "find_line_lib/ring.h"
#include "find_line_lib/road.hpp"
#include "find_line_lib/thinning.h"
find_line_lib::ring ring;
find_line_lib::road road;
int main()
{
	std::cout << "OpenCV loaded OK" << std::endl;

	cv::VideoCapture cap("png/right_ring.mp4");
	if (!cap.isOpened()) {
		std::cout << "无法打开视频" << std::endl;
		return 1;
	}

	int frame_count = 0;
	std::vector<cv::Mat> frame_cache;
	frame_cache.reserve(4330);

	std::cout << "正在加载视频帧..." << std::endl;
	cv::Mat frame;
	while (true) {
		cap >> frame;
		if (frame.empty())
			break;
		frame_cache.push_back(frame.clone());
		++frame_count;
	}
	cap.release();
	std::cout << "已加载 " << frame_count << " 帧" << std::endl;

	bool is_playing = true;
	int current_idx = 0;

	cv::namedWindow("center_line", cv::WINDOW_NORMAL);
	cv::resizeWindow("center_line", 400, 300);

	while (current_idx < frame_count) {
		auto bin_img = ring.check_ring(frame_cache[current_idx]);
		cv::copyMakeBorder(bin_img, bin_img, 1, 0, 1, 1,
				   cv::BORDER_CONSTANT, cv::Scalar(0));
		cv::Mat skeleton_img;
		cv::ximgproc::thinning(bin_img, skeleton_img,
				       cv::ximgproc::THINNING_GUOHALL);
		cv::Mat debug_display;
		cv::resize(skeleton_img, debug_display, cv::Size(400, 300));
		cv::imshow("center_line", debug_display);
		// cv::imwrite("debug_frames/" + std::to_string(current_idx) +
		// 		    "_display" + ".png",
		// 	    bin_img);
		// cv::imwrite("debug_frames/" + std::to_string(current_idx) +
		// 		    "_skeleton" + ".png",
		// 	    skeleton_img);

		int wait_time = is_playing ? 1 : 0;
		char key = cv::waitKey(wait_time) & 0xFF;
		if (key == 'q')
			break;
		if (key == ' ') {
			is_playing = !is_playing;
			printf("%s (帧 %d)\n",
			       is_playing ? "继续播放" : "已暂停",
			       current_idx + 1);
		} else if (key == 'd' || key == 83) {
			current_idx =
				std::min(current_idx + 100, frame_count - 1);
			is_playing = false;
		} else if (key == 'a' || key == 81) {
			current_idx = std::max(current_idx - 100, 0);
			is_playing = false;
		}

		if (is_playing) {
			++current_idx;
		}
	}

	cv::destroyAllWindows();

	printf("\n==================================================\n");
	printf("视频处理结束，总帧数: %d\n", frame_count);
	printf("==================================================\n");

	return 0;
}
