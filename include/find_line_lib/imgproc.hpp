#pragma once
#include "opencv2/opencv.hpp"
#include "find_line_lib/road.hpp"
namespace find_line_lib
{
class imgproc {
	int img_width_ = 160;
	int img_height_ = 120;

    public:
	imgproc(int img_width, int img_height)
		: img_width_(img_width)
		, img_height_(img_height) {};

	auto find_best_contour(cv::Mat &bin_img,
			       std::vector<std::vector<cv::Point> > &contours)
	{
		int ref_center_x = img_width_ / 2;
		int ref_center_y = img_height_ / 2;
		bool has_ref_point = false;
		start_point sp;
		auto start_result = road::get_start_point(bin_img, sp);
		if (start_result) {
			ref_center_x = (sp.left.x + sp.right.x) / 2;
			ref_center_y = (sp.left.y + sp.right.y) / 2;
			has_ref_point = true;
		}

		// 筛选最底部居中轮廓
		int best_dist = img_width_;
		int best_contour_index = -1;
		for (size_t i = 0; i < contours.size(); ++i) {
			double area = cv::contourArea(contours[i]);
			cv::Rect bounding = cv::boundingRect(contours[i]);
			int bottom_y = bounding.y + bounding.height;
			if (bottom_y < img_height_ - 2)
				continue;
			// 参考点必须在轮廓内
			if (has_ref_point) {
				cv::Point ref_pt(ref_center_x, ref_center_y);
				if (cv::pointPolygonTest(contours[i], ref_pt,
							 false) < 0)
					continue;
			}
			int contour_center_x = bounding.x + bounding.width / 2;
			int dist = std::abs(contour_center_x - ref_center_x);
			if (dist < best_dist) {
				best_dist = dist;
				best_contour_index = static_cast<int>(i);
			}
		}
		return best_contour_index;
	}
};
}