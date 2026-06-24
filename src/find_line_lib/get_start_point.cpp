#include "find_line_lib/get_start_point.h"

#include <cstring>
#include <tuple>

namespace find_line_lib
{

static std::tuple<Point, Point> result_buffer;

// ==================== 起点查找实现 ====================
// 核心思路：从图像底部中间开始往上/往左搜索
// 找黑白跳变点作为边界起点
//
// 横向搜索（horizontal）：
//   从底部中间往上扫，找白->黑的跳变
//   左边：往左找白(255)右邻居是黑(0)的点
//   右边：往右找白(255)左邻居是黑(0)的点
//
// 纵向搜索（其他方向）：
//   从中间往两边扫，找白->黑的跳变
//   上边：往上找白(255)下邻居是黑(0)的点
//   下边：往下找白(255)上邻居是黑(0)的点
//
// 如果在指定列找不到跳变，但该列起始位置是白的
// 就用边界限制值作为默认起点
std::tuple<Point, Point> *get_start_point(const uint8_t *bin_img, int width,
					  int height, const Point *start_point,
					  int left_limit, int right_limit,
					  int upper_limit, int lower_limit,
					  const char *direction)
{
	// 参数合法性检查，默认值设置
	if (left_limit <= 0)
		left_limit = 1;
	if (right_limit < 0)
		right_limit = width - 1;
	if (upper_limit <= 0)
		upper_limit = 1;
	if (lower_limit < 0)
		lower_limit = height - 1;

	int center_col = width / 2;
	int init_col = center_col;
	int init_row = height - 1;

	// 如果传入了起始点，就从那里开始搜
	if (start_point != nullptr) {
		init_col = start_point->x;
		init_row = start_point->y;
	}

	Point left_point(0, 0);
	Point right_point(0, 0);

	// ==================== 横向搜索（找左右边界） ====================
	if (strcmp(direction, "horizontal") == 0) {
		// 从底部往上扫
		for (int row = init_row; row >= upper_limit; --row) {
			left_point = Point(0, 0);
			right_point = Point(0, 0);

			// 往左找：白(255) -> 黑(0) 的跳变点（白的右边缘）
			for (int col = init_col; col > left_limit; --col) {
				int idx = row * width + col;
				int idx_prev = row * width + (col - 1);
				if (bin_img[idx] == 255 &&
				    bin_img[idx_prev] == 0) {
					left_point = Point(col, row);
					break;
				}
			}

			// 如果没找到跳变点，但起始位置是白的，用边界限制值
			if (left_point.x == 0 && left_point.y == 0) {
				int init_idx = row * width + init_col;
				if (bin_img[init_idx] == 255) {
					left_point = Point(left_limit, row);
				}
			}

			// 往右找：白(255) -> 黑(0) 的跳变点（白的左边缘）
			for (int col = init_col; col < right_limit; ++col) {
				int idx = row * width + col;
				int idx_next = row * width + (col + 1);
				if (bin_img[idx] == 255 &&
				    bin_img[idx_next] == 0) {
					right_point = Point(col, row);
					break;
				}
			}

			// 如果没找到跳变点，但起始位置是白的，用边界限制值
			if (right_point.x == 0 && right_point.y == 0) {
				int init_idx = row * width + init_col;
				if (bin_img[init_idx] == 255) {
					right_point = Point(right_limit, row);
				}
			}

			// 找到一个有效的行就返回
			if (left_point.x != 0 || left_point.y != 0) {
				if (right_point.x != 0 || right_point.y != 0) {
					result_buffer = std::make_tuple(
						left_point, right_point);
					return &result_buffer;
				}
			}
		}
	}
	// ==================== 纵向搜索（找上下边界） ====================
	else {
		int init_col_v = 0;
		int init_row_v = height / 2;

		if (start_point != nullptr) {
			init_col_v = start_point->x;
			init_row_v = start_point->y;
		}

		// 从中间往右扫
		for (int col = init_col_v; col < right_limit; ++col) {
			Point upper_point(0, 0);
			Point lower_point(0, 0);

			// 往上找：白(255) -> 黑(0) 的跳变点（白的下边缘）
			for (int row = init_row_v; row >= upper_limit; --row) {
				int idx = row * width + col;
				int idx_prev = (row - 1) * width + col;
				if (bin_img[idx] == 255 &&
				    bin_img[idx_prev] == 0) {
					upper_point = Point(col, row);
					break;
				}
			}

			if (upper_point.x == 0 && upper_point.y == 0) {
				int init_idx = init_row_v * width + init_col_v;
				if (bin_img[init_idx] == 255) {
					upper_point = Point(col, upper_limit);
				}
			}

			// 往下找：白(255) -> 黑(0) 的跳变点（白的下边缘）
			for (int row = init_row_v; row <= lower_limit; ++row) {
				int idx = row * width + col;
				int idx_next = (row + 1) * width + col;
				if (bin_img[idx] == 255 &&
				    bin_img[idx_next] == 0) {
					lower_point = Point(col, row);
					break;
				}
			}

			if (lower_point.x == 0 && lower_point.y == 0) {
				int init_idx = init_row_v * width + init_col_v;
				if (bin_img[init_idx] == 255) {
					lower_point = Point(col, lower_limit);
				}
			}

			if (upper_point.x != 0 || upper_point.y != 0) {
				if (lower_point.x != 0 || lower_point.y != 0) {
					result_buffer = std::make_tuple(
						upper_point, lower_point);
					return &result_buffer;
				}
			}
		}
	}

	return nullptr;
}

}