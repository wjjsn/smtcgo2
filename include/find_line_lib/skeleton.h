#pragma once

#include <cstdint>
#include <tuple>
#include <vector>

constexpr int MAX_PIXELS = 640 * 480; // 最大图像尺寸

namespace find_line_lib
{

// ==================== 骨架分析结果 ====================
// 描述一条路径的分支结构
struct SkeletonAnalysisResult {
	int bifurcation_count; // 分叉点数量（邻居>=3）
	int endpoint_count; // 端点数量（邻居=1）
	int branch_count; // 分支数量估算
	std::vector<std::tuple<int, int> > bifurcation_points; // 分叉点坐标列表
	std::vector<std::tuple<int, int> > endpoint_points; // 端点坐标列表
	int left_start_x; // 左起始点x
	int left_start_y; // 左起始点y
	int right_start_x; // 右起始点x
	int right_start_y; // 右起始点y

	SkeletonAnalysisResult()
		: bifurcation_count(0)
		, endpoint_count(0)
		, branch_count(0)
		, left_start_x(0)
		, left_start_y(0)
		, right_start_x(0)
		, right_start_y(0)
	{
	}
};

// ==================== 圆环检测结果 ====================
// 描述检测到的圆环信息
struct RingDetectionResult {
	bool has_ring; // 是否检测到圆环
	int ring_center_x; // 环中心x坐标
	char ring_side; // 环在左边还是右边 ('l'/'r'/'n')
	int ring_area; // 环的面积

	RingDetectionResult()
		: has_ring(false)
		, ring_center_x(0)
		, ring_side('n')
		, ring_area(0)
	{
	}
};

// ==================== 骨架提取（Zhang-Suen算法） ====================
// 输入：二值图（0或255）
// 输出：骨架图（跟输入同尺寸）
// 原理：迭代删除满足条件的边界点，直到不再变化
// 两个条件配合用：
//   条件1：删除东南西四个方向的边界点
//   条件2：删除北西南三个方向的边界点
// 交替执行，直到没有点被删除
void skeletonize(const uint8_t *binary_img, uint8_t *skeleton, int width,
		 int height);

// ==================== 骨架分析 ====================
// 在骨架上找特征点
// 8邻域连接数：
//   = 1：端点（线的尽头）
//   >= 3：分叉点（岔路口）
// 返回：分叉点列表、端点列表、分支数量
SkeletonAnalysisResult analyze_skeleton(const uint8_t *skeleton, int width,
					int height, const uint8_t *binary_img);

// ==================== 圆环检测 ====================
// 原理：
//   1. 对骨架做膨胀（1像素 -> 5x5的块）
//   2. 用连通域分析找所有轮廓
//   3. 看哪些轮廓被其他轮廓包含（内孔）
//   4. 有内孔的说明是闭合环
// 参数：
//   min_area_ratio: 内孔面积占图面积比例阈值（越小越宽松）
//   dilate_kernel_size: 膨胀核大小（越大越容易补闭断裂环）
//   dilate_iterations: 膨胀迭代次数
// 返回：是否有环、环中心位置、环在左还是右
RingDetectionResult detect_ring(const uint8_t *skeleton, int width, int height,
				double min_area_ratio = 0.05,
				int dilate_kernel_size = 5,
				int dilate_iterations = 1);

}