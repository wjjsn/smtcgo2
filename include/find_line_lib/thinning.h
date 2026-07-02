#pragma once

#include <opencv2/core.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>

namespace cv
{
namespace ximgproc
{
enum ThinningTypes {
	THINNING_ZHANGSUEN = 0, // Thinning technique of Zhang-Suen
	THINNING_GUOHALL = 1 // Thinning technique of Guo-Hall
};
void thinning(InputArray src, OutputArray dst,
	      int thinningType = THINNING_ZHANGSUEN);
}
}