// ==================== 状态切换器实现 ====================
//
// 状态切换器负责管理车辆行驶过程中的各种状态转换
// 包括：圆环状态、赛道类型、目标板检测等
//
// 当前实现：StatusSwitcher 结构体已在 common.h 中定义
// 此文件保留用于扩展状态切换逻辑

#include "find_line_lib/status_switcher.h"

namespace find_line_lib
{

// 状态切换器的主要功能：
// 1. 圆环状态机管理
//    NotFound -> Discovered -> PrepareEnter -> PrepareExit -> AboutToExit -> Exiting
//
// 2. 赛道类型识别
//    根据视觉信息判断前方是左岔路、直道还是右岔路
//
// 3. 目标板检测
//    检测是否看到目标板，以及是否正在通过目标板

}