#pragma once

#include "common.h"

namespace find_line_lib
{

// ==================== 状态切换器 ====================
// 状态切换器的实现在 common.h 中定义
// 这里只做声明引用，实际的状态切换逻辑在 status_switcher.cpp 中实现

// 注意：StatusSwitcher 结构体已经在 common.h 中完整定义
// 包括以下状态字段：
//   - ring_status: 圆环检测状态机
//   - ring_type: 圆环类型（左/右）
//   - model_status: 赛道类型识别
//   - target_board_status: 目标板检测状态

}