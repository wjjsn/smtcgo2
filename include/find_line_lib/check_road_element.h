/**
 * @file check_road_element.h
 * @brief 赛道元素处理函数声明
 *
 * 本模块是赛道元素处理的统一入口，根据状态切换器 (StatusSwitcher)
 * 的状态决定如何处理图像中的赛道元素（圆环、路障、目标板等）
 */

#pragma once

#include "common.h"

namespace find_line_lib
{

/**
 * @brief 处理赛道上的各种元素（圆环、路障、目标板等）
 *
 * 当前实现为空框架，各个状态的处理逻辑待填充
 */
uint8_t *process_track_element(const uint8_t *img, int width, int height,
			       StatusSwitcher *ss);

}