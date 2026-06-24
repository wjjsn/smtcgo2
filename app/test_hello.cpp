/**
 * @file test_hello.cpp
 * @brief 测试 find_line_lib::hello() 函数
 *
 * 本测试程序用于验证 find_line_lib 库的基本功能，调用 hello() 函数
 * 并打印返回值。这是一个最简单的冒烟测试，用来确认库能够正常链接和调用。
 *
 * 使用方法：
 *   ./app_hello.exe
 *
 * 预期输出：
 *   显示 hello() 函数返回的字符串内容
 */

#include <iostream>
#include "find_line_lib/hello.h"

/**
 * @brief 主函数
 * @return 程序退出码，0表示正常退出
 *
 * 测试流程：
 *   1. 调用 find_line_lib::hello() 获取问候语
 *   2. 将结果打印到标准输出
 *   3. 程序正常退出
 */
int main()
{
	std::cout << find_line_lib::hello() << std::endl;
	return 0;
}