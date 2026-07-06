#ifndef APP_DEBUG_H
#define APP_DEBUG_H

#include "app_debug_rtt.h"

/**
 * @brief 初始化 Debug 测试环境。
 *
 * 负责执行调试相关的一次性初始化动作，例如环回自检、
 * 采样链路初始化或其他只应在启动阶段执行一次的调试准备。
 *
 * @note 应在系统基础初始化完成后调用。
 */
void app_debug_init(void);

/**
 * @brief 执行一轮 Debug 测试调度。
 *
 * 用于在主循环中周期性推进当前启用的调试测试项。
 * 当前由 Debug 编排层统一调度具体测试用例，调用方无需感知
 * 各子测试模块的实现细节。
 */
void app_debug_run_once(void);

#endif /* APP_DEBUG_H */
