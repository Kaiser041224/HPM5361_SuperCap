#ifndef APP_DEBUG_CAN_H
#define APP_DEBUG_CAN_H

/**
 * @brief 执行 CAN 调试测试。
 *
 * 该测试负责推进当前 CAN 调试用例，包括发送测试帧、轮询接收结果，
 * 以及打印状态与统计信息。
 *
 * @note 适合在主循环中周期性调用。
 */
void app_debug_can_run_tests(void);
void app_debug_can_loopback_test(void);

#endif /* APP_DEBUG_CAN_H */
