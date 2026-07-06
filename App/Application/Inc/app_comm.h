/*
 * Application Communication
 *
 * CAN-based host communication protocol handler.
 *
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef APP_COMM_H
#define APP_COMM_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    HOST_CMD_START          = 0x01, /**< 启动功率输出 */
    HOST_CMD_STOP           = 0x02, /**< 停止功率输出 */
    HOST_CMD_SET_MODE       = 0x03, /**< 切换运行模式 */
    HOST_CMD_SET_TARGET     = 0x04, /**< 设置控制目标值 */
    HOST_CMD_SET_PARAM      = 0x05, /**< 修改控制器参数 */
    HOST_CMD_GET_STATUS     = 0x06, /**< 查询当前状态 */
    HOST_CMD_CLEAR_FAULT    = 0x07, /**< 清除故障锁存 */
    HOST_CMD_ENTER_DIAG     = 0x08, /**< 进入诊断模式 */
    HOST_CMD_EXIT_DIAG      = 0x09, /**< 退出诊断模式 */
} host_cmd_t;

int  app_comm_init(void);
void app_comm_tick(void);
void app_comm_report_fault(uint32_t fault_code);
void app_comm_send_telemetry(void);
bool app_comm_is_timeout(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_COMM_H */
