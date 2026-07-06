/*
 * Debug RTT - SEGGER RTT wrapper for debug output
 *
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef APP_DEBUG_RTT_H
#define APP_DEBUG_RTT_H

#include <stdint.h>

int app_debug_printf(const char *fmt, ...);

/**
 * @brief 获取 RTT 写入成功次数。
 *
 * @return 已成功写入 RTT 上行缓冲的字符串条数。
 */
uint32_t app_debug_rtt_get_write_ok(void);

/**
 * @brief 获取 RTT 写入被丢弃次数。
 *
 * @return 因 RTT 上行缓冲不足被整条丢弃的字符串条数。
 */
uint32_t app_debug_rtt_get_write_dropped(void);

#endif /* APP_DEBUG_RTT_H */
