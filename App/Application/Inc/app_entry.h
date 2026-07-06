/*
 * Application Entry
 *
 * Top-level application orchestration entry points.
 *
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef APP_ENTRY_H
#define APP_ENTRY_H

#ifdef __cplusplus
extern "C" {
#endif

void app_init(void);
void app_run_once(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_ENTRY_H */
