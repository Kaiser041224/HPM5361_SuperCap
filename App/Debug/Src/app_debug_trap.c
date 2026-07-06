/*
 * [TEMP DIAG] CPU 异常(fault)诊断钩子。
 *
 * 背景：SDK 默认 exception_handler(weak, toolchains/trap.c)对所有非中断异常
 * (总线错误/非法指令/未对齐访问等)仅 switch-break，不修正 mepc 就返回。若
 * app_gptmr_start_all() 第二次调用内部某条寄存器写触发了此类异常，CPU 会陷入
 * "trap→mepc 不变→重新执行同一条指令→再次 trap" 的硬件级死循环，表现为主循环
 * 彻底静默、RTT 打印一次都不触发——与观测到的现象(after_set_phase 之后再无
 * 任何打印，等待 20s+ 无恢复)完全吻合。
 *
 * 本文件覆盖该 weak 函数：打印 mcause/mepc 后将 mepc += 4 跳过出错指令，
 * 避免真正陷入死循环，以便能看到诊断输出。定位完成后应删除本文件。
 *
 * Copyright (c) 2026 Alliance HardWare Team
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "SEGGER_RTT.h"

#include <stdint.h>

long exception_handler(long cause, long epc) {
    SEGGER_RTT_printf(
        0, "\r\n[FAULT] mcause=0x%08lx (code=%lu) mepc=0x%08lx\r\n", (unsigned long)cause,
        (unsigned long)(cause & 0x7FFFFFFFL), (unsigned long)epc);
    return epc + 4;
}
