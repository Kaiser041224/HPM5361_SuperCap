/*
 * IRQ Profiler Implementation - Low-intrusion ISR timing
 *
 * Copyright (c) 2026 Alliance HardWare Team
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Author: Kaiser
 */

#include "irq_profiler.h"

#include "hpm_interrupt.h" /* [TEMP DIAG] disable_global_irq for nest-aware busy accounting */

#include <limits.h>
#include <stddef.h>
#include <string.h>

volatile irq_prof_cycle_t g_irq_prof_stamp[IRQ_PROF_MAX_SLOTS];
volatile irq_prof_raw_t   g_irq_prof_raw[IRQ_PROF_MAX_SLOTS];

/* [TEMP DIAG] 嵌套感知的中断总占用测量：只在最外层 ISR 进入(nest 0→1)记起点、
 * 最外层退出(1→0)累加墙钟 cycle，内层嵌套不重复计。得到 CPU 真正处于中断态的
 * 墙钟时间，物理上 ≤ 100%，消除"抢占时间被每层重复计入"的虚高。定位完成后移除。 */
volatile uint64_t g_irq_busy_cycles;
static volatile uint8_t  s_irq_nest_depth;
static volatile uint32_t s_irq_outer_t0;

void irq_prof_nest_enter(void) {
    uint32_t m = disable_global_irq(CSR_MSTATUS_MIE_MASK);
    if (s_irq_nest_depth == 0U) {
        s_irq_outer_t0 = irq_prof_read_cycle();
    }
    s_irq_nest_depth++;
    restore_global_irq(m);
}

void irq_prof_nest_exit(void) {
    uint32_t m = disable_global_irq(CSR_MSTATUS_MIE_MASK);
    if (s_irq_nest_depth > 0U) {
        s_irq_nest_depth--;
        if (s_irq_nest_depth == 0U) {
            g_irq_busy_cycles += (uint64_t)(irq_prof_read_cycle() - s_irq_outer_t0);
        }
    }
    restore_global_irq(m);
}

static const char *g_labels[IRQ_PROF_MAX_SLOTS];
static uint8_t     g_slot_count = 0;
static irq_prof_cycle_t g_overhead_cycles = 0;

irq_prof_id_t irq_prof_register(const char *label)
{
    if (g_slot_count >= IRQ_PROF_MAX_SLOTS) {
        return UINT8_MAX;
    }

    irq_prof_id_t id = g_slot_count++;
    g_labels[id] = (label != NULL) ? label : "???";

    memset((void *)&g_irq_prof_raw[id], 0, sizeof(irq_prof_raw_t));
    g_irq_prof_raw[id].min = UINT32_MAX;

    return id;
}

irq_prof_cycle_t irq_prof_measure_overhead(void)
{
    irq_prof_cycle_t min_diff = UINT32_MAX;

    for (int i = 0; i < 100; i++) {
        irq_prof_cycle_t t0 = irq_prof_read_cycle();
        irq_prof_cycle_t t1 = irq_prof_read_cycle();
        irq_prof_cycle_t diff = t1 - t0;
        if (diff < min_diff) {
            min_diff = diff;
        }
    }

    g_overhead_cycles = min_diff;
    return min_diff;
}

int irq_prof_get_result(irq_prof_id_t id, irq_prof_result_t *result)
{
    if (id >= g_slot_count || result == NULL) {
        return -1;
    }

    irq_prof_raw_t snapshot;

    uint32_t mstatus;
    __asm__ volatile("csrr %0, mstatus" : "=r"(mstatus) :: "memory");
    __asm__ volatile("csrci mstatus, 0x8" ::: "memory");
    snapshot = *(irq_prof_raw_t *)&g_irq_prof_raw[id];
    __asm__ volatile("csrw mstatus, %0" :: "r"(mstatus) : "memory");

    uint32_t valid_hits = snapshot.hits - snapshot.outliers;

    result->last_ns    = snapshot.last;
    result->min_ns     = (valid_hits > 0) ? snapshot.min : 0;
    result->max_ns     = snapshot.max;
    result->avg_ns     = (valid_hits > 0) ? (uint32_t)(snapshot.total / valid_hits) : 0;
    result->hits       = snapshot.hits;
    result->outliers   = snapshot.outliers;
    result->overhead_ns = g_overhead_cycles;

    return 0;
}

uint8_t irq_prof_get_slot_count(void)
{
    return g_slot_count;
}

const char *irq_prof_get_label(irq_prof_id_t id)
{
    return (id < g_slot_count) ? g_labels[id] : "???";
}

irq_prof_cycle_t irq_prof_get_overhead_cycles(void)
{
    return g_overhead_cycles;
}
