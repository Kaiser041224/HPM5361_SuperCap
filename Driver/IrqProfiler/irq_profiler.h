/*
 * IRQ Profiler - Low-intrusion ISR timing analysis
 *
 * Designed for 200kHz/400kHz PWM/ADC fast-loop control on HPM RISC-V MCU.
 * ISR overhead: ~10 cycles (2 reads + 2 writes + 1 branch).
 *
 * Copyright (c) 2026 Alliance HardWare Team
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Author: Kaiser
 */

#ifndef IRQ_PROFILER_H
#define IRQ_PROFILER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Configuration - user can override via build system
 * ============================================================================ */

/* Set to 0 to completely disable profiler (macros become empty) */
#ifndef IRQ_PROF_ENABLED
#define IRQ_PROF_ENABLED 1
#endif

/* Set to 1 to enable GPIO pulse output for oscilloscope verification */
#ifndef IRQ_PROF_GPIO_ENABLED
#define IRQ_PROF_GPIO_ENABLED 0
#endif

/* Outlier threshold in cycles (480MHz * 10us = 4800 cycles) */
#ifndef IRQ_PROF_OUTLIER_CYCLES
#define IRQ_PROF_OUTLIER_CYCLES 4800
#endif

/* Max measurement slots */
#ifndef IRQ_PROF_MAX_SLOTS
#define IRQ_PROF_MAX_SLOTS 16
#endif

/* ============================================================================
 * Types - keep minimal for fast ISR writes
 * ============================================================================ */

typedef uint8_t  irq_prof_id_t;
typedef uint32_t irq_prof_cycle_t;

/* Raw stat stored in ISR context - no float, no conversion */
typedef struct {
    irq_prof_cycle_t last;
    irq_prof_cycle_t min;
    irq_prof_cycle_t max;
    uint64_t         total;
    uint32_t         hits;
    uint32_t         outliers;
} irq_prof_raw_t;

/* Converted result for dump context only */
typedef struct {
    uint32_t last_ns;
    uint32_t min_ns;
    uint32_t max_ns;
    uint32_t avg_ns;
    uint32_t hits;
    uint32_t outliers;
    uint32_t overhead_ns;
} irq_prof_result_t;

/* ============================================================================
 * Hardware abstraction - weak hooks for GPIO observation
 * ============================================================================ */

/* User implements these in board layer for oscilloscope verification */
void irq_prof_gpio_set(uint8_t slot_id)   __attribute__((weak));
void irq_prof_gpio_clear(uint8_t slot_id) __attribute__((weak));

/* ============================================================================
 * Core API - must be inline for minimal ISR overhead
 * ============================================================================ */

static inline irq_prof_cycle_t irq_prof_read_cycle(void)
{
    irq_prof_cycle_t val;
    __asm__ volatile("csrr %0, mcycle" : "=r"(val));
    return val;
}

/* Registration - call from main context */
irq_prof_id_t irq_prof_register(const char *label);

/* ISR context - minimal operations */
static inline void irq_prof_enter(irq_prof_id_t id)
{
    extern volatile irq_prof_cycle_t g_irq_prof_stamp[IRQ_PROF_MAX_SLOTS];
    if (id < IRQ_PROF_MAX_SLOTS) {
        g_irq_prof_stamp[id] = irq_prof_read_cycle();
    }
}

static inline void irq_prof_exit(irq_prof_id_t id)
{
    extern volatile irq_prof_raw_t   g_irq_prof_raw[IRQ_PROF_MAX_SLOTS];
    extern volatile irq_prof_cycle_t g_irq_prof_stamp[IRQ_PROF_MAX_SLOTS];

    if (id >= IRQ_PROF_MAX_SLOTS) {
        return;
    }

    irq_prof_cycle_t elapsed = irq_prof_read_cycle() - g_irq_prof_stamp[id];

    volatile irq_prof_raw_t *r = &g_irq_prof_raw[id];
    r->last = elapsed;
    r->hits++;

    if (elapsed > IRQ_PROF_OUTLIER_CYCLES) {
        r->outliers++;
        return;
    }

    r->total += elapsed;
    if (elapsed < r->min) {
        r->min = elapsed;
    }
    if (elapsed > r->max) {
        r->max = elapsed;
    }
}

/* GPIO hooks for oscilloscope - inline no-op if disabled */
static inline void irq_prof_gpio_high(irq_prof_id_t id)
{
#if IRQ_PROF_GPIO_ENABLED
    extern void irq_prof_gpio_set(uint8_t slot_id);
    if (irq_prof_gpio_set) {
        irq_prof_gpio_set(id);
    }
#else
    (void)id;
#endif
}

static inline void irq_prof_gpio_low(irq_prof_id_t id)
{
#if IRQ_PROF_GPIO_ENABLED
    extern void irq_prof_gpio_clear(uint8_t slot_id);
    if (irq_prof_gpio_clear) {
        irq_prof_gpio_clear(id);
    }
#else
    (void)id;
#endif
}

/* Main context - snapshot + conversion */
int  irq_prof_get_result(irq_prof_id_t id, irq_prof_result_t *result);
uint8_t irq_prof_get_slot_count(void);
const char *irq_prof_get_label(irq_prof_id_t id);
irq_prof_cycle_t irq_prof_get_overhead_cycles(void);

/* Measurement overhead calibration */
irq_prof_cycle_t irq_prof_measure_overhead(void);

/* [TEMP DIAG] 嵌套感知的中断总占用测量。每个 ISR 最外层入口调 enter、出口调 exit，
 * 内层嵌套自动不重复计。g_irq_busy_cycles = CPU 处于中断态的真实墙钟 cycle。 */
extern volatile uint64_t g_irq_busy_cycles;
void irq_prof_nest_enter(void);
void irq_prof_nest_exit(void);

/* ============================================================================
 * Convenience macros - empty when disabled
 * ============================================================================ */

#if IRQ_PROF_ENABLED
    #define IRQ_PROF_ENTER(id)           irq_prof_enter(id)
    #define IRQ_PROF_EXIT(id)            irq_prof_exit(id)
    #define IRQ_PROF_GPIO_HIGH(id)       irq_prof_gpio_high(id)
    #define IRQ_PROF_GPIO_LOW(id)        irq_prof_gpio_low(id)
#else
    #define IRQ_PROF_ENTER(id)           do { (void)(id); } while (0)
    #define IRQ_PROF_EXIT(id)            do { (void)(id); } while (0)
    #define IRQ_PROF_GPIO_HIGH(id)       do { (void)(id); } while (0)
    #define IRQ_PROF_GPIO_LOW(id)        do { (void)(id); } while (0)
#endif

#ifdef __cplusplus
}
#endif

#endif /* IRQ_PROFILER_H */
