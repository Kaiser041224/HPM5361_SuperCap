/*
 * SYNT Driver - HPM5361 Sync Timer implementation
 *
 * SYNT counter generates periodic sync events via compare channels.
 * Events route through TRGM to PWM SYNCI / GPTMR SYNCI for hardware sync.
 * Uses motor system clock (clock_mot0 = 120MHz, same domain as PWM).
 *
 * Copyright (c) 2026 Alliance HardWare Team
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "intf_synt.h"

#include "hpm_soc.h"
#include "hpm_synt_drv.h"
#include "hpm_clock_drv.h"

#include <stddef.h>

#define SYNT_BASE       HPM_SYNT
#define SYNT_CMP_COUNT  4U

static bool synt_initialized = false;

static int synt_drv_init(const intf_synt_cfg_t *cfg)
{
    if (cfg == NULL || cfg->reload_count == 0U) {
        return -1;
    }
    if (cfg->cmp_channel >= SYNT_CMP_COUNT) {
        return -1;
    }

    synt_enable_counter(SYNT_BASE, false);
    synt_reset_counter(SYNT_BASE);

    synt_set_reload(SYNT_BASE, cfg->reload_count);
    synt_set_comparator(SYNT_BASE, cfg->cmp_channel, cfg->cmp_count);

    synt_initialized = true;
    return 0;
}

static int synt_drv_start(void)
{
    if (!synt_initialized) {
        return -1;
    }
    synt_enable_counter(SYNT_BASE, true);
    return 0;
}

static int synt_drv_stop(void)
{
    synt_enable_counter(SYNT_BASE, false);
    return 0;
}

static int synt_drv_reset(void)
{
    synt_enable_counter(SYNT_BASE, false);
    synt_reset_counter(SYNT_BASE);
    return 0;
}

static int synt_drv_set_reload(uint32_t reload_count)
{
    if (reload_count == 0U) {
        return -1;
    }
    synt_set_reload(SYNT_BASE, reload_count);
    return 0;
}

static int synt_drv_set_compare(intf_synt_ch_t ch, uint32_t cmp_count)
{
    if (ch >= SYNT_CMP_COUNT) {
        return -1;
    }
    if (synt_set_comparator(SYNT_BASE, ch, cmp_count) != status_success) {
        return -1;
    }
    return 0;
}

static uint32_t synt_drv_get_count(void)
{
    return synt_get_current_count(SYNT_BASE);
}

static const intf_synt_t synt_ops = {
    .instance_id = 0,
    .init = synt_drv_init,
    .start = synt_drv_start,
    .stop = synt_drv_stop,
    .reset = synt_drv_reset,
    .set_reload = synt_drv_set_reload,
    .set_compare = synt_drv_set_compare,
    .get_count = synt_drv_get_count,
};

void hpm_synt_driver_register(void)
{
    intf_synt_register(&synt_ops);
}
