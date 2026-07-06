/*
 * SYNT Interface - Sync Timer hardware-independent contract
 *
 * SYNT generates periodic sync events via compare channels.
 * Events route through TRGM to synchronize PWM, GPTMR, etc.
 * No interrupt support - pure hardware sync event output.
 *
 * Copyright (c) 2026 Alliance HardWare Team
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef INTF_SYNT_H
#define INTF_SYNT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef uint8_t intf_synt_ch_t;

typedef struct {
    uint32_t reload_count;
    intf_synt_ch_t cmp_channel;
    uint32_t cmp_count;
} intf_synt_cfg_t;

typedef struct {
    uint8_t instance_id;
    struct {
        int (*init)(const intf_synt_cfg_t *cfg);
        int (*start)(void);
        int (*stop)(void);
        int (*reset)(void);
        int (*set_reload)(uint32_t reload_count);
        int (*set_compare)(intf_synt_ch_t ch, uint32_t cmp_count);
        uint32_t (*get_count)(void);
    };
} intf_synt_t;

int intf_synt_register(const intf_synt_t *ops);
int intf_synt_init(const intf_synt_cfg_t *cfg);
int intf_synt_start(void);
int intf_synt_stop(void);
int intf_synt_reset(void);
int intf_synt_set_reload(uint32_t reload_count);
int intf_synt_set_compare(intf_synt_ch_t ch, uint32_t cmp_count);
uint32_t intf_synt_get_count(void);

#ifdef __cplusplus
}
#endif

#endif /* INTF_SYNT_H */
