/*
 * MCAN Driver - HPM MCAN hardware implementation
 *
 * 最大化复用 HPM SDK 已有代码:
 * - mcan_get_default_config() → 填充完整默认配置
 * - mcan_init() → 硬件初始化
 * - mcan_transmit_blocking() / mcan_receive_from_fifo_blocking() → 收发
 * - mcan_parse_protocol_status() / mcan_get_error_counter() → 状态查询
 * - mcan_set_filter_element() → 过滤器配置
 * - 复用 SDK demo 中的 can_info_t[] + ISR 模式
 *
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "intf_can.h"
#include "board.h"

#include "hpm_mcan_drv.h"
#include "hpm_clock_drv.h"
#include "hpm_csr_drv.h"
#include "hpm_interrupt.h"
#include "hpm_soc_irq.h"

#include <stddef.h>
#include <string.h>

/* ============================================================================
 * MCAN 实例数量 (使用 SDK 定义的 MCAN_SOC_MAX_COUNT)
 * ============================================================================ */

#define DRV_MCAN_INSTANCE_COUNT MCAN_SOC_MAX_COUNT

/* ============================================================================
 * DLC 编码表: 字节数 → DLC 寄存器值 (CAN FD)
 * ============================================================================ */

static const uint8_t dlc_encode_table[65] = {
    [0]  = 0,   [1]  = 1,   [2]  = 2,   [3]  = 3,
    [4]  = 4,   [5]  = 5,   [6]  = 6,   [7]  = 7,
    [8]  = 8,   [12] = 9,   [16] = 10,  [20] = 11,
    [24] = 12,  [32] = 13,  [48] = 14,  [64] = 15,
};

static uint8_t can_dlc_encode(uint8_t byte_count)
{
    if (byte_count <= 8) {
        return byte_count;
    }
    if (byte_count <= 64) {
        return dlc_encode_table[byte_count];
    }
    return 15;
}

/* ============================================================================
 * AHB RAM 消息缓冲区 (硬件要求: 必须放在 AHB SRAM 段)
 * 复用 SDK demo 模式: ATTR_PLACE_AT(".ahb_sram")
 * ============================================================================ */

#if defined(HPM_MCAN0)
ATTR_PLACE_AT(".ahb_sram") static uint32_t mcan0_msg_buf[MCAN_MSG_BUF_SIZE_IN_WORDS];
#endif
#if defined(HPM_MCAN1)
ATTR_PLACE_AT(".ahb_sram") static uint32_t mcan1_msg_buf[MCAN_MSG_BUF_SIZE_IN_WORDS];
#endif
#if defined(HPM_MCAN2)
ATTR_PLACE_AT(".ahb_sram") static uint32_t mcan2_msg_buf[MCAN_MSG_BUF_SIZE_IN_WORDS];
#endif
#if defined(HPM_MCAN3)
ATTR_PLACE_AT(".ahb_sram") static uint32_t mcan3_msg_buf[MCAN_MSG_BUF_SIZE_IN_WORDS];
#endif

/* ============================================================================
 * 实例管理结构体 (复用 SDK demo 的 can_info_t 模式)
 * ============================================================================ */

typedef struct {
    MCAN_Type    *base;
    clock_name_t  clock_name;
    uint32_t      irq_num;
    uint32_t      ram_base;
    uint32_t      ram_size;
    uint8_t       std_filter_capacity;
    uint8_t       ext_filter_capacity;
    bool          initialized;
    bool          canfd_enabled;
    uint32_t      interrupt_mask;
    intf_can_irq_callback_t irq_cb;
    void         *irq_user_data;
} mcan_instance_t;

static uint32_t drv_can_enter_critical(void)
{
    return read_clear_csr(CSR_MSTATUS, CSR_MSTATUS_MIE_MASK);
}

static void drv_can_exit_critical(uint32_t irq_state)
{
    write_csr(CSR_MSTATUS, irq_state);
}

static bool mcan_frame_id_is_valid(const intf_can_frame_t *frame)
{
    if (frame->is_ext_id) {
        return frame->id <= 0x1FFFFFFFU;
    }
    return frame->id <= 0x7FFU;
}

static bool mcan_frame_is_valid(const mcan_instance_t *inst,
                                const intf_can_frame_t *frame)
{
    if (inst == NULL || frame == NULL) {
        return false;
    }
    if (!mcan_frame_id_is_valid(frame)) {
        return false;
    }
    if (frame->frame_type == INTF_CAN_FRAME_CLASSIC) {
        return frame->dlc <= 8U;
    }
    if (!inst->canfd_enabled) {
        return false;
    }

    switch (frame->dlc) {
    case 0U:
    case 1U:
    case 2U:
    case 3U:
    case 4U:
    case 5U:
    case 6U:
    case 7U:
    case 8U:
    case 12U:
    case 16U:
    case 20U:
    case 24U:
    case 32U:
    case 48U:
    case 64U:
        return true;
    default:
        return false;
    }
}

static mcan_instance_t mcan_instances[DRV_MCAN_INSTANCE_COUNT];

static void mcan_deinit_impl(uint8_t inst_id);

/* ============================================================================
 * 获取实例基地址 (类似 drv_hrpwm.c 的 hrpwm_get_base)
 * ============================================================================ */

static MCAN_Type *mcan_get_base(uint8_t inst_id)
{
    if (inst_id >= DRV_MCAN_INSTANCE_COUNT) {
        return NULL;
    }
    return mcan_instances[inst_id].base;
}

static mcan_instance_t *mcan_get_instance(uint8_t inst_id)
{
    if (inst_id >= DRV_MCAN_INSTANCE_COUNT) {
        return NULL;
    }
    return &mcan_instances[inst_id];
}

/* ============================================================================
 * 事件掩码映射: intf_can_event_t → SDK MCAN_INT_* / MCAN_EVENT_*
 * ============================================================================ */

static uint32_t mcan_event_to_sdk_mask(uint32_t intf_events)
{
    uint32_t sdk_mask = 0U;

    if (intf_events & INTF_CAN_EVENT_RX_FIFO0_NEW_MSG)  sdk_mask |= MCAN_INT_RXFIFO0_NEW_MSG;
    if (intf_events & INTF_CAN_EVENT_RX_FIFO1_NEW_MSG)  sdk_mask |= MCAN_INT_RXFIFO1_NEW_MSG;
    if (intf_events & INTF_CAN_EVENT_RX_BUF_NEW_MSG)    sdk_mask |= MCAN_INT_MSG_STORE_TO_RXBUF;
    if (intf_events & INTF_CAN_EVENT_RX_FIFO0_FULL)     sdk_mask |= MCAN_INT_RXFIFO0_FULL;
    if (intf_events & INTF_CAN_EVENT_RX_FIFO1_FULL)     sdk_mask |= MCAN_INT_RXFIFO1_FULL;
    if (intf_events & INTF_CAN_EVENT_RX_FIFO0_MSG_LOST) sdk_mask |= MCAN_INT_RXFIFO0_MSG_LOST;
    if (intf_events & INTF_CAN_EVENT_RX_FIFO1_MSG_LOST) sdk_mask |= MCAN_INT_RXFIFO1_MSG_LOST;
    if (intf_events & INTF_CAN_EVENT_TX_COMPLETED)      sdk_mask |= MCAN_INT_TX_COMPLETED;
    if (intf_events & INTF_CAN_EVENT_TX_FIFO_EMPTY)     sdk_mask |= MCAN_INT_TXFIFO_EMPTY;
    if (intf_events & INTF_CAN_EVENT_TX_CANCEL_DONE)    sdk_mask |= MCAN_INT_TX_CANCEL_FINISHED;
    if (intf_events & INTF_CAN_EVENT_TX_EVT_FIFO_NEW)   sdk_mask |= MCAN_INT_TX_EVT_FIFO_NEW_ENTRY;
    if (intf_events & INTF_CAN_EVENT_TX_EVT_FIFO_FULL)  sdk_mask |= MCAN_INT_TX_EVT_FIFO_FULL;
    if (intf_events & INTF_CAN_EVENT_TX_EVT_FIFO_LOST)  sdk_mask |= MCAN_INT_TX_EVT_FIFO_EVT_LOST;
    if (intf_events & INTF_CAN_EVENT_BUS_OFF)           sdk_mask |= MCAN_INT_BUS_OFF_STATUS;
    if (intf_events & INTF_CAN_EVENT_ERROR_WARNING)     sdk_mask |= MCAN_INT_WARNING_STATUS;
    if (intf_events & INTF_CAN_EVENT_ERROR_PASSIVE)     sdk_mask |= MCAN_INT_ERROR_PASSIVE;
    if (intf_events & INTF_CAN_EVENT_PROTOCOL_ERROR) {
        sdk_mask |= (MCAN_INT_PROTOCOL_ERR_IN_DATA_PHASE |
                     MCAN_INT_PROTOCOL_ERR_IN_ARB_PHASE  |
                     MCAN_INT_BIT_ERROR_UNCORRECTED);
    }
    if (intf_events & INTF_CAN_EVENT_TIMEOUT)           sdk_mask |= MCAN_INT_TIMEOUT_OCCURRED;
    if (intf_events & INTF_CAN_EVENT_HIGH_PRIORITY_MSG) sdk_mask |= MCAN_INT_HIGH_PRIORITY_MSG;
    if (intf_events & INTF_CAN_EVENT_TIMESTAMP_WRAP)    sdk_mask |= MCAN_INT_TIMESTAMP_WRAPAROUND;
    if (intf_events & INTF_CAN_EVENT_RAM_ACCESS_FAIL)   sdk_mask |= MCAN_INT_MSG_RAM_ACCESS_FAILURE;

    return sdk_mask;
}

static uint32_t sdk_mask_to_mcan_event(uint32_t sdk_flags)
{
    uint32_t intf_events = 0U;

    if (sdk_flags & MCAN_INT_RXFIFO0_NEW_MSG)  intf_events |= INTF_CAN_EVENT_RX_FIFO0_NEW_MSG;
    if (sdk_flags & MCAN_INT_RXFIFO1_NEW_MSG)  intf_events |= INTF_CAN_EVENT_RX_FIFO1_NEW_MSG;
    if (sdk_flags & MCAN_INT_MSG_STORE_TO_RXBUF) intf_events |= INTF_CAN_EVENT_RX_BUF_NEW_MSG;
    if (sdk_flags & MCAN_INT_RXFIFO0_FULL)     intf_events |= INTF_CAN_EVENT_RX_FIFO0_FULL;
    if (sdk_flags & MCAN_INT_RXFIFO1_FULL)     intf_events |= INTF_CAN_EVENT_RX_FIFO1_FULL;
    if (sdk_flags & MCAN_INT_RXFIFO0_MSG_LOST) intf_events |= INTF_CAN_EVENT_RX_FIFO0_MSG_LOST;
    if (sdk_flags & MCAN_INT_RXFIFO1_MSG_LOST) intf_events |= INTF_CAN_EVENT_RX_FIFO1_MSG_LOST;
    if (sdk_flags & MCAN_INT_TX_COMPLETED)     intf_events |= INTF_CAN_EVENT_TX_COMPLETED;
    if (sdk_flags & MCAN_INT_TXFIFO_EMPTY)     intf_events |= INTF_CAN_EVENT_TX_FIFO_EMPTY;
    if (sdk_flags & MCAN_INT_TX_CANCEL_FINISHED) intf_events |= INTF_CAN_EVENT_TX_CANCEL_DONE;
    if (sdk_flags & MCAN_INT_TX_EVT_FIFO_NEW_ENTRY) intf_events |= INTF_CAN_EVENT_TX_EVT_FIFO_NEW;
    if (sdk_flags & MCAN_INT_TX_EVT_FIFO_FULL)   intf_events |= INTF_CAN_EVENT_TX_EVT_FIFO_FULL;
    if (sdk_flags & MCAN_INT_TX_EVT_FIFO_EVT_LOST) intf_events |= INTF_CAN_EVENT_TX_EVT_FIFO_LOST;
    if (sdk_flags & MCAN_INT_BUS_OFF_STATUS)     intf_events |= INTF_CAN_EVENT_BUS_OFF;
    if (sdk_flags & MCAN_INT_WARNING_STATUS)     intf_events |= INTF_CAN_EVENT_ERROR_WARNING;
    if (sdk_flags & MCAN_INT_ERROR_PASSIVE)      intf_events |= INTF_CAN_EVENT_ERROR_PASSIVE;
    if (sdk_flags & (MCAN_INT_PROTOCOL_ERR_IN_DATA_PHASE |
                     MCAN_INT_PROTOCOL_ERR_IN_ARB_PHASE  |
                     MCAN_INT_BIT_ERROR_UNCORRECTED))
        intf_events |= INTF_CAN_EVENT_PROTOCOL_ERROR;
    if (sdk_flags & MCAN_INT_TIMEOUT_OCCURRED)   intf_events |= INTF_CAN_EVENT_TIMEOUT;
    if (sdk_flags & MCAN_INT_HIGH_PRIORITY_MSG)  intf_events |= INTF_CAN_EVENT_HIGH_PRIORITY_MSG;
    if (sdk_flags & MCAN_INT_TIMESTAMP_WRAPAROUND) intf_events |= INTF_CAN_EVENT_TIMESTAMP_WRAP;
    if (sdk_flags & MCAN_INT_MSG_RAM_ACCESS_FAILURE) intf_events |= INTF_CAN_EVENT_RAM_ACCESS_FAIL;

    return intf_events;
}

/* ============================================================================
 * 帧映射: intf_can_frame_t → mcan_tx_frame_t
 * ============================================================================ */

static void frame_to_sdk_tx(const intf_can_frame_t *src, mcan_tx_frame_t *dst)
{
    memset(dst, 0, sizeof(*dst));

    if (src->is_ext_id) {
        dst->use_ext_id = 1;
        dst->ext_id = src->id & 0x1FFFFFFFU;
    } else {
        dst->std_id = src->id & 0x7FFU;
    }
    dst->rtr = src->is_remote ? 1U : 0U;
    dst->dlc = can_dlc_encode(src->dlc);
    dst->canfd_frame = (src->frame_type != INTF_CAN_FRAME_CLASSIC) ? 1U : 0U;
    dst->bitrate_switch = (src->frame_type == INTF_CAN_FRAME_FD_BRS) ? 1U : 0U;
    dst->message_marker_h = (uint8_t)((src->message_marker >> 8) & 0xFFU);
    dst->message_marker_l = (uint8_t)(src->message_marker & 0xFFU);
    memcpy(dst->data_8, src->data, src->dlc);
}

/* ============================================================================
 * 帧映射: mcan_rx_message_t → intf_can_frame_t
 * ============================================================================ */

static void sdk_rx_to_frame(const mcan_rx_message_t *src, intf_can_frame_t *dst)
{
    uint8_t payload_size;

    memset(dst, 0, sizeof(*dst));

    dst->id = src->use_ext_id ? src->ext_id : src->std_id;
    dst->is_ext_id = (src->use_ext_id != 0U);
    dst->is_remote = (src->rtr != 0U);
    if (src->canfd_frame) {
        dst->frame_type = src->bitrate_switch
                          ? INTF_CAN_FRAME_FD_BRS
                          : INTF_CAN_FRAME_FD_NO_BRS;
    } else {
        dst->frame_type = INTF_CAN_FRAME_CLASSIC;
    }
    payload_size = (uint8_t)mcan_get_message_size_from_dlc(src->dlc);
    if (payload_size > sizeof(dst->data)) {
        payload_size = sizeof(dst->data);
    }
    dst->dlc = payload_size;
    dst->timestamp = src->rx_timestamp;
    dst->filter_index = src->filter_index;
    memcpy(dst->data, src->data_8, dst->dlc);
}

/* ============================================================================
 * 接口实现 — init
 * ============================================================================ */

static void filter_to_sdk(const intf_can_filter_elem_t *src, mcan_filter_elem_t *dst)
{
    memset(dst, 0, sizeof(*dst));

    dst->can_id_type = src->is_ext_id ? MCAN_CAN_ID_TYPE_EXTENDED
                                      : MCAN_CAN_ID_TYPE_STANDARD;

    if (src->type == INTF_CAN_FILTER_STORE_TO_BUF) {
        dst->filter_type = MCAN_FILTER_TYPE_CLASSIC_FILTER;
        dst->filter_config = MCAN_FILTER_ELEM_CFG_STORE_INTO_RX_BUFFER_OR_AS_DBG_MSG;
        dst->match_id = src->id;
        dst->offset = src->rxbuf_idx;
        dst->filter_event = 0U;
        dst->store_location = 0U;
    } else {
        dst->filter_type = (uint8_t)src->type;
        dst->filter_config = (src->target_fifo == INTF_CAN_FILTER_FIFO1)
                           ? MCAN_FILTER_ELEM_CFG_STORE_IN_RX_FIFO1_IF_MATCH
                           : MCAN_FILTER_ELEM_CFG_STORE_IN_RX_FIFO0_IF_MATCH;
        switch (src->type) {
        case INTF_CAN_FILTER_RANGE:
            dst->start_id = src->id;
            dst->end_id = src->mask;
            break;
        case INTF_CAN_FILTER_DUAL_ID:
            dst->id1 = src->id;
            dst->id2 = src->mask;
            break;
        case INTF_CAN_FILTER_CLASSIC:
        default:
            dst->filter_id = src->id;
            dst->filter_mask = src->mask;
            break;
        }
    }
}

/* ============================================================================
 * 接口实现 — init
 * ============================================================================ */

static int mcan_init_impl(uint8_t inst_id, const intf_can_cfg_t *cfg)
{
    mcan_instance_t *inst = &mcan_instances[inst_id];

    if (inst->base == NULL || cfg == NULL) {
        return -1;
    }
    if (cfg->baudrate == 0U) {
        return -1;
    }
    if (cfg->enable_canfd && cfg->baudrate_fd == 0U) {
        return -1;
    }

    if (inst->initialized) {
        mcan_deinit_impl(inst_id);
    }

    mcan_msg_buf_attr_t attr;
    attr.ram_base = inst->ram_base;
    attr.ram_size = inst->ram_size;
    hpm_stat_t st = mcan_set_msg_buf_attr(inst->base, &attr);
    if (st != status_success) {
        return -1;
    }

    mcan_config_t sdk_cfg;
    mcan_ram_config_t ram;
    mcan_get_default_config(inst->base, &sdk_cfg);
    mcan_get_default_ram_config(inst->base, &ram, cfg->enable_canfd);

    sdk_cfg.baudrate = cfg->baudrate;
    sdk_cfg.mode     = (mcan_node_mode_t)cfg->mode;
    sdk_cfg.enable_canfd = cfg->enable_canfd;
    if (cfg->enable_canfd) {
        sdk_cfg.baudrate_fd = cfg->baudrate_fd;
    }
    if (cfg->sample_point > 0U) {
        sdk_cfg.can20_samplepoint_min = cfg->sample_point;
        sdk_cfg.can20_samplepoint_max = cfg->sample_point;
    }
    if (cfg->sample_point_fd > 0U) {
        sdk_cfg.canfd_samplepoint_min = cfg->sample_point_fd;
        sdk_cfg.canfd_samplepoint_max = cfg->sample_point_fd;
    }
    sdk_cfg.disable_auto_retransmission = cfg->disable_auto_retransmission;
    sdk_cfg.enable_restricted_operation_mode = cfg->enable_restricted_mode;

    bool has_custom_ram = (cfg->ram.std_filter_count > 0U)
                       || (cfg->ram.ext_filter_count > 0U)
                       || (cfg->ram.rxfifo0_count > 0U)
                       || (cfg->ram.rxfifo1_count > 0U)
                       || (cfg->ram.rxbuf_count > 0U)
                       || (cfg->ram.txbuf_count > 0U)
                       || (cfg->ram.tx_evt_fifo_count > 0U);
    if (has_custom_ram) {
        if (cfg->ram.std_filter_count > 0U) ram.std_filter_elem_count = cfg->ram.std_filter_count;
        if (cfg->ram.ext_filter_count > 0U) ram.ext_filter_elem_count  = cfg->ram.ext_filter_count;
        if (cfg->ram.rxfifo0_count > 0U)    ram.rxfifos[0].elem_count  = cfg->ram.rxfifo0_count;
        if (cfg->ram.rxfifo1_count > 0U)    ram.rxfifos[1].elem_count  = cfg->ram.rxfifo1_count;
        if (cfg->ram.rxbuf_count > 0U)      ram.rxbuf_elem_count       = cfg->ram.rxbuf_count;
        if (cfg->ram.txbuf_count > 0U)      ram.txbuf_fifo_or_queue_elem_count = cfg->ram.txbuf_count;
        if (cfg->ram.tx_evt_fifo_count > 0U) ram.tx_evt_fifo_elem_count = cfg->ram.tx_evt_fifo_count;
        sdk_cfg.ram_config = ram;
    }

    sdk_cfg.interrupt_mask = mcan_event_to_sdk_mask(cfg->interrupt_mask);

    uint32_t clk_freq = clock_get_frequency(inst->clock_name);
    st = mcan_init(inst->base, &sdk_cfg, clk_freq);
    if (st != status_success) {
        return -1;
    }

    inst->initialized = true;
    inst->std_filter_capacity = ram.std_filter_elem_count;
    inst->ext_filter_capacity = ram.ext_filter_elem_count;
    inst->canfd_enabled = cfg->enable_canfd;
    inst->interrupt_mask = cfg->interrupt_mask;
    return 0;
}

/* ============================================================================
 * 接口实现 — deinit
 * ============================================================================ */

static void mcan_deinit_impl(uint8_t inst_id)
{
    mcan_instance_t *inst;
    uint32_t flags;

    if (inst_id >= DRV_MCAN_INSTANCE_COUNT) {
        return;
    }

    inst = &mcan_instances[inst_id];

    if (inst->base == NULL) {
        return;
    }

    mcan_disable_interrupts(inst->base, UINT32_MAX);
    intc_m_disable_irq(inst->irq_num);
    flags = mcan_get_interrupt_flags(inst->base);
    if (flags != 0U) {
        mcan_clear_interrupt_flags(inst->base, flags);
    }

    mcan_deinit(inst->base);
    inst->std_filter_capacity = 0U;
    inst->ext_filter_capacity = 0U;
    inst->initialized = false;
    inst->canfd_enabled = false;
    inst->interrupt_mask = 0U;
    inst->irq_cb = NULL;
    inst->irq_user_data = NULL;
}

/* ============================================================================
 * 接口实现 — send (阻塞)
 * ============================================================================ */

static int mcan_send_impl(uint8_t inst_id, const intf_can_frame_t *frame,
                          uint32_t timeout_ms)
{
    mcan_instance_t *inst = mcan_get_instance(inst_id);
    MCAN_Type *base = mcan_get_base(inst_id);
    if (base == NULL || !mcan_frame_is_valid(inst, frame)) {
        return -1;
    }

    mcan_tx_frame_t tx;
    frame_to_sdk_tx(frame, &tx);

    (void)timeout_ms;
    hpm_stat_t st = mcan_transmit_blocking(base, &tx);
    return (st == status_success) ? 0 : -1;
}

/* ============================================================================
 * 接口实现 — send_nonblocking
 * ============================================================================ */

static int mcan_send_nonblocking_impl(uint8_t inst_id,
                                       const intf_can_frame_t *frame,
                                       uint8_t *fifo_idx)
{
    mcan_instance_t *inst = mcan_get_instance(inst_id);
    MCAN_Type *base = mcan_get_base(inst_id);
    if (base == NULL || !mcan_frame_is_valid(inst, frame)) {
        return -1;
    }

    mcan_tx_frame_t tx;
    frame_to_sdk_tx(frame, &tx);

    uint32_t idx = 0;
    hpm_stat_t st = mcan_transmit_via_txfifo_nonblocking(base, &tx, &idx);
    if (st == status_success && fifo_idx != NULL) {
        *fifo_idx = (uint8_t)idx;
    }
    return (st == status_success) ? 0 : -1;
}

/* ============================================================================
 * 接口实现 — send_add_request
 * ============================================================================ */

static int mcan_send_add_request_impl(uint8_t inst_id, uint8_t fifo_idx)
{
    MCAN_Type *base = mcan_get_base(inst_id);
    if (base == NULL) {
        return -1;
    }
    mcan_send_add_request(base, (uint32_t)fifo_idx);
    return 0;
}

/* ============================================================================
 * 接口实现 — receive (阻塞)
 * ============================================================================ */

static int mcan_receive_impl(uint8_t inst_id, intf_can_frame_t *frame,
                              uint32_t timeout_ms)
{
    MCAN_Type *base = mcan_get_base(inst_id);
    if (base == NULL || frame == NULL) {
        return -1;
    }

    mcan_rx_message_t rx;
    memset(&rx, 0, sizeof(rx));

    (void)timeout_ms;
    hpm_stat_t st = mcan_receive_from_fifo_blocking(base, 0U, &rx);
    if (st != status_success) {
        return -1;
    }
    sdk_rx_to_frame(&rx, frame);
    return 0;
}

/* ============================================================================
 * 接口实现 — receive_nonblocking
 * ============================================================================ */

static int mcan_receive_nonblocking_impl(uint8_t inst_id,
                                          intf_can_frame_t *frame)
{
    MCAN_Type *base = mcan_get_base(inst_id);
    if (base == NULL || frame == NULL) {
        return -1;
    }

    mcan_rx_message_t rx;
    memset(&rx, 0, sizeof(rx));

    hpm_stat_t st = mcan_read_rxfifo(base, 0U, &rx);
    if (st != status_success) {
        return -1;
    }
    sdk_rx_to_frame(&rx, frame);
    return 0;
}

/* ============================================================================
 * 接口实现 — config_filter
 * ============================================================================ */

static int mcan_config_filter_impl(uint8_t inst_id, uint32_t index,
                                     const intf_can_filter_elem_t *elem)
{
    mcan_instance_t *inst = mcan_get_instance(inst_id);
    MCAN_Type *base = mcan_get_base(inst_id);
    if (base == NULL || inst == NULL || elem == NULL) {
        return -1;
    }
    if (elem->is_ext_id) {
        if (elem->id > 0x1FFFFFFFU || elem->mask > 0x1FFFFFFFU) {
            return -1;
        }
        if (index >= inst->ext_filter_capacity) {
            return -1;
        }
    } else {
        if (elem->id > 0x7FFU || elem->mask > 0x7FFU) {
            return -1;
        }
        if (index >= inst->std_filter_capacity) {
            return -1;
        }
    }
    if (elem->type == INTF_CAN_FILTER_STORE_TO_BUF && elem->rxbuf_idx >= 64U) {
        return -1;
    }

    mcan_filter_elem_t sdk_elem;
    filter_to_sdk(elem, &sdk_elem);

    hpm_stat_t st = mcan_set_filter_element(base, &sdk_elem, index);
    return (st == status_success) ? 0 : -1;
}

/* ============================================================================
 * 接口实现 — enable_interrupt / disable_interrupt
 * ============================================================================ */

static int mcan_enable_interrupt_impl(uint8_t inst_id, uint32_t event_mask)
{
    mcan_instance_t *inst = mcan_get_instance(inst_id);
    MCAN_Type *base = mcan_get_base(inst_id);
    if (base == NULL || inst == NULL) {
        return -1;
    }
    uint32_t sdk_mask = mcan_event_to_sdk_mask(event_mask);
    mcan_enable_interrupts(base, sdk_mask);
    inst->interrupt_mask |= event_mask;
    return 0;
}

static int mcan_disable_interrupt_impl(uint8_t inst_id, uint32_t event_mask)
{
    mcan_instance_t *inst = mcan_get_instance(inst_id);
    MCAN_Type *base = mcan_get_base(inst_id);
    if (base == NULL || inst == NULL) {
        return -1;
    }
    uint32_t sdk_mask = mcan_event_to_sdk_mask(event_mask);
    mcan_disable_interrupts(base, sdk_mask);
    inst->interrupt_mask &= ~event_mask;
    return 0;
}

/* ============================================================================
 * 接口实现 — config_irq_callback
 * ============================================================================ */

static int mcan_config_irq_callback_impl(uint8_t inst_id,
                                           intf_can_irq_callback_t cb,
                                           void *user_data)
{
    mcan_instance_t *inst = mcan_get_instance(inst_id);
    uint32_t irq_state;

    if (inst == NULL) {
        return -1;
    }

    irq_state = drv_can_enter_critical();
    inst->irq_cb = cb;
    inst->irq_user_data = user_data;
    if (inst->initialized && inst->interrupt_mask != 0U) {
        if (cb != NULL) {
            intc_m_enable_irq_with_priority(inst->irq_num, 1);
        } else {
            intc_m_disable_irq(inst->irq_num);
        }
    }
    drv_can_exit_critical(irq_state);
    return 0;
}

/* ============================================================================
 * 接口实现 — get_status (复用 SDK mcan_parse_protocol_status!)
 * ============================================================================ */

static int mcan_get_status_impl(uint8_t inst_id, intf_can_status_t *status)
{
    MCAN_Type *base = mcan_get_base(inst_id);
    if (base == NULL || status == NULL) {
        return -1;
    }

    mcan_protocol_status_t ps;
    mcan_get_protocol_status(base, &ps);

    mcan_error_count_t err_cnt;
    mcan_get_error_counter(base, &err_cnt);

    status->tx_error_count     = err_cnt.transmit_error_count;
    status->rx_error_count     = err_cnt.receive_error_count;
    status->last_error_code    = (uint8_t)ps.last_error_code;
    status->activity           = (uint8_t)ps.activity;
    status->bus_off            = ps.in_bus_off_state;
    status->error_warning      = ps.in_warning_state;
    status->error_passive      = ps.in_error_passive_state;
    status->protocol_exception = ps.protocol_exception_evt_occurred;
    return 0;
}

/* ============================================================================
 * 接口实现 — read_tx_event
 * ============================================================================ */

static int mcan_read_tx_event_impl(uint8_t inst_id, intf_can_tx_event_t *tx_evt)
{
    MCAN_Type *base = mcan_get_base(inst_id);
    if (base == NULL || tx_evt == NULL) {
        return -1;
    }

    mcan_tx_event_fifo_elem_t sdk_evt;
    hpm_stat_t st = mcan_read_tx_evt_fifo(base, &sdk_evt);
    if (st != status_success) {
        return -1;
    }

    tx_evt->id = sdk_evt.extend_id ? sdk_evt.ext_id : sdk_evt.std_id;
    tx_evt->is_ext_id = (sdk_evt.extend_id != 0U);
    tx_evt->is_remote = (sdk_evt.rtr != 0U);
    tx_evt->event_type = (uint8_t)sdk_evt.event_type;
    tx_evt->message_marker = sdk_evt.message_marker;
    tx_evt->timestamp = sdk_evt.tx_timestamp;
    return 0;
}

/* ============================================================================
 * 接口实现 — get_timestamp
 * ============================================================================ */

static int mcan_get_timestamp_impl(uint8_t inst_id,
                                    const intf_can_tx_event_t *tx_evt,
                                    intf_can_timestamp_t *ts)
{
    (void)inst_id;
    if (tx_evt == NULL || ts == NULL) {
        return -1;
    }
    ts->is_64bit = false;
    ts->ts_low = tx_evt->timestamp;
    ts->ts_high = 0U;
    return 0;
}

/* ============================================================================
 * ISR 处理 (复用 SDK demo 模式)
 * ============================================================================ */

static void mcan_isr_handler(uint8_t inst_id)
{
    mcan_instance_t *inst = &mcan_instances[inst_id];
    if (inst->base == NULL) {
        return;
    }

    uint32_t flags = mcan_get_interrupt_flags(inst->base);
    if (flags == 0U) {
        return;
    }

    uint32_t intf_events = sdk_mask_to_mcan_event(flags);

    mcan_clear_interrupt_flags(inst->base, flags);

    if (inst->irq_cb != NULL && intf_events != 0U) {
        inst->irq_cb(inst_id, intf_events, inst->irq_user_data);
    }
}

#if defined(HPM_MCAN0)
SDK_DECLARE_EXT_ISR_M(IRQn_MCAN0, isr_mcan0)
void isr_mcan0(void) { mcan_isr_handler(0); }
#endif

#if defined(HPM_MCAN1)
SDK_DECLARE_EXT_ISR_M(IRQn_MCAN1, isr_mcan1)
void isr_mcan1(void) { mcan_isr_handler(1); }
#endif

#if defined(HPM_MCAN2)
SDK_DECLARE_EXT_ISR_M(IRQn_MCAN2, isr_mcan2)
void isr_mcan2(void) { mcan_isr_handler(2); }
#endif

#if defined(HPM_MCAN3)
SDK_DECLARE_EXT_ISR_M(IRQn_MCAN3, isr_mcan3)
void isr_mcan3(void) { mcan_isr_handler(3); }
#endif

/* ============================================================================
 * 实例信息表初始化
 * ============================================================================ */

static void mcan_init_instance_table(void)
{
    static bool table_initialized = false;
    if (table_initialized) {
        return;
    }
    table_initialized = true;

#if defined(HPM_MCAN0)
    mcan_instances[0].base = HPM_MCAN0;
    mcan_instances[0].clock_name = clock_can0;
    mcan_instances[0].irq_num = IRQn_MCAN0;
    mcan_instances[0].ram_base = (uint32_t)&mcan0_msg_buf;
    mcan_instances[0].ram_size = sizeof(mcan0_msg_buf);
#endif

#if defined(HPM_MCAN1)
    mcan_instances[1].base = HPM_MCAN1;
    mcan_instances[1].clock_name = clock_can1;
    mcan_instances[1].irq_num = IRQn_MCAN1;
    mcan_instances[1].ram_base = (uint32_t)&mcan1_msg_buf;
    mcan_instances[1].ram_size = sizeof(mcan1_msg_buf);
#endif

#if defined(HPM_MCAN2)
    mcan_instances[2].base = HPM_MCAN2;
    mcan_instances[2].clock_name = clock_can2;
    mcan_instances[2].irq_num = IRQn_MCAN2;
    mcan_instances[2].ram_base = (uint32_t)&mcan2_msg_buf;
    mcan_instances[2].ram_size = sizeof(mcan2_msg_buf);
#endif

#if defined(HPM_MCAN3)
    mcan_instances[3].base = HPM_MCAN3;
    mcan_instances[3].clock_name = clock_can3;
    mcan_instances[3].irq_num = IRQn_MCAN3;
    mcan_instances[3].ram_base = (uint32_t)&mcan3_msg_buf;
    mcan_instances[3].ram_size = sizeof(mcan3_msg_buf);
#endif
}

/* ============================================================================
 * 每实例 init 包装 (捕获 inst_id)
 * ============================================================================ */

#if defined(HPM_MCAN0)
static int mcan0_init(const intf_can_cfg_t *cfg) { return mcan_init_impl(0, cfg); }
static void mcan0_deinit(void) { mcan_deinit_impl(0); }
static int mcan0_send(const intf_can_frame_t *f, uint32_t t) { return mcan_send_impl(0, f, t); }
static int mcan0_send_nb(const intf_can_frame_t *f, uint8_t *idx) { return mcan_send_nonblocking_impl(0, f, idx); }
static int mcan0_send_add_req(uint8_t idx) { return mcan_send_add_request_impl(0, idx); }
static int mcan0_receive(intf_can_frame_t *f, uint32_t t) { return mcan_receive_impl(0, f, t); }
static int mcan0_receive_nb(intf_can_frame_t *f) { return mcan_receive_nonblocking_impl(0, f); }
static int mcan0_cfg_filter(uint32_t i, const intf_can_filter_elem_t *e) { return mcan_config_filter_impl(0, i, e); }
static int mcan0_enable_int(uint32_t m) { return mcan_enable_interrupt_impl(0, m); }
static int mcan0_disable_int(uint32_t m) { return mcan_disable_interrupt_impl(0, m); }
static int mcan0_cfg_irq_cb(intf_can_irq_callback_t cb, void *d) { return mcan_config_irq_callback_impl(0, cb, d); }
static int mcan0_get_status(intf_can_status_t *s) { return mcan_get_status_impl(0, s); }
static int mcan0_read_tx_evt(intf_can_tx_event_t *e) { return mcan_read_tx_event_impl(0, e); }
static int mcan0_get_ts(const intf_can_tx_event_t *e, intf_can_timestamp_t *t) { return mcan_get_timestamp_impl(0, e, t); }

static const intf_can_t mcan0_ops = {
    .instance_id = 0,
    .init = mcan0_init,
    .deinit = mcan0_deinit,
    .send = mcan0_send,
    .send_nonblocking = mcan0_send_nb,
    .send_add_request = mcan0_send_add_req,
    .receive = mcan0_receive,
    .receive_nonblocking = mcan0_receive_nb,
    .config_filter = mcan0_cfg_filter,
    .enable_interrupt = mcan0_enable_int,
    .disable_interrupt = mcan0_disable_int,
    .config_irq_callback = mcan0_cfg_irq_cb,
    .get_status = mcan0_get_status,
    .read_tx_event = mcan0_read_tx_evt,
    .get_timestamp = mcan0_get_ts,
};
#endif

#if defined(HPM_MCAN1)
static int mcan1_init(const intf_can_cfg_t *cfg) { return mcan_init_impl(1, cfg); }
static void mcan1_deinit(void) { mcan_deinit_impl(1); }
static int mcan1_send(const intf_can_frame_t *f, uint32_t t) { return mcan_send_impl(1, f, t); }
static int mcan1_send_nb(const intf_can_frame_t *f, uint8_t *idx) { return mcan_send_nonblocking_impl(1, f, idx); }
static int mcan1_send_add_req(uint8_t idx) { return mcan_send_add_request_impl(1, idx); }
static int mcan1_receive(intf_can_frame_t *f, uint32_t t) { return mcan_receive_impl(1, f, t); }
static int mcan1_receive_nb(intf_can_frame_t *f) { return mcan_receive_nonblocking_impl(1, f); }
static int mcan1_cfg_filter(uint32_t i, const intf_can_filter_elem_t *e) { return mcan_config_filter_impl(1, i, e); }
static int mcan1_enable_int(uint32_t m) { return mcan_enable_interrupt_impl(1, m); }
static int mcan1_disable_int(uint32_t m) { return mcan_disable_interrupt_impl(1, m); }
static int mcan1_cfg_irq_cb(intf_can_irq_callback_t cb, void *d) { return mcan_config_irq_callback_impl(1, cb, d); }
static int mcan1_get_status(intf_can_status_t *s) { return mcan_get_status_impl(1, s); }
static int mcan1_read_tx_evt(intf_can_tx_event_t *e) { return mcan_read_tx_event_impl(1, e); }
static int mcan1_get_ts(const intf_can_tx_event_t *e, intf_can_timestamp_t *t) { return mcan_get_timestamp_impl(1, e, t); }

static const intf_can_t mcan1_ops = {
    .instance_id = 1,
    .init = mcan1_init,
    .deinit = mcan1_deinit,
    .send = mcan1_send,
    .send_nonblocking = mcan1_send_nb,
    .send_add_request = mcan1_send_add_req,
    .receive = mcan1_receive,
    .receive_nonblocking = mcan1_receive_nb,
    .config_filter = mcan1_cfg_filter,
    .enable_interrupt = mcan1_enable_int,
    .disable_interrupt = mcan1_disable_int,
    .config_irq_callback = mcan1_cfg_irq_cb,
    .get_status = mcan1_get_status,
    .read_tx_event = mcan1_read_tx_evt,
    .get_timestamp = mcan1_get_ts,
};
#endif

#if defined(HPM_MCAN2)
static int mcan2_init(const intf_can_cfg_t *cfg) { return mcan_init_impl(2, cfg); }
static void mcan2_deinit(void) { mcan_deinit_impl(2); }
static int mcan2_send(const intf_can_frame_t *f, uint32_t t) { return mcan_send_impl(2, f, t); }
static int mcan2_send_nb(const intf_can_frame_t *f, uint8_t *idx) { return mcan_send_nonblocking_impl(2, f, idx); }
static int mcan2_send_add_req(uint8_t idx) { return mcan_send_add_request_impl(2, idx); }
static int mcan2_receive(intf_can_frame_t *f, uint32_t t) { return mcan_receive_impl(2, f, t); }
static int mcan2_receive_nb(intf_can_frame_t *f) { return mcan_receive_nonblocking_impl(2, f); }
static int mcan2_cfg_filter(uint32_t i, const intf_can_filter_elem_t *e) { return mcan_config_filter_impl(2, i, e); }
static int mcan2_enable_int(uint32_t m) { return mcan_enable_interrupt_impl(2, m); }
static int mcan2_disable_int(uint32_t m) { return mcan_disable_interrupt_impl(2, m); }
static int mcan2_cfg_irq_cb(intf_can_irq_callback_t cb, void *d) { return mcan_config_irq_callback_impl(2, cb, d); }
static int mcan2_get_status(intf_can_status_t *s) { return mcan_get_status_impl(2, s); }
static int mcan2_read_tx_evt(intf_can_tx_event_t *e) { return mcan_read_tx_event_impl(2, e); }
static int mcan2_get_ts(const intf_can_tx_event_t *e, intf_can_timestamp_t *t) { return mcan_get_timestamp_impl(2, e, t); }

static const intf_can_t mcan2_ops = {
    .instance_id = 2,
    .init = mcan2_init,
    .deinit = mcan2_deinit,
    .send = mcan2_send,
    .send_nonblocking = mcan2_send_nb,
    .send_add_request = mcan2_send_add_req,
    .receive = mcan2_receive,
    .receive_nonblocking = mcan2_receive_nb,
    .config_filter = mcan2_cfg_filter,
    .enable_interrupt = mcan2_enable_int,
    .disable_interrupt = mcan2_disable_int,
    .config_irq_callback = mcan2_cfg_irq_cb,
    .get_status = mcan2_get_status,
    .read_tx_event = mcan2_read_tx_evt,
    .get_timestamp = mcan2_get_ts,
};
#endif

#if defined(HPM_MCAN3)
static int mcan3_init(const intf_can_cfg_t *cfg) { return mcan_init_impl(3, cfg); }
static void mcan3_deinit(void) { mcan_deinit_impl(3); }
static int mcan3_send(const intf_can_frame_t *f, uint32_t t) { return mcan_send_impl(3, f, t); }
static int mcan3_send_nb(const intf_can_frame_t *f, uint8_t *idx) { return mcan_send_nonblocking_impl(3, f, idx); }
static int mcan3_send_add_req(uint8_t idx) { return mcan_send_add_request_impl(3, idx); }
static int mcan3_receive(intf_can_frame_t *f, uint32_t t) { return mcan_receive_impl(3, f, t); }
static int mcan3_receive_nb(intf_can_frame_t *f) { return mcan_receive_nonblocking_impl(3, f); }
static int mcan3_cfg_filter(uint32_t i, const intf_can_filter_elem_t *e) { return mcan_config_filter_impl(3, i, e); }
static int mcan3_enable_int(uint32_t m) { return mcan_enable_interrupt_impl(3, m); }
static int mcan3_disable_int(uint32_t m) { return mcan_disable_interrupt_impl(3, m); }
static int mcan3_cfg_irq_cb(intf_can_irq_callback_t cb, void *d) { return mcan_config_irq_callback_impl(3, cb, d); }
static int mcan3_get_status(intf_can_status_t *s) { return mcan_get_status_impl(3, s); }
static int mcan3_read_tx_evt(intf_can_tx_event_t *e) { return mcan_read_tx_event_impl(3, e); }
static int mcan3_get_ts(const intf_can_tx_event_t *e, intf_can_timestamp_t *t) { return mcan_get_timestamp_impl(3, e, t); }

static const intf_can_t mcan3_ops = {
    .instance_id = 3,
    .init = mcan3_init,
    .deinit = mcan3_deinit,
    .send = mcan3_send,
    .send_nonblocking = mcan3_send_nb,
    .send_add_request = mcan3_send_add_req,
    .receive = mcan3_receive,
    .receive_nonblocking = mcan3_receive_nb,
    .config_filter = mcan3_cfg_filter,
    .enable_interrupt = mcan3_enable_int,
    .disable_interrupt = mcan3_disable_int,
    .config_irq_callback = mcan3_cfg_irq_cb,
    .get_status = mcan3_get_status,
    .read_tx_event = mcan3_read_tx_evt,
    .get_timestamp = mcan3_get_ts,
};
#endif

/* ============================================================================
 * 驱动注册 — 对 App 层暴露的唯一入口
 * ============================================================================ */

void hpm_can_driver_register(void)
{
    mcan_init_instance_table();

#if defined(HPM_MCAN0)
    intf_can_register(&mcan0_ops);
#endif
#if defined(HPM_MCAN1)
    intf_can_register(&mcan1_ops);
#endif
#if defined(HPM_MCAN2)
    intf_can_register(&mcan2_ops);
#endif
#if defined(HPM_MCAN3)
    intf_can_register(&mcan3_ops);
#endif
}
