#include "app_can.h"

#include "intf_can.h"

#include <stddef.h>
#include <string.h>

/*
 * 临界区保护：直接操作 RISC-V MSTATUS.MIE，避免引入驱动层依赖。
 * TODO: 后续可提升为 Interface 层通用抽象。
 */
static inline uint32_t can_critical_enter(void)
{
    uint32_t mie;
    __asm__ volatile("csrrc %0, mstatus, %1" : "=r"(mie) : "i"(0x8));
    return mie;
}

static inline void can_critical_exit(uint32_t state)
{
    __asm__ volatile("csrw mstatus, %0" :: "r"(state));
}

#define APP_CAN_INST      (0U)
#define APP_CAN_BAUDRATE  (1000000U)
#define APP_CAN_INT_MASK  (INTF_CAN_EVENT_RX_FIFO0_NEW_MSG   \
                           | INTF_CAN_EVENT_RX_FIFO0_FULL     \
                           | INTF_CAN_EVENT_RX_FIFO0_MSG_LOST \
                           | INTF_CAN_EVENT_TX_COMPLETED      \
                           | INTF_CAN_EVENT_BUS_OFF           \
                           | INTF_CAN_EVENT_ERROR_WARNING     \
                           | INTF_CAN_EVENT_ERROR_PASSIVE     \
                          | INTF_CAN_EVENT_PROTOCOL_ERROR    \
                          | INTF_CAN_EVENT_RAM_ACCESS_FAIL)

static app_can_rx_callback_t s_rx_callback;
static bool s_initialized;

static struct {
    app_can_msg_t buf[APP_CAN_RX_BUF_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
    volatile uint8_t count;
} s_rx_ring;

static app_can_stats_t s_stats;
static uint32_t s_filter_index;

static void app_can_driver_register_once(void)
{
    static bool registered = false;

    if (registered) {
        return;
    }
    registered = true;
}

extern void hpm_can_driver_register(void);
void app_can_register_driver(void)
{
    hpm_can_driver_register();
    app_can_driver_register_once();
}

static bool app_can_is_std_id_valid(uint32_t id)
{
    return id <= 0x7FFU;
}

static bool app_can_is_ext_id_valid(uint32_t id)
{
    return id <= 0x1FFFFFFFU;
}

static bool app_can_ring_pop_locked(app_can_msg_t *msg)
{
    if (s_rx_ring.count == 0U) {
        return false;
    }

    *msg = s_rx_ring.buf[s_rx_ring.tail];
    s_rx_ring.tail = (s_rx_ring.tail + 1U) % APP_CAN_RX_BUF_SIZE;
    s_rx_ring.count--;
    return true;
}

static void app_can_refresh_status_snapshot(void)
{
    intf_can_status_t status;

    if (intf_can_get_status(APP_CAN_INST, &status) == 0) {
        uint32_t irq_state = can_critical_enter();
        s_stats.last_status = status;
        can_critical_exit(irq_state);
    }
}

static void app_can_reset_state(void)
{
    uint32_t irq_state = can_critical_enter();

    memset(&s_rx_ring, 0, sizeof(s_rx_ring));
    memset(&s_stats, 0, sizeof(s_stats));
    s_rx_callback = NULL;
    s_filter_index = 0U;

    can_critical_exit(irq_state);
}

static void app_can_note_event_flags(uint32_t event_flags)
{
    s_stats.last_event_flags = event_flags;

    if ((event_flags & INTF_CAN_EVENT_RX_FIFO0_FULL) != 0U) {
        s_stats.rx_fifo_full_count++;
    }
    if ((event_flags & INTF_CAN_EVENT_RX_FIFO0_MSG_LOST) != 0U) {
        s_stats.rx_fifo_lost_count++;
    }
    if ((event_flags & INTF_CAN_EVENT_TX_COMPLETED) != 0U) {
        s_stats.tx_ok_count++;
    }
    if ((event_flags & INTF_CAN_EVENT_BUS_OFF) != 0U) {
        s_stats.bus_off_count++;
    }
    if ((event_flags & INTF_CAN_EVENT_ERROR_WARNING) != 0U) {
        s_stats.error_warning_count++;
    }
    if ((event_flags & INTF_CAN_EVENT_ERROR_PASSIVE) != 0U) {
        s_stats.error_passive_count++;
    }
    if ((event_flags & INTF_CAN_EVENT_PROTOCOL_ERROR) != 0U) {
        s_stats.protocol_error_count++;
    }
    if ((event_flags & INTF_CAN_EVENT_RAM_ACCESS_FAIL) != 0U) {
        s_stats.ram_access_fail_count++;
    }
}

static int app_can_send_internal(uint32_t id, bool is_ext_id,
                                 const uint8_t *data, uint8_t len)
{
    int ret;
    intf_can_frame_t frame;
    uint32_t irq_state;

    if (!s_initialized) {
        return -1;
    }
    if (is_ext_id) {
        if (!app_can_is_ext_id_valid(id)) {
            return -1;
        }
    } else {
        if (!app_can_is_std_id_valid(id)) {
            return -1;
        }
    }
    if (len > 8U) {
        return -1;
    }
    if (data == NULL && len != 0U) {
        return -1;
    }

    memset(&frame, 0, sizeof(frame));
    frame.id         = id;
    frame.is_ext_id  = is_ext_id;
    frame.frame_type = INTF_CAN_FRAME_CLASSIC;
    frame.dlc        = len;
    if (len != 0U) {
        memcpy(frame.data, data, len);
    }

    ret = intf_can_send_nonblocking(APP_CAN_INST, &frame, NULL);

    irq_state = can_critical_enter();
    s_stats.last_tx_ret = ret;
    if (ret == 0) {
        s_stats.tx_enqueue_ok_count++;
    } else {
        s_stats.tx_fail_count++;
    }
    can_critical_exit(irq_state);

    if (ret != 0) {
        app_can_refresh_status_snapshot();
    }
    return ret;
}

static int app_can_add_filter_internal(uint32_t id, bool is_ext_id,
                                       uint32_t mask)
{
    intf_can_filter_elem_t filter;
    int ret;

    if (!s_initialized) {
        return -1;
    }
    if (s_filter_index >= APP_CAN_FILTER_COUNT) {
        return -1;
    }
    if (is_ext_id) {
        if (!app_can_is_ext_id_valid(id) || !app_can_is_ext_id_valid(mask)) {
            return -1;
        }
    } else {
        if (!app_can_is_std_id_valid(id) || !app_can_is_std_id_valid(mask)) {
            return -1;
        }
    }

    memset(&filter, 0, sizeof(filter));
    filter.type        = INTF_CAN_FILTER_CLASSIC;
    filter.target_fifo = INTF_CAN_FILTER_FIFO0;
    filter.is_ext_id   = is_ext_id;
    filter.id          = id;
    filter.mask        = mask;

    ret = intf_can_config_filter(APP_CAN_INST, s_filter_index, &filter);
    if (ret == 0) {
        s_filter_index++;
    }
    return ret;
}

static void can_irq_handler(intf_can_inst_t inst, uint32_t event_flags,
                            void *user_data)
{
    (void)inst;
    (void)user_data;

    s_stats.rx_irq_count++;
    app_can_note_event_flags(event_flags);

    if ((event_flags & INTF_CAN_EVENT_RX_FIFO0_NEW_MSG) != 0U) {
        intf_can_frame_t frame;
        memset(&frame, 0, sizeof(frame));

        while (intf_can_receive_nonblocking(APP_CAN_INST, &frame) == 0) {
            app_can_msg_t local_msg;

            memset(&local_msg, 0, sizeof(local_msg));
            local_msg.id        = frame.id;
            local_msg.is_ext_id = frame.is_ext_id;
            local_msg.dlc       = frame.dlc;
            local_msg.timestamp = frame.timestamp;
            if (frame.dlc != 0U) {
                memcpy(local_msg.data, frame.data, frame.dlc);
            }

            s_stats.rx_count++;
            s_stats.last_rx_id = local_msg.id;
            s_stats.last_rx_dlc = local_msg.dlc;

            if (s_rx_ring.count < APP_CAN_RX_BUF_SIZE) {
                s_rx_ring.buf[s_rx_ring.head] = local_msg;
                s_rx_ring.head = (s_rx_ring.head + 1U) % APP_CAN_RX_BUF_SIZE;
                s_rx_ring.count++;
            } else {
                s_stats.rx_drop_count++;
                s_stats.rx_overflow_count++;
            }
        }
    }

    if ((event_flags & INTF_CAN_EVENT_BUS_OFF) != 0U) {
        s_rx_ring.count = 0U;
        s_rx_ring.head  = 0U;
        s_rx_ring.tail  = 0U;
    }
}

int app_can_init(void)
{
    int ret;

    app_can_driver_register_once();

    if (s_initialized) {
        app_can_deinit();
    }

    app_can_reset_state();

    {
        intf_can_cfg_t cfg = {
            .baudrate     = APP_CAN_BAUDRATE,
            .mode         = INTF_CAN_MODE_NORMAL,
            .enable_canfd = false,
            .interrupt_mask = APP_CAN_INT_MASK,
            .ram = {
                .std_filter_count = APP_CAN_FILTER_COUNT,
                .ext_filter_count = APP_CAN_FILTER_COUNT,
            },
        };

        ret = intf_can_init(APP_CAN_INST, &cfg);
    }
    if (ret != 0) {
        return -1;
    }

    ret = intf_can_config_irq_callback(APP_CAN_INST, can_irq_handler, NULL);
    if (ret != 0) {
        intf_can_deinit(APP_CAN_INST);
        return -1;
    }

    s_initialized = true;
    app_can_refresh_status_snapshot();
    return 0;
}

void app_can_deinit(void)
{
    if (!s_initialized) {
        app_can_reset_state();
        return;
    }

    intf_can_config_irq_callback(APP_CAN_INST, NULL, NULL);
    intf_can_deinit(APP_CAN_INST);
    app_can_reset_state();
    s_initialized = false;
}

void app_can_set_rx_callback(app_can_rx_callback_t cb)
{
    uint32_t irq_state = can_critical_enter();
    s_rx_callback = cb;
    can_critical_exit(irq_state);
}

void app_can_poll(void)
{
    for (;;) {
        app_can_msg_t msg;
        app_can_rx_callback_t cb;
        uint32_t irq_state = can_critical_enter();

        cb = s_rx_callback;
        if (cb == NULL || !app_can_ring_pop_locked(&msg)) {
            can_critical_exit(irq_state);
            return;
        }

        can_critical_exit(irq_state);
        cb(&msg);
    }
}

int app_can_send(uint32_t id, const uint8_t *data, uint8_t len)
{
    return app_can_send_internal(id, id > 0x7FFU, data, len);
}

int app_can_send_std(uint16_t id, const uint8_t *data, uint8_t len)
{
    return app_can_send_internal(id, false, data, len);
}

int app_can_send_ext(uint32_t id, const uint8_t *data, uint8_t len)
{
    return app_can_send_internal(id, true, data, len);
}

int app_can_receive(app_can_msg_t *msg)
{
    bool popped;
    uint32_t irq_state;

    if (!s_initialized || msg == NULL) {
        return -1;
    }

    irq_state = can_critical_enter();
    popped = app_can_ring_pop_locked(msg);
    can_critical_exit(irq_state);
    return popped ? 0 : -1;
}

int app_can_add_filter(uint32_t id, uint32_t mask)
{
    if (app_can_is_std_id_valid(id) && app_can_is_std_id_valid(mask)) {
        return app_can_add_std_filter((uint16_t)id, (uint16_t)mask);
    }
    return app_can_add_ext_filter(id, mask);
}

int app_can_add_std_filter(uint16_t id, uint16_t mask)
{
    return app_can_add_filter_internal(id, false, mask);
}

int app_can_add_ext_filter(uint32_t id, uint32_t mask)
{
    return app_can_add_filter_internal(id, true, mask);
}

int app_can_get_status(intf_can_status_t *status)
{
    if (!s_initialized || status == NULL) {
        return -1;
    }
    if (intf_can_get_status(APP_CAN_INST, status) != 0) {
        return -1;
    }

    {
        uint32_t irq_state = can_critical_enter();
        s_stats.last_status = *status;
        can_critical_exit(irq_state);
    }
    return 0;
}

int app_can_get_stats(app_can_stats_t *stats)
{
    uint32_t irq_state;

    if (stats == NULL) {
        return -1;
    }

    irq_state = can_critical_enter();
    *stats = s_stats;
    stats->rx_pending_count = s_rx_ring.count;
    can_critical_exit(irq_state);
    return 0;
}

void app_can_clear_stats(void)
{
    uint32_t irq_state = can_critical_enter();

    memset(&s_stats, 0, sizeof(s_stats));

    can_critical_exit(irq_state);
    if (s_initialized) {
        app_can_refresh_status_snapshot();
    }
}

bool app_can_is_bus_off(void)
{
    intf_can_status_t status;

    if (app_can_get_status(&status) != 0) {
        return true;
    }
    return status.bus_off;
}
