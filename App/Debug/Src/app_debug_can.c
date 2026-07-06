#include "app_debug_can.h"

#include "app_can.h"
#include "app_debug_rtt.h"
#include "intf_can.h"
#include "intf_clock.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

extern void hpm_can_driver_register(void);

static void can_test_rx_callback(const app_can_msg_t *msg)
{
    app_debug_printf("[CAN] RX ID=0x%03lX DLC=%u data=", (unsigned long) msg->id, msg->dlc);
    for (uint8_t i = 0U; i < msg->dlc; i++) {
        app_debug_printf("%02X ", msg->data[i]);
    }
    app_debug_printf("\r\n");
}

static void app_debug_can_dump_status_and_stats(void)
{
    intf_can_status_t status;
    app_can_stats_t stats;

    if (app_can_get_status(&status) == 0) {
        app_debug_printf("[CAN] STS tx_err=%u rx_err=%u bus_off=%d warn=%d passive=%d lec=%u\r\n",
                         status.tx_error_count, status.rx_error_count, status.bus_off,
                         status.error_warning, status.error_passive, status.last_error_code);
    } else {
        app_debug_printf("[CAN] STS read FAILED\r\n");
    }

    if (app_can_get_stats(&stats) == 0) {
        app_debug_printf("[CAN] STAT rx=%lu irq=%lu drop=%lu ovf=%lu fifo_full=%lu fifo_lost=%lu tx_enq=%lu tx_ok=%lu tx_fail=%lu pending=%u evt=0x%08lX\r\n",
                         (unsigned long) stats.rx_count, (unsigned long) stats.rx_irq_count,
                         (unsigned long) stats.rx_drop_count,
                         (unsigned long) stats.rx_overflow_count,
                         (unsigned long) stats.rx_fifo_full_count,
                         (unsigned long) stats.rx_fifo_lost_count,
                         (unsigned long) stats.tx_enqueue_ok_count,
                         (unsigned long) stats.tx_ok_count,
                         (unsigned long) stats.tx_fail_count, stats.rx_pending_count,
                         (unsigned long) stats.last_event_flags);
        app_debug_printf("[CAN] ERR bus_off=%lu warn=%lu passive=%lu proto=%lu ram=%lu last_tx_ret=%d last_rx_id=0x%03lX last_rx_dlc=%u\r\n",
                         (unsigned long) stats.bus_off_count,
                         (unsigned long) stats.error_warning_count,
                         (unsigned long) stats.error_passive_count,
                         (unsigned long) stats.protocol_error_count,
                         (unsigned long) stats.ram_access_fail_count, stats.last_tx_ret,
                         (unsigned long) stats.last_rx_id, stats.last_rx_dlc);
    } else {
        app_debug_printf("[CAN] STAT read FAILED\r\n");
    }
}

void app_debug_can_run_tests(void)
{
    static bool initialized = false;
    static uint32_t tx_count = 0U;
    int ret;
    int filter_ret;

    if (!initialized) {
        intf_can_status_t tmp;
        app_debug_printf("\r\n[CAN] === CAN Test ===\r\n");

        if (app_can_get_status(&tmp) == 0) {
            app_debug_printf("[CAN] init OK\r\n");
        } else {
            app_debug_printf("[CAN] init FAILED\r\n");
            return;
        }

        app_can_set_rx_callback(can_test_rx_callback);
        app_can_clear_stats();

        filter_ret = app_can_add_std_filter(0x114U, 0x7FFU);
        if (filter_ret != 0) {
            app_debug_printf("[CAN] add std filter FAILED\r\n");
            return;
        }
        filter_ret = app_can_add_std_filter(0x000U, 0x000U);
        if (filter_ret != 0) {
            app_debug_printf("[CAN] add catch-all filter FAILED\r\n");
            return;
        }

        initialized = true;
        app_debug_can_dump_status_and_stats();
    }

    uint8_t tx_data[8];
    for (uint8_t i = 0U; i < 8U; i++) {
        tx_data[i] = (uint8_t) ((tx_count >> (i * 4U)) & 0x0FU) | (uint8_t) (i << 4U);
    }

    ret = app_can_send_std(0x114U, tx_data, 8U);
    app_debug_printf("[CAN] TX seq=%lu ret=%d\r\n", (unsigned long) tx_count, ret);
    tx_count++;

    intf_clock_delay_ms(100);
    app_can_poll();
    app_debug_can_dump_status_and_stats();
}

static volatile bool lb_rx_done;
static volatile app_can_msg_t lb_rx_msg;

static void lb_irq_handler(intf_can_inst_t inst, uint32_t events, void *user_data)
{
    (void)inst;
    (void)events;
    (void)user_data;
    if ((events & INTF_CAN_EVENT_RX_FIFO0_NEW_MSG) != 0U) {
        intf_can_frame_t f;
        memset(&f, 0, sizeof(f));
        if (intf_can_receive_nonblocking(0, &f) == 0) {
            lb_rx_msg.id = f.id;
            lb_rx_msg.is_ext_id = f.is_ext_id;
            lb_rx_msg.dlc = f.dlc;
            if (f.dlc != 0U) {
                memcpy((void *) lb_rx_msg.data, f.data, f.dlc);
            }
            lb_rx_done = true;
        }
    }
}

void app_debug_can_loopback_test(void)
{
    int ret;
    uint32_t wait_ms;

    app_debug_printf("\r\n[CAN] === CAN Internal Loopback Test ===\r\n");

    app_can_deinit();
    hpm_can_driver_register();
    memset((void *) &lb_rx_msg, 0, sizeof(lb_rx_msg));

    intf_can_cfg_t cfg = {
        .baudrate = 1000000U,
        .mode = INTF_CAN_MODE_LOOPBACK_INTERNAL,
        .enable_canfd = false,
        .interrupt_mask = INTF_CAN_EVENT_RX_FIFO0_NEW_MSG | INTF_CAN_EVENT_RX_FIFO0_FULL |
                          INTF_CAN_EVENT_RX_FIFO0_MSG_LOST,
        .ram = {
            .std_filter_count = APP_CAN_FILTER_COUNT,
            .ext_filter_count = APP_CAN_FILTER_COUNT,
        },
    };

    ret = intf_can_init(0, &cfg);
    if (ret != 0) {
        app_debug_printf("[CAN] LB init FAILED\r\n");
        return;
    }
    app_debug_printf("[CAN] LB init OK (internal loopback mode)\r\n");

    ret = intf_can_config_irq_callback(0, lb_irq_handler, NULL);
    if (ret != 0) {
        app_debug_printf("[CAN] LB IRQ callback config FAILED\r\n");
        intf_can_deinit(0);
        return;
    }

    intf_can_filter_elem_t filter = {
        .type = INTF_CAN_FILTER_CLASSIC,
        .target_fifo = INTF_CAN_FILTER_FIFO0,
        .id = 0U,
        .mask = 0U,
    };
    ret = intf_can_config_filter(0, 0, &filter);
    if (ret != 0) {
        app_debug_printf("[CAN] LB filter config FAILED\r\n");
        intf_can_config_irq_callback(0, NULL, NULL);
        intf_can_deinit(0);
        return;
    }

    uint8_t data[8] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11};
    intf_can_frame_t tx_frame = {
        .id = 0x114U,
        .dlc = 8U,
        .frame_type = INTF_CAN_FRAME_CLASSIC,
    };
    memcpy(tx_frame.data, data, 8);

    lb_rx_done = false;
    ret = intf_can_send_nonblocking(0, &tx_frame, NULL);
    app_debug_printf("[CAN] LB TX nonblocking ret=%d\r\n", ret);
    if (ret != 0) {
        intf_can_config_irq_callback(0, NULL, NULL);
        intf_can_deinit(0);
        return;
    }

    for (wait_ms = 0U; wait_ms < 100U && !lb_rx_done; wait_ms++) {
        intf_clock_delay_ms(1U);
    }

    if (lb_rx_done) {
        app_debug_printf("[CAN] LB RX OK: ID=0x%03lX DLC=%u data=", (unsigned long) lb_rx_msg.id,
                         lb_rx_msg.dlc);
        for (uint8_t i = 0U; i < lb_rx_msg.dlc; i++) {
            app_debug_printf("%02X ", lb_rx_msg.data[i]);
        }
        app_debug_printf("\r\n");

        bool match = true;
        for (uint8_t i = 0U; i < 8U; i++) {
            if (lb_rx_msg.data[i] != data[i]) {
                match = false;
                break;
            }
        }
        app_debug_printf("[CAN] LB data match: %s\r\n", match ? "YES" : "NO");
    } else {
        app_debug_printf("[CAN] LB RX TIMEOUT after %lu ms — MCAN internal loopback FAILED\r\n",
                         (unsigned long) wait_ms);
        intf_can_status_t st;
        intf_can_get_status(0, &st);
        app_debug_printf("[CAN] LB status: tx_err=%u rx_err=%u bus_off=%d\r\n",
                         st.tx_error_count, st.rx_error_count, st.bus_off);
    }

    intf_can_config_irq_callback(0, NULL, NULL);
    intf_can_deinit(0);
    app_can_init();
}
