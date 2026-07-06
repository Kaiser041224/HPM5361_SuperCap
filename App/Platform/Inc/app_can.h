#ifndef APP_CAN_H
#define APP_CAN_H

#include "intf_can.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define APP_CAN_RX_BUF_SIZE  (16U)
#define APP_CAN_FILTER_COUNT (16U)

typedef struct {
    uint32_t id;
    bool     is_ext_id;
    uint8_t  dlc;
    uint8_t  data[64];
    uint32_t timestamp;
} app_can_msg_t;

typedef void (*app_can_rx_callback_t)(const app_can_msg_t *msg);

typedef struct {
    uint32_t rx_count;
    uint32_t rx_irq_count;
    uint32_t rx_drop_count;
    uint32_t rx_overflow_count;
    uint32_t rx_fifo_full_count;
    uint32_t rx_fifo_lost_count;
    uint32_t tx_enqueue_ok_count;
    uint32_t tx_ok_count;
    uint32_t tx_fail_count;
    uint32_t bus_off_count;
    uint32_t error_warning_count;
    uint32_t error_passive_count;
    uint32_t protocol_error_count;
    uint32_t ram_access_fail_count;
    uint32_t last_event_flags;
    uint32_t last_rx_id;
    uint8_t  last_rx_dlc;
    int      last_tx_ret;
    uint8_t  rx_pending_count;
    intf_can_status_t last_status;
} app_can_stats_t;

int  app_can_init(void);
void app_can_register_driver(void);
void app_can_deinit(void);
void app_can_set_rx_callback(app_can_rx_callback_t cb);
void app_can_poll(void);
int  app_can_send(uint32_t id, const uint8_t *data, uint8_t len);
int  app_can_send_std(uint16_t id, const uint8_t *data, uint8_t len);
int  app_can_send_ext(uint32_t id, const uint8_t *data, uint8_t len);
int  app_can_receive(app_can_msg_t *msg);
int  app_can_add_filter(uint32_t id, uint32_t mask);
int  app_can_add_std_filter(uint16_t id, uint16_t mask);
int  app_can_add_ext_filter(uint32_t id, uint32_t mask);
int  app_can_get_status(intf_can_status_t *status);
int  app_can_get_stats(app_can_stats_t *stats);
void app_can_clear_stats(void);
bool app_can_is_bus_off(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_CAN_H */
