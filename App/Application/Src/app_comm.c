#include "app_comm.h"

int app_comm_init(void) { return 0; }
void app_comm_tick(void) { }
void app_comm_report_fault(uint32_t code) { (void)code; }
void app_comm_send_telemetry(void) { }
bool app_comm_is_timeout(void) { return false; }
