#include "ctrl_fault.h"

void ctrl_fault_init(const ctrl_fault_thresholds_t *t)                 { (void)t; }
void ctrl_fault_set_thresholds(const ctrl_fault_thresholds_t *t)      { (void)t; }
uint32_t ctrl_fault_check(void)                                        { return 0U; }
uint32_t ctrl_fault_get_active(void)                                   { return 0U; }
int ctrl_fault_clear(uint32_t code)                                    { (void)code; return 0; }
int ctrl_fault_clear_all(void)                                         { return 0; }
bool ctrl_fault_is_hardware(void)                                      { return false; }
void ctrl_fault_set_callback(void (*cb)(uint32_t))                     { (void)cb; }
