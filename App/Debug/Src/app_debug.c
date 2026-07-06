#include "app_debug.h"
#include "app_analog_signal.h"
#include "app_debug_adc.h"
#include "app_debug_hrpwm.h"
#include "app_debug_profiler.h"
#include "intf_clock.h"

void app_debug_init(void) { }

void app_debug_run_once(void)
{
    static uint32_t last_cycle;
    static bool initialized;

    uint32_t cycle = intf_clock_get_cycle();
    uint32_t elapsed = initialized ? (cycle - last_cycle) : 0U;

    app_analog_signal_process();
    app_debug_adc_dump_pmt();
    app_debug_adc_dump_analog_signal();
    app_debug_dump_hrpwm_freq();
    app_debug_adc_dump_diag();
    app_debug_profiler_dump(intf_clock_get_cpu_freq(), elapsed);

    last_cycle = cycle;
    initialized = true;
}
