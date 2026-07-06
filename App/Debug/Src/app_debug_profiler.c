#include "app_debug_profiler.h"

#include "app_debug_rtt.h"
#include "irq_profiler.h"

#include <stdint.h>

static uint32_t rate_hz(uint32_t delta, uint32_t elapsed_cycles, uint32_t cpu_freq)
{
    if (elapsed_cycles == 0U || cpu_freq == 0U) {
        return 0U;
    }

    return (uint32_t)(((uint64_t)delta * cpu_freq + elapsed_cycles / 2U) / elapsed_cycles);
}

void app_debug_profiler_dump(uint32_t cpu_freq, uint32_t elapsed_cycles)
{
    irq_prof_result_t r;
    uint8_t slot_count = irq_prof_get_slot_count();
    irq_prof_cycle_t overhead = irq_prof_get_overhead_cycles();

    static uint32_t last_hits[IRQ_PROF_MAX_SLOTS];
    static uint32_t last_outliers[IRQ_PROF_MAX_SLOTS];
    static bool initialized;

    float factor_ns = (cpu_freq > 0U) ? (1e9f / (float)cpu_freq) : 0.0f;

    app_debug_printf("\r\n[IRQ_PROF] CPU=%uMHz, overhead=%u cycles, elapsed=%lu us\r\n",
                     cpu_freq / 1000000, overhead,
                     (unsigned long)((uint64_t)elapsed_cycles * 1000000ULL / cpu_freq));
    app_debug_printf("----------------------------------------------------------------------------\r\n");
    app_debug_printf("  Slot  Label         last    min    max    avg  ns  hits(+d,Hz)        out(+d)\r\n");
    app_debug_printf("----------------------------------------------------------------------------\r\n");

    for (uint8_t i = 0; i < slot_count; i++) {
        if (irq_prof_get_result(i, &r) != 0) {
            continue;
        }

        uint32_t delta = initialized ? r.hits - last_hits[i] : 0U;
        uint32_t outlier_delta = initialized ? r.outliers - last_outliers[i] : 0U;
        uint32_t hz = rate_hz(delta, elapsed_cycles, cpu_freq);

        app_debug_printf("  [%02u]  %-12s %5u %5u %5u %5u      n=%lu(+%lu,%luHz) out=%lu(+%lu)\r\n",
                         i, irq_prof_get_label(i),
                         (uint32_t)(r.last_ns * factor_ns),
                         (uint32_t)(r.min_ns * factor_ns),
                         (uint32_t)(r.max_ns * factor_ns),
                         (uint32_t)(r.avg_ns * factor_ns),
                         (unsigned long)r.hits,
                         (unsigned long)delta,
                         (unsigned long)hz,
                         (unsigned long)r.outliers,
                         (unsigned long)outlier_delta);

        last_hits[i] = r.hits;
        last_outliers[i] = r.outliers;
    }

    initialized = true;

    app_debug_printf("----------------------------------------------------------------------------\r\n");
}
