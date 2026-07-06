#include "app_debug_adc.h"

#include "app_adc.h"
#include "app_analog_signal.h"
#include "app_debug_rtt.h"
#include "intf_adc.h"
#include "intf_clock.h"

#include <stdbool.h>
#include <stdint.h>

static const char *adc_ch_names[ADC_CH_COUNT] = {
    [ADC_CH_VCAP] = "VCAP ",
    [ADC_CH_VOUT] = "VOUT ",
    [ADC_CH_I_IN] = "I_IN ",
    [ADC_CH_I_L]  = "I_L  ",
};

static const char *analog_signal_names[APP_ANALOG_SIGNAL_ITEM_COUNT] = {
    [APP_ANALOG_SIGNAL_ITEM_VCAP] = "VCAP ",
    [APP_ANALOG_SIGNAL_ITEM_VOUT] = "VOUT ",
    [APP_ANALOG_SIGNAL_ITEM_I_IN] = "I_IN ",
    [APP_ANALOG_SIGNAL_ITEM_I_L]  = "I_L  ",
};

static const char *analog_signal_units[APP_ANALOG_SIGNAL_ITEM_COUNT] = {
    [APP_ANALOG_SIGNAL_ITEM_VCAP] = "V",
    [APP_ANALOG_SIGNAL_ITEM_VOUT] = "V",
    [APP_ANALOG_SIGNAL_ITEM_I_IN] = "A",
    [APP_ANALOG_SIGNAL_ITEM_I_L]  = "A",
};

static uint8_t app_debug_adc_inst(adc_channel_t ch)
{
    return (ch == ADC_CH_I_L) ? 0U : 1U;
}

static uint32_t app_debug_adc_rate_hz(uint32_t delta, uint32_t elapsed_cycles)
{
    if (elapsed_cycles == 0U) {
        return 0U;
    }

    uint32_t cpu_hz = intf_clock_get_cpu_freq();
    if (cpu_hz == 0U) {
        return 0U;
    }

    return (uint32_t)(((uint64_t)delta * cpu_hz + elapsed_cycles / 2U) / elapsed_cycles);
}

static uint32_t app_debug_adc_elapsed_us(uint32_t elapsed_cycles)
{
    uint32_t cpu_hz = intf_clock_get_cpu_freq();
    if (cpu_hz == 0U) {
        return 0U;
    }

    return (uint32_t)(((uint64_t)elapsed_cycles * 1000000ULL + cpu_hz / 2U) / cpu_hz);
}

static void app_debug_adc_print_ratio(const char *label, uint32_t numerator, uint32_t denominator)
{
    if (denominator == 0U) {
        app_debug_printf("%s=%s", label, (numerator > 0U) ? "INF" : "NA");
        return;
    }

    uint32_t permille = (uint32_t)(((uint64_t)numerator * 1000ULL + denominator / 2U) / denominator);
    app_debug_printf("%s=%lu.%03lu", label,
                     (unsigned long)(permille / 1000U),
                     (unsigned long)(permille % 1000U));
}

void app_debug_adc_dump_channels(void)
{
    app_debug_printf("[ADC]  Channel  Inst  Raw(hex)  Raw(dec)   ADC(mV)  Sense(mV)  Physical\r\n");

    for (adc_channel_t ch = ADC_CH_VCAP; ch < ADC_CH_COUNT; ch++) {
        uint16_t raw = app_adc_read_raw(ch);
        float adc_mv = 0.0f;
        float sense_mv = 0.0f;
        float physical = 0.0f;

        (void)app_adc_read_adc_voltage_mv(ch, &adc_mv);
        (void)app_adc_read_sense_voltage_mv(ch, &sense_mv);
        (void)app_adc_read_physical(ch, &physical);

        app_debug_printf(
            "[ADC]  %s   ADC%d  0x%04X   %5u   %8.1f   %9.1f   %8.3f\r\n",
            adc_ch_names[ch], app_debug_adc_inst(ch), raw, raw, adc_mv, sense_mv, physical);
    }
}

void app_debug_adc_dump_pmt(void)
{
    uint16_t val[ADC_CH_COUNT];
    bool valid = false;

    for (adc_channel_t ch = ADC_CH_VCAP; ch < ADC_CH_COUNT; ch++) {
        if (app_adc_get_pmt_raw(ch, &val[ch]) != 0) {
            val[ch] = 0;
        } else {
            valid = true;
        }
    }

    if (!valid) {
        return;
    }
    app_debug_printf("--------------------------------------------------\r\n");
    app_debug_printf("[ADC]  Channel  Inst  Raw(hex)  Raw(dec)     mV\r\n");

    for (adc_channel_t ch = ADC_CH_VCAP; ch < ADC_CH_COUNT; ch++) {
        float mv = (float)val[ch] * 3300.0f / 65535.0f;
        app_debug_printf(
            "[ADC]  %s   ADC%d  0x%04X   %5u   %7.1f\r\n", adc_ch_names[ch],
            app_debug_adc_inst(ch), val[ch], val[ch], mv);
    }
}

void app_debug_adc_dump_analog_signal(void)
{
    const float *raw_vals[] = {
        &g_analog_signal_snapshot.raw.vcap_v,
        &g_analog_signal_snapshot.raw.vout_v,
        &g_analog_signal_snapshot.raw.i_in_a,
        &g_analog_signal_snapshot.raw.i_l_a,
    };
    const float *flt_vals[] = {
        &g_analog_signal_snapshot.filtered.vcap_v,
        &g_analog_signal_snapshot.filtered.vout_v,
        &g_analog_signal_snapshot.filtered.i_in_a,
        &g_analog_signal_snapshot.filtered.i_l_a,
    };

    app_debug_printf("--------------------------------------------------\r\n");
    app_debug_printf("[ADC]  Signal   Unit     Raw        Filtered\r\n");

    for (app_analog_signal_item_t item = APP_ANALOG_SIGNAL_ITEM_VCAP;
         item < APP_ANALOG_SIGNAL_ITEM_COUNT; item++) {
        app_debug_printf("[ADC]  %s   %s    %8.3f    %8.3f\r\n",
                         analog_signal_names[item], analog_signal_units[item],
                         *raw_vals[item], *flt_vals[item]);
    }
}

void app_debug_adc_dump_diag(void)
{
    intf_adc_diag_snapshot_t diag;
    intf_adc_get_diag_snapshot(&diag);

    app_debug_printf("[ADC] diag: irq0=%lu irq1=%lu max0=%lu max1=%lu invalid0=%lu invalid1=%lu\r\n",
                     (unsigned long)diag.irq_entry[0],
                     (unsigned long)diag.irq_entry[1],
                     (unsigned long)diag.isr_cycles_max[0],
                     (unsigned long)diag.isr_cycles_max[1],
                     (unsigned long)diag.pmt_invalid[0],
                     (unsigned long)diag.pmt_invalid[1]);

    app_debug_printf("[ADC] miss: ADC0 cycle=%lu trig=%lu ch=%lu | ADC1 cycle=%lu trig=%lu ch=%lu\r\n",
                     (unsigned long)diag.pmt_invalid_cycle[0],
                     (unsigned long)diag.pmt_invalid_trig[0],
                     (unsigned long)diag.pmt_invalid_channel[0],
                     (unsigned long)diag.pmt_invalid_cycle[1],
                     (unsigned long)diag.pmt_invalid_trig[1],
                     (unsigned long)diag.pmt_invalid_channel[1]);
}

void app_debug_adc_run_tests(void)
{
    app_debug_printf("\r\n[ADC] === ADC Validation Tests ===\r\n");

    app_debug_adc_dump_channels();
    app_debug_adc_dump_pmt();
    app_analog_signal_snapshot_refresh_raw();
    app_debug_adc_dump_analog_signal();
    app_debug_adc_dump_diag();

    uint32_t start_cycles = intf_clock_get_cycle();
    intf_adc_diag_snapshot_t diag0;
    intf_adc_get_diag_snapshot(&diag0);
    intf_clock_delay_ms(1000);
    intf_adc_diag_snapshot_t diag1;
    intf_adc_get_diag_snapshot(&diag1);
    uint32_t elapsed_cycles = intf_clock_get_cycle() - start_cycles;

    uint32_t adc0_rate = app_debug_adc_rate_hz(diag1.irq_entry[0] - diag0.irq_entry[0], elapsed_cycles);
    uint32_t adc1_rate = app_debug_adc_rate_hz(diag1.irq_entry[1] - diag0.irq_entry[1], elapsed_cycles);
    app_debug_printf("[ADC] IRQ rate: ADC0=%luHz ADC1=%luHz elapsed=%luus ",
                     (unsigned long)adc0_rate, (unsigned long)adc1_rate,
                     (unsigned long)app_debug_adc_elapsed_us(elapsed_cycles));
    app_debug_adc_print_ratio("ratio", adc0_rate, adc1_rate);
    app_debug_printf("\r\n");
}
