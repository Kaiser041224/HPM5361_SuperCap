/*
 * Analog Signal Conditioning Module
 *
 * Centralizes ADC calibration defaults and physical measurement conversion.
 */

#include "app_analog_signal.h"

#include "algo_filter.h"
#include "intf_attr.h"

#include <stdbool.h>
#include <stddef.h>

#define APP_ANALOG_SIGNAL_VOLTAGE_DIVIDER_GAIN ((100.0f + 3.3f) / 3.3f)
#define APP_ANALOG_SIGNAL_INA241A3_GAIN        (50.0f)
#define APP_ANALOG_SIGNAL_I_IN_RSENSE_OHM      (0.002f)
#define APP_ANALOG_SIGNAL_I_L_RSENSE_OHM       (0.001f)
#define APP_ANALOG_SIGNAL_I_IN_GAIN_A_PER_V \
    (1.0f / (APP_ANALOG_SIGNAL_I_IN_RSENSE_OHM * APP_ANALOG_SIGNAL_INA241A3_GAIN))
#define APP_ANALOG_SIGNAL_I_L_GAIN_A_PER_V \
    (1.0f / (APP_ANALOG_SIGNAL_I_L_RSENSE_OHM * APP_ANALOG_SIGNAL_INA241A3_GAIN))
#define APP_ANALOG_SIGNAL_BIDIR_BIAS_MV (INTF_ADC_DEFAULT_VREF_MV * 0.5f)

#define APP_ANALOG_SIGNAL_FILTER_MA_MAX_WINDOW (8U)
#define I_IN_FIX_A                             (0.1f) /* IIN 偏置修正 */
typedef enum {
    APP_ANALOG_SIGNAL_FILTER_MA = 0,
    APP_ANALOG_SIGNAL_FILTER_LPF,
} app_analog_signal_filter_type_t;

typedef struct {
    app_analog_signal_filter_type_t type;
    union {
        struct {
            uint16_t window_size;
        } ma;
        struct {
            float cutoff_hz;
            float sample_rate_hz;
        } lpf;
    } cfg;
} app_analog_signal_filter_cfg_t;

ATTR_PLACE_AT_FAST_RAM_INIT static const adc_channel_t
    s_item_to_channel[APP_ANALOG_SIGNAL_ITEM_COUNT] = {
        [APP_ANALOG_SIGNAL_ITEM_VCAP] = ADC_CH_VCAP,
        [APP_ANALOG_SIGNAL_ITEM_VOUT] = ADC_CH_VOUT,
        [APP_ANALOG_SIGNAL_ITEM_I_IN] = ADC_CH_I_IN,
        [APP_ANALOG_SIGNAL_ITEM_I_L] = ADC_CH_I_L,
    };

ATTR_PLACE_AT_FAST_RAM_INIT const app_analog_signal_item_t s_channel_to_item[ADC_CH_COUNT] = {
    [ADC_CH_VCAP] = APP_ANALOG_SIGNAL_ITEM_VCAP,
    [ADC_CH_VOUT] = APP_ANALOG_SIGNAL_ITEM_VOUT,
    [ADC_CH_I_IN] = APP_ANALOG_SIGNAL_ITEM_I_IN,
    [ADC_CH_I_L] = APP_ANALOG_SIGNAL_ITEM_I_L,
};

static const app_adc_calibration_t s_default_calibration[ADC_CH_COUNT] = {
    [ADC_CH_VCAP] =
        {
                       .sense_gain = 1.0f,
                       .sense_offset_mv = 0.0f,
                       .physical_gain = 0.001f * APP_ANALOG_SIGNAL_VOLTAGE_DIVIDER_GAIN,
                       .physical_offset = 0.0f,
                       },
    [ADC_CH_VOUT] =
        {
                       .sense_gain = 1.0f,
                       .sense_offset_mv = 0.0f,
                       .physical_gain = 0.001f * APP_ANALOG_SIGNAL_VOLTAGE_DIVIDER_GAIN,
                       .physical_offset = 0.0f,
                       },
    [ADC_CH_I_IN] =
        {
                       .sense_gain = 1.0f,
                       .sense_offset_mv = 0.0f,
                       .physical_gain = 0.001f * APP_ANALOG_SIGNAL_I_IN_GAIN_A_PER_V,
                       .physical_offset =
                -0.001f * APP_ANALOG_SIGNAL_BIDIR_BIAS_MV * APP_ANALOG_SIGNAL_I_IN_GAIN_A_PER_V
                + I_IN_FIX_A,
                       },
    [ADC_CH_I_L] =
        {.sense_gain = 1.0f,
                       .sense_offset_mv = 0.0f,
                       .physical_gain = 0.001f * APP_ANALOG_SIGNAL_I_L_GAIN_A_PER_V,
                       .physical_offset =
             -0.001f * APP_ANALOG_SIGNAL_BIDIR_BIAS_MV * APP_ANALOG_SIGNAL_I_L_GAIN_A_PER_V},
};

static const app_analog_signal_filter_cfg_t s_default_filter_cfg[ADC_CH_COUNT] = {
    [ADC_CH_VCAP] =
        {.type = APP_ANALOG_SIGNAL_FILTER_LPF,
                       .cfg.lpf = {.cutoff_hz = 40000.0f, .sample_rate_hz = 200000.0f}     },
    [ADC_CH_VOUT] = { .type = APP_ANALOG_SIGNAL_FILTER_MA,  .cfg.ma = {.window_size = 4U}},
    [ADC_CH_I_IN] = { .type = APP_ANALOG_SIGNAL_FILTER_MA, .cfg.ma = {.window_size = 16U}},
    [ADC_CH_I_L] =
        {.type = APP_ANALOG_SIGNAL_FILTER_LPF,
                       .cfg.lpf = {.cutoff_hz = 20000.0f, .sample_rate_hz = 200000.0f}     },
};

ATTR_PLACE_AT_FAST_RAM_BSS algo_ma_t s_ma_filters[APP_ANALOG_SIGNAL_ITEM_COUNT];
ATTR_PLACE_AT_FAST_RAM_BSS algo_lpf_t s_lpf_filters[APP_ANALOG_SIGNAL_ITEM_COUNT];
ATTR_PLACE_AT_FAST_RAM_BSS static float s_ma_filter_buffers[APP_ANALOG_SIGNAL_ITEM_COUNT]
                                                           [APP_ANALOG_SIGNAL_FILTER_MA_MAX_WINDOW];

static uint16_t s_raw_cache[ADC_CH_COUNT];
static bool s_raw_cache_valid[ADC_CH_COUNT];
static float s_physical_cache[ADC_CH_COUNT];
static float s_filtered_cache[ADC_CH_COUNT];
static bool s_filtered_cache_valid[ADC_CH_COUNT];

ATTR_PLACE_AT_FAST_RAM_BSS static float s_cal_gain[ADC_CH_COUNT];
ATTR_PLACE_AT_FAST_RAM_BSS static float s_cal_offset[ADC_CH_COUNT];

app_analog_signal_snapshot_t g_analog_signal_snapshot;

static bool app_analog_signal_item_is_valid(app_analog_signal_item_t item) {
    return item < APP_ANALOG_SIGNAL_ITEM_COUNT;
}

static void
    app_analog_signal_update_fast_calibration(adc_channel_t ch, const app_adc_calibration_t* cal) {
    float vref_over_range = INTF_ADC_DEFAULT_VREF_MV / 65535.0f;
    s_cal_gain[ch] = vref_over_range * cal->sense_gain * cal->physical_gain;
    s_cal_offset[ch] = cal->sense_offset_mv * cal->physical_gain + cal->physical_offset;
}

static void app_analog_signal_filter_init_one(app_analog_signal_item_t item) {
    adc_channel_t ch = s_item_to_channel[item];
    const app_analog_signal_filter_cfg_t* cfg = &s_default_filter_cfg[ch];

    algo_ma_ctor(&s_ma_filters[item]);
    algo_lpf_ctor(&s_lpf_filters[item]);

    if (cfg->type == APP_ANALOG_SIGNAL_FILTER_MA) {
        algo_ma_cfg_t ma_cfg = {
            .window_size = cfg->cfg.ma.window_size,
            .buffer = s_ma_filter_buffers[item],
        };
        (void)s_ma_filters[item].init(&s_ma_filters[item], &ma_cfg);
    } else if (cfg->type == APP_ANALOG_SIGNAL_FILTER_LPF) {
        algo_lpf_cfg_t lpf_cfg = {
            .cutoff_hz = cfg->cfg.lpf.cutoff_hz,
            .sample_rate_hz = cfg->cfg.lpf.sample_rate_hz,
        };
        (void)s_lpf_filters[item].init(&s_lpf_filters[item], &lpf_cfg);
    }

    if (item == APP_ANALOG_SIGNAL_ITEM_VCAP) {
        algo_ma_cfg_t ma_cfg = {
            .window_size = 4U,
            .buffer = s_ma_filter_buffers[item],
        };
        (void)s_ma_filters[item].init(&s_ma_filters[item], &ma_cfg);
    }
}

static float app_analog_signal_filter_step(app_analog_signal_item_t item, float value) {
    adc_channel_t ch = s_item_to_channel[item];
    const app_analog_signal_filter_cfg_t* cfg = &s_default_filter_cfg[ch];

    switch (cfg->type) {
    case APP_ANALOG_SIGNAL_FILTER_MA:
        return s_ma_filters[item]._inited ? algo_ma_step_fast(&s_ma_filters[item], value) : value;
    case APP_ANALOG_SIGNAL_FILTER_LPF:
        return s_lpf_filters[item]._inited ? algo_lpf_step_fast(&s_lpf_filters[item], value)
                                           : value;
    default: return value;
    }
}

static float*
    snapshot_field_ptr(app_analog_signal_measurements_t* m, app_analog_signal_item_t item) {
    switch (item) {
    case APP_ANALOG_SIGNAL_ITEM_VCAP: return &m->vcap_v;
    case APP_ANALOG_SIGNAL_ITEM_VOUT: return &m->vout_v;
    case APP_ANALOG_SIGNAL_ITEM_I_IN: return &m->i_in_a;
    case APP_ANALOG_SIGNAL_ITEM_I_L: return &m->i_l_a;
    default: return NULL;
    }
}

void app_analog_signal_load_default_calibration(void) {
    for (adc_channel_t ch = ADC_CH_VCAP; ch < ADC_CH_COUNT; ch++) {
        app_adc_set_calibration(ch, &s_default_calibration[ch]);
        app_analog_signal_update_fast_calibration(ch, &s_default_calibration[ch]);
    }
}

void app_analog_signal_init(void) {
    app_analog_signal_load_default_calibration();

    for (app_analog_signal_item_t item = APP_ANALOG_SIGNAL_ITEM_VCAP;
         item < APP_ANALOG_SIGNAL_ITEM_COUNT; item++) {
        app_analog_signal_filter_init_one(item);
    }
}

void app_analog_signal_update_raw(adc_channel_t ch, uint16_t raw) {
    if (ch >= ADC_CH_COUNT) {
        return;
    }

    s_raw_cache[ch] = raw;
    s_raw_cache_valid[ch] = true;

    float physical = (float)raw * s_cal_gain[ch] + s_cal_offset[ch];
    s_physical_cache[ch] = physical;

    app_analog_signal_item_t item = s_channel_to_item[ch];
    s_filtered_cache[ch] = app_analog_signal_filter_step(item, physical);
    s_filtered_cache_valid[ch] = true;

    float* raw_field = snapshot_field_ptr(&g_analog_signal_snapshot.raw, item);
    float* filtered_field = snapshot_field_ptr(&g_analog_signal_snapshot.filtered, item);
    if (raw_field != NULL) {
        *raw_field = physical;
    }
    if (filtered_field != NULL) {
        *filtered_field = s_filtered_cache[ch];
    }
}

void app_analog_signal_process(void) {
    for (adc_channel_t ch = ADC_CH_VCAP; ch < ADC_CH_COUNT; ch++) {
        uint16_t raw;
        if (app_adc_get_pmt_raw(ch, &raw) == 0) {
            app_analog_signal_update_raw(ch, raw);
        }
    }
}

void app_analog_signal_snapshot_refresh_raw(void) {
    for (adc_channel_t ch = ADC_CH_VCAP; ch < ADC_CH_COUNT; ch++) {
        uint16_t raw;
        if (app_adc_get_pmt_raw(ch, &raw) == 0) {
            float physical;
            app_analog_signal_convert_raw(ch, raw, &physical);
            float* raw_field =
                snapshot_field_ptr(&g_analog_signal_snapshot.raw, s_channel_to_item[ch]);
            if (raw_field != NULL) {
                *raw_field = physical;
            }
        }
    }
}

int app_analog_signal_get_cached_raw(adc_channel_t ch, uint16_t* raw) {
    if (ch >= ADC_CH_COUNT || raw == NULL || !s_raw_cache_valid[ch]) {
        return -1;
    }
    *raw = s_raw_cache[ch];
    return 0;
}

void app_analog_signal_convert_raw(adc_channel_t ch, uint16_t raw, float* physical) {
    if (physical == NULL) {
        return;
    }
    if (ch >= ADC_CH_COUNT) {
        *physical = 0.0f;
        return;
    }
    *physical = (float)raw * s_cal_gain[ch] + s_cal_offset[ch];
}

int app_analog_signal_get_physical(adc_channel_t ch, float* physical) {
    if (ch >= ADC_CH_COUNT || physical == NULL || !s_raw_cache_valid[ch]) {
        return -1;
    }
    *physical = s_physical_cache[ch];
    return 0;
}

int app_analog_signal_read_item(
    app_analog_signal_item_t item, app_analog_signal_value_mode_t mode, float* value) {
    if (!app_analog_signal_item_is_valid(item) || value == NULL) {
        return -1;
    }

    adc_channel_t ch = s_item_to_channel[item];
    if (mode == APP_ANALOG_SIGNAL_VALUE_FILTERED && s_filtered_cache_valid[ch]) {
        *value = s_filtered_cache[ch];
        return 0;
    }
    if (s_raw_cache_valid[ch]) {
        *value = s_physical_cache[ch];
        return 0;
    }
    return app_adc_read_physical(ch, value);
}

int app_analog_signal_get_measurements(
    app_analog_signal_measurements_t* measurements, app_analog_signal_value_mode_t mode) {
    if (measurements == NULL) {
        return -1;
    }
    *measurements = (app_analog_signal_measurements_t){0};

    if (app_analog_signal_read_item(APP_ANALOG_SIGNAL_ITEM_VCAP, mode, &measurements->vcap_v)
        != 0) {
        return -1;
    }
    if (app_analog_signal_read_item(APP_ANALOG_SIGNAL_ITEM_VOUT, mode, &measurements->vout_v)
        != 0) {
        return -1;
    }
    if (app_analog_signal_read_item(APP_ANALOG_SIGNAL_ITEM_I_IN, mode, &measurements->i_in_a)
        != 0) {
        return -1;
    }
    if (app_analog_signal_read_item(APP_ANALOG_SIGNAL_ITEM_I_L, mode, &measurements->i_l_a) != 0) {
        return -1;
    }
    return 0;
}

int app_analog_signal_set_channel_calibration(
    adc_channel_t ch, const app_adc_calibration_t* calibration) {
    if (ch >= ADC_CH_COUNT || calibration == NULL) {
        return -1;
    }
    app_adc_set_calibration(ch, calibration);
    app_analog_signal_update_fast_calibration(ch, calibration);
    return 0;
}

int app_analog_signal_get_channel_calibration(
    adc_channel_t ch, app_adc_calibration_t* calibration) {
    if (ch >= ADC_CH_COUNT || calibration == NULL) {
        return -1;
    }
    return app_adc_get_calibration(ch, calibration);
}
