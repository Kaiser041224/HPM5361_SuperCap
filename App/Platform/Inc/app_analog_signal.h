/*
 * Analog Signal Conditioning Module
 *
 * Converts ADC readings into physical measurements for controllers,
 * protection logic, and communication services.
 */

#ifndef APP_ANALOG_SIGNAL_H
#define APP_ANALOG_SIGNAL_H

#include "app_adc.h"
#include "algo_filter.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    APP_ANALOG_SIGNAL_ITEM_VCAP = 0, /* SuperCap VCAP, control-model VLINK */
    APP_ANALOG_SIGNAL_ITEM_VOUT,     /* Three-port VOUT, control-model VIN */
    APP_ANALOG_SIGNAL_ITEM_I_IN,
    APP_ANALOG_SIGNAL_ITEM_I_L,
    APP_ANALOG_SIGNAL_ITEM_COUNT,
} app_analog_signal_item_t;

#define APP_ANALOG_SIGNAL_ITEM_VLINK APP_ANALOG_SIGNAL_ITEM_VCAP
#define APP_ANALOG_SIGNAL_ITEM_VIN   APP_ANALOG_SIGNAL_ITEM_VOUT

/* 内部符号声明，仅供 inline 函数访问，不作为公共 API */
extern algo_lpf_t s_lpf_filters[];
extern algo_ma_t s_ma_filters[];
extern const app_analog_signal_item_t s_channel_to_item[];

/**
 * @brief 读取结果模式。
 *
 * - `APP_ANALOG_SIGNAL_VALUE_RAW`：返回未经滤波的真实物理量。
 * - `APP_ANALOG_SIGNAL_VALUE_FILTERED`：返回经过当前配置滤波器处理后的物理量。
 */
typedef enum {
    APP_ANALOG_SIGNAL_VALUE_RAW = 0,
    APP_ANALOG_SIGNAL_VALUE_FILTERED = 1,
} app_analog_signal_value_mode_t;

/**
 * @brief 全量模拟量读取结果。
 *
 * 所有字段均为已经完成 ADC 换算后的真实物理量；
 * 是否经过滤波，由 `app_analog_signal_get_measurements()` 调用时传入的
 * `app_analog_signal_value_mode_t` 决定。
 */
typedef struct {
    float vcap_v;
    float vout_v;
    float i_in_a;
    float i_l_a;
} app_analog_signal_measurements_t;

typedef struct {
    app_analog_signal_measurements_t raw;
    app_analog_signal_measurements_t filtered;
} app_analog_signal_snapshot_t;

extern app_analog_signal_snapshot_t g_analog_signal_snapshot;

/**
 * @brief 初始化模拟量调理模块。
 *
 * 完成 ADC 初始化、校准、默认标定参数装载，以及每路滤波器初始化。
 */
void app_analog_signal_init(void);

/**
 * @brief 装载默认 ADC 标定参数。
 */
void app_analog_signal_load_default_calibration(void);

/**
 * @brief 更新指定通道的最新原始 ADC 结果。
 *
 * 用于将 PWM/TRGM/PMT 等外部采样链得到的原始值推送进模拟量模块，
 * 供后续物理量换算和滤波读取使用。
 *
 * @param ch  目标 ADC 逻辑通道。
 * @param raw 原始 16-bit ADC 结果。
 */
void app_analog_signal_update_raw(adc_channel_t ch, uint16_t raw);

/**
 * @brief 从 PMT 缓存拉取全部通道原始值并完成物理量换算 + 滤波。
 *
 * 应在主循环中周期调用，禁止在 ISR 中调用。
 */
void app_analog_signal_process(void);

/**
 * @brief 从 PMT 缓存直接刷新快照中的 raw 值（不做滤波）。
 *
 * 轻量级，可在调试 dump 中高频调用。
 */
void app_analog_signal_snapshot_refresh_raw(void);

int app_analog_signal_get_cached_raw(adc_channel_t ch, uint16_t *raw);

/**
 * @brief 轻量级 raw → 物理量转换（ISR 专用，无滤波/快照更新）。
 *
 * 在 init 时预计算合并校准系数，调用时仅做 1 次乘法 + 1 次加法。
 *
 * @param ch        ADC 通道。
 * @param raw       原始 ADC 值。
 * @param physical  输出物理量。
 */
void app_analog_signal_convert_raw(adc_channel_t ch, uint16_t raw, float *physical);

/**
 * @brief 单步 LPF 滤波（ISR 安全，内联零开销）。
 *
 * 对指定通道的 LPF 实例执行一步一阶低通滤波。
 * 滤波参数由 app_analog_signal 统一配置。
 * 非法通道直接返回原值，不会触发异常。
 */
ALGO_RAMFUNC
static inline float app_analog_signal_lpf_step_fast(adc_channel_t ch, float value)
{
    if (ch >= ADC_CH_COUNT) return value;
    algo_lpf_t *lpf = &s_lpf_filters[s_channel_to_item[ch]];
    return lpf->_inited ? algo_lpf_step_fast(lpf, value) : value;
}

/**
 * @brief 单步 MA 滤波（ISR 安全，内联零开销）。
 *
 * 对指定通道的 MA 实例执行一步滑动平均滤波。
 * 滤波参数由 app_analog_signal 统一配置。
 * 非法通道或未初始化通道直接返回原值，不会触发异常。
 */
ALGO_RAMFUNC
static inline float app_analog_signal_ma_step(adc_channel_t ch, float value)
{
    if (ch >= ADC_CH_COUNT) return value;
    algo_ma_t *ma = &s_ma_filters[s_channel_to_item[ch]];
    return ma->_inited ? algo_ma_step_fast(ma, value) : value;
}

/**
 * @brief 读取缓存中的物理量（ISR 专用，无函数调用开销）。
 *
 * @param ch        ADC 通道。
 * @param physical  输出物理量。
 * @return 0 成功，-1 缓存无效。
 */
int app_analog_signal_get_physical(adc_channel_t ch, float *physical);

/**
 * @brief 读取单个目标模拟量。
 *
 * @param item  目标信号项。
 * @param mode  读取模式：原始值或滤波值。
 * @param value 输出物理量。
 * @return 0 成功，-1 失败。
 */
int app_analog_signal_read_item(app_analog_signal_item_t item, app_analog_signal_value_mode_t mode, float *value);

/**
 * @brief 一次读取全部目标模拟量。
 *
 * @param measurements 输出测量结构体。
 * @param mode         读取模式：原始值或滤波值。
 * @return 0 成功，-1 失败。
 */
int app_analog_signal_get_measurements(app_analog_signal_measurements_t *measurements,
                                       app_analog_signal_value_mode_t mode);

/**
 * @brief 设置指定 ADC 通道的标定参数。
 *
 * @param ch          ADC 通道。
 * @param calibration 标定参数。
 * @return 0 成功，-1 失败。
 */
int app_analog_signal_set_channel_calibration(adc_channel_t ch, const app_adc_calibration_t *calibration);

/**
 * @brief 获取指定 ADC 通道的当前标定参数。
 *
 * @param ch          ADC 通道。
 * @param calibration 输出标定参数。
 * @return 0 成功，-1 失败。
 */
int app_analog_signal_get_channel_calibration(adc_channel_t ch, app_adc_calibration_t *calibration);

#ifdef __cplusplus
}
#endif

#endif /* APP_ANALOG_SIGNAL_H */
