#ifndef APP_DEBUG_HRPWM_H
#define APP_DEBUG_HRPWM_H

#include <stdint.h>

/**
 * @brief 打印 HRPWM compare 寄存器快照。
 *
 * 用于观察当前 PWM reload / compare 配置是否符合预期，
 * 可辅助分析对齐模式、移相结果和占空比配置。
 */
void app_debug_dump_hrpwm_cmp(void);

void app_debug_dump_hrpwm_freq(void);

/**
 * @brief PWM 中断用户回调类型。
 *
 * 注册后会在对应 PWM reload IRQ 中被调用，适合插入轻量级
 * 调试观测逻辑。
 */
typedef void (*pwm_irq_user_callback_t)(void);

/**
 * @brief 使能指定 HRPWM 实例的 reload IRQ 调试。
 *
 * @param inst HRPWM 实例号，当前支持 0 或 1。
 */
void app_debug_pwm_irq_enable(uint8_t inst);

/**
 * @brief 关闭指定 HRPWM 实例的 reload IRQ 调试。
 *
 * @param inst HRPWM 实例号，当前支持 0 或 1。
 */
void app_debug_pwm_irq_disable(uint8_t inst);

/**
 * @brief 获取指定 HRPWM 实例的 IRQ 计数值。
 *
 * @param inst HRPWM 实例号，当前支持 0 或 1。
 * @return IRQ 累计触发次数；若实例非法则返回 0。
 */
uint32_t app_debug_pwm_irq_get_count(uint8_t inst);

/**
 * @brief 清零指定 HRPWM 实例的 IRQ 计数值。
 *
 * @param inst HRPWM 实例号，当前支持 0 或 1。
 */
void app_debug_pwm_irq_reset_count(uint8_t inst);

/**
 * @brief 打印全部 HRPWM IRQ 调试状态。
 *
 * 输出每个实例的 IRQ 使能状态和累计计数。
 */
void app_debug_pwm_irq_dump_status(void);

/**
 * @brief 注册指定 HRPWM 实例的 IRQ 用户回调。
 *
 * @param inst HRPWM 实例号，当前支持 0 或 1。
 * @param callback 用户回调函数；传入非空指针后会在 IRQ 中执行。
 * @return 0 表示成功，-1 表示参数非法。
 *
 * @warning 回调运行在中断上下文中，必须保持轻量、非阻塞。
 */
int app_debug_pwm_irq_register_callback(uint8_t inst, pwm_irq_user_callback_t callback);

/**
 * @brief 注销指定 HRPWM 实例的 IRQ 用户回调。
 *
 * @param inst HRPWM 实例号，当前支持 0 或 1。
 */
void app_debug_pwm_irq_unregister_callback(uint8_t inst);

/**
 * @brief 执行 HRPWM 变频扫描测试。
 *
 * @param inst HRPWM 实例号，当前支持 0 或 1。
 * @param freq_start 起始频率，单位 Hz。
 * @param freq_end 结束频率，单位 Hz。
 * @param freq_step 步进频率，单位 Hz。
 * @param delay_ms 每步之间的等待时间，单位 ms。
 */
void app_debug_pwm_test_frequency_sweep(uint8_t inst, uint32_t freq_start, uint32_t freq_end,
                                        uint32_t freq_step, uint32_t delay_ms);

/**
 * @brief 执行 HRPWM 占空比分辨率测试。
 *
 * @param inst HRPWM 实例号，当前支持 0 或 1。
 * @param pair PWM pair 编号。
 * @param duty_start 起始占空比，归一化范围通常为 [0.0, 1.0]。
 * @param duty_end 结束占空比，归一化范围通常为 [0.0, 1.0]。
 * @param duty_step 每步占空比增量。
 * @param delay_ms 每步之间的等待时间，单位 ms。
 */
void app_debug_pwm_test_duty_resolution(uint8_t inst, uint8_t pair, float duty_start,
                                        float duty_end, float duty_step, uint32_t delay_ms);

/**
 * @brief 执行 HRPWM 综合验证测试。
 *
 * 该入口会按预设顺序执行寄存器快照、IRQ 使能、变频扫描和占空比分辨率测试。
 */
void app_debug_hrpwm_run_tests(void);

#endif /* APP_DEBUG_HRPWM_H */
