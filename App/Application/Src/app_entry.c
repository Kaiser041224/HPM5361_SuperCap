#include "app_entry.h"

#include "app_adc.h"
#include "app_comm.h"
#include "app_control.h"

#include "app_analog_signal.h"
#include "app_buzzer.h"
#include "app_can.h"
#include "app_gpio.h"
#include "app_hrpwm.h"

#include "board.h"
#include "intf_clock.h"

/* PMT 预热等待时间：计数器启动后需跳过驱动内部丢弃的前 8 帧，
 * 并让 VCAP/VOUT 的模拟前级 LPF 稳定，1ms 足够覆盖两者余量 */
#define APP_INIT_PMT_WARMUP_MS 1U

void app_init(void) {
    board_init();
    intf_clock_init();

    app_gpio_init();
    app_gpio_set(PIN_DRVPWR, true); /* Ensure DRVPWR is low before PWM starts */

    app_buzzer_init();
    //
    app_can_register_driver();
    app_can_init();
    app_comm_init();
    //
    app_hrpwm_init();               /* PWM configured, NOT started */
    app_adc_init();                 /* ADC calibration + PMT setup, trigger source still idle */
    app_analog_signal_init();       /* Load calibration params for raw → physical conversion */
    app_hrpwm_start_counter_only(); /* Counter runs, PMT starts sampling; A/B pins stay LOW */
    intf_clock_delay_ms(APP_INIT_PMT_WARMUP_MS); /* Let PMT cache VOUT/VCAP settle */
    app_control_init();             /* Register ISR + soft-start using PMT-sampled VOUT/VCAP */
    app_hrpwm_start_all();          /* Enable A/B output — duty begins at soft-start value */
}

void app_run_once(void) {
    app_control_tick();
    app_comm_tick();
}
