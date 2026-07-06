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
    app_hrpwm_init();         /* PWM configured, NOT started (quiet for ADC cal) */
    app_adc_init();           /* ADC calibration + PMT setup in quiet environment */
    app_analog_signal_init(); /* Load calibration params for raw → physical conversion */
    app_control_init();       /* Register ISR callbacks + pre-stage duty=0 BEFORE PWM starts */
    app_hrpwm_start_all();    /* PWM starts — ISR already armed, duty begins at 0 */
}

void app_run_once(void) {
    app_control_tick();
    app_comm_tick();
}
