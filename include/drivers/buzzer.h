#include <hardware/pwm.h>
#include <hardware/gpio.h>

#pragma once

void buzzer_tone(uint pin, uint freq)
{
	// get slice number
	uint slicenum = pwm_gpio_to_slice_num(pin);
	
	// assign GPIO to pwm functionality
	gpio_set_function(pin, GPIO_FUNC_PWM);

	// gpio_set_drive_strength(pin, GPIO_DRIVE_STRENGTH_12MA);

	// set the clkdiv mode (this might not actually do anything)
	pwm_set_clkdiv_mode(slicenum, PWM_DIV_FREE_RUNNING);

	// disable phase correct (if enabled, frequency is halved and duty cycle is doubled)
	pwm_set_phase_correct(slicenum, false);

	// scale PWM clock divider by the cpu clock
	pwm_set_clkdiv_int_frac(slicenum, clock_get_hz(clk_sys)/1000000, 0);

	// set wrap level
	pwm_set_wrap(slicenum, (uint)(1000000/freq));

	// enable PWM slice
	pwm_set_enabled(slicenum, true);

	// enable PWM pin
	pwm_set_gpio_level(pin, (uint)((1000000/freq)/2));

};

void buzzer_disable(uint pin) {
	uint slicenum = pwm_gpio_to_slice_num(pin);
	pwm_set_enabled(slicenum, false);
	gpio_init(pin);
	gpio_set_dir(pin, true);
	gpio_put(pin, false);
}