#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "gps.h"
#include "core.h"
#include "task.h"
#include "control.h"
#include "telemetry.h"
#include "state_ctrl.h"
#include "simulation.h"
#include "navigation.h"
#include "datalogging.h"

#include "drivers/radio.h"
#include "drivers/perif_ctrl.h"
#include "drivers/core_interface.h"

task_t task_gps = { (callback_t)gps::update, 10000, 0, 0, 0 };
task_t task_nav = { (callback_t)nav::update, 10000, 0, 0, 0 };
task_t task_perif = { (callback_t)perif::update, 10000, 0, 0, 0 };
task_t task_state = { (callback_t)state::update, 10000, 0, 0, 0 };
task_t task_radio = { (callback_t)radio::update, 10000, 0, 0, 0 };
task_t task_datalog = { (callback_t)datalog::update, 10000, 0, 0, 0 };
task_t task_control = { (callback_t)control::update, 10000, 0, 0, 0 };
task_t task_telemetry = { (callback_t)telemetry::update, 10000, 0, 0, 0 };

int main(void)
{ 
	stdio_init_all();

	datalog::ptrs.points = {

		.state = &vehicle_state,
		.time = &timing::RUNTIME,
		.flag_gpio = &flags::perif_flags::gpio_sts,
		.flag_pyro = &flags::perif_flags::pyro_sts,
		.flag_state = &flags::state_flags::sts_bitmap,
		.flag_control = &flags::control_flags::control_bitmap,
		.flag_nav = &flags::nav_flags::nav_bitmap,
		.active_state_timer = &state::current_state_timer,
		.flash_errors = &datalog::flash_errors,
		
		.voltage_batt = &perif::voltage_batt_raw,
		.voltage_pyro = &perif::voltage_pyro_raw,
		.core_usage_percent = &timing::core_usage_percent,
				
		.position = &nav::position,
		.velocity = &nav::velocity,
		.rotation = &nav::rotation,
		.accel_bias = &nav::acceleration_b,
		
		.acceleration = &nav::raw::accel,
		.ori_rate = &nav::raw::gyro,

		.baro_alt = &nav::altitude,
		.baro_pressure = &nav::pressure,
		.baro_temp = &nav::temperature,

		.mag = &nav::raw::mag,

		.lat = &nav::lat,
		.lon = &nav::lon,
		.gps_pos_y = &nav::position_gps.y,
		.gps_pos_z = &nav::position_gps.z,
		.h_acc = &nav::gps_horizontal_accuracy,
		.n_sats = &nav::gps_n_sats,

		.thrust = &control::thrust,

		.target_vector = &control::target_vector,
		.ang_acc_output = &control::ang_acc_out,
		.ang_acc_error = &control::ang_acc_error,
		.angle_out = &control::angle_out,
		
		.burn_alt                = &control::gfield::burn_alt,
		.comp                    = &control::gfield::comp,
		.desired_accel			 = &control::gfield::desired_accel,
		.throttle_ratio			 = &control::gfield::throttle_ratio,
		.divert_angle			 = &control::gfield::divert_angle,

		.simulation_energy_est   = &control::simulation_energy_est,
		.simulation_work_est     = &control::simulation_work_est,
		.simulation_position_est = &control::simulation_position_est,
		.simulation_velocity_est = &control::simulation_velocity_est,
		.simulation_time_est     = &control::simulation_time_est,
		.simulation_time_taken   = &control::simulation_time_taken

	};
	
	sleep_ms(2000);

	// while(1) {
	// 	printf("hello\n");
	// 	sleep_ms(1000);
	// }

	core1_interface::core1_landing_sim_func = (core1_interface::callback_t)simulation::run_landing_sim;
	core1_interface::core1_ascent_sim_func = (core1_interface::callback_t)simulation::run_ascent_sim;
	core1_interface::core1_divert_sim_func = (core1_interface::callback_t)simulation::run_divert_sim;
	core1_interface::init();

	perif::init();
	print_compile_config();
	nav::init();
	datalog::init();
	telemetry::init();
	radio::init();
	#ifdef USE_GPS
		gps::init();
	#endif

	// while(1) {
	// 	printf("hello\n");
	// 	sleep_ms(1000);
	// }


	// spi_set_baudrate(spi0, 1000000);

	// flash_erase_chip(pin_cs_flash);

	// while(flash_busy(pin_cs_flash)) {;}

	// uint8_t buf[256] = {0};
	// for ( int i = 0; i < 256; i++ ) { buf[i] = 0; }
	
	// uint8_t buf_rx[256] = {0};
	// // spi_set_baudrate(spi0, 10000000);
	// for ( int i = 0; i < 100; i++ ) {
	// 	buf[5] = i&0xff;
	// 	buf[6] = (i>>8)&0xff;
	// 	buf[7] = (i>>16)&0xff;
	// 	buf[8] = (i>>24)&0xff;
		
	// 	flash_write_page(pin_cs_flash, i, buf);

	// 	if ( i % 100 == 0 ) { printf("%i\n", i); }
	// }

	// for ( int i = 0; i < 100; i++ ) {
	// 	buf[5] = i&0xff;
	// 	buf[6] = (i>>8)&0xff;
	// 	buf[7] = (i>>16)&0xff;
	// 	buf[8] = (i>>24)&0xff;
	// 	flash_read_page(pin_cs_flash, i, buf_rx);

	// 	bool valid = true;
	// 	for ( int j = 5; j < 256; j++ ) { valid &= buf[j] == buf_rx[j]; }
	// 	if ( !valid ) { 
	// 		printf("invalid at page %i\n", i);
	// 		printf("expected:\n");
	// 		for ( int j = 0; j < 256; j++ ) { printf("%2x ", buf[j]); if ( j % 8 == 0 ) { printf("\n"); } }
	// 		printf("actual:\n");
	// 		for ( int j = 0; j < 256; j++ ) { printf("%2x ", buf_rx[j]); if ( j % 8 == 0 ) { printf("\n"); } }
	// 	}
	// 	if ( i % 100 == 0 ) { printf("%i\n", i); }
	// }

	// while(1) {;}

	vehicle_state = state_idle;

	while(1) {

		uint64_t t_start = time_us_64();

		timing::update();
		flags::update();

		#ifdef USE_GPS
		update_task(task_gps);			
		#endif
		update_task(task_nav);
		update_task(task_state);
		update_task(task_control);
		update_task(task_datalog);
		update_task(task_telemetry);
		update_task(task_perif);
		update_task(task_radio);

		timing::average_runtime = (time_us_64()-t_start);

		// wait for next cycle
		int t_sleep = 10000-(timing::average_runtime);
		if ( t_sleep > 0 ) { sleep_us(t_sleep); }

	}

}