#pragma once

#include <stdio.h>
#include <string.h>
#include <pico/stdlib.h>
#include <hardware/timer.h>

#include "physics.h"
#include "mmath.h"
#include "drivers/pins.h"

// ============================================================================
// general settings

// enable pyros
#define PYRO_EN

// enable datalogging
#define DATALOG_EN

// enable telemetry
#define TELEMETRY_EN

// use radio instead of usb serial
// #define USE_RADIO

// log data to internal flash
// #define USE_INTERNAL_FLASH

// enable gps integration to the kalman filter
// #define USE_GPS

// simulate flight (SITL)
// #define SITL

// enable / disable buzzer
#define USE_BUZZER

// ============================================================================
// thresholds for state switches

// launch detect

// acceleration that the rocket must feel to increase the acceleration counter when in launch detect mode
const static float launch_detect_accel_threshold = 12.f;

// number of acceleration readings over the acceleration threshold required to trigger launch detection
const static int launch_detect_accel_count = 10;

// burnout detect

// acceleration that the rocket must be below to increase the acceleration counter when in burnout detect mode
const static float burnout_detect_accel_threshold = 2.f;

// number of acceleration readings under the acceleration threshold required to trigger burnout detection
const static int burnout_detect_accel_count = 15;

// apogee detect

// velocity that the rocket must be below to detect apogee
const static float apogee_detect_vel_threshold = 0.0f;

// landing burn detect
const static float landing_burn_detect_accel_threshold = 6.f;

// deviation from 1g that rocket can feel to be landed
const static float landing_detect_accel_threshold = 0.8f;

// deviation from 0 rad/s that rocket can feel to be landed
const static float landing_detect_ori_threshold = 0.0698132f;

// ============================================================================
// NAV settings

// number of samples to take to determine gyroscope bias
const static uint16_t gyro_bias_count = 2000;

// number of samples to take to determine altitude offset
const static uint16_t baro_bias_count = 200;

// ============================================================================
// pyro settings

uint32_t pyro_1_fire_dur_us = 500000;
uint32_t pyro_2_fire_dur_us = 0;
uint32_t pyro_3_fire_dur_us = 0;

const static bool pyro_1_en = true;
const static bool pyro_2_en = false;
const static bool pyro_3_en = false;

// ============================================================================
// internal communication bus settings

#define spi_default_baud 2000000
#define spi_flash_baud 4000000

// ============================================================================
// global flags and variables

#define PI 3.14159265359

namespace flags {
	
	namespace state_flags {
	
		volatile bool accel_over_ld_threshold = false;

		volatile bool accel_under_burnout_threshold = false;
		volatile bool time_over_burnout_threshold = false;

		volatile bool velocity_over_apogee_threshold = false;

		volatile bool accel_over_landing_threshold = false;

		volatile bool accel_within_landed_threshold = false;
		volatile bool gyro_within_landed_threshold = false;

		volatile bool baro_below_alt_threshold = false;
		volatile bool velocity_below_landed_threshold = false;

		uint8_t sts_bitmap = 0;

		// b0: accel_over_ld_threshold
		// b1: accel_under_burnout_threshold
		// b2: velocity_over_apogee_threshold
		// b3: accel_over_landing_threshold
		// b4: accel_within_landed_threshold
		// b5: gyro_within_landed_threshold
		// b6: baro_below_alt_threshold
		// b7: velocity_below_landed_threshold

	}

	namespace boot {

		volatile bool boot_fail = false;

	}

	namespace nav_flags {
		
		volatile bool gyro_debiased = false;
		volatile bool baro_debiased = false;

		volatile bool gps_lock = false;
		volatile bool gps_initial_position_lock = false;

		volatile bool kalman_x_converged = false;
		volatile bool kalman_y_converged = false;
		volatile bool kalman_z_converged = false;

		volatile bool orientation_converged = false;
		
		volatile bool gps_drdy = false;

		uint8_t nav_bitmap;

		// b0: orientation_converged
		// b1: kalman_z_converged
		// b2: kalman_y_converged
		// b3: kalman_x_converged
		// b4: gps_initial_position_lock
		// b5: gps_lock
		// b6: baro_debiased
		// b7: gyro_debiased

	}

	namespace control_flags {

		volatile bool start_landing_burn = false;
		volatile bool burn_alt_over_safe_thresh = false;

		volatile bool new_ascent_sim_result = false;
		volatile bool new_landing_sim_result = false;
		volatile bool new_divert_sim_result = false;

		volatile bool core1_communication_failure = false;

		// b0: start_landing_burn
		// b1: burn_alt_over_safe_thresh
		// b2: core1_communication_failure

		uint8_t control_bitmap = 0;

	}

	namespace perif_flags {

		uint8_t gpio_sts = 0;
		// switch_sts

		bool pyro_1_fire = false;
		bool pyro_2_fire = false;
		bool pyro_3_fire = false;
		bool pyro_1_cont = false;
		bool pyro_2_cont = false;
		bool pyro_3_cont = false;
		
		// b1: pyro_has_power
		// b2: pyro_1_fire
		// b3: pyro_2_fire
		// b4: pyro_3_fire
		// b5: pyro_1_cont
		// b6: pyro_2_cont
		// b7: pyro_3_cont
		
		uint8_t pyro_sts = 0;

		volatile bool running_from_lipo = false;
		volatile bool pyro_has_power = false;
		volatile bool switch_sts = false;

	}

	void update() {
		
		nav_flags::nav_bitmap = (nav_flags::orientation_converged) |
								(nav_flags::kalman_z_converged<<1) |
								(nav_flags::kalman_y_converged<<2) |
								(nav_flags::kalman_x_converged<<3) |
								(nav_flags::gps_initial_position_lock<<4) |
								(nav_flags::gps_lock<<5) |
								(nav_flags::baro_debiased<<6) |
								(nav_flags::gyro_debiased<<7);

		

		control_flags::control_bitmap = (control_flags::start_landing_burn) |
										(control_flags::burn_alt_over_safe_thresh<<1) |
										(control_flags::core1_communication_failure<<2);

		perif_flags::pyro_sts = (perif_flags::pyro_has_power<<1) |
								(perif_flags::pyro_1_fire<<2) |
								(perif_flags::pyro_2_fire<<3) |
								(perif_flags::pyro_3_fire<<4) |
								(perif_flags::pyro_1_cont<<5) |
								(perif_flags::pyro_2_cont<<6) |
								(perif_flags::pyro_3_cont<<7);

		perif_flags::gpio_sts = perif_flags::switch_sts;

	}

}

namespace timing { 
	
	uint64_t RUNTIME = 0;
	uint64_t MET = 0;
	uint64_t T_launch = 0;
	uint64_t T_landing = 0;
	uint64_t T_landing_burn_start = 0;
	uint64_t T_apogee = 0;

	uint64_t average_runtime = 0;

	uint8_t core_usage_percent = 0;

	void set_MET(uint64_t t) { MET = t; }
	uint64_t get_MET() { return MET; }

	void set_t_launch(uint64_t t) { T_launch = t; }
	uint64_t get_t_launch() { return T_launch; }

	void set_t_landing(uint64_t t) { T_landing = t; }
	uint64_t get_t_landing() { return T_landing; }

	void set_t_landing_burn_start(uint64_t t) { T_landing_burn_start = t; }
	uint64_t get_t_landing_burn_start() { return T_landing_burn_start; }

	void set_t_apogee(uint64_t t) { T_apogee = t; }
	uint64_t get_t_apogee() { return T_apogee; }

	void update() {

		RUNTIME = time_us_64();

		if ( get_t_launch() > 0 ) { set_MET(time_us_64()-get_t_launch()); }

		core_usage_percent = (uint8_t)(average_runtime/100);

	}

}

// ============================================================================
// vehicle state declaration and getters/setters

enum system_state_t : uint8_t {
	// initialization of all systems
	state_boot,

	// 1st idle state
	state_idle,

	// filter convergence, sensor debiasing, gps initialization
	state_nav_init,

	// 2nd idle state
	state_launch_idle,

	// waiting for launch
	state_launch_detect,

	// ascent under thrust and guidance
	state_powered_ascent,

	// coasting to apogee
	state_ascent_coast,

	// costing to landing burn
	state_descent_coast,

	// time between motor ignition and peak thrust
	state_landing_start,

	// powered descent up until 0.25s before burnout
	state_landing_guidance,

	// final 0.25s of landing burn, nulling angular rates and preparing to land
	state_landing_terminal,

	// on the ground post-flight
	state_landed,

	// flight has been aborted
	state_abort
};

system_state_t vehicle_state = state_boot;
system_state_t get_vehicle_state() { return vehicle_state; }
void set_vehicle_state(system_state_t state) { vehicle_state = state; } 

bool vehicle_is_in_flight() {
	switch(vehicle_state) {
		case(state_boot): { return 0; }
		case(state_idle): { return 0; }
		case(state_nav_init): { return 0; }
		case(state_launch_idle): { return 0; }
		case(state_launch_detect): { return 0; }
		case(state_powered_ascent): { return 1; }
		case(state_ascent_coast): { return 1; }
		case(state_descent_coast): { return 1; }
		case(state_landing_start): { return 1; }
		case(state_landing_guidance): { return 1; }
		case(state_landing_terminal): { return 1; }
		case(state_landed): { return 0; }
		case(state_abort): { return 0; }
		default: { return 0; }
	}
}

bool vehicle_has_launched() {
	switch(vehicle_state) {
		case(state_boot): { return 0; }
		case(state_idle): { return 0; }
		case(state_nav_init): { return 0; }
		case(state_launch_idle): { return 0; }
		case(state_launch_detect): { return 0; }
		case(state_powered_ascent): { return 1; }
		case(state_ascent_coast): { return 1; }
		case(state_descent_coast): { return 1; }
		case(state_landing_start): { return 1; }
		case(state_landing_guidance): { return 1; }
		case(state_landing_terminal): { return 1; }
		case(state_landed): { return 1; }
		case(state_abort): { return 1; }
		default: { return 0; }
	}
}

bool vehicle_has_control() {
	switch(vehicle_state) {
		case(state_boot): { return 0; }
		case(state_idle): { return 0; }
		case(state_nav_init): { return 0; }
		case(state_launch_idle): { return 0; }
		case(state_launch_detect): { return 0; }
		case(state_powered_ascent): { return 1; }
		case(state_ascent_coast): { return 0; }
		case(state_descent_coast): { return 0; }
		case(state_landing_start): { return 1; }
		case(state_landing_guidance): { return 1; }
		case(state_landing_terminal): { return 1; }
		case(state_landed): { return 0; }
		case(state_abort): { return 0; }
		default: { return 0; }
	}
}

// ============================================================================
// peripheral bus flags and config

struct _port_t {

	// pins used (scl/sda, tx/rx, etc.)
	uint pin0;
	uint pin1;

	// protocol (and protocol instance) being used by the port ( i2c0, i2c1, uart0, uart1, etc. )
	enum port_protocol {
		i2c_0,
		i2c_1,
		uart_0,
		uart_1,
		gpio
	} protocol;

	// gpio function to be used when initializing the port
	gpio_function gpio_func;
	
	// flag indicate if the communication protocol is being bit-banged or controlled by a peripheral
	bool bitbang;

};

_port_t qwiic_port0 = { 14,                             // pin0
						15,                             // pin1
						_port_t::port_protocol::gpio,   // protocol
						GPIO_FUNC_SIO,                  // gpio func
						true};                          // bitbang flag

_port_t qwiic_port1 = { 13,                             // pin0
						12,                             // pin1
						_port_t::port_protocol::i2c_0,  // protocol
						GPIO_FUNC_I2C,                  // gpio func
						false};                         // bitbang flag

_port_t qwiic_port2 = { 9,                              // pin0
						8,                              // pin1
						_port_t::port_protocol::gpio, // protocol
						GPIO_FUNC_SIO,                 // gpio func
						true};                         // bitbang flag

_port_t qwiic_port3 = { 1,                              // pin0
						0,                              // pin1
						_port_t::port_protocol::uart_0,   // protocol
						GPIO_FUNC_UART,                  // gpio func
						false};                          // bitbang flag

// ============================================================================
// functions that dont really belong anywhere else

template <class T>
auto clamp(const T& x, const T& min, const T& max) { return x < min ? min : (x > max ? max : x); }

void print_compile_config() {

	printf("============================================================================\n\n");
	printf("COMPILE CONFIGS\n\n");
	
	#ifdef PYRO_EN
		printf("PYRO_EN is ENABLED\n");
	#else
		printf("PYRO_EN is DISABLED\n");
	#endif

	#ifdef DATALOG_EN
		printf("DATALOG_EN is ENABLED\n");
	#else
		printf("DATALOG_EN is DISABLED\n");
	#endif
	
	#ifdef TELEMETRY_EN
		printf("TELEMETRY_EN is ENABLED\n");
	#else
		printf("TELEMETRY_EN is DISABLED\n");
	#endif

	#ifdef SITL
		printf("SITL is ENABLED\n");
	#else
		printf("SITL is DISABLED\n");
	#endif

	#ifdef NAV_SLOW_UPDATE
		printf("NAV_UPDATE is set to SLOW (100Hz)\n");
	#else
		printf("NAV_UPDATE is set to FAST (400Hz)\n");
	#endif

	#define vname(x) #x
	printf("\n");

	printf("%s is set to %f\n", vname(launch_detect_accel_threshold), launch_detect_accel_threshold);
	printf("%s is set to %i\n", vname(launch_detect_accel_count), launch_detect_accel_count);
	printf("\n");

	printf("%s is set to %f\n", vname(burnout_detect_accel_threshold), burnout_detect_accel_threshold);
	printf("%s is set to %i\n", vname(burnout_detect_accel_count), burnout_detect_accel_count);
	printf("\n");

	printf("%s is set to %f\n", vname(apogee_detect_vel_threshold), apogee_detect_vel_threshold);
	printf("\n");

	printf("============================================================================\n");
	
}

#define boot_warning(message) printf("[WARNING] from function %s:\n%s\n", __FUNCTION__, message)
#define boot_panic(message) printf("[ERROR] from function %s:\n%s\n", __FUNCTION__, message); flags::boot::boot_fail = true