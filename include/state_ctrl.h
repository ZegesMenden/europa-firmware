#include "core.h"
#pragma once

namespace state {

	bool switch_has_been_low = false;

	// timers
	uint8_t accel_ld_timer = 0;
	uint8_t accel_burnout_timer = 0;
	uint8_t accel_landing_burn_timer = 0;
	uint8_t accel_landing_timer = 0;
	uint8_t velocity_apogee_timer = 0;
	uint8_t landing_detect_timer = 0;

	// active timer being used for state switch ( for datalogging? )
	uint8_t current_state_timer = 0;

	void update() {

		system_state_t state = get_vehicle_state();
		current_state_timer = 0;

		switch(state) {
			case (state_boot) : {
				set_vehicle_state(state_idle);
				break;
			}

			case (state_idle) : {

				// if ( !flags::perif_flags::switch_sts ) { switch_has_been_low = true; }

				// if ( flags::perif_flags::switch_sts && switch_has_been_low ) {
				// 	switch_has_been_low = false;
				// 	set_vehicle_state(state_nav_init);
				// }

				break;
			}

			case (state_nav_init) : {

				#ifdef USE_GPS
				if ( flags::nav_flags::baro_debiased && flags::nav_flags::gyro_debiased && flags::nav_flags::gps_initial_position_lock ) {
				#else
				if ( flags::nav_flags::baro_debiased && flags::nav_flags::gyro_debiased ) {
				#endif
					set_vehicle_state(state_launch_idle);
				}

				break;
			}

			case (state_launch_idle) : {

				if ( !flags::perif_flags::switch_sts ) { switch_has_been_low = true; }

				if ( flags::perif_flags::switch_sts && switch_has_been_low ) {
					switch_has_been_low = false;
					set_vehicle_state(state_launch_detect);
				}

				break;
			}

			case (state_launch_detect) : {

				if ( flags::state_flags::accel_over_ld_threshold ) { accel_ld_timer++; }
				else { accel_ld_timer = 0; }

				if ( accel_ld_timer > 10 ) { set_vehicle_state(state_powered_ascent); }

				current_state_timer = accel_ld_timer;

				break;
			}

			case (state_powered_ascent) : {

				if ( flags::state_flags::accel_under_burnout_threshold ) { accel_burnout_timer++; }
				else { accel_burnout_timer = 0; }

				if ( accel_burnout_timer > 25 ) { set_vehicle_state(state_ascent_coast); }

				current_state_timer = accel_burnout_timer;

				break;
			}

			case (state_ascent_coast) : {

				if ( flags::state_flags::velocity_over_apogee_threshold ) { velocity_apogee_timer++; }
				else { velocity_apogee_timer = 0; }

				if ( velocity_apogee_timer > 10 ) { set_vehicle_state(state_descent_coast); }

				current_state_timer = velocity_apogee_timer;

				break;
			}

			case (state_descent_coast) : {

				if ( timing::get_t_apogee() == 0 ) { timing::set_t_apogee(time_us_64()); }

				#ifdef LANDING_HW_EN

				if ( flags::control_flags::start_landing_burn ) {
				    set_vehicle_state(state_landing_start);
					#ifdef SITL
					int32_t rand_offset = (rand()%100000)-50000;
					timing::set_t_landing_burn_start(time_us_64()+200000+rand_offset);
					#endif
				}

				#else

				if ( flags::state_flags::accel_within_landed_threshold || flags::state_flags::gyro_within_landed_threshold ) {
					landing_detect_timer++;
				} else {
					landing_detect_timer = 0;
				}

				if ( landing_detect_timer > 100 ) { set_vehicle_state(state_landed); }
				
				#endif

				break;
			}

			case (state_landing_start) : {

				if ( timing::get_t_apogee() == 0 ) { timing::set_t_apogee(time_us_64()); }

				#ifdef LANDING_HW_EN

				if ( flags::state_flags::accel_decreasing && flags::state_flags::accel_over_peak_landing_thresh ) { accel_landing_burn_timer++; }
				else { accel_landing_burn_timer = 0; }

				if ( accel_landing_burn_timer > 2 ) {
					set_vehicle_state(state_landing_guidance);

					// subtract (419000+3000) because landing is detected after burn peak, which is 0.419s after ignition
					timing::set_t_landing_burn_start(time_us_64() - (419000+30000));
				}

				current_state_timer = accel_landing_burn_timer;

				#endif

				if ( flags::state_flags::accel_within_landed_threshold || flags::state_flags::gyro_within_landed_threshold ) {
					landing_detect_timer++;
				} else {
					landing_detect_timer = 0;
				}

				if ( landing_detect_timer > 100 ) { set_vehicle_state(state_landed); }

				break;
			}

			// landing guidance is from 0.0 to 1.8s of landing burn OR alt > 0.5m
			case (state_landing_guidance) : {

				if ( (time_us_64() > timing::get_t_landing_burn_start()+1700000) || ( nav::position.x < 0.5f ) ) { set_vehicle_state(state_landing_terminal); }

				if ( flags::state_flags::accel_within_landed_threshold || flags::state_flags::gyro_within_landed_threshold ) {
					landing_detect_timer++;
				} else {
					landing_detect_timer = 0;
				}

				if ( landing_detect_timer > 100 ) { set_vehicle_state(state_landed); }

				break;
			}

			case (state_landing_terminal) : {

				if ( flags::state_flags::accel_within_landed_threshold || flags::state_flags::gyro_within_landed_threshold ) {
					landing_detect_timer++;
				} else {
					landing_detect_timer = 0;
				}

				if ( landing_detect_timer > 100 ) { set_vehicle_state(state_landed); }

				break;
			}

			case (state_landed) : {

				if ( timing::get_t_landing() == 0 ) { timing::set_t_landing(time_us_64()); }

				break;
			}

			case (state_abort) : {

				break;
			}
		}
	}


};