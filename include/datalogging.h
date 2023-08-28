#include "core.h"
#include "drivers/flash.h"
#include "control.h"
#include "state_ctrl.h"
#include "radio.h"

#pragma once

// datalog header
//sys_time,sys_state,sys_core_usage,pyro_has_power,pyro_1_fire,pyro_2_fire,pyro_3_fire,pyro_1_cont,pyro_2_cont,pyro_3_cont,switch_sts,sys_flag_accel_over_ld_threshold,sys_flag_accel_under_burnout_threshold,sys_flag_velocity_over_apogee_threshold,sys_flag_accel_over_landing_threshold,sys_flag_accel_within_landed_threshold,sys_flag_gyro_within_landed_threshold,sys_flag_baro_below_alt_threshold,sys_flag_velocity_below_landed_threshold,gnc_flag_start_landing_burn,gnc_flag_burn_alt_over_safe_thresh,sys_flag_core1_communication_failure,gnc_flag_3,gnc_flag_4,gnc_flag_5,gnc_flag_6,gnc_flag_7,gnc_flag_gyro_debiased,gnc_flag_baro_debiased,gnc_flag_gps_lock,gnc_flag_gps_initial_position_lock,gnc_flag_kalman_x_converged,gnc_flag_kalman_y_converged,gnc_flag_kalman_z_converged,gnc_flag_orientation_converged,sys_active_state_timer,sys_flash_errors,sys_voltage_batt,sys_voltage_pyro,nav_position_x,nav_position_y,nav_position_z,nav_velocity_x,nav_velocity_y,nav_velocity_z,nav_accel_bias_x,nav_accel_bias_y,nav_accel_bias_z,nav_rotation_w,nav_rotation_x,nav_rotation_y,nav_rotation_z,nav_rotation_euler_x,nav_rotation_euler_y,nav_rotation_euler_z,nav_acceleration_l_x,nav_acceleration_l_y,nav_acceleration_l_z,nav_acceleration_l_debiased_x,nav_acceleration_l_debiased_y,nav_acceleration_l_debiased_z,nav_acceleration_i_x,nav_acceleration_i_y,nav_acceleration_i_z,nav_acceleration_i_debiased_x,nav_acceleration_i_debiased_y,nav_acceleration_i_debiased_z,nav_ori_rate_x,nav_ori_rate_y,nav_ori_rate_z,nav_baro_alt,nav_baro_pressure,nav_baro_temp,nav_mag_x,nav_mag_y,nav_mag_z,gnc_target_vector_x,gnc_target_vector_y,gnc_target_vector_z,gnc_angle_setpoint_y,gnc_angle_setpoint_z,gnc_angle_y,gnc_angle_z,gnc_angle_error_y,gnc_angle_error_z,gnc_thrust,gnc_ang_acc_output_x,gnc_ang_acc_output_y,gnc_ang_acc_output_z,gnc_ang_acc_error_x,gnc_ang_acc_error_y,gnc_ang_acc_error_z,gnc_ang_acc_x,gnc_ang_acc_y,gnc_ang_acc_z,gnc_angle_out_x,gnc_angle_out_y,gnc_angle_out_z,gnc_burn_alt,gnc_comp,gnc_desired_accel,gnc_throttle_ratio,gnc_divert_angle,simulation_energy_est,simulation_work_est,simulation_position_est,simulation_velocity_est,simulation_time_est,simulation_time_taken

namespace datalog {

	#pragma pack(1)

	struct points {
		
		uint64_t padding;

		uint64_t time;
		system_state_t state;
		uint8_t flag_gpio;
		uint8_t flag_pyro;
		uint8_t flag_state;
		uint8_t flag_control;
		uint8_t flag_nav;
		uint8_t active_state_timer;

		uint8_t flash_errors;

		uint16_t voltage_batt;
		uint16_t voltage_pyro;

		uint8_t core_usage_percent;
		
		vec3<float> position;
		vec3<float> velocity;
		vec3<float> accel_bias;
		quat<float> rotation;
		
		vec3<int16_t> acceleration;
		vec3<int16_t> ori_rate;

		float baro_alt;
		float baro_pressure;
		float baro_temp;

		vec3<int16_t> mag;

		float lat;
		float lon;
		float gps_pos_y;
		float gps_pos_z;
		float h_acc;
		uint8_t n_sats;
		
		float mass;

		float thrust;

		vec3<float> target_vector;
		vec3<float> ang_acc_output;
		vec3<float> ang_acc_error;
		vec3<float> angle_out;

		float burn_alt;
		float comp;

		float desired_accel;
		float throttle_ratio;
		float divert_angle;

		uint64_t t_landing_burn_start;

		float simulation_energy_est;
		float simulation_work_est;
		float simulation_position_est;
		float simulation_velocity_est;
		float simulation_time_est;
		uint32_t simulation_time_taken;

		uint8_t extra[16];

	} points;

	static_assert(sizeof(points) == 256, "data points have more than 256 bytes!");

	union {
		struct points {

			system_state_t* state;
			uint64_t* time;
			uint8_t* flag_gpio;
			uint8_t* flag_pyro;
			uint8_t* flag_state;
			uint8_t* flag_control;
			uint8_t* flag_nav;
			uint8_t* active_state_timer;

			uint8_t* flash_errors;

			uint16_t* voltage_batt;
			uint16_t* voltage_pyro;

			uint8_t* core_usage_percent;

			vec3<float>* position;
			vec3<float>* velocity;
			quat<float>* rotation;
			vec3<float>* accel_bias;
			
			vec3<int16_t>* acceleration;
			vec3<int16_t>* ori_rate;

			float* baro_alt;
			float* baro_pressure;
			float* baro_temp;

			vec3<int16_t>* mag;

			float* lat;
			float* lon;
			float* gps_pos_y;
			float* gps_pos_z;
			float* h_acc;
			uint8_t* n_sats;

			float *mass;

			float *thrust;

			vec3<float> *target_vector;
			vec3<float> *ang_acc_output;
			vec3<float> *ang_acc_error;
			vec3<float> *angle_out;

			float* burn_alt;
			float* comp;

			float* desired_accel;
			float* throttle_ratio;
			float* divert_angle;

			uint64_t *t_landing_burn_start;

			float* simulation_energy_est;
			float* simulation_work_est;
			float* simulation_position_est;
			float* simulation_velocity_est;
			float* simulation_time_est;
			uint32_t* simulation_time_taken;

		} points;

		void *raw[46] = {NULL};

		static_assert(sizeof(points) == sizeof(raw));
	} ptrs;

	#pragma pop()

	uint8_t log_timing_idx = 0;

	uint16_t start_page = 0;
	uint16_t page = 0;
	uint16_t export_page = 0;

	uint8_t flash_errors = 0;

	bool export_current_flight = false;
	bool export_past_flights = false;
	bool erase_all_flights = false;
	
	uint16_t find_flash_start() {

		#ifdef DATALOG_EN

		#ifndef USE_INTERNAL_FLASH

			int last_page_was_valid = 0;
			uint8_t read_buf[256] = {0};

			spi_set_baudrate(spi0, spi_flash_baud);

			for ( int i = 0; i < 15625; i++ ) {
				
				while(flash_busy(pin_cs_flash)) { ; }
				if (!flash_read_page(pin_cs_flash, i, read_buf) ) { return -1; }

				page = i;
				start_page = i;
				bool valid_page = 1;
				for ( int j = 0; j < 256; j++ ) { valid_page &= read_buf[j]; }
				if ( valid_page ) { 
					last_page_was_valid ++;
					if ( last_page_was_valid > 2 ) { 
						page = i-1;
						start_page = i-1;
						return page; 
					}             
				}
				
			}
			
			spi_set_baudrate(spi0, spi_default_baud);

			return page;

		#else

			// multicore_lockout_start_blocking();

			int last_page_was_valid = 0;
			uint8_t read_buf[256] = {0};

			for ( int i = 0; i < 12288; i++ ) {
				
				if (!flash_read_page(pin_cs_flash, i, read_buf) ) { return -1; }

				page = i;
				start_page = i;
				bool valid_page = 1;
				for ( int j = 0; j < 256; j++ ) { valid_page &= read_buf[j]; }
				if ( valid_page ) { 
					last_page_was_valid ++;
					if ( last_page_was_valid > 2 ) { 
						page = i-1;
						start_page = i-1;
						return page; 
					}             
				}
				
			}

			// multicore_lockout_end_blocking();

			return page;


		#endif

		#endif

		return -1;

	}

	bool valid_pointers() {
		#ifdef DATALOG_EN
			for ( int i = 0; i < sizeof(ptrs.raw); i++ ) { if ( ptrs.raw[i] == NULL ) { return 0; } }
		#endif
		return 1;
	}

	void log_flash_data() {
		
		#ifdef DATALOG_EN

		points.padding = 0;
		points.time						= *ptrs.points.time;
		points.state					= *ptrs.points.state;
		points.flag_gpio				= *ptrs.points.flag_gpio;
		points.flag_pyro 				= *ptrs.points.flag_pyro;
		points.flag_state				= *ptrs.points.flag_state;
		points.flag_control				= *ptrs.points.flag_control;
		points.flag_nav					= *ptrs.points.flag_nav;
		points.active_state_timer		= *ptrs.points.active_state_timer;
		points.flash_errors				= *ptrs.points.flash_errors;

		points.voltage_batt				= *ptrs.points.voltage_batt;
		points.voltage_pyro				= *ptrs.points.voltage_pyro;
		points.core_usage_percent		= *ptrs.points.core_usage_percent;

		points.position             	= *ptrs.points.position;
		points.velocity             	= *ptrs.points.velocity;
		points.rotation             	= *ptrs.points.rotation;
		points.accel_bias           	= *ptrs.points.accel_bias;

		points.acceleration         	= *ptrs.points.acceleration;

		points.ori_rate             	= *ptrs.points.ori_rate;
		points.baro_alt             	= *ptrs.points.baro_alt;
		points.baro_pressure        	= *ptrs.points.baro_pressure;
		points.baro_temp            	= *ptrs.points.baro_temp;

		points.mag                  	= *ptrs.points.mag;

		points.lat						= *ptrs.points.lat;
		points.lon						= *ptrs.points.lon;
		points.gps_pos_y				= *ptrs.points.gps_pos_y;
		points.gps_pos_z				= *ptrs.points.gps_pos_z;
		points.h_acc					= *ptrs.points.h_acc;
		points.n_sats					= *ptrs.points.n_sats;

		points.mass 					= *ptrs.points.mass;

		points.thrust 					= *ptrs.points.thrust;

		points.target_vector        	= *ptrs.points.target_vector;
		points.ang_acc_output       	= *ptrs.points.ang_acc_output;
		points.ang_acc_error        	= *ptrs.points.ang_acc_error;
		points.angle_out            	= *ptrs.points.angle_out;

		points.burn_alt             	= *ptrs.points.burn_alt;
		points.comp                 	= *ptrs.points.comp;

		points.desired_accel			= *ptrs.points.desired_accel;
		points.throttle_ratio			= *ptrs.points.throttle_ratio;
		points.divert_angle				= *ptrs.points.divert_angle;

		points.t_landing_burn_start 	= *ptrs.points.t_landing_burn_start;

		points.simulation_energy_est	= *ptrs.points.simulation_energy_est;
		points.simulation_work_est     	= *ptrs.points.simulation_work_est;
		points.simulation_position_est 	= *ptrs.points.simulation_position_est;
		points.simulation_velocity_est 	= *ptrs.points.simulation_velocity_est;
		points.simulation_time_est     	= *ptrs.points.simulation_time_est;
		points.simulation_time_taken   	= *ptrs.points.simulation_time_taken;

		page++;
		spi_set_baudrate(spi0, spi_flash_baud);
		while(flash_busy(pin_cs_flash)) { ; }
		if ( !flash_write_page(pin_cs_flash, page, (uint8_t*)&points) ) { flash_errors++; }
		spi_set_baudrate(spi0, spi_default_baud);

		#endif

	}

	uint8_t *get_flash_tx_buf() { return (uint8_t*)&points; }

	bool init() {
		#ifdef DATALOG_EN

		#ifndef USE_INTERNAL_FLASH

			uint8_t a = 0;
			uint8_t b = 0;
			uint8_t c = 0;
			get_jdec(pin_cs_flash, &a, &b, &c);

			printf("%i, %i, %i\n", a, b, c);

			get_sreg(pin_cs_flash, &a, &b);

			printf("%i, %i, %i\n", a, b);

			if ( (a == 0) && (b == 0) && (c == 0) ) { boot_panic("no flash chip found!\n"); return 0; }
		#endif

		printf("============================================================================\n");
		printf("locating flash start page...\n");

		spi_set_baudrate(spi0, spi_flash_baud);     
		
		// flash_erase_chip(pin_cs_flash);
		find_flash_start();

		#ifndef USE_INTERNAL_FLASH
			printf("start page: %i\nremaining pages: %i (%.1f%c / %.1fs)\n", page, (15625-page), 100.0f - 100.f*(float(page)/float(15625)), '%', float(15625-page)/100.f);
			printf("flash storage: [");
			for ( int i = 0; i < 20; i++ ) {
				if ( i*5 < (100.f*(float(page)/float(15625))) ) { printf("="); }
				else { printf(" "); }
			}
		#else
			printf("start page: %i\nremaining pages: %i (%.1f%c / %.1fs)\n", page, (12288-page), 100.0f - 100.f*(float(page)/float(12288)), '%', float(12288-page)/100.f);
			printf("flash storage: [");
			for ( int i = 0; i < 20; i++ ) {
				if ( i*5 < (100.f*(float(page)/float(12288))) ) { printf("="); }
				else { printf(" "); }
			}
		#endif

		printf("]\n");
		printf("============================================================================\n");

		spi_set_baudrate(spi0, spi_default_baud);
		return 1;
		#endif
		return true;
	}

	void export_flash_data_blocking(int page_start, int page_end) {
		#ifdef DATALOG_EN

		for ( int i = page_start; i < page_end; i++ ) {
			
			flash_read_page(pin_cs_flash, i, (uint8_t*)&points);

			quat<float> rotation = points.rotation;
			vec3<float> target_vec = points.target_vector;
			vec3<float> rotation_euler = rotation.euler_angles() * 180.0f/PI;
			
			#ifndef ACCEL_DEBIAS_ON_PAD
			vec3<float> accel_l = vec3<float>(points.acceleration.x, points.acceleration.y, points.acceleration.z) * 0.009765625f;
			vec3<float> accel_i = rotation.rotate_vec(accel_l);
			vec3<float> accel_i_debiased = accel_i - points.accel_bias;
			vec3<float> accel_l_debiased = rotation.conjugate().rotate_vec(accel_i_debiased);
			#else
			vec3<float> accel_l = vec3<float>(points.acceleration.x, points.acceleration.y, points.acceleration.z) * 0.009765625f;
			vec3<float> accel_i = rotation.rotate_vec(accel_l)-vec3<float>(9.816, 0.0, 0.0);
			vec3<float> accel_i_debiased = accel_i - rotation.rotate_vec(points.accel_bias);
			vec3<float> accel_l_debiased = accel_l - points.accel_bias;
			#endif

			vec3<float> angle_error;
			vec3<float> target_vector_;
			target_vector_ = rotation.conjugate().rotate_vec(target_vec.norm());
			angle_error.y = atan2f(-target_vector_.z, target_vector_.x) * 180.0/PI;
			angle_error.z = atan2f( target_vector_.y, target_vector_.x) * 180.0/PI;

			vec3<float> vector_heading = rotation.conjugate().rotate_vec(vec3<float>(1.0, 0.0, 0.0));
			vec3<float> vector_angle;
			vector_angle.y = atan2f(-vector_heading.z, vector_heading.x) * 180.0/PI;
			vector_angle.z = atan2f( vector_heading.y, vector_heading.x) * 180.0/PI;

			printf("$");

			if ( points.state == 0xff ) { printf("\n$\n");continue; }

			printf("%llu,", points.time);
			printf("%i,",   points.state);
			printf("%i,",   points.core_usage_percent);

			// printf("%i,",   (points.flag_pyro   )&1);
			printf("%i,",   (points.flag_pyro>>1)&1);
			printf("%i,",   (points.flag_pyro>>2)&1);
			printf("%i,",   (points.flag_pyro>>3)&1);
			printf("%i,",   (points.flag_pyro>>4)&1);
			printf("%i,",   (points.flag_pyro>>5)&1);
			printf("%i,",   (points.flag_pyro>>6)&1);
			printf("%i,",   (points.flag_pyro>>7)&1);

			printf("%i,",   (points.flag_gpio   )&1);
			// printf("%i,",   (points.flag_gpio>>1)&1);
			// printf("%i,",   (points.flag_gpio>>2)&1);
			// printf("%i,",   (points.flag_gpio>>3)&1);
			// printf("%i,",   (points.flag_gpio>>4)&1);
			// printf("%i,",   (points.flag_gpio>>5)&1);
			// printf("%i,",   (points.flag_gpio>>6)&1);
			// printf("%i,",   (points.flag_gpio>>7)&1);

			printf("%i,",   (points.flag_state   )&1);
			printf("%i,",   (points.flag_state>>1)&1);
			printf("%i,",   (points.flag_state>>2)&1);
			printf("%i,",   (points.flag_state>>3)&1);
			printf("%i,",   (points.flag_state>>4)&1);
			printf("%i,",   (points.flag_state>>5)&1);
			printf("%i,",   (points.flag_state>>6)&1);
			printf("%i,",   (points.flag_state>>7)&1);

			printf("%i,",   (points.flag_control   )&1);
			printf("%i,",   (points.flag_control>>1)&1);
			printf("%i,",   (points.flag_control>>2)&1);
			printf("%i,",   (points.flag_control>>3)&1);
			printf("%i,",   (points.flag_control>>4)&1);
			printf("%i,",   (points.flag_control>>5)&1);
			printf("%i,",   (points.flag_control>>6)&1);
			printf("%i,",   (points.flag_control>>7)&1);

			printf("%i,",   (points.flag_nav   )&1);
			printf("%i,",   (points.flag_nav>>1)&1);
			printf("%i,",   (points.flag_nav>>2)&1);
			printf("%i,",   (points.flag_nav>>3)&1);
			printf("%i,",   (points.flag_nav>>4)&1);
			printf("%i,",   (points.flag_nav>>5)&1);
			printf("%i,",   (points.flag_nav>>6)&1);
			printf("%i,",   (points.flag_nav>>7)&1);

			printf("%i,",   points.active_state_timer);
			printf("%i,",   points.flash_errors);

			printf("%f,",   ((float)points.voltage_batt)/484.07f);
			printf("%f,",   ((float)points.voltage_pyro)/484.07f);

			printf("%f,", 	points.mass);

			printf("%f,",   points.position.x);
			printf("%f,",   points.position.y);
			printf("%f,",   points.position.z);

			printf("%f,",   points.velocity.x);
			printf("%f,",   points.velocity.y);
			printf("%f,",   points.velocity.z);

			printf("%f,",   points.accel_bias.x);
			printf("%f,",   points.accel_bias.y);
			printf("%f,",   points.accel_bias.z);

			printf("%f,",   points.rotation.w);
			printf("%f,",   points.rotation.x);
			printf("%f,",   points.rotation.y);
			printf("%f,",   points.rotation.z);

			printf("%f,",   rotation_euler.x);
			printf("%f,",   rotation_euler.y);
			printf("%f,",   rotation_euler.z);

			printf("%f,",   accel_l.x);
			printf("%f,",   accel_l.y);
			printf("%f,",   accel_l.z);

			printf("%f,",   accel_l_debiased.x);
			printf("%f,",   accel_l_debiased.y);
			printf("%f,",   accel_l_debiased.z);

			printf("%f,",   accel_i.x);
			printf("%f,",   accel_i.y);
			printf("%f,",   accel_i.z);

			printf("%f,",   accel_i_debiased.x);
			printf("%f,",   accel_i_debiased.y);
			printf("%f,",   accel_i_debiased.z);

			printf("%f,",   (float)points.ori_rate.x*0.00106526443f*180.0f/PI);
			printf("%f,",   (float)points.ori_rate.y*0.00106526443f*180.0f/PI);
			printf("%f,",   (float)points.ori_rate.z*0.00106526443f*180.0f/PI);

			printf("%f,",   points.baro_alt);
			printf("%f,",   points.baro_pressure);
			printf("%f,",   points.baro_temp);
			
			printf("%i,",   points.mag.x);
			printf("%i,",   points.mag.y);
			printf("%i,",   points.mag.z);

			printf("%f,",	points.target_vector.x);
			printf("%f,",	points.target_vector.y);
			printf("%f,",	points.target_vector.z);

			printf("%f,", 	atan2f(-points.target_vector.z, points.target_vector.x) * 180.0/PI);
			printf("%f,", 	atan2f( points.target_vector.y, points.target_vector.x) * 180.0/PI);

			printf("%f,", 	vector_angle.y);
			printf("%f,", 	vector_angle.z);

			printf("%f,", 	angle_error.y);
			printf("%f,", 	angle_error.z);

			printf("%f,", 	points.thrust);
			
			printf("%f,",	points.ang_acc_output.x*180/PI);
			printf("%f,",	points.ang_acc_output.y*180/PI);
			printf("%f,",	points.ang_acc_output.z*180/PI);
			
			printf("%f,",	points.ang_acc_error.x*180/PI);
			printf("%f,",	points.ang_acc_error.y*180/PI);
			printf("%f,",	points.ang_acc_error.z*180/PI);

			printf("%f,",	(points.ang_acc_error.x + points.ang_acc_output.x)*180/PI);
			printf("%f,",	(points.ang_acc_error.y + points.ang_acc_output.y)*180/PI);
			printf("%f,",	(points.ang_acc_error.z + points.ang_acc_output.z)*180/PI);

			printf("%f,",	points.angle_out.x);
			printf("%f,",	points.angle_out.y);
			printf("%f,",	points.angle_out.z);

			printf("%f,",   points.burn_alt);
			printf("%f,",   points.comp);
			printf("%f,", 	points.desired_accel);
			printf("%f,", 	points.throttle_ratio);
			printf("%f,", 	points.divert_angle*180.f/PI);

			printf("%f,", 	points.t_landing_burn_start);
			
			printf("%f,",   points.simulation_energy_est);
			printf("%f,",   points.simulation_work_est);
			printf("%f,",   points.simulation_position_est);
			printf("%f,",   points.simulation_velocity_est);
			printf("%f,",   points.simulation_time_est);
			printf("%i\n",  points.simulation_time_taken);

		};
		#endif
	}

	void update() {
		#ifdef DATALOG_EN

		if ( export_current_flight && !vehicle_is_in_flight() ) {
			// export_flash_data_async();
			export_flash_data_blocking(start_page, page);
			export_current_flight = false;
		}

		if ( export_past_flights && !vehicle_is_in_flight() ) {
			// export_flash_data_async();
			export_flash_data_blocking(0, start_page);
			export_past_flights = false;
		}

		if ( erase_all_flights && !vehicle_is_in_flight() ) {
			flash_erase_chip(pin_cs_flash);
			find_flash_start();
			erase_all_flights = false;
		}

		switch(get_vehicle_state()) {
			case(state_boot): { 
				break;
			}
			case(state_idle): { 
				break;
			}
			case(state_nav_init): { 
				if ( ++log_timing_idx >= 20 ) { log_flash_data(); log_timing_idx = 0; } // 5hz
				break;
			}
			case(state_launch_idle): { 
				if ( ++log_timing_idx >= 20 ) { log_flash_data(); log_timing_idx = 0; } // 5hz
				break;
			}
			case(state_launch_detect): { 
				if ( ++log_timing_idx >= 1 ) { log_flash_data(); log_timing_idx = 0; } // 100hz
				break;
			}
			case(state_powered_ascent): { 
				if ( ++log_timing_idx >= 1 ) { log_flash_data(); log_timing_idx = 0; } // 100hz
				break;
			}
			case(state_ascent_coast): { 
				if ( ++log_timing_idx >= 1 ) { log_flash_data(); log_timing_idx = 0; } // 100hz
				break;
			}
			case(state_descent_coast): { 
				if ( ++log_timing_idx >= 1 ) { log_flash_data(); log_timing_idx = 0; } // 100hz
				break;
			}
			case(state_landing_start): { 
				if ( ++log_timing_idx >= 1 ) { log_flash_data(); log_timing_idx = 0; } // 100hz
				break;
			}
			case(state_landing_guidance): { 
				if ( ++log_timing_idx >= 1 ) { log_flash_data(); log_timing_idx = 0; } // 100hz
				break;
			}
			case(state_landing_terminal): { 
				if ( ++log_timing_idx >= 1 ) { log_flash_data(); log_timing_idx = 0; } // 100hz
				break;
			}
			case(state_landed): { 

				// only log for 5 seconds after landing
				if ( time_us_64() < timing::get_t_landing()+5000000 ) {

					if ( ++log_timing_idx >= 2 ) {
						log_flash_data();
						log_timing_idx = 0;
					}

				} 
				
				// else {

				//     // send data back at 25hz
				//     // radio operates at 115200 bits/s, each flash page is 256*8 bits + 16 bit checksum
				//     // 2064 (page size) * 25 ~= 1/2 of the radio's max data throughput, which should prevent corruption

				//     if ( ++log_timing_idx >= 20 ) {
				//         export_flash_data_async();
				//         log_timing_idx = 0;
				//     }

				// }
				
				break;
			}
			case(state_abort): { 
				if ( ++log_timing_idx >= 1 ) { log_flash_data(); log_timing_idx = 0; }
				break;
			}
			default: { 
				
				break;
			}
		}

		#endif
	}

}
