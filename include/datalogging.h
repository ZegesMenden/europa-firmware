#include "core.h"
#include "drivers/flash.h"
#include "control.h"
#include "state_ctrl.h"
#include "radio.h"
#pragma once

namespace datalog {

	#pragma pack(1)


	struct points {
		
		uint64_t padding;

		uint64_t time;
		system_state_t state;
		uint8_t flag_gpio;
		uint8_t flag_state;
		uint8_t flag_control;
		uint8_t flag_nav;
		uint8_t active_state_timer;

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
		
		vec3<float> target_vector;
		vec3<float> ang_acc_output;
		vec3<float> ang_acc_error;
		vec3<float> angle_out;

		float burn_alt;
		float comp;
		float simulation_energy_est;
		float simulation_work_est;
		float simulation_position_est;
		float simulation_velocity_est;
		float simulation_time_est;
		uint32_t simulation_time_taken;

		uint8_t extra[67];

	} points;

	int a = sizeof(points);

	typedef union log_ptrs_t {
		struct points {

			system_state_t* state;
			uint64_t* time;
			uint8_t* flag_gpio;
			uint8_t* flag_state;
			uint8_t* flag_control;
			uint8_t* flag_nav;
			uint8_t* active_state_timer;

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

			vec3<float> *target_vector;
			vec3<float> *ang_acc_output;
			vec3<float> *ang_acc_error;
			vec3<float> *angle_out;

			float* burn_alt;
			float* comp;
			float* simulation_energy_est;
			float* simulation_work_est;
			float* simulation_position_est;
			float* simulation_velocity_est;
			float* simulation_time_est;
			uint32_t* simulation_time_taken;

		} points;

		void *raw[32] = {NULL};

		static_assert(sizeof(points) == sizeof(raw));
	} log_ptrs_t;

	#pragma pop()

	uint8_t log_timing_idx = 0;

	uint16_t start_page = 0;
	uint16_t page = 0;
	uint16_t export_page = 0;

	uint8_t flash_errors = 0;

	log_ptrs_t ptrs;

	bool start_flash_export = false;

	uint16_t find_flash_start() {

		#ifdef DATALOG_EN

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

		flags::update();

		points.padding = 0x0011223344556677;
		points.time						= *ptrs.points.time;
		points.state					= *ptrs.points.state;
		points.flag_gpio				= *ptrs.points.flag_gpio;
		points.flag_state				= *ptrs.points.flag_state;
		points.flag_control				= *ptrs.points.flag_control;
		points.flag_nav					= *ptrs.points.flag_nav;
		points.active_state_timer		= *ptrs.points.active_state_timer;
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

		points.target_vector        	= *ptrs.points.target_vector;
		points.ang_acc_output       	= *ptrs.points.ang_acc_output;
		points.ang_acc_error        	= *ptrs.points.ang_acc_error;
		points.angle_out            	= *ptrs.points.angle_out;

		points.burn_alt             	= *ptrs.points.burn_alt;
		points.comp                 	= *ptrs.points.comp;
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

		// uint8_t buf[256];

		// if ( !flash_read_page(pin_cs_flash, page, buf) ) { flash_errors++; }
		// bool matches = true;

		// for ( int i = i; i < 256; i++ ) { matches &= (buf[i]==((uint8_t*)&points)[i]); }   

		// if ( !matches ) { 
		//     printf("WARNING: datalog mismatch!\n"); 
			
		//     printf("written data:\n");
		//     for ( int i = 0; i < 8; i++ ) {
		//         int j = i*8;
		//         printf("%x,%x,%x,%x,%x,%x,%x,%x\n", buf[j],buf[j+1],buf[j+2],buf[j+3],buf[j+4],buf[j+5],buf[j+6],buf[j+7]);
		//     }

		//     printf("re-read data:\n");
		//     for ( int i = 0; i < 8; i++ ) {
		//         int j = i*8;
		//         printf("%x,%x,%x,%x,%x,%x,%x,%x\n", ((uint8_t*)&points)[j],((uint8_t*)&points)[j+1],((uint8_t*)&points)[j+2],((uint8_t*)&points)[j+3],((uint8_t*)&points)[j+4],((uint8_t*)&points)[j+5],((uint8_t*)&points)[j+6],((uint8_t*)&points)[j+7]);
		//     }
		// }

		spi_set_baudrate(spi0, spi_default_baud);

		#endif

	}

	uint8_t *get_flash_tx_buf() { return (uint8_t*)&points; }

	bool init() {
		#ifdef DATALOG_EN

		uint8_t a = 0;
		uint8_t b = 0;
		uint8_t c = 0;
		get_jdec(pin_cs_flash, &a, &b, &c);

		printf("%i, %i, %i\n", a, b, c);

		if ( (a == 0) && (b == 0) && (c == 0) ) { boot_panic("no flash chip found!\n"); return 0; }

		printf("============================================================================\n");
		printf("locating flash start page...\n");

		spi_set_baudrate(spi0, spi_flash_baud);     

		flash_erase_chip(pin_cs_flash);
		while(flash_busy(pin_cs_flash)) {;}
		
		find_flash_start();

		printf("start page: %i\nremaining pages: %i (%.1f%c / %.1fs)\n", page, (15625-page), 100.0f - 100.f*(float(page)/float(15625)), '%', float(15625-page)/100.f);
		printf("flash storage: [");
		for ( int i = 0; i < 20; i++ ) {
			if ( i*5 < (100.f*(float(page)/float(15625))) ) { printf("="); }
			else { printf(" "); }
		}
		printf("]\n");
		printf("============================================================================\n");

		spi_set_baudrate(spi0, spi_default_baud);
		return 1;
		#endif
		return 1;
	}

	void export_flash_data_blocking() {
		#ifdef DATALOG_EN

		// printf("FLASHDATA\n");
		// printf("nlines%i\n", page-start_page);

		for ( int i = start_page; i < page; i++ ) {
			
			flash_read_page(pin_cs_flash, i, (uint8_t*)&points);

			printf("$");

			quat<float> rotation = points.rotation;
			vec3<float> target_vec = points.target_vector;

			vec3<float> rotation_euler = rotation.euler_angles() * 180.0f/PI;
			vec3<float> accel_l = points.acceleration;
			accel_l *= 0.009765625f;
			vec3<float> accel_i = rotation.rotate_vec(accel_l);
			vec3<float> accel_i_debiased = accel_i - points.accel_bias;
			vec3<float> accel_l_debiased = rotation.conjugate().rotate_vec(accel_i_debiased);
			vec3<float> angle_error;
			vec3<float> target_vector_;
			target_vector_ = rotation.conjugate().rotate_vec(target_vec.norm());
			angle_error.y = atan2f(-target_vector_.z, target_vector_.x) * 180.0/PI;
			angle_error.z = atan2f(target_vector_.y, target_vector_.x) * 180.0/PI;

			printf("%llu,", points.time);
			printf("%i,",   points.state);
			printf("%i,",   (points.flag_gpio&1));
			printf("%i,",   (points.flag_gpio&0b10)>>1);
			printf("%i,",   (points.flag_gpio&0b100)>>2);
			printf("%i,",   (points.flag_gpio&0b1000)>>3);
			printf("%i,",   (points.flag_gpio&0b10000)>>4);
			printf("%i,",   (points.flag_gpio&0b100000)>>5);
			printf("%i,",   (points.flag_gpio&0b1000000)>>6);
			printf("%i,",   (points.flag_gpio&0b10000000)>>7);
			printf("%i,",   (points.flag_state&1));
			printf("%i,",   (points.flag_state&0b10)>>1);
			printf("%i,",   (points.flag_state&0b100)>>2);
			printf("%i,",   (points.flag_state&0b1000)>>3);
			printf("%i,",   (points.flag_state&0b10000)>>4);
			printf("%i,",   (points.flag_state&0b100000)>>5);
			printf("%i,",   (points.flag_state&0b1000000)>>6);
			printf("%i,",   (points.flag_state&0b10000000)>>7);
			printf("%i,",   (points.flag_control&1));
			printf("%i,",   (points.flag_control&0b10)>>1);
			printf("%i,",   (points.flag_control&0b100)>>2);
			printf("%i,",   (points.flag_control&0b1000)>>3);
			printf("%i,",   (points.flag_control&0b10000)>>4);
			printf("%i,",   (points.flag_control&0b100000)>>5);
			printf("%i,",   (points.flag_control&0b1000000)>>6);
			printf("%i,",   (points.flag_control&0b10000000)>>7);
			printf("%i,",   (points.flag_nav&1));
			printf("%i,",   (points.flag_nav&0b10)>>1);
			printf("%i,",   (points.flag_nav&0b100)>>2);
			printf("%i,",   (points.flag_nav&0b1000)>>3);
			printf("%i,",   (points.flag_nav&0b10000)>>4);
			printf("%i,",   (points.flag_nav&0b100000)>>5);
			printf("%i,",   (points.flag_nav&0b1000000)>>6);
			printf("%i,",   (points.flag_nav&0b10000000)>>7);
			printf("%i,",   points.active_state_timer);
			printf("%f,",   ((float)points.voltage_batt)/484.07f);
			printf("%f,",   ((float)points.voltage_pyro)/484.07f);
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

			printf("%f,",points.target_vector.x);
			printf("%f,",points.target_vector.y);
			printf("%f,",points.target_vector.z);

			printf("%f,", atan2f(-points.target_vector.z, points.target_vector.x) * 180.0/PI);
			printf("%f,", atan2f(points.target_vector.y, points.target_vector.x) * 180.0/PI);

			printf("%f,", angle_error.y);
			printf("%f,", angle_error.z);
			
			printf("%f,",points.ang_acc_output.x*180/PI);
			printf("%f,",points.ang_acc_output.y*180/PI);
			printf("%f,",points.ang_acc_output.z*180/PI);
			
			printf("%f,",points.ang_acc_error.x*180/PI);
			printf("%f,",points.ang_acc_error.y*180/PI);
			printf("%f,",points.ang_acc_error.z*180/PI);
			
			printf("%f,",points.angle_out.x);
			printf("%f,",points.angle_out.y);
			printf("%f,",points.angle_out.z);

			printf("%f,",   points.burn_alt);
			printf("%f,",   points.comp);
			printf("%f,",   points.simulation_energy_est);
			printf("%f,",   points.simulation_work_est);
			printf("%f,",   points.simulation_position_est);
			printf("%f,",   points.simulation_velocity_est);
			printf("%f,",   points.simulation_time_est);
			printf("%i\n",  points.simulation_time_taken);

		};
		#endif
	}

	void export_flash_data_async() {
		#ifdef DATALOG_EN

		// set read start page
		// if ( export_page == 0 && start_page != 0 ) { export_page = start_page; }

		// dont send the current datalog
		if ( export_page >= start_page ) { return; }

		uint8_t buf[256];

		flash_read_page(pin_cs_flash, export_page++, buf);

		// calculate checksum for sending flash data
		uint8_t checksum_a = 0;
		uint8_t checksum_b = 0;

		// header
		// radio::radio_tx_buf[radio::radio_tx_buf_position++] = '$';
		printf("$");

		for ( int i = 0; i < 256; i++ ) { 
			// get value from most recent data log
			uint8_t v = buf[i];

			// send value to radio buffer
			printf(" %x", v);
			
			// calculate checksum on value
			checksum_a += v; 
			checksum_b += checksum_a; 
		}

		printf("\n");

		// old, prints data as a string

		// radio::radio_tx_buf_position = sprintf((radio::radio_tx_buf+radio::radio_tx_buf_position), "%i,%llu,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%i,%i,%i,%i,%i,%i,%f,%f,%f,%i,%i,%i,%f,%f,%f,%f,%f,%f,%f\n",
		//     points.state,
		//     points.time,
		//     (points.flag_gpio&1),
		//     (points.flag_gpio&0b10)>>1,
		//     (points.flag_gpio&0b100)>>2,
		//     (points.flag_gpio&0b1000)>>3,
		//     (points.flag_gpio&0b10000)>>4,
		//     (points.flag_gpio&0b100000)>>5,
		//     (points.flag_gpio&0b1000000)>>6,
		//     (points.flag_gpio&0b10000000)>>7,
		//     (points.flag_state&1),
		//     (points.flag_state&0b10)>>1,
		//     (points.flag_state&0b100)>>2,
		//     (points.flag_state&0b1000)>>3,
		//     (points.flag_state&0b10000)>>4,
		//     (points.flag_state&0b100000)>>5,
		//     (points.flag_state&0b1000000)>>6,
		//     (points.flag_state&0b10000000)>>7,
		//     points.active_state_timer,
		//     points.voltage_batt,
		//     points.voltage_pyro,
		//     points.position.x,
		//     points.position.y,
		//     points.position.z,
		//     points.velocity.x,
		//     points.velocity.y,
		//     points.velocity.z,
		//     points.accel_bias.x,
		//     points.accel_bias.y,
		//     points.accel_bias.z,
		//     points.rotation.w,
		//     points.rotation.x,
		//     points.rotation.y,
		//     points.rotation.z,
		//     points.acceleration.x,
		//     points.acceleration.y,
		//     points.acceleration.z,
		//     points.ori_rate.x,
		//     points.ori_rate.y,
		//     points.ori_rate.z,
		//     points.baro_alt,
		//     points.baro_pressure,
		//     points.baro_temp,
		//     points.mag.x,
		//     points.mag.y,
		//     points.mag.z,
		//     points.burn_alt,
		//     points.comp,
		//     points.simulation_energy_est,
		//     points.simulation_work_est,
		//     points.simulation_position_est,
		//     points.simulation_velocity_est,
		//     points.simulation_time_est);
		
		#endif
	}

	void update() {
		#ifdef DATALOG_EN

		if ( start_flash_export ) {
			// export_flash_data_async();
			export_flash_data_blocking();
			start_flash_export = false;
		}

		switch(get_vehicle_state()) {
			case(state_boot): { 
				break;
			}
			case(state_idle): { 
				break;
			}
			case(state_nav_init): { 
				if ( ++log_timing_idx >= 20 ) { log_flash_data(); log_timing_idx = 0; }
				break;
			}
			case(state_launch_idle): { 
				if ( ++log_timing_idx >= 20 ) { log_flash_data(); log_timing_idx = 0; }
				break;
			}
			case(state_launch_detect): { 
				if ( ++log_timing_idx >= 2 ) { log_flash_data(); log_timing_idx = 0; }
				break;
			}
			case(state_powered_ascent): { 
				if ( ++log_timing_idx >= 1 ) { log_flash_data(); log_timing_idx = 0; }
				break;
			}
			case(state_ascent_coast): { 
				if ( ++log_timing_idx >= 1 ) { log_flash_data(); log_timing_idx = 0; }
				break;
			}
			case(state_descent_coast): { 
				if ( ++log_timing_idx >= 1 ) { log_flash_data(); log_timing_idx = 0; }
				break;
			}
			case(state_landing_start): { 
				if ( ++log_timing_idx >= 1 ) { log_flash_data(); log_timing_idx = 0; }
				break;
			}
			case(state_landing_guidance): { 
				if ( ++log_timing_idx >= 1 ) { log_flash_data(); log_timing_idx = 0; }
				break;
			}
			case(state_landing_terminal): { 
				if ( ++log_timing_idx >= 1 ) { log_flash_data(); log_timing_idx = 0; }
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
				if ( ++log_timing_idx >= 2 ) { log_flash_data(); log_timing_idx = 0; }
				break;
			}
			default: { 
				
				break;
			}
		}

		#endif
	}

}
