#include "core.h"
#include "navigation.h"
#include "datalogging.h"
#include "drivers/radio.h"
#pragma once

namespace telemetry {

	uint16_t telem_idx = 0;

	bool init() {
		return 1;
	}

	void process_message(char msg_id, char msg_payload) {

		switch(msg_id) {  
			case('a'): {
				if ( get_vehicle_state() == state_idle ) {
					#ifndef SITL
					set_vehicle_state(state_nav_init);
					#else
					set_vehicle_state(state_powered_ascent);
					timing::set_t_launch(time_us_64());
					#endif
				}
				break;
			}
			case('b'): {
				if ( get_vehicle_state() == state_launch_idle ) {
					set_vehicle_state(state_launch_detect);
				}
				break;
			}
			case('c'): {
				if ( get_vehicle_state() == state_launch_detect ) {
					set_vehicle_state(state_launch_idle);
				}
				break;
			}
			case('d'): {
				#ifndef SITL
				if ( !vehicle_is_in_flight() && (vehicle_state != state_launch_detect) ) {
					datalog::export_current_flight = true;
				}
				#else
				datalog::export_current_flight = true;
				#endif
				break;
			}
			case('e'): {
				#ifndef SITL
				if ( !vehicle_is_in_flight() && (vehicle_state != state_launch_detect) ) {
					datalog::export_past_flights = true;
				}
				#else
				datalog::export_past_flights = true;
				#endif
				break;
			}
			case('f'): {
				#ifndef SITL
				if ( !vehicle_is_in_flight() && (vehicle_state != state_launch_detect) ) {
					datalog::erase_all_flights = true;
				}
				#else
				datalog::erase_all_flights = true;
				#endif
				break;
			}
			default: {
				break;
			}
		}

	}

	void update() {

		// sending telemetry data

		uint8_t telem_speed = 10;
		if ( vehicle_is_in_flight() ) { telem_speed = 20; }

		if ( telem_idx >= telem_speed ) {

			// print radio data as a string

			// if ( datalog::export_current_flight || datalog::export_past_flights ) { return; }

			#if true

			vec3<float> rotation_euler = nav::rotation.euler_angles();

			#ifdef USE_RADIO
				radio::radio_tx_buf_position = sprintf((radio::radio_tx_buf+radio::radio_tx_buf_position), "%llu,%i,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%i,%f,%f,%f,%i\n",
						timing::RUNTIME,
						get_vehicle_state(),
						rotation_euler.x,
						rotation_euler.y,
						rotation_euler.z,
						nav::rotational_velocity.x,
						nav::rotational_velocity.y,
						nav::rotational_velocity.z,
						nav::acceleration_l.x,
						nav::acceleration_l.y,
						nav::acceleration_l.z,
						float(timing::average_runtime)/100.f,
						float(perif::voltage_batt_raw)/484.07f,
						nav::position.x,
						nav::position.y,
						nav::position.z,
						nav::velocity.x,
						nav::velocity.y,
						nav::velocity.z,
						flags::perif_flags::pyro_sts,
						(float)gps::gps_pvt_raw.lat*1e-7,
						(float)gps::gps_pvt_raw.lon*1e-7,
						(float)gps::gps_pvt_raw.h_acc*1e-3,
						gps::gps_pvt_raw.n_sat);
			#else
			 
				printf("%llu,%i,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%i,%f,%f,%f,%i\n",
						timing::RUNTIME,
						get_vehicle_state(),
						rotation_euler.x,
						rotation_euler.y,
						rotation_euler.z,
						nav::rotational_velocity.x,
						nav::rotational_velocity.y,
						nav::rotational_velocity.z,
						nav::acceleration_l.x,
						nav::acceleration_l.y,
						nav::acceleration_l.z,
						float(timing::average_runtime)/100.f,
						float(perif::voltage_batt_raw)/484.07f,
						nav::position.x,
						nav::position.y,
						nav::position.z,
						nav::velocity.x,
						nav::velocity.y,
						nav::velocity.z,
						flags::perif_flags::pyro_sts,
						(float)gps::gps_pvt_raw.lat*1e-7,
						(float)gps::gps_pvt_raw.lon*1e-7,
						(float)gps::gps_pvt_raw.h_acc*1e-3,
						gps::gps_pvt_raw.n_sat);

			#endif

			#else

			#ifdef USE_RADIO

				memmove(radio::radio_tx_buf, &datalog::points, 256);
				radio::radio_tx_buf_position = 256;
			
			#else

				for ( int i = 0; i < 256; i++ ) {
					printf("%c", ((char*)(&datalog::points))[i]);
				}

			#endif

			#endif
			
			// ====================================================================================
			// print data as raw values

			// // calculate checksum for sending flash data
			// uint8_t checksum_a = 0;
			// uint8_t checksum_b = 0;
			
			// for ( int i = 0; i < 256; i++ ) { 
			//     // get value from most recent data log
			//     uint8_t v = datalog::get_flash_tx_buf()[i];

			//     #ifdef USE_RADIO
			//         // send value to radio buffer
			//         radio::radio_tx_buf[radio::radio_tx_buf_position++] = v;
			//     #else
			//         printf("%x,", v);
			//     #endif

			//     // calculate checksum on value
			//     checksum_a += v; 
			//     checksum_b += checksum_a; 
			// }

			// #ifdef USE_RADIO
			//     radio::radio_tx_buf[radio::radio_tx_buf_position++] = checksum_a;
			//     radio::radio_tx_buf[radio::radio_tx_buf_position++] = checksum_b;
			//     radio::radio_tx_buf[radio::radio_tx_buf_position++] = '\n';
			// #else
			//     printf("%x,", checksum_a);
			//     printf("%x", checksum_b);
			//     printf("\n");
			// #endif

			telem_idx = 0;
		
		}
		
		telem_idx++;

		// receiving telemetry data
		#ifndef USE_RADIO

			while(1) {
				int t = getchar_timeout_us(0);
				if ( t == PICO_ERROR_TIMEOUT ) { break; }
				radio::radio_rx_buf[radio::radio_rx_buf_position++] = (char)t;
			}
			
		#endif

		if ( radio::radio_rx_buf_position ) {

			// printf("packet recieved, contents:\n");
			// for ( int i = 0; i < radio::radio_rx_buf_position; i++ ) { printf("%c", radio::radio_rx_buf[i]); }
			// printf("\n\n");
			int msg_start_position = -1;

			char msg_id;
			char msg_payload;
			char msg_checksum;
			bool checksum_is_valid;

			for ( int i = 0; i < radio::radio_rx_buf_position; i++ ) {

				if ( radio::radio_rx_buf[i] == '$' ) { msg_start_position = i; }
				if ( msg_start_position != -1 ) {
					
					if ( i == msg_start_position+1 ) {
						msg_id = radio::radio_rx_buf[i]; 
						// printf("id: %c (%x)\n", msg_id, msg_id); 
					}

					if ( i == msg_start_position+2 ) {
						// if ( atoi(&radio::radio_rx_buf[i]) ) {
						// 	if ( msg_id == 'a' ) { perif::servo_1_position = atoi(&radio::radio_rx_buf[i]); }
						// 	if ( msg_id == 'b' ) { perif::servo_2_position = atoi(&radio::radio_rx_buf[i]); }
						// }
						msg_payload = radio::radio_rx_buf[i]; 
						// printf("payload: %c (%x)\n", msg_id, msg_id);
					}

					if ( i == msg_start_position+3 ) { 
						msg_checksum = radio::radio_rx_buf[i]; 
						// printf("checksum: %c (%x)\n", msg_checksum, msg_checksum);
						checksum_is_valid = (msg_checksum) == (msg_id+msg_payload);
						// printf("%s", checksum_is_valid ? "checksum is valid\n" : "checksum failed\n");
						// printf("checksum value: %x\ncalculated value: %x\n", msg_checksum, (msg_id+msg_payload));
						if ( checksum_is_valid ) {
							process_message(msg_id, msg_payload);
						}
					}
				}
			}
			radio::radio_rx_buf_position = 0;
		}
	}
}