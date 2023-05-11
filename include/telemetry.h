#include "core.h"
#include "navigation.h"
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
					set_vehicle_state(state_nav_init);
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
				if ( !vehicle_is_in_flight() && (vehicle_state != state_launch_detect) ) {
					datalog::start_flash_export = true;
				}
				break;
			}
			default: {
				break;
			}
		}

	}

	void update() {

		// sending telemetry data

		uint8_t telem_speed = 4;
		if ( vehicle_is_in_flight() ) { telem_speed = 2; }

		if ( telem_idx >= telem_speed ) {

			// print radio data as a string

			if ( datalog::start_flash_export ) { return; }

			#ifdef USE_RADIO
				radio::radio_tx_buf_position = sprintf((radio::radio_tx_buf+radio::radio_tx_buf_position), "%llu,%i,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%i,%f,%f,%f,%i\n",
						timing::RUNTIME,
						get_vehicle_state(),
						0.0f,
						control::angle_error.y,
						control::angle_error.z,
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
						gps::lat,
						gps::lon,
						gps::hdop,
						gps::n_sattelites);
			#else
			 
				printf("%llu,%i,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%i,%f,%f,%f,%i\n",
						timing::RUNTIME,
						get_vehicle_state(),
						0.0f,
						control::angle_error.y,
						control::angle_error.z,
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
						gps::lat,
						gps::lon,
						gps::hdop,
						gps::n_sattelites);

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

			printf("packet recieved, contents:\n");
			for ( int i = 0; i < radio::radio_rx_buf_position; i++ ) { printf("%c", radio::radio_rx_buf[i]); }
			printf("\n\n");
			int msg_start_position = -1;

			char msg_id;
			char msg_payload;
			char msg_checksum;
			bool checksum_is_valid;

			for ( int i = 0; i < radio::radio_rx_buf_position; i++ ) {

				if ( radio::radio_rx_buf[i] == '$' ) { msg_start_position = i; }
				if ( msg_start_position != -1 ) {

					if ( i == msg_start_position+1 ) { msg_id = radio::radio_rx_buf[i]; printf("id: %c (%x)\n", msg_id, msg_id); }
					if ( i == msg_start_position+2 ) { msg_payload = radio::radio_rx_buf[i]; printf("payload: %c (%x)\n", msg_id, msg_id);}
					if ( i == msg_start_position+3 ) { 
						msg_checksum = radio::radio_rx_buf[i]; 
						printf("checksum: %c (%x)\n", msg_checksum, msg_checksum);
						checksum_is_valid = (msg_checksum) == (msg_id+msg_payload);
						printf("%s", checksum_is_valid ? "checksum is valid\n" : "checksum failed\n");
						printf("checksum value: %x\ncalculated value: %x\n", msg_checksum, (msg_id+msg_payload));
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