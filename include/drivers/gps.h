#include "core.h"
#include "hardware/i2c.h"

#pragma once

namespace gps {

	uart_inst_t *inst = uart1;

	int baudrate = 115200;

	uint8_t rx_buf[2048];
	int rx_buf_position = 0;

	uint32_t utc = 0;
	char west_or_east;
	char north_or_south;

	uint8_t fix_type;
	uint8_t n_sattelites;
	int32_t ialtitude;

	uint8_t checksum;

	float lat;
	float lon;
	float alt;
	float hdop;

	bool gps_has_new_data = false;

	uint8_t unread_messages = 0;

	void uart_rx_handler() {

		while ( uart_is_readable(inst) ) {
			char c = uart_getc(inst);
			rx_buf[rx_buf_position++] = c;
			if ( rx_buf_position > 2047 ) { rx_buf_position = 2047; }
			if ( c == '$' ) { unread_messages++; }
		}

	}

	bool init() {
		#ifdef USE_GPS
		if ( !uart_is_enabled(inst) ) {
			uart_init(inst, baudrate);
		}

		uart_set_baudrate(inst, baudrate);

		uart_set_fifo_enabled(inst, false);
		irq_set_exclusive_handler(UART1_IRQ, uart_rx_handler);
		irq_set_enabled(UART1_IRQ, true);
		uart_set_irq_enables(uart1, true, false);
		#endif
		return 1;

	}

	void update() {
		#ifdef USE_GPS
		while ( unread_messages ) {
			
			uint16_t start_char = 0;
			bool has_full_sentence = false;
			for ( start_char = 0; start_char < 2048; start_char++ ) { if ( rx_buf[start_char] == '$' ) { break; } }
			for ( int i = start_char; i < 2047; i++ ) { if ( rx_buf[i] == '*' ) { has_full_sentence = true; rx_buf_position = 0; }  }
			if ( !has_full_sentence ) { break; }
			gps_has_new_data = true;
			char * NMEA_sentence = (char*)(&rx_buf[start_char]);

			unread_messages--;

			char msgid[5]; 
			int message_is_gpgga = 0;

			int n_commas = 0;
			int last_start_idx = 7;

			for ( int i = 0; i < 72; i++ ) {

				if ( i < 6 && i > 0 ) { msgid[i-1] = NMEA_sentence[i]; }
				
				if ( i > 6 ) {

					message_is_gpgga = (msgid[2] == 'G' && msgid[3] == 'G' && msgid[4] == 'A' );       

					if ( NMEA_sentence[i] == ',' ) {
						NMEA_sentence[i] = '\0';
						switch(n_commas++) {

							case(0): {
								utc = atoi(&NMEA_sentence[last_start_idx]);
								// printf("UTC: %i\n", utc);
								break;
							}case(1): {
								lat = atof(&NMEA_sentence[last_start_idx])*0.01;
								// printf("LAT: %f\n", lat);
								break;
							}case(2): {
								if ( NMEA_sentence[last_start_idx] == 'S' ) { lat *= -1; }
								break;
							}case(3): {
								lon = atof(&NMEA_sentence[last_start_idx])*0.01;
								// printf("LON: %f\n", lon);                     
								break;
							}case(4): {
								if ( NMEA_sentence[last_start_idx] == 'W' ) { lat *= -1; }
								break;
							}case(5): {
								fix_type = atoi(&NMEA_sentence[last_start_idx]);
								// printf("posfix: %i\n", fix_type);
								break;
							}case(6): {
								n_sattelites = atoi(&NMEA_sentence[last_start_idx]);
								// printf("nsats: %i\n", n_sattelites);
								break;
							}case(7): {
								hdop = atof(&NMEA_sentence[last_start_idx]);
								// printf("hdop: %f\n", hdop);    
								break;
							}case(8): {
								alt = atof(&NMEA_sentence[last_start_idx]);
								// printf("alt: %f\n", alt);    
								break;
							}

						}
						last_start_idx = i+1;
					}

				}
			}

		}
		#endif

	}

};