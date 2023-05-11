#include "core.h"
#include "hardware/uart.h"
#include "hardware/irq.h"

#pragma once

namespace radio {

	char radio_tx_buf[2048];
	uint radio_tx_buf_position = 0;

	char radio_rx_buf[1024];
	uint radio_rx_buf_position = 0;
	
	void uart1_rx_handler() {

		while ( uart_is_readable(uart1) ) {
			radio_rx_buf[radio_rx_buf_position++] = uart_getc(uart1);
			if ( radio_rx_buf_position >= 1023 ) { radio_rx_buf_position = 1023; }
		}

	}

	bool init() {

		gpio_set_function(0, GPIO_FUNC_UART);
		gpio_set_function(1, GPIO_FUNC_UART);
		uart_init(uart1, 115200);
		uart_set_baudrate(uart1, 115200);

		uart_set_fifo_enabled(uart1, false);
		irq_set_exclusive_handler(UART1_IRQ, uart1_rx_handler);
		irq_set_enabled(UART1_IRQ, true);
		uart_set_irq_enables(uart1, true, false);

		return 1;

	}

	void update() {

		if ( radio_tx_buf_position ) {
			uart_write_blocking(uart1, (uint8_t*)radio_tx_buf, radio_tx_buf_position);
			radio_tx_buf_position = 0;
		}

	}

};