#include "core.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/dma.h"

#pragma once

namespace radio {

	char radio_tx_buf[4096];
	uint radio_tx_buf_position = 0;

	char radio_rx_buf[1024];
	uint radio_rx_buf_position = 0;
	
	int dma_channel;

	void uart0_rx_handler() {

		while ( uart_is_readable(uart0) ) {
			radio_rx_buf[radio_rx_buf_position++] = uart_getc(uart0);
			if ( radio_rx_buf_position >= 1023 ) { radio_rx_buf_position = 1023; }
		}

	}

	bool init() {

		#ifdef USE_RADIO

		gpio_set_function(0, GPIO_FUNC_UART);
		gpio_set_function(1, GPIO_FUNC_UART);
		uart_set_baudrate(uart0, 115200);

		uart_set_fifo_enabled(uart0, false);
		irq_set_exclusive_handler(UART0_IRQ, uart0_rx_handler);
		irq_set_enabled(UART0_IRQ, true);
		uart_set_irq_enables(uart0, true, false);

		dma_channel = dma_claim_unused_channel(true);	

		if ( dma_channel == -1 ) { boot_panic("could not claim DMA channel!"); }

		dma_channel_config cfg = dma_channel_get_default_config(dma_channel);

		channel_config_set_read_increment(&cfg, true);
		channel_config_set_write_increment(&cfg, false);
		channel_config_set_dreq(&cfg, uart_get_dreq(uart0, true));
		channel_config_set_transfer_data_size(&cfg, DMA_SIZE_8);

		dma_channel_configure(	dma_channel, 
								&cfg, 
								&uart0_hw->dr,
								radio_tx_buf,
								0,
								false);

		#endif
		return 1;

	}

	void update() {

		#ifdef USE_RADIO

		if ( radio_tx_buf_position ) {

			// old code, takes too long

			// uart_write_blocking(uart0, (uint8_t*)radio_tx_buf, radio_tx_buf_position);
			// radio_tx_buf_position = 0;

			if ( !dma_channel_is_busy(dma_channel) ) {
				dma_channel_set_read_addr(dma_channel, radio_tx_buf, false);
				dma_channel_set_trans_count(dma_channel, radio_tx_buf_position, true);
			}

			radio_tx_buf_position = 0;
		}

		#endif

	}

};