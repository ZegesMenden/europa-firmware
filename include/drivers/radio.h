#include "core.h"
#include "hardware/uart.h"
#include "hardware/irq.h"

#pragma once

namespace radio {

    char radio_tx_buf[1024];
    uint radio_tx_buf_position = 0;

    char radio_rx_buf[1024];
    uint radio_rx_buf_position = 0;
    
    void uart0_rx_handler() {

        while ( uart_is_readable(uart0) ) {
            radio_rx_buf[radio_rx_buf_position++] = uart_getc(uart0);
        }

    }

    bool init() {

        gpio_set_function(0, GPIO_FUNC_UART);
        gpio_set_function(1, GPIO_FUNC_UART);
        uart_init(uart0, 115200);
        uart_set_baudrate(uart0, 115200);

        uart_set_fifo_enabled(uart0, false);
        irq_set_exclusive_handler(UART0_IRQ, uart0_rx_handler);
        irq_set_enabled(UART0_IRQ, true);
        uart_set_irq_enables(uart0, true, false);

        // dma_channel_config tx_config = dma_channel_get_default_config(kUartTxChannel);
        // channel_config_set_transfer_data_size(&tx_config, DMA_SIZE_8);
        // channel_config_set_read_increment(&tx_config, true);
        // channel_config_set_write_increment(&tx_config, false);
        // channel_config_set_ring(&tx_config, false, kTxBuffLengthPow);
        // channel_config_set_dreq(&tx_config, DREQ_UART0_TX);
        // dma_channel_set_config(kUartTxChannel, &tx_config, false);
        // dma_channel_set_write_addr(kUartTxChannel, &uart0_hw->dr, false);

        return 1;

    }

    void update() {

        if ( radio_tx_buf_position ) {
            // uart_controller.write((uint8_t*)radio_tx_buf, radio_tx_buf_position);
            uart_write_blocking(uart0, (uint8_t*)radio_tx_buf, radio_tx_buf_position);
            radio_tx_buf_position = 0;
        }

    }

};