// ================================================
// low level ws25qXX driver for RP2040 written in c
// ================================================
#pragma once
#include <hardware/spi.h>
#include <hardware/dma.h>
#include <hardware/gpio.h>
#include <hardware/flash.h>
#include <pico/multicore.h>

enum flash_commands {
  write_enable    = 0x06,
  write_disable   = 0x04,
  chip_erase      = 0xc7,
  alt_chip_erase  = 0x60,
  status_reg_1    = 0x05,
  read_data       = 0x03,
  read_data_fast  = 0x0b,
  page_program    = 0x02,
  jdec_id         = 0x9f
} flash_commands;

int flash_send_byte(uint8_t data) {
	return spi_write_blocking(spi0, &data, 1);
}

bool flash_busy(int cs) {

	#ifndef USE_INTERNAL_FLASH

	gpio_put(cs, 1);
	gpio_put(cs, 0);

	flash_send_byte(status_reg_1);
	uint8_t buf;
	spi_read_blocking(spi0, 0, &buf, 1);

	gpio_put(cs, 1);
	
	return (buf&1);

	#endif

	return 0;

}

bool get_jdec(uint cs, uint8_t *b1, uint8_t *b2, uint8_t *b3) {
	
	#ifndef USE_INTERNAL_FLASH

	if (flash_busy(cs)) {return 0;}

	gpio_put(cs, 1);
	gpio_put(cs, 0);

	flash_send_byte(jdec_id);

	spi_read_blocking(spi0, 0, b1, 1);
	spi_read_blocking(spi0, 0, b2, 1);
	spi_read_blocking(spi0, 0, b3, 1);
	
	gpio_put(cs, 1);

	#endif
	
	return 1;

}

bool get_sreg(uint cs, uint8_t *b1, uint8_t *b2) {
	
	#ifndef USE_INTERNAL_FLASH

	if (flash_busy(cs)) {return 0;}

	gpio_put(cs, 1);
	gpio_put(cs, 0);
	
	flash_send_byte(write_enable);

	gpio_put(cs, 1);
	gpio_put(cs, 0);

	flash_send_byte(0x05);
	spi_read_blocking(spi0, 0, b1, 1);

	gpio_put(cs, 1);
	gpio_put(cs, 0);

	flash_send_byte(0x35);
	spi_read_blocking(spi0, 0, b2, 1);

	gpio_put(cs, 1);

	#endif
	
	return 1;

}

bool flash_write_page(uint cs, uint16_t page_number, uint8_t *buf) {
	
	#ifndef USE_INTERNAL_FLASH

	while(flash_busy(cs)) {;}

	uint64_t t_start = time_us_64();

	while(1) {
		uint8_t t;

		gpio_put(cs, 1);
		gpio_put(cs, 0);
		
		flash_send_byte(write_enable);

		gpio_put(cs, 1);
		gpio_put(cs, 0);

		flash_send_byte(0x05);
		spi_read_blocking(spi0, 0, &t, 1);

		gpio_put(cs, 1);

		if ( (t&2) == 2) {
			break;
		}

		if ( time_us_64() > t_start+1000 ) { return 0; }

	}

	gpio_put(cs, 0);
	flash_send_byte(page_program);
	flash_send_byte((page_number>>8) & 0xff);
	flash_send_byte(page_number & 0xff);
	flash_send_byte(0);
		
	int err = spi_write_blocking(spi0, buf, 256);

	gpio_put(cs, 1);

	return err > 1 ? 1 : 0;

	#else

	multicore_lockout_start_blocking();
	uint32_t ints = save_and_disable_interrupts();
	flash_range_program( (1024 * 1024) + page_number*256, buf, 256);
	restore_interrupts(ints);
	multicore_lockout_end_blocking();

	return 1;

	#endif

}

// poggers DMA??
bool flash_dma_write_page(uint cs, uint16_t page_number, uint8_t *buf) {

	#ifndef USE_INTERNAL_FLASH

	while(flash_busy(cs)) {;}

	// attempt to claim a channel
	int dma_chan = dma_claim_unused_channel(true);
	// if ( dma_chan == -1 ) { printf("uh oh"); }
	dma_channel_config cfg = dma_channel_get_default_config(dma_chan);
	channel_config_set_dreq(&cfg, DREQ_SPI0_TX);
	channel_config_set_transfer_data_size(&cfg, DMA_SIZE_8);
	channel_config_set_read_increment(&cfg, true);
	channel_config_set_write_increment(&cfg, false);
	
	dma_channel_configure(	dma_chan,     			// dma channel 
							&cfg,					// config value?
							&spi_get_hw(spi0)->dr,	// TO here
							buf,					// FROM here
							256,					// number of bytes to transfer?
							false);					// dont immediately start
	
	gpio_put(cs, 1);
	gpio_put(cs, 0);
	
	flash_send_byte(write_enable);

	gpio_put(cs, 1);
	gpio_put(cs, 0);

	flash_send_byte(page_program);
	flash_send_byte((page_number>>8) & 0xff);
	flash_send_byte(page_number & 0xff);
	flash_send_byte(0);
	
	spi0_hw->dmacr = 2;
	dma_channel_start(dma_chan);

	// for debugging
	dma_channel_wait_for_finish_blocking(dma_chan);

	gpio_put(cs, 1);	

	return 1;

	#endif

	return 1;

}

bool flash_read_page(uint cs, uint16_t page_number, uint8_t *buf) {

	#ifndef USE_INTERNAL_FLASH

  	while(flash_busy(cs)) {;}

	gpio_put(cs, 1);
	gpio_put(cs, 0);

	flash_send_byte(read_data);
	flash_send_byte((page_number>>8) & 0xff);
	flash_send_byte(page_number & 0xff);
	flash_send_byte(0);

	int err = spi_read_blocking(spi0, 0, buf, 256);

	gpio_put(cs, 1);

	return err > 1 ? 1 : 0;

	#else

	// multicore_lockout_start_blocking();
	// uint32_t ints = save_and_disable_interrupts();

	uint8_t* ptr = (uint8_t*)(XIP_BASE + ( (1024 * 512) + page_number*256));

	for ( int i = 0; i < 256; i++ ) {
		buf[i] = ptr[i];
	}

	// memcpy(buf, (uint8_t*) ( XIP_BASE + ( (1024 * 512) + page_number*256) ), 256);
	
	// restore_interrupts(ints);
	// multicore_lockout_end_blocking();

	return 1;

	#endif

}

bool flash_read_bytes(uint cs, uint16_t page_number, uint8_t *buf, uint n_bytes) {

	#ifndef USE_INTERNAL_FLASH

  	while(flash_busy(cs)) {;}

	gpio_put(cs, 1);
	gpio_put(cs, 0);

	flash_send_byte(read_data);
	flash_send_byte((page_number>>8) & 0xff);
	flash_send_byte(page_number & 0xff);
	flash_send_byte(0);

	int err = spi_read_blocking(spi0, 0, buf, n_bytes);

	gpio_put(cs, 1);

	return err > 1 ? 1 : 0;

	#endif

	return 1;

}

bool flash_erase_chip(uint cs) {

	#ifndef USE_INTERNAL_FLASH

	// if ( flash_busy(cs) ) {return 0;}
	while(flash_busy(cs)) {;}

	gpio_put(cs, 1);
	gpio_put(cs, 0);

	flash_send_byte(write_enable);

	gpio_put(cs, 1);
	gpio_put(cs, 0);

	flash_send_byte(chip_erase);

	gpio_put(cs, 1);

	// while(flash_busy(cs)) {;}
	// gpio_put(cs, 0);

	// flash_send_byte(WB_WRITE_DISABLE);

	// gpio_put(cs, 1);

	#else

	multicore_lockout_start_blocking();
	uint32_t ints = save_and_disable_interrupts();

	// erase 3MB of flash (0.5 - 3.5MB of 4 total)
	flash_range_erase((1024 * 512), 1024*1024*1);

	restore_interrupts(ints);
	multicore_lockout_end_blocking();

	#endif

	return 1;

}