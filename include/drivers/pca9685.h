#include "hardware/i2c.h"
#include "hardware/gpio.h"
#pragma once 
class pca9685 {

private:

	uint8_t i2c_addr;
	i2c_inst_t *inst;

	// internal default frequency, allegedly
	uint32_t osc_freq = 25000000;
	uint8_t prescale = 0;

	int write_to_device(uint8_t addr, uint8_t data) {
		uint8_t buf[2] = { addr , data };
		int ret = i2c_write_blocking(inst, i2c_addr, buf, 2, false);
		return ret == 2;
	}

	int write_buf_to_device(uint8_t addr, uint8_t *data, int nbytes) {
		int reta = i2c_write_blocking(inst, i2c_addr, &addr, 1, false);
		int retb = i2c_write_blocking(inst, i2c_addr, data, nbytes, false);
		return (reta == 1)&(retb==nbytes);       
	}

	int write_buf_to_device(uint8_t *data, int nbytes) {
		int retb = i2c_write_blocking(inst, i2c_addr, data, nbytes, false);
		return (retb==nbytes);       
	}

	int read_from_device(uint8_t addr, uint8_t *ret) {
		int reta = i2c_write_blocking(inst, i2c_addr, &addr, 1, true);
		return (1 == i2c_read_blocking(inst, i2c_addr, ret, 1, false)) & ( reta == 1 );
	}

public:

	pca9685() {}
	pca9685(uint8_t addr, i2c_inst_t *_inst) { i2c_addr = addr; inst = _inst; }

	bool init() {

		// restart device
		return write_to_device(0x0, 0x80);
		
	}

	bool set_osc_freq(int freq) {
		return 1;
	}

	void set_pwm_freq(float freq) {
		float prescaleval = ((osc_freq / (freq * 4096.0)) + 0.5) - 1;
		if (prescaleval < 3)
			prescaleval = 3;
		if (prescaleval > 255)
			prescaleval = 255;
		uint8_t prescale = (uint8_t)prescaleval;

		uint8_t oldmode;
		read_from_device(0x0, &oldmode);

		uint8_t newmode = (oldmode & (~0x80)) | 0x10;
		write_to_device(0x00, newmode);
		write_to_device(0xfe, prescale);
		write_to_device(0x00, oldmode);
		sleep_ms(5);
		write_to_device(0x0, oldmode | 0x80 | 0x20 );
	}

	/// @brief 
	/// @param idx pin index
	/// @param t_on time for pulse to start
	/// @param t_off time for pulse to end
	/// @return 
	bool set_pin_timing_raw(uint8_t idx, uint16_t t_on, uint16_t t_off) {
		uint8_t buf[5] = { 0x06+4*idx, (uint8_t)t_on, uint8_t(t_on>>8), (uint8_t)t_off, uint8_t(t_off>>8) };
		return write_buf_to_device(buf, 5);
	}

	bool set_pin(uint8_t idx, uint16_t val) {
		val = val > 4095 ? 4095 : val;

		switch(val) {
			case(0): {
				return set_pin_timing_raw(idx, 0, 4096);
			}
			case(4095): {
				return set_pin_timing_raw(idx, 4096, 0);
			}
			default: {
				return set_pin_timing_raw(idx, 0, val);
			}
		}

	}

	bool set_pin_pwm(uint8_t idx, uint16_t duty_cycle) {
		
		// read prescalar
		uint8_t prescale;
		if ( !read_from_device(0xfe, &prescale) ) { return 0; }

		prescale ++;
		uint32_t pulselength = 10000000*prescale;
		pulselength/=osc_freq;
		uint32_t pulse = (duty_cycle*10)/pulselength;

		return set_pin_timing_raw(idx, pulse, 0);

	}

};