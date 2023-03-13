#include "core.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

#include "hardware/spi.h"
#include "hardware/uart.h"
#include "hardware/i2c.h"
#include "drivers/ws2812.pio.h"
#include "drivers/buzzer.h"
#include "drivers/pca9685.h"
#include "drivers/servo.h"

#pragma once

// TODO
// add servo support
// verify pyro code

namespace perif {

    pca9685 pca(0b10011110, i2c0);

    uint16_t voltage_pyro_raw = 0;
    uint16_t voltage_batt_raw = 0;
    uint16_t voltage_switch_raw = 0;

    char stdio_char_buf[1024] = {0};
    int stdio_char_buf_position = 0;

    bool pyro_1_cont = false;
    bool pyro_2_cont = false;
    bool pyro_3_cont = false;
    
    bool pyro_1_fire = false;
    bool pyro_2_fire = false;
    bool pyro_3_fire = false;

    uint64_t pyro_1_fire_start = 0;
    uint64_t pyro_2_fire_start = 0;
    uint64_t pyro_3_fire_start = 0;

    uint8_t io_timing_idx = 0;

    uint8_t neopixel_timing_idx = 0;
    uint8_t neopixel_timing_freq = 0;

    uint8_t buzzer_timing_idx = 0;
    uint8_t buzzer_timing_freq = 0;
        
    uint16_t servo_1_position = 1500;
    uint16_t servo_2_position = 1500;
    
    bool pyro_1_fire_condition() {
        
        return 0;
    }

    bool pyro_2_fire_condition() {
        
        return 0;
    }

    bool pyro_3_fire_condition() {
        
        return 0;
    }

    uint32_t rgb_from_state(system_state_t state) {

        switch(state) {
            case(state_boot): { return urgb_u32(5, 5, 5); }
            case(state_idle): { return urgb_u32(5, 5, 5); }
            case(state_nav_init): { return urgb_u32(5, 5, 5); }
            case(state_launch_idle): { return urgb_u32(5, 5, 5); }
            case(state_launch_detect): { return urgb_u32(5, 5, 5); }
            case(state_powered_ascent): { return urgb_u32(5, 5, 5); }
            case(state_ascent_coast): { return urgb_u32(5, 5, 5); }
            case(state_descent_coast): { return urgb_u32(5, 5, 5); }
            case(state_landing_start): { return urgb_u32(5, 5, 5); }
            case(state_landing_guidance): { return urgb_u32(5, 5, 5); }
            case(state_landing_terminal): { return urgb_u32(5, 5, 5); }
            case(state_landed): { return urgb_u32(5, 5, 5); }
            case(state_abort): { return urgb_u32(5, 5, 5); }
            default: { return urgb_u32(5, 5, 5); }
        }

    }

    uint8_t neopixel_freq_from_state(system_state_t state) {

        switch(state) {
            case(state_boot): { return 4; }
            case(state_idle): { return 4; }
            case(state_nav_init): { return 4; }
            case(state_launch_idle): { return 4; }
            case(state_launch_detect): { return 4; }
            case(state_powered_ascent): { return 4; }
            case(state_ascent_coast): { return 4; }
            case(state_descent_coast): { return 4; }
            case(state_landing_start): { return 4; }
            case(state_landing_guidance): { return 4; }
            case(state_landing_terminal): { return 4; }
            case(state_landed): { return 4; }
            case(state_abort): { return 4; }
            default: { return 4; }
        }

    }

    uint8_t buzzer_freq_from_state(system_state_t state) {

        return neopixel_freq_from_state(state);

    }

    uint16_t buzzer_tone_from_state(system_state_t state) {

        switch(state) {
            case(state_boot): { return 500; }
            case(state_idle): { return 500; }
            case(state_nav_init): { return 500; }
            case(state_launch_idle): { return 500; }
            case(state_launch_detect): { return 500; }
            case(state_powered_ascent): { return 500; }
            case(state_ascent_coast): { return 500; }
            case(state_descent_coast): { return 500; }
            case(state_landing_start): { return 500; }
            case(state_landing_guidance): { return 500; }
            case(state_landing_terminal): { return 500; }
            case(state_landed): { return 500; }
            case(state_abort): { return 500; }
            default: { return 500; }
        }

    }

    void stdio_new_character_callback(void * ptr) {
        stdio_char_buf[stdio_char_buf_position++] = getchar();
    }

    void init() {

        // gpio init

        // pyro enable pins
        gpio_init(pin_pyro_1_fire);
        gpio_set_dir(pin_pyro_1_fire, 1);
        gpio_put(pin_pyro_1_fire, 0);
        
        gpio_init(pin_pyro_2_fire);
        gpio_set_dir(pin_pyro_2_fire, 1);
        gpio_put(pin_pyro_2_fire, 0);
        
        gpio_init(pin_pyro_3_fire);
        gpio_set_dir(pin_pyro_3_fire, 1);
        gpio_put(pin_pyro_3_fire, 0);
        
        // pyro continuity pins
        gpio_init(pin_pyro_1_cont);
        gpio_set_dir(pin_pyro_1_cont, 0);

        gpio_init(pin_pyro_2_cont);
        gpio_set_dir(pin_pyro_2_cont, 0);

        gpio_init(pin_pyro_3_cont);
        gpio_set_dir(pin_pyro_3_cont, 0);

        // ADC
        adc_init();

        adc_gpio_init(pin_adc_batt);
        adc_gpio_init(pin_adc_pyro);
        adc_gpio_init(pin_adc_switch);

        // SPI cs pins
        #define gpio_init_with_pullup(x) gpio_init(x); gpio_set_dir(x, 1); gpio_put(x, 1)

        gpio_init_with_pullup(pin_cs_baro);
        gpio_init_with_pullup(pin_cs_imu);
        gpio_init_with_pullup(pin_cs_mag);
        gpio_init_with_pullup(pin_cs_flash);
        gpio_init_with_pullup(pin_cs_extra);
        
        // communication peripherals

        gpio_init(pin_spi0_clk);
        gpio_init(pin_spi0_sdi);
        gpio_init(pin_spi0_sdo);

        gpio_set_function(pin_spi0_clk, GPIO_FUNC_SPI);
        gpio_set_function(pin_spi0_sdi, GPIO_FUNC_SPI);
        gpio_set_function(pin_spi0_sdo, GPIO_FUNC_SPI);
        
        spi_init(spi0, spi_default_baud);

        gpio_set_function(qwiic_port0.pin0, qwiic_port0.gpio_func);
        gpio_set_function(qwiic_port0.pin1, qwiic_port0.gpio_func);
        switch(qwiic_port0.protocol) {  
            case(_port_t::port_protocol::i2c_0): { i2c_init(i2c0, 400000); break; }
            case(_port_t::port_protocol::i2c_1): { i2c_init(i2c1, 400000); break; }
            case(_port_t::port_protocol::uart_0): { uart_init(uart0, 115200); break; }
            case(_port_t::port_protocol::uart_1): { uart_init(uart1, 115200); break; }
            case(_port_t::port_protocol::gpio): { break; }
        }
        
        gpio_set_function(qwiic_port1.pin0, qwiic_port1.gpio_func);
        gpio_set_function(qwiic_port1.pin1, qwiic_port1.gpio_func);
        switch(qwiic_port1.protocol) {  
            case(_port_t::port_protocol::i2c_0): { i2c_init(i2c0, 400000); break; }
            case(_port_t::port_protocol::i2c_1): { i2c_init(i2c1, 400000); break; }
            case(_port_t::port_protocol::uart_0): { uart_init(uart0, 115200); break; }
            case(_port_t::port_protocol::uart_1): { uart_init(uart1, 115200); break; }
            case(_port_t::port_protocol::gpio): { break; }
        }

        gpio_set_function(qwiic_port2.pin0, qwiic_port2.gpio_func);
        gpio_set_function(qwiic_port2.pin1, qwiic_port2.gpio_func);
        switch(qwiic_port2.protocol) {  
            case(_port_t::port_protocol::i2c_0): { i2c_init(i2c0, 400000); break; }
            case(_port_t::port_protocol::i2c_1): { i2c_init(i2c1, 400000); break; }
            case(_port_t::port_protocol::uart_0): { uart_init(uart0, 115200); break; }
            case(_port_t::port_protocol::uart_1): { uart_init(uart1, 115200); break; }
            case(_port_t::port_protocol::gpio): { break; }
        }

        gpio_set_function(qwiic_port3.pin0, qwiic_port3.gpio_func);
        gpio_set_function(qwiic_port3.pin1, qwiic_port3.gpio_func);
        switch(qwiic_port3.protocol) {  
            case(_port_t::port_protocol::i2c_0): { i2c_init(i2c0, 400000); break; }
            case(_port_t::port_protocol::i2c_1): { i2c_init(i2c1, 400000); break; }
            case(_port_t::port_protocol::uart_0): { uart_init(uart0, 115200); break; }
            case(_port_t::port_protocol::uart_1): { uart_init(uart1, 115200); break; }
            case(_port_t::port_protocol::gpio): { break; }
        }
        
        // buzzer and neopixel

        neopix_init(pin_neopixel);
        buzzer_disable(pin_buzzer);
        
        servo_init(24);
        servo_init(25);

        servo_write(24, servo_1_position);
        servo_write(25, servo_2_position);

    }

    void update() {

        // ============================================================================================
        // pyro logic

        #ifdef PYRO_EN

        pyro_1_fire = gpio_get(pin_pyro_1_cont);
        pyro_2_fire = gpio_get(pin_pyro_2_cont);
        pyro_3_fire = gpio_get(pin_pyro_3_cont);
        
        if ( !vehicle_has_launched() ) {
            gpio_put(pin_pyro_1_fire, 0);
            gpio_put(pin_pyro_2_fire, 0);
            gpio_put(pin_pyro_3_fire, 0);
        }

        if ( pyro_1_en && pyro_1_fire && time_us_64() < pyro_1_fire_start+pyro_1_fire_dur_us ) { gpio_put(pin_pyro_1_fire, 1); }
        else { gpio_put(pin_pyro_1_fire, 0); }

        if ( pyro_2_en && pyro_2_fire && time_us_64() < pyro_2_fire_start+pyro_2_fire_dur_us ) { gpio_put(pin_pyro_2_fire, 1); }
        else { gpio_put(pin_pyro_2_fire, 0); }

        if ( pyro_3_en && pyro_3_fire && time_us_64() < pyro_3_fire_start+pyro_3_fire_dur_us ) { gpio_put(pin_pyro_3_fire, 1); }
        else { gpio_put(pin_pyro_3_fire, 0); }

        if ( vehicle_is_in_flight() ) {

            if ( pyro_1_en ) {

                if ( pyro_1_fire_condition() && !pyro_1_fire ) {
                    pyro_1_fire = true;
                    pyro_1_fire_start = time_us_64();
                    gpio_put(pin_pyro_1_fire, 1);
                }

            }

            if ( pyro_2_en ) {

                if ( pyro_2_fire_condition() && !pyro_2_fire ) {
                    pyro_2_fire = true;
                    pyro_2_fire_start = time_us_64();
                    gpio_put(pin_pyro_2_fire, 1);
                }

            }

            if ( pyro_3_en ) {

                if ( pyro_3_fire_condition() && !pyro_3_fire ) {
                    pyro_3_fire = true;
                    pyro_3_fire_start = time_us_64();
                    gpio_put(pin_pyro_3_fire, 1);
                }

            }

        }

        #endif

        // ============================================================================================
        // ADC logic

        adc_select_input(1);
        voltage_pyro_raw = adc_read();

        adc_select_input(2);
        voltage_batt_raw = adc_read();

        adc_select_input(3);
        voltage_switch_raw = adc_read();

        flags::perif::switch_sts = voltage_switch_raw > 3500;

        // ============================================================================================
        // neopixel and buzzer

        io_timing_idx++;
        if ( io_timing_idx > 100 ) { io_timing_idx = 0; }

        uint8_t neopix_freq = neopixel_freq_from_state(get_vehicle_state());
        bool write_neopix = 0;
        for ( int i = 0; i < neopix_freq; i++ ) {
            // if time is greater than start time and less than end time for any blink
            write_neopix |= ( io_timing_idx > (100/neopix_freq)*i && io_timing_idx < ((100/neopix_freq)*i)+10 );
        }

        if (write_neopix) { neopix_write(rgb_from_state(get_vehicle_state())); }
        else { neopix_write(0); }

        uint8_t buzzer_freq = buzzer_freq_from_state(get_vehicle_state());
        bool buzzer_en = 0;
        for ( int i = 0; i < buzzer_freq; i++ ) {
            // if time is greater than start time and less than end time for any blink
            buzzer_en |= ( io_timing_idx > (100/buzzer_freq)*i && io_timing_idx < ((100/buzzer_freq)*i)+10 );
        }

        if (buzzer_en) { buzzer_tone(pin_buzzer, buzzer_tone_from_state(get_vehicle_state())); }
        else { buzzer_disable(pin_buzzer); }
       
        // ============================================================================================
        // servos

        servo_write(24, servo_1_position);
        servo_write(25, servo_2_position);

    }

};