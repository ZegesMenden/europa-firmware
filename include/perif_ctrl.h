#include "core.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

// output drivers
#include "drivers/ws2812.pio.h"
#include "drivers/buzzer.h"
#include "drivers/pca9685.h"

// datalogging drivers
#include "drivers/flash.h"

// sensor drivers
#include "drivers/bmp581.h"
#include "drivers/lsm6dso32.h"
#include "drivers/lis2mdl.h"

// state identification

#define pin_neopixel    16
#define pin_buzzer      10

// pyros

#define pin_pyro_3_cont 18
#define pin_pyro_3_fire 19

#define pin_pyro_2_cont 20
#define pin_pyro_2_fire 21

#define pin_pyro_1_cont 22
#define pin_pyro_1_fire 23

// io

#define pin_qwiic_port0_0 14
#define pin_qwiic_port0_1 15

#define pin_qwiic_port1_0 13
#define pin_qwiic_port1_1 12

#define pin_qwiic_port2_0 9
#define pin_qwiic_port2_1 8

#define pin_qwiic_port3_0 1
#define pin_qwiic_port3_1 0

#define pin_spi0_clk 2
#define pin_spi0_sdo 3
#define pin_spi0_sdi 4

#define pin_cs_mag   5
#define pin_cs_imu   6
#define pin_cs_baro  7
#define pin_cs_flash 7
#define pin_cs_extra 17

#define pin_adc_pyro   27
#define pin_adc_switch 28
#define pin_adc_batt   29

void perif_init() {

    // gpio init

    // pyro enable pins
    gpio_init(pin_pyro_1_fire);
    gpio_put(pin_pyro_1_fire, 0);
    
    gpio_init(pin_pyro_2_fire);
    gpio_put(pin_pyro_2_fire, 0);
    
    gpio_init(pin_pyro_3_fire);
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
    #define gpio_init_with_pullup(x) gpio_init(x); gpio_set_dir(x, 1); gpio_pull_up(x)

    gpio_init_with_pullup(pin_cs_baro);
    gpio_init_with_pullup(pin_cs_imu);
    gpio_init_with_pullup(pin_cs_mag);
    gpio_init_with_pullup(pin_cs_flash);
    gpio_init_with_pullup(pin_cs_extra);
    
    // communication peripherals
    spi_init(spi0, 1000000);

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

}



