#include "core.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

#include "hardware/spi.h"
#include "hardware/uart.h"
#include "hardware/i2c.h"
#include "drivers/ws2812.pio.h"
#include "drivers/buzzer.h"

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
    
    spi_init(spi0, 8000000);

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

