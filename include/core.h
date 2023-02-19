#pragma once

#include <stdio.h>
#include <string.h>
#include <pico/stdlib.h>

#include "physics.h"
#include "mmath.h"
#include "drivers/pins.h"

// ============================================================================
// compile-time settings

// enable pyros
#define PYRO_FIRE_EN

// enable datalogging
#define DATALOG_EN

// enable telemetry
#define TELEMETRY_EN

// ============================================================================
// global flags and variables

// vehicle state declaration and getters/setters

enum system_state_t {
    state_boot,
    state_idle,
    state_nav_init,
    state_launch_idle,
    state_launch_detect,
    state_powered_ascent,
    state_ascent_coast,
    state_descent_coast,
    state_landing_start,
    state_landing_guidance,
    state_landing_terminal,
    state_landed,
    state_abort
};

system_state_t vehicle_state;
system_state_t get_vehicle_state() { return vehicle_state; }
void set_vehicle_state(system_state_t state) { vehicle_state = state; } 

bool vehicle_is_in_flight() {
    switch(vehicle_state) {
        case(state_boot): { return 0; }
        case(state_idle): { return 0; }
        case(state_nav_init): { return 0; }
        case(state_launch_idle): { return 0; }
        case(state_launch_detect): { return 0; }
        case(state_powered_ascent): { return 1; }
        case(state_ascent_coast): { return 1; }
        case(state_descent_coast): { return 1; }
        case(state_landing_start): { return 1; }
        case(state_landing_guidance): { return 1; }
        case(state_landing_terminal): { return 1; }
        case(state_landed): { return 0; }
        case(state_abort): { return 0; }
        default: { return 0; }
    }
}

bool vehicle_has_launched() {
    switch(vehicle_state) {
        case(state_boot): { return 0; }
        case(state_idle): { return 0; }
        case(state_nav_init): { return 0; }
        case(state_launch_idle): { return 0; }
        case(state_launch_detect): { return 0; }
        case(state_powered_ascent): { return 1; }
        case(state_ascent_coast): { return 1; }
        case(state_descent_coast): { return 1; }
        case(state_landing_start): { return 1; }
        case(state_landing_guidance): { return 1; }
        case(state_landing_terminal): { return 1; }
        case(state_landed): { return 1; }
        case(state_abort): { return 1; }
        default: { return 0; }
    }
}

// peripheral bus flags and config

struct _port_t {

    // pins used (scl/sda, tx/rx, etc.)
    uint pin0;
    uint pin1;

    // protocol (and protocol instance) being used by the port ( i2c0, i2c1, uart0, uart1, etc. )
    enum port_protocol {
        i2c_0,
        i2c_1,
        uart_0,
        uart_1,
        gpio
    } protocol;

    // gpio function to be used when initializing the port
    gpio_function gpio_func;
    
    // flag indicate if the communication protocol is being bit-banged or controlled by a peripheral
    bool bitbang;

};

_port_t qwiic_port0 = { 14,                             // pin0
                        15,                             // pin1
                        _port_t::port_protocol::i2c_1,  // protocol
                        GPIO_FUNC_I2C,                  // gpio func
                        false};                         // bitbang flag

_port_t qwiic_port1 = { 13,                             // pin0
                        12,                             // pin1
                        _port_t::port_protocol::i2c_0,  // protocol
                        GPIO_FUNC_I2C,                  // gpio func
                        false};                         // bitbang flag

_port_t qwiic_port2 = { 9,                              // pin0
                        8,                              // pin1
                        _port_t::port_protocol::uart_1, // protocol
                        GPIO_FUNC_UART,                 // gpio func
                        false};                         // bitbang flag

_port_t qwiic_port3 = { 1,                              // pin0
                        0,                              // pin1
                        _port_t::port_protocol::uart_0, // protocol
                        GPIO_FUNC_UART,                 // gpio func
                        false};                         // bitbang flag

// ============================================================================
// functions that dont really belong anywhere else

void print_compile_config() {

    printf("============================================================================\n");
    printf("COMPILE CONFIGS\n\n");

    #ifdef PYRO_FIRE_EN
        printf("PYRO_FIRE_EN is ENABLED\n");
    #else
        printf("PYRO_FIRE_EN is DISABLED\n");
    #endif

    #ifdef DATALOG_EN
        printf("DATALOG_EN is ENABLED\n");
    #else
        printf("DATALOG_EN is DISABLED\n");
    #endif
    
    #ifdef TELEMETRY_EN
        printf("TELEMETRY_EN is ENABLED\n");
    #else
        printf("TELEMETRY_EN is DISABLED\n");
    #endif

    printf("\n");
    printf("============================================================================\n");
    
}