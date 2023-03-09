#pragma once

#include <stdio.h>
#include <string.h>
#include <pico/stdlib.h>
#include <hardware/timer.h>

#include "physics.h"
#include "mmath.h"
#include "drivers/pins.h"

// ============================================================================
// general settings

// enable pyros
#define PYRO_EN

// enable datalogging
#define DATALOG_EN

// enable telemetry
#define TELEMETRY_EN

// simulate flight (SITL)
// #define SITL

// ============================================================================
// thresholds for state switches

// launch detect

// acceleration that the rocket must feel to increase the acceleration counter when in launch detect mode
constexpr static float launch_detect_accel_threshold = 12.f;

// number of acceleration readings over the acceleration threshold required to trigger launch detection
constexpr static int launch_detect_accel_count = 10;

// burnout detect

// acceleration that the rocket must be below to increase the acceleration counter when in burnout detect mode
constexpr static float burnout_detect_accel_threshold = 2.f;

// number of acceleration readings under the acceleration threshold required to trigger burnout detection
constexpr static int burnout_detect_accel_count = 15;

// apogee detect

// velocity that the rocket must be below to detect apogee
constexpr static float apogee_detect_vel_threshold = -2.f;

// ============================================================================
// NAV settings

// number of samples to take to determine gyroscope bias
constexpr static uint16_t gyro_bias_count = 400;

// number of samples to take to determine altitude offset
constexpr static uint16_t baro_bias_count = 100;

// ============================================================================
// pyro settings

uint32_t pyro_1_fire_dur_us = 0;
uint32_t pyro_2_fire_dur_us = 0;
uint32_t pyro_3_fire_dur_us = 0;

constexpr static bool pyro_1_en = false;
constexpr static bool pyro_2_en = false;
constexpr static bool pyro_3_en = false;

// ============================================================================
// global flags and variables

namespace flags {
    
    namespace state {
    
        volatile bool accel_over_ld_threshold = false;

        volatile bool accel_under_burnout_threshold = false;
        volatile bool time_over_burnout_threshold = false;

        volatile bool velocity_over_apogee_threshold = false;

        uint8_t sts_bitmap = 0;

    }

    namespace boot {

        volatile bool boot_fail = false;

    }

    namespace nav {
        
        volatile bool gyro_debiased = false;
        volatile bool baro_debiased = false;

        volatile bool gps_lock = false;
        volatile bool gps_initial_position_lock = false;

        volatile bool kalman_x_converged = false;
        volatile bool kalman_y_converged = false;
        volatile bool kalman_z_converged = false;

    }

    namespace perif {

        uint8_t gpio_sts = 0;
        uint8_t pyro_sts = 0;

        volatile bool running_from_lipo = false;
        volatile bool pyro_has_power = false;
        volatile bool switch_sts = false;

    }

}

// ============================================================================
// vehicle state declaration and getters/setters

enum system_state_t {
    // initialization of all systems
    state_boot,

    // 1st idle state
    state_idle,

    // filter convergence, sensor debiasing, gps initialization
    state_nav_init,

    // 2nd idle state
    state_launch_idle,

    // waiting for launch
    state_launch_detect,

    // ascent under thrust and guidance
    state_powered_ascent,

    // coasting to apogee
    state_ascent_coast,

    // costing to landing burn
    state_descent_coast,

    // time between motor ignition and peak thrust
    state_landing_start,

    // powered descent up until 0.25s before burnout
    state_landing_guidance,

    // final 0.25s of landing burn, nulling angular rates and preparing to land
    state_landing_terminal,

    // on the ground post-flight
    state_landed,

    // flight has been aborted
    state_abort
};

system_state_t vehicle_state = state_boot;
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

bool vehicle_has_control() {
    switch(vehicle_state) {
        case(state_boot): { return 0; }
        case(state_idle): { return 0; }
        case(state_nav_init): { return 0; }
        case(state_launch_idle): { return 0; }
        case(state_launch_detect): { return 0; }
        case(state_powered_ascent): { return 1; }
        case(state_ascent_coast): { return 0; }
        case(state_descent_coast): { return 0; }
        case(state_landing_start): { return 1; }
        case(state_landing_guidance): { return 1; }
        case(state_landing_terminal): { return 1; }
        case(state_landed): { return 0; }
        case(state_abort): { return 0; }
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
                        _port_t::port_protocol::gpio,  // protocol
                        GPIO_FUNC_SIO,                  // gpio func
                        true};                         // bitbang flag

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

template <class T>
auto clamp(const T& x, const T& min, const T& max) { return x < min ? min : (x > max ? max : x); }

void print_compile_config() {

    printf("============================================================================\n\n");
    printf("COMPILE CONFIGS\n\n");
    
    #ifdef PYRO_EN
        printf("PYRO_EN is ENABLED\n");
    #else
        printf("PYRO_EN is DISABLED\n");
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

    #ifdef SITL
        printf("SITL is ENABLED\n");
    #else
        printf("SITL is DISABLED\n");
    #endif

    #ifdef NAV_SLOW_UPDATE
        printf("NAV_UPDATE is set to SLOW (100Hz)\n");
    #else
        printf("NAV_UPDATE is set to FAST (400Hz)\n");
    #endif

    #define vname(x) #x
    printf("\n");

    printf("%s is set to %f\n", vname(launch_detect_accel_threshold), launch_detect_accel_threshold);
    printf("%s is set to %i\n", vname(launch_detect_accel_count), launch_detect_accel_count);
    printf("\n");

    printf("%s is set to %f\n", vname(burnout_detect_accel_threshold), burnout_detect_accel_threshold);
    printf("%s is set to %i\n", vname(burnout_detect_accel_count), burnout_detect_accel_count);
    printf("\n");

    printf("%s is set to %f\n", vname(apogee_detect_vel_threshold), apogee_detect_vel_threshold);
    printf("\n");

    printf("============================================================================\n");
    
}

#define boot_warning(message) printf("[WARNING] from function %s:\n%s\n", __FUNCTION__, message)
#define boot_panic(message) printf("[ERROR] from function %s:\n%s\n", __FUNCTION__, message); flags::boot::boot_fail = true;