#include "core.h"
#include "drivers/flash.h"
#pragma once

#ifdef DATALOG_EN

namespace datalog {

    typedef union log_t {

        log_t() {};

        struct points {

            system_state_t state;
            uint64_t time;
            uint8_t flag_gpio;
            uint8_t flag_state;

            uint16_t voltage_batt;
            uint16_t voltage_pyro;
            
            vec3<float> position;
            vec3<float> velocity;
            vec3<float> accel_bias;

            quat<float> rotation;
            
            vec3<int16_t> acceleration;
            vec3<int16_t> ori_rate;

            float baro_alt;
            float baro_pressure;
            float baro_temp;

            vec3<int16_t> mag;
            
            uint32_t gps_latitude;
            uint32_t gps_longitude;

            uint32_t gps_accuracy_h;
            uint32_t gps_accuracy_v;
            uint16_t gps_pdop;
            uint8_t gps_n_sats;

        } points;

        uint8_t raw[256];

        static_assert(sizeof(points) <= sizeof(raw));
    } log_t;

    typedef union log_ptrs_t {
        struct points {

            system_state_t* state;
            uint32_t* time;
            uint8_t* flag_gpio;
            uint8_t* flag_state;
            uint16_t* voltage_batt;
            uint16_t* voltage_pyro;
                        
            vec3<float>* position;
            vec3<float>* velocity;
            quat<float>* rotation;
            vec3<float>* accel_bias;
            
            vec3<int16_t>* acceleration;
            vec3<int16_t>* ori_rate;

            float* baro_alt;
            float* baro_pressure;
            float* baro_temp;

            vec3<int16_t>* mag;
            
            uint32_t* gps_latitude;
            uint32_t* gps_longitude;

            uint32_t* gps_accuracy_h;
            uint32_t* gps_accuracy_v;
            uint16_t* gps_pdop;
            uint8_t* gps_n_sats;

        } points;

        void *raw[22] = {NULL};

        static_assert(sizeof(points) == sizeof(raw));
    } log_ptrs_t;

    uint16_t page = 0;
    uint8_t flash_errors = 0;
    log_ptrs_t ptrs;
    log_t data;
    uint8_t log_idx = 0;

    bool valid_pointers() {
        for ( int i = 0; i < sizeof(ptrs.raw); i++ ) { if ( ptrs.raw[i] == NULL ) { return 0; } }
        return 1;
    }

    void log_flash_data() {

        data.points.state          = *ptrs.points.state;
        data.points.time           = *ptrs.points.time;
        data.points.flag_gpio      = *ptrs.points.flag_gpio;
        data.points.flag_state     = *ptrs.points.flag_state;
        data.points.voltage_batt   = *ptrs.points.voltage_batt;
        data.points.voltage_pyro   = *ptrs.points.voltage_pyro;

        data.points.position       = *ptrs.points.position;
        data.points.velocity       = *ptrs.points.velocity;
        data.points.rotation       = *ptrs.points.rotation;
        data.points.accel_bias     = *ptrs.points.accel_bias;

        data.points.acceleration   = *ptrs.points.acceleration;

        data.points.ori_rate       = *ptrs.points.ori_rate;
        data.points.baro_alt       = *ptrs.points.baro_alt;
        data.points.baro_pressure  = *ptrs.points.baro_pressure;
        data.points.baro_temp      = *ptrs.points.baro_temp;

        data.points.mag            = *ptrs.points.mag;

        data.points.gps_latitude   = *ptrs.points.gps_latitude;
        data.points.gps_longitude  = *ptrs.points.gps_longitude;

        data.points.gps_accuracy_h = *ptrs.points.gps_accuracy_h;
        data.points.gps_accuracy_v = *ptrs.points.gps_accuracy_v;
        data.points.gps_pdop       = *ptrs.points.gps_pdop;
        data.points.gps_n_sats     = *ptrs.points.gps_n_sats;

        spi_set_baudrate(spi0, 8000000);
        flash_write_page(pin_cs_flash, page, data.raw);
        spi_set_baudrate(spi0, 2000000);

    }

    uint8_t *get_flash_tx_buf() { return data.raw; }

    bool init() {

        uint8_t a, b, c;
        get_jdec(pin_cs_flash, &a, &b, &c);

        if ( a == b == c == 0 ) { return 0; }

        // erase chip?

        return 1;

    }

    void update() {

        switch(get_vehicle_state()) {
            case(state_boot): { 
                if ( ++log_idx >= 10 ) { log_flash_data(); log_idx = 0; }
                break;
            }
            case(state_idle): { 
                if ( ++log_idx >= 10 ) { log_flash_data(); log_idx = 0; }
                break;
            }
            case(state_nav_init): { 
                if ( ++log_idx >= 2 ) { log_flash_data(); log_idx = 0; }
                break;
            }
            case(state_launch_idle): { 
                if ( ++log_idx >= 5 ) { log_flash_data(); log_idx = 0; }
                break;
            }
            case(state_launch_detect): { 
                if ( ++log_idx >= 2 ) { log_flash_data(); log_idx = 0; }
                break;
            }
            case(state_powered_ascent): { 
                if ( ++log_idx >= 1 ) { log_flash_data(); log_idx = 0; }
                break;
            }
            case(state_ascent_coast): { 
                if ( ++log_idx >= 1 ) { log_flash_data(); log_idx = 0; }
                break;
            }
            case(state_descent_coast): { 
                if ( ++log_idx >= 1 ) { log_flash_data(); log_idx = 0; }
                break;
            }
            case(state_landing_start): { 
                if ( ++log_idx >= 1 ) { log_flash_data(); log_idx = 0; }
                break;
            }
            case(state_landing_guidance): { 
                if ( ++log_idx >= 1 ) { log_flash_data(); log_idx = 0; }
                break;
            }
            case(state_landing_terminal): { 
                if ( ++log_idx >= 1 ) { log_flash_data(); log_idx = 0; }
                break;
            }
            case(state_landed): { 
                if ( ++log_idx >= 10 ) { log_flash_data(); log_idx = 0; }
                break;
            }
            case(state_abort): { 
                if ( ++log_idx >= 1 ) { log_flash_data(); log_idx = 0; }
                break;
            }
            default: { 
                
                break;
            }
        }

    }

}

#endif