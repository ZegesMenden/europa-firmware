#pragma once

#include "core.h"

#ifdef DATALOG_EN

union datalog {
    struct points {

        system_state_t state;
        uint32_t time;
        uint8_t flag_gpio;
        uint8_t flag_state;
        
        vec3<float> position;
        vec3<float> velocity;
        quat<float> rotation;
        
        uint16_t accel_x;
        uint16_t accel_y;
        uint16_t accel_z;

        uint16_t ori_rate_x;
        uint16_t ori_rate_y;
        uint16_t ori_rate_z;

        float baro_alt;
        float baro_pressure;
        float baro_temp;

        uint16_t mag_x;
        uint16_t mag_y;
        uint16_t mag_z;
        
        uint32_t gps_latitude;
        uint32_t gps_longitude;

        uint32_t gps_accuracy_h;
        uint32_t gps_accuracy_v;
        uint16_t gps_pdop;
        uint8_t gps_n_sats;

    };

    uint8_t raw[256];

    static_assert(sizeof(points) <= sizeof(raw));
};

struct datalog_ptrs {
    uint32_t* time;
    system_state_t* state;
    uint8_t* flag_gpio;
    uint8_t* flag_state;
    
    vec3<float>* position;
    vec3<float>* velocity;
    quat<float>* rotation;
    
    uint16_t* accel_x;
    uint16_t* accel_y;
    uint16_t* accel_z;

    uint16_t* ori_rate_x;
    uint16_t* ori_rate_y;
    uint16_t* ori_rate_z;

    uint16_t* mag_x;
    uint16_t* mag_y;
    uint16_t* mag_z;
};

#endif