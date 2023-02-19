#include "core.h"

#ifdef DATALOG_EN

#include "drivers/flash.h"

namespace datalog {

    typedef union log_t {

        log_t() {};

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
            
            vec3<float>* position;
            vec3<float>* velocity;
            quat<float>* rotation;
            
            uint16_t* accel_x;
            uint16_t* accel_y;
            uint16_t* accel_z;

            uint16_t* ori_rate_x;
            uint16_t* ori_rate_y;
            uint16_t* ori_rate_z;

            float* baro_alt;
            float* baro_pressure;
            float* baro_temp;

            uint16_t* mag_x;
            uint16_t* mag_y;
            uint16_t* mag_z;
            
            uint32_t* gps_latitude;
            uint32_t* gps_longitude;

            uint32_t* gps_accuracy_h;
            uint32_t* gps_accuracy_v;
            uint16_t* gps_pdop;
            uint8_t* gps_n_sats;
        } points;

        void *raw[25] = {NULL};

        static_assert(sizeof(points) == sizeof(raw));
    } log_ptrs_t;

    uint16_t page = 0;
    uint8_t flash_errors = 0;
    log_ptrs_t ptrs;
    log_t data;

    bool valid_pointers() {
        for ( int i = 0; i < sizeof(ptrs.raw); i++ ) { if ( ptrs.raw[i] == NULL ) { return 0; } }
        return 1;
    }

    void log_flash_data() {

        data.points.state          = *ptrs.points.state;
        data.points.time           = *ptrs.points.time;
        data.points.flag_gpio      = *ptrs.points.flag_gpio;
        data.points.flag_state     = *ptrs.points.flag_state;

        data.points.position       = *ptrs.points.position;
        data.points.velocity       = *ptrs.points.velocity;
        data.points.rotation       = *ptrs.points.rotation;

        data.points.accel_x        = *ptrs.points.accel_x;
        data.points.accel_y        = *ptrs.points.accel_y;
        data.points.accel_z        = *ptrs.points.accel_z;

        data.points.ori_rate_x     = *ptrs.points.ori_rate_x;
        data.points.ori_rate_y     = *ptrs.points.ori_rate_y;
        data.points.ori_rate_z     = *ptrs.points.ori_rate_z;

        data.points.baro_alt       = *ptrs.points.baro_alt;
        data.points.baro_pressure  = *ptrs.points.baro_pressure;
        data.points.baro_temp      = *ptrs.points.baro_temp;

        data.points.mag_x          = *ptrs.points.mag_x;
        data.points.mag_y          = *ptrs.points.mag_y;
        data.points.mag_z          = *ptrs.points.mag_z;

        data.points.gps_latitude   = *ptrs.points.gps_latitude;
        data.points.gps_longitude  = *ptrs.points.gps_longitude;

        data.points.gps_accuracy_h = *ptrs.points.gps_accuracy_h;
        data.points.gps_accuracy_v = *ptrs.points.gps_accuracy_v;
        data.points.gps_pdop       = *ptrs.points.gps_pdop;
        data.points.gps_n_sats     = *ptrs.points.gps_n_sats;

        flash_write_page(pin_cs_flash, page, data.raw);

    }

    uint8_t *get_flash_tx_buf() { return data.raw; }

}

#endif