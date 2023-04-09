#include "core.h"
#include "drivers/flash.h"
#pragma once



namespace datalog {

    #pragma pack(1)

    typedef union log_t {

        log_t() {};

        struct points {

            system_state_t state;
            uint64_t time;
            uint8_t flag_gpio;
            uint8_t flag_state;
            uint8_t active_state_timer;

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
            
            float burn_alt;
            float comp;
            float simulation_energy_est;
            float simulation_work_est;
            float simulation_position_est;
            float simulation_velocity_est;
            float simulation_time_est;

        } points;

        uint8_t raw[256];

        static_assert(sizeof(points) <= sizeof(raw));

    } log_t;

    typedef union log_ptrs_t {
        struct points {

            system_state_t* state;
            uint64_t* time;
            uint8_t* flag_gpio;
            uint8_t* flag_state;
            uint8_t* active_state_timer;

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

            float* burn_alt;
            float* comp;
            float* simulation_energy_est;
            float* simulation_work_est;
            float* simulation_position_est;
            float* simulation_velocity_est;
            float* simulation_time_est;

        } points;

        void *raw[24] = {NULL};

        static_assert(sizeof(points) == sizeof(raw));
    } log_ptrs_t;

    #pragma pop()

    uint16_t start_page = 0;
    uint16_t page = 0;
    uint8_t flash_errors = 0;
    log_ptrs_t ptrs;
    log_t data;
    uint8_t log_idx = 0;

    uint16_t find_flash_start() {

        #ifdef DATALOG_EN

        bool last_page_was_valid = 0;
        uint8_t read_buf[256];
        for ( int i = 0; i < 256; i++ ) { read_buf[i] = 0; }

        spi_set_baudrate(spi0, 8000000);

        for ( int i = 0; i < 15625; i++ ) {
            
            while(flash_busy(pin_cs_flash)) { ; }
            if (!flash_read_page(pin_cs_flash, i, read_buf) ) { return -1; }

            page = i;
            start_page = i;
            if ( read_buf[0] == 0xff ) { 
                if ( last_page_was_valid ) { return i; }
                last_page_was_valid = true;
            }
            
        }
        
        spi_set_baudrate(spi0, spi_default_baud);

        return page;

        #endif

        return 0;

    }

    bool valid_pointers() {
        #ifdef DATALOG_EN
        for ( int i = 0; i < sizeof(ptrs.raw); i++ ) { if ( ptrs.raw[i] == NULL ) { return 0; } }
        #endif
        return 1;
    }

    void log_flash_data() {
        
        #ifdef DATALOG_EN

        data.points.state                   = *ptrs.points.state;
        data.points.time                    = *ptrs.points.time;
        data.points.flag_gpio               = *ptrs.points.flag_gpio;
        data.points.flag_state              = *ptrs.points.flag_state;
        data.points.active_state_timer      = *ptrs.points.active_state_timer;
        data.points.voltage_batt            = *ptrs.points.voltage_batt;
        data.points.voltage_pyro            = *ptrs.points.voltage_pyro;

        data.points.position                = *ptrs.points.position;
        data.points.velocity                = *ptrs.points.velocity;
        data.points.rotation                = *ptrs.points.rotation;
        data.points.accel_bias              = *ptrs.points.accel_bias;

        data.points.acceleration            = *ptrs.points.acceleration;

        data.points.ori_rate                = *ptrs.points.ori_rate;
        data.points.baro_alt                = *ptrs.points.baro_alt;
        data.points.baro_pressure           = *ptrs.points.baro_pressure;
        data.points.baro_temp               = *ptrs.points.baro_temp;

        data.points.mag                     = *ptrs.points.mag;

        data.points.burn_alt                = *ptrs.points.burn_alt;
        data.points.comp                    = *ptrs.points.comp;
        data.points.simulation_energy_est   = *ptrs.points.simulation_energy_est;
        data.points.simulation_work_est     = *ptrs.points.simulation_work_est;
        data.points.simulation_position_est = *ptrs.points.simulation_position_est;
        data.points.simulation_velocity_est = *ptrs.points.simulation_velocity_est;
        data.points.simulation_time_est     = *ptrs.points.simulation_time_est;

        spi_set_baudrate(spi0, 6000000);
        flash_write_page(pin_cs_flash, page++, data.raw);
        spi_set_baudrate(spi0, spi_default_baud);

        #endif

    }

    uint8_t *get_flash_tx_buf() { return data.raw; }

    bool init() {
        #ifdef DATALOG_EN
        uint8_t a = 0;
        uint8_t b = 0;
        uint8_t c = 0;
        get_jdec(pin_cs_flash, &a, &b, &c);

        // if ( a == b == c == 0 ) { return 0; }

        printf("============================================================================\n");
        printf("locating flash start page...\n");

        spi_set_baudrate(spi0, 8000000);      
        flash_erase_chip(pin_cs_flash);
        while(flash_busy(pin_cs_flash)) {;}
        find_flash_start();

        printf("start page: %i\nremaining pages: %i (%.1f%c / %.1fs)\n", page, (15625-page), 100.f*(float(page)/float(15625)), '%', float(15625-page)/100.f);
        printf("flash storage: [");
        for ( int i = 0; i < 20; i++ ) {
            if ( i*5 < (100.f*(float(page)/float(15625))) ) { printf("="); }
            else { printf(" "); }
        }
        printf("]\n");
        printf("============================================================================\n");

        spi_set_baudrate(spi0, spi_default_baud);
        return 1;
        #endif
        return 1;
    }

    void update() {
        #ifdef DATALOG_EN
        switch(get_vehicle_state()) {
            case(state_boot): { 
                break;
            }
            case(state_idle): { 
                break;
            }
            case(state_nav_init): { 
                if ( ++log_idx >= 10 ) { log_flash_data(); log_idx = 0; }
                break;
            }
            case(state_launch_idle): { 
                if ( ++log_idx >= 10 ) { log_flash_data(); log_idx = 0; }
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
                if ( ++log_idx >= 2 ) {

                    // only log for 5 seconds after landing
                    if ( time_us_64() < timing::get_t_landing()+5000000 ) { log_flash_data(); }
                    log_idx = 0;

                }
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
        #endif
    }

    void export_flash_data(uint16_t page_start) {
        #ifdef DATALOG_EN
        printf("FLASHDATA\n");

        printf("nlines%i\n", page-page_start);

        for ( int i = page_start; i < page; i++ ) {

            flash_read_page(pin_cs_flash, i, data.raw);

            uint8_t checksum = 0;

            printf("%i,", data.points.state);
            printf("%i,", data.points.time);
            printf("%i,%i,%i,%i,%i,%i,%i,%i,", (data.points.flag_gpio>>7)&1, (data.points.flag_gpio>>6)&1, (data.points.flag_gpio>>5)&1, (data.points.flag_gpio>>4)&1, (data.points.flag_gpio>>3)&1, (data.points.flag_gpio>>2)&1, (data.points.flag_gpio>>1)&1, (data.points.flag_gpio)&1);
            printf("%i,%i,%i,%i,%i,%i,%i,%i,", (data.points.flag_state>>7)&1, (data.points.flag_state>>6)&1, (data.points.flag_state>>5)&1, (data.points.flag_state>>4)&1, (data.points.flag_state>>3)&1, (data.points.flag_state>>2)&1, (data.points.flag_state>>1)&1, (data.points.flag_state)&1);
            printf("%i,", data.points.active_state_timer);
            printf("%f,", float(data.points.voltage_batt)/484.07f);
            printf("%f,", float(data.points.voltage_pyro)/484.07f);

            printf("%f,%f,%f,", data.points.position.x, data.points.position.y, data.points.position.z);
            printf("%f,%f,%f,", data.points.velocity.x, data.points.velocity.y, data.points.velocity.z);
            printf("%f,%f,%f,%f,", data.points.rotation.w, data.points.rotation.x, data.points.rotation.y, data.points.rotation.z);
            printf("%f,%f,%f,", data.points.accel_bias.x, data.points.accel_bias.y, data.points.accel_bias.z);
            printf("%f,%f,%f,", data.points.acceleration.x*0.009765625f, data.points.acceleration.y*0.009765625f, data.points.acceleration.z*0.009765625f);
            printf("%f,%f,%f,", data.points.ori_rate.x*0.00106526443f, data.points.ori_rate.y*0.00106526443f, data.points.ori_rate.z*0.00106526443f);

            printf("%f,", data.points.baro_alt);
            printf("%f,", data.points.baro_pressure);
            printf("%f,", data.points.baro_temp);

            printf("%f,%f,%f,", float(data.points.mag.x), float(data.points.mag.y), float(data.points.mag.z));

            printf("%f,", data.points.burn_alt);
            printf("%f,", data.points.comp);
            printf("%f,", data.points.simulation_energy_est);
            printf("%f,", data.points.simulation_work_est);
            printf("%f,", data.points.simulation_position_est);
            printf("%f,", data.points.simulation_velocity_est);
            printf("%f\n", data.points.simulation_time_est);
        };
        #endif
    }

}
