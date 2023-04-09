#include "core.h"
#include "navigation.h"
#include "drivers/radio.h"
#pragma once

namespace telemetry {

    uint16_t telem_idx = 0;

    bool init() {
        return 1;
    }

    void update() {

        if ( telem_idx == 10 ) {

            // printf("%llu,", timing::RUNTIME);
            // printf("%i,", get_vehicle_state());

            // printf("%f,", 0.0f);
            // printf("%f,", control::angle_error.y);
            // printf("%f,", control::angle_error.z);

            // printf("%f,", nav::rotational_velocity.x);
            // printf("%f,", nav::rotational_velocity.y);
            // printf("%f,", nav::rotational_velocity.z);

            // printf("%f,", nav::acceleration_l.x);
            // printf("%f,", nav::acceleration_l.y);
            // printf("%f,", nav::acceleration_l.z);

            // printf("%f,", float(timing::average_runtime)/100.f);
            // printf("%f,", nav::temperature);
            
            // printf("%f,", nav::position.x);
            // printf("%f,", nav::position.y);
            // printf("%f,", nav::position.z);

            // printf("%f,", nav::velocity.x);
            // printf("%f,", nav::velocity.y);
            // printf("%f,", nav::velocity.z);
            
            // printf("%i\n", flags::perif::pyro_sts);

            radio::radio_tx_buf_position = sprintf((radio::radio_tx_buf), "%llu,%i,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%i\n",
                    timing::RUNTIME,
                    get_vehicle_state(),
                    0.0f,
                    control::angle_error.y,
                    control::angle_error.z,
                    nav::rotational_velocity.x,
                    nav::rotational_velocity.y,
                    nav::rotational_velocity.z,
                    nav::acceleration_l.x,
                    nav::acceleration_l.y,
                    nav::acceleration_l.z,
                    float(timing::average_runtime)/100.f,
                    float(perif::voltage_batt_raw)/484.07f,
                    nav::position.x,
                    nav::position.y,
                    nav::position.z,
                    nav::velocity.x,
                    nav::velocity.y,
                    nav::velocity.z,
                    flags::perif::pyro_sts);
            
            // printf("%s", radio::radio_tx_buf);

            telem_idx = 0;
        
        }
        
        telem_idx++;
    }

}