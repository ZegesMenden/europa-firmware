#include "core.h"

#include "navigation.h"
#include "perif_ctrl.h"

#pragma once

namespace control {

    enum ctrl_mode_t {
        ctrl_none,
        ctrl_ori,
        ctrl_vel,
        ctrl_pos_vel,
    };

    ctrl_mode_t guidance_ctrl_mode = ctrl_none;

    vec3<float> setpoint_position;
    vec3<float> setpoint_velocity;
    vec3<float> setpoint_orientation;
    vec3<float> setpoint_tvc;

    vec3<float> desired_torque;
    vec3<float> tvc_position;

    float thrust;
    
    uint16_t servo_1_offset = 75;
    uint16_t servo_2_offset = 50;

    uint16_t servo_1_max = 1500;
    uint16_t servo_1_min = 1500;
    
    uint16_t servo_2_max = 1500;
    uint16_t servo_2_min = 1500;

    void clear_outputs() {
        
        setpoint_position = vec3();
        setpoint_velocity = vec3();
        setpoint_orientation = vec3();

        setpoint_tvc = vec3();
        desired_torque = vec3();

        thrust = 0;

    }

    void init() {

    }

    void update() {

        if ( !vehicle_has_control() ) {
            perif::servo_1_position = 1500 + servo_1_offset;
            perif::servo_2_position = 1500 + servo_2_offset;            
        }

        if ( ((time_us_32()/1000) % 1000) < 100) { perif::servo_2_position += 250; }
        if ( ((time_us_32()/1000) % 1000) > 200 && ((time_us_32()/1000) % 1000) < 300) { perif::servo_2_position -= 250; }
        if ( ((time_us_32()/1000) % 1000) > 400 && ((time_us_32()/1000) % 1000) < 500) { perif::servo_1_position += 250; }
        if ( ((time_us_32()/1000) % 1000) > 600 && ((time_us_32()/1000) % 1000) < 700) { perif::servo_1_position -= 250; }

    }

};