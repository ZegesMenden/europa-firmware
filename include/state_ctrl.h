#include "core.h"
#pragma once

namespace state {

    bool switch_has_been_low = false;

    // timers
    uint8_t accel_ld_timer = 0;
    uint8_t accel_burnout_timer = 0;
    uint8_t accel_landing_burn_timer = 0;
    uint8_t accel_landing_timer = 0;
    uint8_t velocity_apogee_timer = 0;

    // active timer being used for state switch ( for datalogging? )
    uint8_t current_state_timer = 0;
    
    void update() {

        system_state_t state = get_vehicle_state();
        current_state_timer = 0;

        switch(state) {
            case (state_boot) : {
                set_vehicle_state(state_idle);
                break;
            }

            case (state_idle) : {
                
                if ( !flags::perif::switch_sts ) { switch_has_been_low = true; }

                if ( flags::perif::switch_sts && switch_has_been_low ) {
                    switch_has_been_low = false;
                    set_vehicle_state(state_nav_init);
                }

                break;
            }

            case (state_nav_init) : {
                set_vehicle_state(state_launch_idle);
                if ( flags::nav::baro_debiased && flags::nav::gyro_debiased && flags::nav::orientation_converged ) {
                    set_vehicle_state(state_launch_idle);
                }

                break;
            }

            case (state_launch_idle) : {

                if ( !flags::perif::switch_sts ) { switch_has_been_low = true; }            
                
                if ( flags::perif::switch_sts && switch_has_been_low ) {
                    switch_has_been_low = false;
                    set_vehicle_state(state_launch_detect);
                }

                break;
            }

            case (state_launch_detect) : {
                
                if ( flags::state::accel_over_ld_threshold ) { accel_ld_timer++; }
                else { accel_ld_timer = 0; }

                if ( accel_ld_timer > 10 ) { set_vehicle_state(state_powered_ascent); }

                current_state_timer = accel_ld_timer;

                break;
            }

            case (state_powered_ascent) : {

                if ( flags::state::accel_under_burnout_threshold ) { accel_burnout_timer++; }
                else { accel_burnout_timer = 0; }

                if ( accel_burnout_timer > 25 ) { set_vehicle_state(state_ascent_coast); }

                current_state_timer = accel_burnout_timer;
                
                break;
            }

            case (state_ascent_coast) : {
                
                if ( flags::state::velocity_over_apogee_threshold ) { velocity_apogee_timer++; }
                else { velocity_apogee_timer = 0; }

                if ( velocity_apogee_timer > 10 ) { set_vehicle_state(state_descent_coast); }

                current_state_timer = velocity_apogee_timer;

                break;
            }

            case (state_descent_coast) : {
                
                if ( flags::control::start_landing_burn ) {
                    set_vehicle_state(state_landing_start);
                }

                break;
            }

            case (state_landing_start) : {
                
                if ( flags::state::accel_over_landing_threshold ) { accel_landing_burn_timer++; }
                else { accel_landing_burn_timer = 0; }

                if ( accel_landing_burn_timer > 5 ) { 
                    set_vehicle_state(state_landing_guidance); 
                    timing::set_t_landing_burn_start(time_us_64());
                }

                current_state_timer = accel_landing_burn_timer;

                break;
            }

            // landing guidance is from 0.0 to 1.8s of landing burn OR alt > 0.5m
            case (state_landing_guidance) : {
                
                if ( (time_us_64() > timing::get_t_landing_burn_start()+1700000) || ( nav::position.x < 0.5f ) ) { set_vehicle_state(state_landing_terminal); }

                break;
            }

            case (state_landing_terminal) : {
                
                if ( ( flags::state::accel_within_landed_threshold && flags::state::gyro_within_landed_threshold && ( time_us_64() > timing::get_t_landing_burn_start()+1500000 ) ) || ( time_us_64() > timing::get_t_landing_burn_start()+6000000 ) ) {
                    set_vehicle_state(state_landed);
                }

                break;
            }

            case (state_landed) : {
                
                break;
            }

            case (state_abort) : {
                
                break;
            }
        }
    }


};