#include "core.h"

void update_sys_state() {

    system_state_t state = get_vehicle_state();

    switch(state) {
        case (state_boot) : {
            set_vehicle_state(state_idle);
            break;
        }

        case (state_idle) : {
            
            break;
        }

        case (state_nav_init) : {
            
            break;
        }

        case (state_launch_idle) : {
            
            break;
        }

        case (state_launch_detect) : {
            
            break;
        }

        case (state_powered_ascent) : {
            
            break;
        }

        case (state_ascent_coast) : {
            
            break;
        }

        case (state_descent_coast) : {
            
            break;
        }

        case (state_landing_start) : {
            
            break;
        }

        case (state_landing_guidance) : {
            
            break;
        }

        case (state_landing_terminal) : {
            
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