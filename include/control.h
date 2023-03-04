#include "core.h"

namespace control {

    enum guidance_ctrl_mode_t {
        ctrl_none,
        ctrl_ori,
        ctrl_vel,
        ctrl_pos_vel,
    };

    guidance_ctrl_mode_t guidance_ctrl_mode = ctrl_none;

    vec3<float> setpoint_position;
    vec3<float> setpoint_velocity;
    vec3<float> setpoint_orientation;
    
        

};