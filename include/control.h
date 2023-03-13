#include "core.h"

#include "navigation.h"
#include "perif_ctrl.h"

#pragma once

namespace control {

    class pid {
        public:
            float kP, kI, kD, iMax, setpoint, error, output;
            pid();
            pid(float p, float i, float d, float setPoint) { kP = p; kI = i; kD = d; setpoint = setPoint; }; // initializer
            pid(float p, float i, float d, float setPoint, float imax) { kP = p; kI = i; kD = d; iMax = imax; setpoint = setPoint; }; // initializer with integral cap

            void set_kp(float p) { kP = p; };
            void set_ki(float i) { kI = i; };
            void set_kd(float d) { kD = d; };
            void set_imax(float imax) { iMax = imax; };

            void setSetpoint(float setPoint) { setpoint = setpoint; };

            void update(float input, float dt) {
                error = setpoint - input;
                P = kP * error;
                float D = (error - lastError) * dt;
                I += error * kI * dt;
                output = P + I + D;
            }

            void updateWithDerivative(float input, float derivative, float dt) {
                error = setpoint - input;
                P = kP * error;
                I += error * kI * dt;
                output = P + (derivative*kD) + I;
            }

        private:
            float lastError, initInput, lastInput, P, I;
    };

    enum ctrl_mode_t {
        ctrl_none,
        ctrl_ori,
        ctrl_vel,
        ctrl_pos_vel,
    };

    ctrl_mode_t control_mode = ctrl_none;

    vec3<float> setpoint_position;
    vec3<float> setpoint_velocity;
    vec3<float> setpoint_orientation;
    vec3<float> setpoint_tvc;

    vec3<float> angle_error;

    vec3<float> desired_torque;
    vec3<float> tvc_position;
        
    float thrust;
    
    const uint16_t servo_neutral = 1500;

    uint16_t servo_1_offset = 75;
    uint16_t servo_2_offset = 50;

    uint16_t servo_1_min = 1300;
    uint16_t servo_1_max = 1700;

    uint16_t servo_2_min = 1300;
    uint16_t servo_2_max = 1700;

    pid pid_ori_y(0.6*(180.f/PI), 0.0, 0.2*(180.f/PI), 0.0);
    pid pid_ori_z(0.6*(180.f/PI), 0.0, 0.2*(180.f/PI), 0.0);

    namespace gfield {

        float burn_alt = 0;
        float energy = 0;
        float energy_potential = 0;
        float energy_kinetic = 0;
        float comp = 0;
        float velocity_at_burn_alt = 0;
        float burn_time = 0;

        float average_thrust = 5;

        float alt_to_start_burn = 0;

        void update() {

            float drag_est = 0.5*nav::acceleration_l.len();

            // energy calculation

            // m*g*h
            energy_potential = nav::position.x*nav::mass*9.816;
            
            // 0.5*m*v^2
            energy_kinetic = nav::velocity.len();
            energy_kinetic *= energy_kinetic;
            energy_kinetic *= nav::mass * 0.5;
        
            energy = energy_kinetic + energy_potential;

            // landing burn calculation

            float adjusted_thrust = average_thrust+comp+drag_est;

            if ( adjusted_thrust > 0.0f ) {

                // B = E/F
                burn_alt = energy/(adjusted_thrust);

                if ( burn_alt > 12.f ) { burn_alt = 12.f; flags::control::burn_alt_over_safe_thresh = true; }
                else { flags::control::burn_alt_over_safe_thresh = false; }

                velocity_at_burn_alt = -sqrtf(nav::velocity.x*nav::velocity.x + 2*nav::acceleration_l.len() * ( nav::position.x - burn_alt ));
                
                if ( !isnanf(velocity_at_burn_alt) ) {

                    burn_time = -velocity_at_burn_alt / (adjusted_thrust);
                    if ( burn_time > 2.2 ) { comp += 0.005; }
                    if ( burn_time < 1.9 ) { comp -= 0.005; }

                }

                alt_to_start_burn = burn_alt - nav::velocity.x*0.6;
                if ( nav::position.x + nav::velocity.x*0.005 < alt_to_start_burn ) { flags::control::start_landing_burn = true; }

            }
        }
    }

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
            perif::servo_1_position = servo_neutral + servo_1_offset;
            perif::servo_2_position = servo_neutral + servo_2_offset;            
        }
        
        if ( vehicle_has_control() ) {

            vec3<float> target_vector = vec3(1.0, 0.0, 0.0);
            
            // ================================================================
            // landing burn?

            if ( get_vehicle_state() == state_descent_coast ) {

            }

            // ================================================================
            // position and velocity control

            if ( control_mode == ctrl_vel ) {

                target_vector = vec3(abs(nav::velocity.x), -nav::velocity.y, -nav::velocity.z)*0.25;

                // clamp output to +- 15 degrees
                target_vector /= target_vector.x;
                target_vector.y = clamp(target_vector.y, -0.268f, 0.268f);
                target_vector.z = clamp(target_vector.z, -0.268f, 0.268f);
                
            }

            // ================================================================
            // orientation control

            target_vector = nav::rotation.conjugate().rotate_vec(target_vector.norm());
            angle_error.y = atan2f(-target_vector.z, target_vector.x);
            angle_error.z = atan2f(target_vector.y, target_vector.x);

            pid_ori_y.updateWithDerivative(angle_error.y, nav::rotational_velocity.y, 0.01);
            pid_ori_z.updateWithDerivative(angle_error.z, nav::rotational_velocity.z, 0.01);
            
            perif::servo_1_position = uint16_t( -pid_ori_y.output * 5.555555556f + \
                                                servo_1_offset + \
                                                servo_neutral);

            perif::servo_2_position = uint16_t( -pid_ori_z.output * 5.555555556f + \
                                                servo_2_offset + \
                                                servo_neutral);

            perif::servo_1_position = clamp(perif::servo_1_position, servo_1_min, servo_1_max);
            perif::servo_2_position = clamp(perif::servo_2_position, servo_2_min, servo_2_max);            

        }

    }

};