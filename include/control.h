#include "core.h"

#include "navigation.h"
#include "perif_ctrl.h"

#include "drivers/core_interface.h"
#include "simulation.h"

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

	vec3<float> target_vector;
	vec3<float> ang_acc_out;
	vec3<float> ang_acc_error;
	vec3<float> angle_out;

	vec3<float> angle_error;

	vec3<float> desired_torque;
	vec3<float> tvc_position;
	vec3<float> tvc_lever = vec3(0.26, 0.0, 0.0);
		
	float thrust;

	float simulation_energy_est;
	float simulation_work_est;
	float simulation_position_est;
	float simulation_velocity_est;
	float simulation_time_est;
	uint32_t simulation_time_taken;
	
	const uint16_t servo_neutral = 1500;

	int16_t servo_1_offset = -75;
	int16_t servo_2_offset = -75;

	uint16_t servo_1_min = 1250;
	uint16_t servo_1_max = 1650;

	uint16_t servo_2_min = 1225;
	uint16_t servo_2_max = 1625;

	pid pid_ori_y(8, 0.0, 6, 0.0);
	pid pid_ori_z(8, 0.0, 6, 0.0);

	namespace gfield {

		float burn_alt = 0;
		float energy = 0;
		float energy_potential = 0;
		float energy_kinetic = 0;
		float comp = 0;
		float velocity_at_burn_alt = 0;
		float burn_time = 0;

		float average_thrust = 10.5;
		float target_landing_alt = 0.25;

		float alt_to_start_burn = 0;

		vec3<float> target_orientation;
		float desired_force = 0;

		// percentage of the current thrust that would result in an ideal landing
		float desired_force_coefficient = 0;

		// angle to obtain desired force
		float desired_divert_vertical_ang = 0;

		// angle of horizontal course over the ground
		float lateral_course_ang = 0;

		void update() {

			// =========================================================
			// update burn alt

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

				if ( burn_alt > 12.f ) { burn_alt = 12.f; flags::control_flags::burn_alt_over_safe_thresh = true; }
				else { flags::control_flags::burn_alt_over_safe_thresh = false; }

				velocity_at_burn_alt = -sqrtf(nav::velocity.x*nav::velocity.x + 2*nav::acceleration_l.len() * ( nav::position.x - burn_alt ));
				
				if ( !isnanf(velocity_at_burn_alt) ) {

					burn_time = -velocity_at_burn_alt / (adjusted_thrust);
					if ( burn_time > 2.2 ) { comp += 0.005; }
					if ( burn_time < 1.9 ) { comp -= 0.005; }

				}

				alt_to_start_burn = burn_alt - nav::velocity.x*0.6;
				if ( nav::position.x + nav::velocity.x*0.005 < alt_to_start_burn ) { flags::control_flags::start_landing_burn = true; }

			}

			// =========================================================
			// update landing burn guidance

			if ( time_us_64() > timing::get_t_landing_burn_start() ) {

				// F = W/D
				desired_force = energy / ( nav::position.x - target_landing_alt );
				desired_force = clamp(desired_force, 0.f, 20.f);

				desired_force_coefficient = desired_force/(nav::acceleration_l.len()*nav::mass);

				// clamp to 0-25 degrees
				desired_force_coefficient = clamp(desired_force_coefficient, 0.9f, 1.0f);
				
				desired_divert_vertical_ang = acosf(desired_force_coefficient);
				// desired_divert_vertical_ang = 1.57079632679f-desired_force_coefficient;

				float horizontal_magnitude = sinf(desired_divert_vertical_ang);

				// float vertical_magnitude = cosf(desired_divert_vertical_ang);
				float vertical_magnitude = desired_divert_vertical_ang;

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

		/*

		// servo sweep code
		perif::servo_1_position = servo_neutral + servo_1_offset;
		perif::servo_2_position = servo_neutral + servo_2_offset;
		if ( ((time_us_32()/1000) % 1000) < 100 ) { perif::servo_1_position += 250; }
		if ( ((time_us_32()/1000) % 1000) > 200 && ((time_us_32()/1000) % 1000) < 300 ) { perif::servo_1_position -= 250; }

		if ( ((time_us_32()/1000) % 1000) > 550 && ((time_us_32()/1000) % 1000) < 650 ) { perif::servo_2_position += 250; }
		if ( ((time_us_32()/1000) % 1000) > 750 && ((time_us_32()/1000) % 1000) < 850 ) { perif::servo_2_position -= 250; }
		
		*/

		target_vector = vec3(1.0, 0.0, 0.0);
		
		// ================================================================
		// live simulation

		if ( get_vehicle_state() == state_powered_ascent || get_vehicle_state() == state_ascent_coast ) {

			// process previous simulation if there is one

			if ( flags::control_flags::new_ascent_sim_result ) {
				flags::control_flags::new_ascent_sim_result = false;
				
				simulation_energy_est = simulation::ascent_sim_output.energy;
				simulation_position_est = simulation::ascent_sim_output.position;
				simulation_time_est = simulation::ascent_sim_output.time;
				simulation_time_taken = simulation::ascent_sim_output.time_taken;

				simulation_work_est = 0.0;
				simulation_velocity_est = 0.0;
			}

			// prepare new simulation

			simulation::ascent_sim_input.mass = nav::mass;
			simulation::ascent_sim_input.acceleration = nav::acceleration_l.len();
			simulation::ascent_sim_input.position = nav::position.x;
			simulation::ascent_sim_input.velocity = nav::velocity.x;
			simulation::ascent_sim_input.time = float(timing::get_MET()) / 1000000.f;

			flags::control_flags::core1_communication_failure = !multicore_fifo_push_timeout_us(core1_interface::CORE0_NEW_ASCENT_SIM_INPUT, 10);

		}

		if ( get_vehicle_state() == state_descent_coast ) {

			// process previous simulation if there is one

			if ( flags::control_flags::new_landing_sim_result ) {
				flags::control_flags::new_landing_sim_result = false;
				
				simulation_work_est = simulation::landing_sim_output.work_done;
				simulation_position_est = simulation::landing_sim_output.position;
				simulation_velocity_est = simulation::landing_sim_output.velocity;
				simulation_time_est = simulation::landing_sim_output.time + float(timing::get_MET()) / 1000000.f;
				simulation_time_taken = simulation::landing_sim_output.time_taken;

				simulation_energy_est = 0.0;
			}

			// prepare new simulation

			simulation::landing_sim_input.mass = nav::mass;
			simulation::landing_sim_input.acceleration = nav::acceleration_l.len();
			simulation::landing_sim_input.position = nav::position.x;
			simulation::landing_sim_input.velocity = nav::velocity.x;
			simulation::landing_sim_input.burn_alt = gfield::burn_alt;

			flags::control_flags::core1_communication_failure = !multicore_fifo_push_timeout_us(core1_interface::CORE0_NEW_LANDING_SIM_INPUT, 10);

			gfield::update();

		}

		if ( get_vehicle_state() == state_landing_start || get_vehicle_state() == state_landing_guidance || get_vehicle_state() == state_landing_terminal ) {

			if ( flags::control_flags::new_divert_sim_result ) {
				flags::control_flags::new_divert_sim_result = false;
				
				simulation_work_est = simulation::divert_sim_output.work_done;
				simulation_position_est = simulation::divert_sim_output.position;
				simulation_velocity_est = simulation::divert_sim_output.velocity;
				simulation_time_est = simulation::divert_sim_output.time + float(timing::get_MET()) / 1000000.f;
				simulation_time_taken = simulation::divert_sim_output.time_taken;

				simulation_energy_est = 0.0;
			}

			// prepare new simulation

			simulation::divert_sim_input.mass = nav::mass;
			simulation::divert_sim_input.acceleration = nav::acceleration_l.len();
			simulation::divert_sim_input.position = nav::position.x;
			simulation::divert_sim_input.velocity = nav::velocity.x;
			simulation::divert_sim_input.time_since_burn_start = time_us_64() - timing::get_t_landing_burn_start();
			
			flags::control_flags::core1_communication_failure = !multicore_fifo_push_timeout_us(core1_interface::CORE0_NEW_DIVERT_SIM_INPUT, 10);

			gfield::update();

		}

		if ( vehicle_has_control() ) {

			// ================================================================
			// thrust estimation

			// TODO:
			// drag estimation ?

			thrust = (nav::acceleration_l.len()*nav::mass);

			// ================================================================
			// position and velocity control

			if ( control_mode == ctrl_vel ) {

			}

			// ================================================================
			// orientation control

			vec3<float> error_vector = nav::rotation.conjugate().rotate_vec(target_vector.norm());
			angle_error.y = atan2f(-error_vector.z, error_vector.x);
			angle_error.z = atan2f(error_vector.y, error_vector.x);

			ang_acc_error = nav::rotational_acceleration;
			ang_acc_error.y -= pid_ori_y.output;
			ang_acc_error.z -= pid_ori_z.output;

			pid_ori_y.updateWithDerivative(angle_error.y, ((-angle_error.y*3)-nav::rotational_velocity.y), 0.01);
			pid_ori_z.updateWithDerivative(angle_error.z, ((-angle_error.z*3)-nav::rotational_velocity.z), 0.01);

			ang_acc_out.y = pid_ori_y.output;
			ang_acc_out.z = pid_ori_z.output;           

			angle_out.y = 0.0;
			angle_out.z = 0.0;

			// cant divide by thrust if it's zero
			if ( thrust > 0.1 ) {

				pid_ori_y.output -= (ang_acc_error.y*0.25);
				pid_ori_z.output -= (ang_acc_error.z*0.25);

				angle_out.y = (pid_ori_y.output * nav::moment_of_inertia.y)/(control::thrust * control::tvc_lever.x);
				angle_out.z = (pid_ori_z.output * nav::moment_of_inertia.z)/(control::thrust * control::tvc_lever.x);
			
			} else if ( get_vehicle_state() == state_landing_start ) {
			
				// calculate angle for the peak thrust of the landing burn
				angle_out.y = (pid_ori_y.output * nav::moment_of_inertia.y)/(11.f * control::tvc_lever.x);
				angle_out.z = (pid_ori_z.output * nav::moment_of_inertia.z)/(11.f * control::tvc_lever.x);
			
			}    

			angle_out.y *= 180.f/PI;
			angle_out.z *= 180.f/PI;
			
			perif::servo_1_position = uint16_t( (angle_out.z * 37.5f) + servo_1_offset + servo_neutral);
			perif::servo_2_position = uint16_t( (angle_out.y * 37.5f) + servo_2_offset + servo_neutral);

			perif::servo_1_position = clamp(perif::servo_1_position, servo_1_min, servo_1_max);
			perif::servo_2_position = clamp(perif::servo_2_position, servo_2_min, servo_2_max);            
		
		}
	}

};