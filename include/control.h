#include "core.h"
#include "navigation.h"
#include "perif_ctrl.h"
#include "simulation.h"
#include "drivers/core_interface.h"

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

			void update_with_derivative(float input, float derivative, float dt) {
				error = setpoint - input;
				P = kP * error;
				I += error * kI * dt;
				output = P + (-derivative*kD) + I;
			}

			void update_with_target_derivative(float input, float derivative, float target_derivative, float dt) {
				error = setpoint - input;
				P = kP * error;
				I += error * kI * dt;
				output = P + ((target_derivative-derivative)*kD) + I;
			}

		private:
			float lastError, initInput, lastInput, P, I;
	};

	// ===========================================================================
	// setpoints

	vec3<float> setpoint_position;
	vec3<float> setpoint_velocity;
	vec3<float> setpoint_orientation;
	vec3<float> setpoint_tvc;

	// ===========================================================================
	// orientation control

	vec3<float> target_vector;
	vec3<float> angle_error;

	vec3<float> ang_acc_out;
	vec3<float> ang_acc_error;
	vec3<float> angle_out;
	
	float thrust;

	// ===========================================================================
	// simulation values

	float simulation_energy_est;
	float simulation_work_est;
	float simulation_position_est;
	float simulation_velocity_est;
	float simulation_time_est;
	uint32_t simulation_time_taken;

	// ===========================================================================
	// control coefficients

	const float kp_ascent = 12.f;
	const float ki_ascent = 0.f;
	const float kd_ascent = 8.f;
	const float target_d_coeff_ascent = 2.f;
	
	const float kp_descent = 12.f;
	const float ki_descent = 0.f;
	const float kd_descent = 8.f;
	const float target_d_coeff_descent = 7.f;
	
	// ===========================================================================
	// control variables

	vec3<float> tvc_lever = tvc_lever_ascent;
	
	pid pid_ori_y(kp_ascent, ki_ascent, kd_ascent, 0.0);
	pid pid_ori_z(kp_ascent, ki_ascent, kd_ascent, 0.0);

	namespace gfield {

		// motor charictaristics for F15
		const float average_thrust = 14.5f;
		const float motor_burn_time = 3.4f;
		const float max_target_burn_time = 3.5f;
		const float min_target_burn_time = 3.3f;
		const float ignition_delay_time = 0.25f;

		// burn alt constraints

		const float max_burn_alt = 32.f;
		const float min_burn_alt = 20.f;

		// percent of drag force to include in GFIELD calculations
		const float drag_compensation_coeff = 0.15f;
		
		// landing divert constraints

		const float max_accel_for_divert = 16.f;
		const float target_landing_alt = 0.5f;

		const float divert_min_throttle = 0.8f;
		const float divert_enable_throttle = 0.95f;

		// point between upright (0) and retrograde (1) to point when slowing down while landing
		const float divert_retrograde_coeff = 0.5f;

		// commitment percentage from no divert (0) to full target divert angle (1)
		const float throttle_divert_commitment_coeff = 1.f;

		// scalar to apply to y component of velocity direction to make sure the rocket doesnt hit me
		const float y_vel_direction_scalar = 2.0f;

		float energy = 0;
		float energy_potential = 0;
		float energy_kinetic = 0;
		
		float burn_alt = 0;
		
		float comp = 0;
		float velocity_at_burn_alt = 0;
		float burn_time = 0;
		float alt_to_start_burn = 0;

		vec3<float> target_orientation;
		float desired_accel = 0;

		// percentage of the current thrust that would result in an ideal landing
		float throttle_ratio = 0;

		// angle to obtain desired force
		float divert_angle = 0;

		void update() {

			// reset target orientation
			target_orientation = vec3<float>(1.0, 0.0, 0.0);

			// ===========================================================================
			// GFIELD burn altitude calculation

			float drag_est = drag_compensation_coeff*nav::acceleration_l.len();

			// =========================
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

				if ( burn_alt > max_burn_alt ) { burn_alt = max_burn_alt; flags::control_flags::burn_alt_over_safe_thresh = true; }
				else { flags::control_flags::burn_alt_over_safe_thresh = false; }

				if ( burn_alt < min_burn_alt ) { burn_alt = min_burn_alt; flags::control_flags::burn_alt_under_safe_thresh = true; }
				else { flags::control_flags::burn_alt_under_safe_thresh = false; }

				// velocity_at_burn_alt = -sqrtf(nav::velocity.x*nav::velocity.x + 2*nav::acceleration_l.len() * ( nav::position.x - burn_alt ));
				
				// if ( !isnanf(velocity_at_burn_alt) ) {

				// 	burn_time = -velocity_at_burn_alt / (adjusted_thrust);
				// 	if ( burn_time > max_target_burn_time ) { comp += 0.005; }
				// 	if ( burn_time < min_target_burn_time ) { comp -= 0.005; }

				// }

				alt_to_start_burn = burn_alt - nav::velocity.x*ignition_delay_time;
				if ( nav::position.x + nav::velocity.x*0.005 < alt_to_start_burn ) { flags::control_flags::start_landing_burn = true; }

			}

			// ===========================================================================
			// update landing burn guidance (starts 0.4s after ignition)

			if ( time_us_64() > (timing::get_t_landing_burn_start()+400000) ) {

				float time_to_impact = motor_burn_time-(((float)(time_us_64()-timing::get_t_landing_burn_start()))/1000000.f);

				// desired accel = -2(pos-1)/t^2 - 2(vel/t)
				desired_accel = -2.f*(nav::position.x-target_landing_alt)/(time_to_impact*time_to_impact) - 2*nav::velocity.x/time_to_impact;

				// clamp current accel as to avoid excessive diverting during thrust spike
				float accel_current = nav::acceleration_l.len() - 9.816;
				accel_current = clamp(accel_current, 0.1f, max_accel_for_divert);

				// calculate and clamp throttle ratio
				throttle_ratio = desired_accel/accel_current;
				throttle_ratio = clamp(throttle_ratio, divert_min_throttle, 1.f);
				divert_angle = acosf(throttle_ratio);

				// find lateral velocity angle and direction for diverting
				float lateral_velocity_ang = atan2(nav::velocity.y * y_vel_direction_scalar, nav::velocity.z);
				float lateral_velocity_magnitude = sqrtf(nav::velocity.y*nav::velocity.y + nav::velocity.z*nav::velocity.z);

				// calculate horizontal component of thrust and desired horizontal acceleration
				float horizontal_divert_component = sinf(divert_angle);
				float horizontal_acceleration = accel_current*horizontal_divert_component;

				// if target thrust is low enough AND there is enough time to slow down laterally, then increase lateral velocity
				if ( throttle_ratio < divert_enable_throttle && !flags::control_flags::divert_end ) {
					
					// cos(ang) = ratio because cos(ang) = cos(arccos(ratio))
					target_orientation.x = throttle_ratio;

					// move into the current velocity (increase)
					target_orientation.y = horizontal_divert_component*sinf(lateral_velocity_ang)*throttle_divert_commitment_coeff;
					target_orientation.z = horizontal_divert_component*cosf(lateral_velocity_ang)*throttle_divert_commitment_coeff;
				
				} else {
					// otherwise, point retrograde and slow down

					target_orientation = -nav::velocity;

					// ensure that the rocket always points up
					target_orientation.x = abs(target_orientation.x);

					// dont necessarily point fully retrograde
					target_orientation.y *= divert_retrograde_coeff;
					target_orientation.z *= divert_retrograde_coeff;

				}
			
			}

		}

	}

	void clear_outputs() {
		
		setpoint_position = vec3();
		setpoint_velocity = vec3();
		setpoint_orientation = vec3();
		setpoint_tvc = vec3();

		thrust = 0;

	}

	void init() {

	}

	void update() {

		/*

		// servo sweep code
		perif::servo_1_position = servo_neutral + servo_1_offset;
		perif::servo_2_position = servo_neutral + servo_2_offset;
		if ( ((time_us_32()/1000) % 1000) < 100 ) { perif::servo_1_position += 250; }
		if ( ((time_us_32()/1000) % 1000) > 200 && ((time_us_32()/1000) % 1000) < 300 ) { perif::servo_1_position -= 250; }

		if ( ((time_us_32()/1000) % 1000) > 550 && ((time_us_32()/1000) % 1000) < 650 ) { perif::servo_2_position += 250; }
		if ( ((time_us_32()/1000) % 1000) > 750 && ((time_us_32()/1000) % 1000) < 850 ) { perif::servo_2_position -= 250; }
		
		*/
	
		// ===========================================================================
		// in flight simulation

		if ( get_vehicle_state() == state_powered_ascent || get_vehicle_state() == state_ascent_coast ) {

			// process simulation data

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

			float deviation_from_up = nav::rotation.rotate_vec(vec3(1.0, 0.0, 0.0)).angle_between_vectors(vec3(1.0, 0.0, 0.0));

			simulation::ascent_sim_input.mass = nav::mass;
			simulation::ascent_sim_input.acceleration = nav::acceleration_l.len();
			simulation::ascent_sim_input.deviation_from_up = deviation_from_up;
			simulation::ascent_sim_input.position = nav::position.x;
			simulation::ascent_sim_input.velocity = nav::velocity.x;
			simulation::ascent_sim_input.time = float(timing::get_MET()) / 1000000.f;

			flags::control_flags::core1_communication_failure = !multicore_fifo_push_timeout_us(core1_interface::CORE0_NEW_ASCENT_SIM_INPUT, 10);

		}

		if ( get_vehicle_state() == state_descent_coast ) {

			// process simulation data

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

		}

		if ( get_vehicle_state() == state_landing_start || get_vehicle_state() == state_landing_guidance || get_vehicle_state() == state_landing_terminal ) {

			// process simulation data

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

		}

		// ===========================================================================
		// guidance

		// reset target orientation
		target_vector = vec3(1.0, 0.0, 0.0);

		// pitch over on ascent
		if ( timing::get_MET() > 1000000 && timing::get_MET() < 2500000 ) {
			target_vector.y = 0.05;
		}

		// ===========================================================================
		// GFIELD
		
		// update GFIELD for landing burn calculation
		if ( get_vehicle_state() == state_descent_coast ) { gfield::update(); }

		if ( get_vehicle_state() == state_landing_start || get_vehicle_state() == state_landing_guidance || get_vehicle_state() == state_landing_terminal ) {
			gfield::update();
			target_vector = gfield::target_orientation;
		}

		// ===========================================================================
		// update TVC

		if ( vehicle_has_control() ) {

			// ===========================================================================
			// thrust estimation

			thrust = (nav::acceleration_l.len()*nav::mass);

			// ===========================================================================
			// orientation control

			// calculate vector guidance error values

			vec3<float> error_vector = nav::rotation.conjugate().rotate_vec(target_vector.norm());
			angle_error.y = atan2f(-error_vector.z, error_vector.x);
			angle_error.z = atan2f(error_vector.y, error_vector.x);

			// update angular acceleration error estimates
			
			ang_acc_error = nav::rotational_acceleration;
			ang_acc_error.y -= pid_ori_y.output;
			ang_acc_error.z -= pid_ori_z.output;

			// control during ascent
			if ( get_vehicle_state() == state_powered_ascent ) {

				pid_ori_y.update_with_target_derivative(angle_error.y, nav::rotational_velocity.y, angle_error.y*2.0f, 0.01);	
				pid_ori_z.update_with_target_derivative(angle_error.z, nav::rotational_velocity.z, angle_error.z*2.0f, 0.01);

			}

			// before ignition, move the tvc mount to the angle needed when the motor comes up to thrust
			if ( get_vehicle_state() == state_landing_start ) {

			}

			// control during landing burn
			if ( get_vehicle_state() == state_landing_guidance || get_vehicle_state() == state_landing_terminal ) {

			}


			// pid_ori_y.update_with_derivative(angle_error.y, -nav::rotational_velocity.y, 0.01);
			// pid_ori_z.update_with_derivative(angle_error.z, -nav::rotational_velocity.z, 0.01);

			// ang_acc_out.y = pid_ori_y.output;
			// ang_acc_out.z = pid_ori_z.output;           

			// angle_out.y = 0.0;
			// angle_out.z = 0.0;

			// // cant divide by thrust if it's zero
			// if ( thrust > 0.1 ) {

			// 	// pid_ori_y.output -= (ang_acc_error.y*0.25);
			// 	// pid_ori_z.output -= (ang_acc_error.z*0.25);

			// 	angle_out.y = asinf(clamp((pid_ori_y.output * nav::moment_of_inertia.y)/(control::thrust * control::tvc_lever.x), -1.f, 1.f));
			// 	angle_out.z = asinf(clamp((pid_ori_z.output * nav::moment_of_inertia.z)/(control::thrust * control::tvc_lever.x), -1.f, 1.f));
			
			// } else if ( get_vehicle_state() == state_landing_start ) {
			
			// 	// calculate angle for the peak thrust of the landing burn
			// 	angle_out.y = asinf(clamp((pid_ori_y.output * nav::moment_of_inertia.y)/(11.f * control::tvc_lever.x), -1.f, 1.f));
			// 	angle_out.z = asinf(clamp((pid_ori_z.output * nav::moment_of_inertia.z)/(11.f * control::tvc_lever.x), -1.f, 1.f));
			
			// }    

			angle_out.y *= 180.f/PI;
			angle_out.z *= 180.f/PI;
			
			perif::servo_1_position = uint16_t( perif::servo_1_offset + perif::servo_neutral + (angle_out.z * 37.5f) );
			perif::servo_2_position = uint16_t( perif::servo_2_offset + perif::servo_neutral + (angle_out.y * 37.5f) );
		
		} else {
			thrust = 0.0f;
			angle_out = vec3(0.0, 0.0, 0.0);
			
			perif::servo_1_position = perif::servo_neutral + perif::servo_1_offset;
			perif::servo_2_position = perif::servo_neutral + perif::servo_2_offset;
		}

	}

};