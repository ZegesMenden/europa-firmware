#pragma once

#include "core.h"

#include "drivers/pins.h"
#include "drivers/bmp581.h"
#include "drivers/lis2mdl.h"
#include "drivers/lsm6dso32.h"
#include "drivers/gps.h"
#include "kalman.h"

#ifdef SITL

#include "drivers/perif_ctrl.h"
#include "simulation.h"

#endif

namespace nav {

	float mass = mass_ascent_start;
	vec3<float> moment_of_inertia = moi_ascent;

	const vec3<float> gravity(9.816, 0.0, 0.0);

	quat<float> rotation;
	vec3<float> rotational_velocity;
	vec3<float> rotational_velocity_bias;
	vec3<float> rotational_acceleration;
	
	vec3<float> acceleration_l;
	vec3<float> acceleration_i;
	
	vec3<float> position;
	vec3<float> velocity;

	vec3<float> mag_field;

	vec3<float> acceleration_b;
	vec3<float> covariance_position;
	vec3<float> covariance_velocity;
	vec3<float> covariance_acceleration;

	float lat;
	float lon;

	float gps_horizontal_accuracy;
	uint8_t gps_n_sats;

	float lat_initial;
	float lon_initial;

	vec3<float> position_gps;

	uint16_t gps_n_zero_offset_measurements = 0;
	bool gps_zero_offset_lock = false;

	float pressure;
	float altitude;
	float altitude_asl;
	float temperature;

	float pad_altitude;

	float max_measured_landing_acc = 0.0;

	namespace nav_timing {
	
		uint64_t last_nav_update_us = 0;
		
		uint16_t gyro_bias_count = 0;
		uint16_t baro_bias_count = 0;
		
	}

	namespace raw {
		
		// 64 frames for now, probably wont need more
		fifo_data_t fifo_frame_buf[64];

		vec3<int16_t> fifo_gyro[64];
		uint8_t fifo_gyro_pos = 0;
		
		vec3<int16_t> fifo_accel[64];
		uint8_t fifo_accel_pos = 0;

		vec3<int32_t> gyro_bias;

		vec3<int16_t> gyro;
		vec3<int16_t> accel;
		
		uint32_t pressure;
		int32_t temperature;

		vec3<int16_t> mag;

	}

	kalman_1dof kalman_x;
	kalman_1dof kalman_y;
	kalman_1dof kalman_z;

	lsm6dso32 imu(spi0, pin_cs_imu);
	bmp581 baro(spi0, pin_cs_baro);
	lis2mdl mag(spi0, pin_cs_mag);

	// initialize sensors and configure data frequencies / sensitivity
	bool init() {

		// Q is state uncertainty
		// R is measurement uncertainty

		kalman_x.set_Q(100.0f, 1600.f, 1600.f);
		kalman_x.set_R(125000.0f, 1.0f, 1.0f);

		kalman_y.set_Q(1200.f, 800.f, 1600.f);
		kalman_y.set_R(25000.f, 1.f, 1.f);

		kalman_z.set_Q(1200.f, 800.f, 1600.f);
		kalman_z.set_R(25000.f, 1.f, 1.f);

		printf("============================================================================\nNAV init:\n\nInitializing IMU\n");
		
		if ( !imu.init() ) { boot_panic("IMU failed to initialize"); return 0; }
		printf("Done\n\n");

		printf("Configuring accel ODR/OSR\n");
		if ( !imu.set_accel_settings(0b0100, 1) ) { boot_panic("Failed to configure IMU"); return 0; }
		printf("Done\n\n");

		printf("Configuring gyro ODR/OSR\n");
		if ( !imu.set_gyro_settings(0b0110, 0b11) ) { boot_panic("Failed to configure IMU"); return 0; }
		printf("Done\n\n");

		printf("Configuring IMU FIFO\n");
		if ( !imu.set_fifo_batch_rate(0b0110, 0b0110) ) { boot_panic("Failed to configure IMU"); return 0; }
		if ( !imu.set_fifo_page_4(0, 0, 0b110) ) { boot_panic("Failed to configure IMU"); return 0; }
		printf("Done\n\n");
		
		printf("Initializing Barometer\n");
		uint64_t t_start = time_us_64();
		bool sts = 0;
		while ( time_us_64() < t_start + 5000 ) {
			if ( baro.init() ) { sts = 1; break; }
		}
		if ( !sts ) { boot_panic("Barometer failed to initialize"); return 0; }
		printf("Done\n\n");

		baro.set_reg(0x37, 0xf0);
		baro.set_reg(0x36, 0x40);
		baro.set_reg(0x37, 0x80);
		baro.set_reg(0x37, 0x81);
		
		printf("initializing mag\n");

		if ( !mag.init() ) { boot_panic("Failed to initialize mag"); return 0; }
		else { printf("done\n"); }

		printf("Done\n\n");	

		#ifdef SITL
		printf("WARNING: SITL IS ENABLED - ALL SENSOR DATA WILL BE IGNORED\n");
		#endif

		printf("============================================================================\n");

		return 1;
	};

	void update() {

		// mass tracking

		vec3<float> accel_prev = acceleration_l;

		if ( timing::get_MET() ) {

			// powered ascent
			if ( timing::get_MET() < 3400000 ) {

				float percent_burn_remaining = (float)(3400000-timing::get_MET())/(float)(3400000);
				mass = clamp((mass_ascent_start*percent_burn_remaining) + (mass_ascent_end*(1-percent_burn_remaining)), mass_ascent_end, mass_ascent_start);

			} 
			// ascent / descent coast
			else if ( timing::get_MET() >= 3400000 && !timing::get_t_landing_burn_start() ) {

				mass = mass_ascent_end;

			} 
			
			// landing
			else if ( timing::get_MET() > timing::get_t_landing_burn_start() && timing::get_t_landing_burn_start() ) {

				float percent_burn_remaining = (float)((timing::get_t_landing_burn_start()+3400000)-timing::get_MET())/(float)(3400000);
				mass = clamp((mass_descent_start*percent_burn_remaining) + (mass_descent_end*(1-percent_burn_remaining)), mass_descent_end, mass_descent_start);

			} 
			
			// post-landing
			else if ( timing::get_MET() > timing::get_t_landing_burn_start()+3400000 && timing::get_t_landing_burn_start() ) {

				mass = mass_descent_end;

			}

		}

		#ifndef SITL

		// ============================================================================
		// barometer
		
		uint32_t last_raw_pres = raw::pressure;
		baro.read_all_data(&raw::temperature, &raw::pressure);
		
		if ( raw::pressure != last_raw_pres ) {
			pressure = (float)(raw::pressure / 64.0);
			temperature = float(raw::temperature)/65536.0f;
			
			// H = 44330 * [1 - (P/p0)^(1/5.255) ]

			//H = altitude (m)
			//P = measured pressure (Pa) from the sensor
			//p0 = reference pressure at sea level (e.g. 1013.25hPa)

			// altitude_asl = ( 1.f - fast_pow(pressure/101325.f) ) * 44330.f;
			altitude_asl = ( 1.f - powf(pressure/101325.f, 1/5.255) ) * 44330.f;

			if ( !flags::nav_flags::baro_debiased && altitude_asl < 500.f) {

				pad_altitude += altitude_asl;
				nav_timing::baro_bias_count++;

				if ( nav_timing::baro_bias_count == baro_bias_count ) {
					flags::nav_flags::baro_debiased = true;
					pad_altitude /= (float)baro_bias_count;
				}

			} else {

				altitude = altitude_asl - pad_altitude;
				if ( get_vehicle_state() == state_nav_init && altitude > 0.25 ) { flags::nav_flags::baro_debiased = false; nav_timing::baro_bias_count = 0; }

				kalman_x.update_position(altitude);

			}
		}	

		// ============================================================================
		// GPS

		#ifdef USE_GPS

		if ( flags::nav_flags::gps_drdy ) {
			lat = float(gps::gps_pvt_raw.lat)*1e-7;
			lon = float(gps::gps_pvt_raw.lon)*1e-7;
			gps_horizontal_accuracy = float(gps::gps_pvt_raw.h_acc)*1e-3;
			gps_n_sats = gps::gps_pvt_raw.n_sat;
			flags::nav_flags::gps_drdy = false;
			
			if ( lat != 0.0 && lon != 0.0 && gps_n_sats > 5 ) {
				
				float rlat = lat*PI/180.f;
				float rlon = lon*PI/180.f;

				if ( gps_n_zero_offset_measurements < 100 && !gps_zero_offset_lock ) {

					gps_n_zero_offset_measurements++;

					if ( lat_initial == 0 ) { 
						lat_initial = rlat; 
						lon_initial = rlon; 
					}
					else {

						float measurement_displacement = DBPf(lat_initial/gps_n_zero_offset_measurements, lon_initial/gps_n_zero_offset_measurements, rlat, rlon);
						if ( measurement_displacement > 5 ) { 
							gps_n_zero_offset_measurements = 0;
							lat_initial = 0;
							lon_initial = 0;
						} else {
							lat_initial += rlat;
							lon_initial += rlon;
						}

					}

					if ( gps_n_zero_offset_measurements >= 100 ) {
						gps_zero_offset_lock = true;
						flags::nav_flags::gps_initial_position_lock = true;
						lat_initial /= 100.0f;
						lon_initial /= 100.0f;
					}

				} else {

					float distance_from_launchpad = DBPf(lat_initial, lon_initial, rlat, rlon);
					float angle_from_launchpad = CBPf(lat_initial, lon_initial, rlat, rlon);

					// position_gps.x = gps_alt - baro_alt_offs;
					position_gps.y = distance_from_launchpad * cosf(angle_from_launchpad);
					position_gps.z = distance_from_launchpad * sinf(angle_from_launchpad);

					kalman_y.update_position(position_gps.y);
					kalman_z.update_position(position_gps.z);
					
				}
			}
		}

		#else
		if ( get_vehicle_state() == state_nav_init || get_vehicle_state() == state_launch_idle || get_vehicle_state() == state_launch_detect ) {			
			kalman_y.update_posvel(0, 0);
			kalman_z.update_posvel(0, 0);
		}
		#endif

		// ============================================================================
		// IMU

		// get the amount of available fifo frames
		int n_fifo_frames = imu.n_fifo_frames();
		if ( n_fifo_frames > 64 ) { n_fifo_frames = 64; }

		imu.read_fifo_data(raw::fifo_frame_buf, n_fifo_frames);

		for ( int i = 0; i < n_fifo_frames; i++ ) {

			uint8_t fifo_tag = (raw::fifo_frame_buf[i].data[0]&0xf8)>>3;

			switch(fifo_tag) {

				// gyroscope data
				case(1): {
					if ( raw::fifo_gyro_pos > 64 ) { break; }
					raw::fifo_gyro[raw::fifo_gyro_pos++] = vec3<int16_t>( int16_t((raw::fifo_frame_buf[i].data[1]) | (raw::fifo_frame_buf[i].data[2]<<8)), 
																		 -int16_t((raw::fifo_frame_buf[i].data[3]) | (raw::fifo_frame_buf[i].data[4]<<8)),
																		  int16_t((raw::fifo_frame_buf[i].data[5]) | (raw::fifo_frame_buf[i].data[6]<<8)));
					break;
				}

				// accelerometer data
				case(2): {
					if ( raw::fifo_accel_pos > 64 ) { break; }
					raw::fifo_accel[raw::fifo_accel_pos++] = vec3<int16_t>( int16_t((raw::fifo_frame_buf[i].data[1]) | (raw::fifo_frame_buf[i].data[2]<<8)), 
																		   -int16_t((raw::fifo_frame_buf[i].data[3]) | (raw::fifo_frame_buf[i].data[4]<<8)),
																			int16_t((raw::fifo_frame_buf[i].data[5]) | (raw::fifo_frame_buf[i].data[6]<<8)));
					break;
				}

				default: {
					break;
				}

			}

		}

		// ================================================
		// gyroscope

		for ( int i = 0; i < raw::fifo_gyro_pos; i++ ) {

			float gyro_read_dt = 1.f/416.f;

			raw::gyro = raw::fifo_gyro[i];

			vec3<float> last_rotational_velocity = rotational_velocity;
			rotational_velocity = ((vec3<float>)(raw::gyro)) * 0.00106526443f;

			rotational_acceleration = rotational_acceleration*0.3 + (rotational_velocity-last_rotational_velocity)*0.7;

			if ( get_vehicle_state() != state_boot && get_vehicle_state() != state_idle ) {

				// sensor debiasing
				if ( get_vehicle_state() == state_nav_init ) {

					if ( !flags::nav_flags::gyro_debiased ) {
						float rotational_velocity_magnitude = rotational_velocity.len();
						
						if ( rotational_velocity_magnitude < 0.04 ) {
							rotational_velocity_bias += rotational_velocity;
							nav_timing::gyro_bias_count++;

							if ( nav_timing::gyro_bias_count >= gyro_bias_count ) {
								flags::nav_flags::gyro_debiased = true;
								rotational_velocity_bias /= float(gyro_bias_count);
							}
						} else { 
							if ( rotational_velocity_magnitude > 0.08 ) {
								nav_timing::gyro_bias_count = 0;
								rotational_velocity_bias = vec3();
							} 
						}
						
					}

				}

				// gyro integration
				if ( flags::nav_flags::gyro_debiased ) {

					rotational_velocity -= rotational_velocity_bias;
					float rotational_velocity_magnitude = rotational_velocity.len();
					
					// if there is rotation
					if ( rotational_velocity_magnitude > 0.00001f ) {

						quat<float> q = quat<float>().from_axis_angle(rotational_velocity_magnitude * gyro_read_dt, rotational_velocity/rotational_velocity_magnitude);
						rotation *= q;
						rotation = rotation.normalize();

					}

				}

			}

		}

		// ================================================
		// accelerometer

		for ( int i = 0; i < raw::fifo_accel_pos; i++ ) {

			float accel_read_dt = 1.f/104.f;

			raw::accel = raw::fifo_accel[i];
			acceleration_l = (vec3<float>)raw::accel * 0.009765625f;
			acceleration_i = rotation.rotate_vec(acceleration_l);
			acceleration_i -= gravity;

			if ( get_vehicle_state() != state_boot && get_vehicle_state() != state_idle ) {

				kalman_x.predict(acceleration_i.x, accel_read_dt);
				kalman_y.predict(acceleration_i.y, accel_read_dt);
				kalman_z.predict(acceleration_i.z, accel_read_dt);

				kalman_x.get_states(position.x, velocity.x, acceleration_b.x);
				kalman_y.get_states(position.y, velocity.y, acceleration_b.y);
				kalman_z.get_states(position.z, velocity.z, acceleration_b.z);
				
				kalman_x.get_covariances(covariance_position.x, covariance_velocity.x, covariance_acceleration.x);
				kalman_y.get_covariances(covariance_position.y, covariance_velocity.y, covariance_acceleration.y);
				kalman_z.get_covariances(covariance_position.z, covariance_velocity.z, covariance_acceleration.z);

			}

			if ( get_vehicle_state() == state_nav_init || get_vehicle_state() == state_launch_idle || get_vehicle_state() == state_launch_detect ) {

				// if accel is within +- 6 degrees of current orientation, mark orientation as converged
				float angle = acosf(clamp(acceleration_i.dot(vec3<float>(1.0, 0.0, 0.0)), -1.0f, 1.0f));
				flags::nav_flags::orientation_converged = ((1.45f < angle) & (angle < 1.68f));

				// complementary filter with accelerometer
				vec3<float> axis = acceleration_i.cross(vec3<float>(1.0, 0.0, 0.0));
				rotation = quat<float>().from_axis_angle(0.01f * angle, axis) * rotation;
				rotation = rotation.normalize();

			}

		}

		#else
		
		float current_motor_thrust = 0;

		if ( timing::get_MET() && timing::get_MET() < 3560000) {
			int idx = timing::get_MET()/10000;
			if ( idx > 356 ) { idx = 356; }
			current_motor_thrust += simulation::f15_thrust[idx];
		}

		if ( timing::get_t_landing_burn_start() && (time_us_64()-timing::get_t_landing_burn_start()) < 3560000) {
			int idx = (time_us_64()-timing::get_t_landing_burn_start())/10000;
			if ( idx > 356 ) { idx = 356; }
			current_motor_thrust += simulation::f15_thrust[idx];
		}
		
		vec3<float> tvc_angle = vec3<float>(0.0, perif::servo_2_position + ((time_us_32()&0xf0)>>4) - 8, perif::servo_1_position + (time_us_32()&0xf) - 8);
		tvc_angle.y -= perif::servo_2_offset + perif::servo_neutral;
		tvc_angle.z -= perif::servo_1_offset + perif::servo_neutral;
		tvc_angle /= 37.5f;

		vec3<float> motor_forces = quat<float>().from_eulers(0.0, tvc_angle.z*PI/180.f, tvc_angle.y*PI/180.f).rotate_vec(vec3<float>(current_motor_thrust, 0.0, 0.0));
		vec3<float> motor_torques = vec3<float>(-0.2, 0.0, 0.0).cross(motor_forces);
		rotational_acceleration = motor_torques/nav::moment_of_inertia;
		
		acceleration_i = vec3<float>(0.0, 0.0, 0.0);
		acceleration_l = vec3<float>(0.0, 0.0, 0.0);
		
		if ( velocity.len() != 0.0 ) {

			// self.aoa = velocity_relative_wind.angle_between_vectors(
            //     self.rotation.rotate(Vec3(1.0, 0.0, 0.0)))

			float aoa = velocity.angle_between_vectors(rotation.rotate_vec(vec3<float>(1.0, 0.0, 0.0)));
			if ( aoa > PI/2 ) { aoa = PI/2 - aoa; }

			// self.drag_coefficient = self.drag_coefficient_forwards + ((-np.cos(self.aoa)/2.0 + 0.5) * (self.drag_coefficient_sideways - self.drag_coefficient_forwards))
            // self.drag_force = -velocity_relative_wind.normalize() * (self.drag_coefficient/2.0 * self.air_density * self.ref_area * (velocity_relative_wind.length()*velocity_relative_wind.length()))

            // self.apply_point_force(self.drag_force, self.cp_location)
            // self.apply_torque(self.rotational_velocity * -self.friction_coeff * self.air_density)

			float drag_coeff = 0.56 + ((-cosf(aoa)/2.0 + 0.5) * (1.35-0.56));
			vec3<float> drag_force = -velocity.norm() * ( drag_coeff/2.0 * 1.225 * 0.018 * (velocity.len()*velocity.len()) );
			acceleration_i += drag_force/mass;
			// rotational_acceleration += vec3<float>(-0.2, 0.0, 0.0).cross(drag_force)/moment_of_inertia;
			rotational_acceleration -= rotational_velocity.norm()*0.001*1.225;

		}

		acceleration_i += rotation.rotate_vec(motor_forces)/mass;
		acceleration_l = rotation.conjugate().rotate_vec(acceleration_i);
		acceleration_i.x -= 9.816f;

		rotational_velocity += rotational_acceleration*0.01;

		float rotational_velocity_magnitude = rotational_velocity.len();
		if ( rotational_velocity_magnitude > 0.00001f ) {
			quat<float> q = quat<float>().from_axis_angle(rotational_velocity_magnitude * 0.01, rotational_velocity/rotational_velocity_magnitude);
			rotation *= q;
			rotation = rotation.normalize();
		}

		velocity += acceleration_i*0.01;
		position += velocity*0.01;

		if ( position.x <= 0.0 ) { 
			position.x = 0.0; 
			velocity.x = 0.0; 
			velocity.y = 0.0; 
			velocity.z = 0.0; 
			rotational_velocity.x = 0.0;
			rotational_velocity.y = 0.0;
			rotational_velocity.z = 0.0;
		}

		raw::gyro = vec3<int16_t>(rotational_velocity/0.00106526443f);
		raw::accel = vec3<int16_t>(acceleration_l/0.009765625f);
		
		#endif

		// update all flags

		if ( get_vehicle_state() == state_landing_start ) {

			float acc_cur = acceleration_l.len();
			if ( acc_cur > max_measured_landing_acc ) { max_measured_landing_acc = acc_cur; }
			if ( max_measured_landing_acc > 20.f ) { flags::state_flags::accel_over_peak_landing_thresh = true; }

			flags::state_flags::accel_decreasing = ( acc_cur < accel_prev.len() );

		}

		flags::state_flags::accel_over_ld_threshold = (get_vehicle_state() == state_launch_detect) & ( acceleration_l.x > launch_detect_accel_threshold );
		flags::state_flags::accel_under_burnout_threshold = (get_vehicle_state() == state_powered_ascent) & ( acceleration_l.x < burnout_detect_accel_threshold );
		flags::state_flags::accel_over_landing_threshold = (get_vehicle_state() == state_landing_start) & ( acceleration_l.x > landing_burn_detect_accel_threshold );

		float deviation_from_g = (acceleration_l.len());
		flags::state_flags::accel_within_landed_threshold = ( deviation_from_g < 9.816+landing_detect_accel_threshold ) & ( deviation_from_g > 9.816-landing_detect_accel_threshold );

		flags::state_flags::velocity_over_apogee_threshold = ( get_vehicle_state() == state_ascent_coast ) & ( velocity.x < apogee_detect_vel_threshold );

		float gyro_deviation_from_0 = rotational_velocity.len();
		flags::state_flags::gyro_within_landed_threshold = ( gyro_deviation_from_0 < landing_detect_ori_threshold );

		flags::state_flags::baro_below_alt_threshold = position.x < 0.25;
		flags::state_flags::velocity_below_landed_threshold = (velocity.x > -0.1) && (velocity.x < 0.1);

		raw::fifo_gyro_pos = 0;
		raw::fifo_accel_pos = 0;		

	}

}

