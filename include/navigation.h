#include "core.h"

#include "drivers/pins.h"
#include "drivers/bmp581.h"
#include "drivers/lis2mdl.h"
#include "drivers/lsm6dso32.h"

#pragma once

namespace nav {

	class kalman_1dof {

		matrix<float, 3, 3> Hk_x    = { 1.0f, 0.0f, 0.0f,
										0.0f, 0.0f, 0.0f,
										0.0f, 0.0f, 0.0f};

		matrix<float, 3, 3> Hk_xdx  = { 1.0f, 0.0f, 0.0f,
										0.0f, 1.0f, 0.0f,
										0.0f, 0.0f, 0.0f};

		matrix<float, 3, 3> Hk_dx   = { 0.0f, 0.0f, 0.0f,
										0.0f, 1.0f, 0.0f,
										0.0f, 0.0f, 0.0f};

		matrix<float, 3, 3> Hk_ddx  = { 0.0f, 0.0f, 0.0f,
										0.0f, 0.0f, 0.0f,
										0.0f, 0.0f, 1.0f};

		matrix<float, 3, 3> Hk_xddx = { 1.0f, 0.0f, 0.0f,
										0.0f, 0.0f, 0.0f,
										0.0f, 0.0f, 1.0f};

		matrix<float, 3, 3> K       = { 0.0f, 0.0f, 0.0f,
										0.0f, 0.0f, 0.0f,
										0.0f, 0.0f, 0.0f};

		matrix<float, 3, 3> Rk      = { 0.0f, 0.0f, 0.0f,
										0.0f, 0.0f, 0.0f,
										0.0f, 0.0f, 0.0f};

		matrix<float, 3, 3> Qk      = { 0.0f, 0.0f, 0.0f,
										0.0f, 0.0f, 0.0f,
										0.0f, 0.0f, 0.0f};

		matrix<float, 3, 3> Pk      = { 0.0f, 0.0f, 0.0f,
										0.0f, 0.0f, 0.0f,
										0.0f, 0.0f, 0.0f};

		matrix<float, 3, 3> Fk      = { 1.0f, 0.0f, 0.0f, 
										0.0f, 1.0f, 0.0f, 
										0.0f, 0.0f, 1.0f};

		matrix<float, 3, 1> Xk      = {0.0f, 0.0f, 0.0f};
		matrix<float, 3, 1> Zk      = {0.0f, 0.0f, 0.0f};
		matrix<float, 3, 1> BUk     = {0.0f, 0.0f, 0.0f};

	public:

		kalman_1dof() {};

		void set_R(float R_a, float R_b, float R_c) {
			this->Rk(0, 0) = R_a;
			this->Rk(1, 1) = R_b;
			this->Rk(2, 2) = R_c;
		}

		void set_Q(float Q_a, float Q_b, float Q_c) {
			this->Qk(0, 0) = Q_a;
			this->Qk(1, 1) = Q_b;
			this->Qk(2, 2) = Q_c;
		}

		void predict(float accel, float dt) {
			float dt_dt_2 = (dt*dt)/2.0f;

			Fk(0, 1) = dt;
			Fk(1, 2) = dt;
			Fk(0, 2) = dt_dt_2;

			BUk(0, 0) = accel * dt_dt_2;
			BUk(0, 1) = accel * dt;

			// Xk = (Fk*Xk)+BUk;
			// Pk = Fk*Pk*~Fk + Qk;

			Xk = (Fk*Xk)+BUk;
			Pk = Fk*Pk*Fk + Qk;
		};

		void update_position(float pos) {
			Zk(0, 0) = pos;
			Zk(0, 1) = 0.0f;
			Zk(0, 2) = 0.0f;
			
			// temporary inverted matrix
			// auto tmp = Hk_x*Pk*~Hk_x + Rk; // unnecesarry transpose
			auto tmp = Hk_x*Pk*Hk_x + Rk;

			// returned true if inversion was successful? 
			bool good_invert;
			tmp = tmp.invert(&good_invert, 0.001f);
			
			if ( good_invert ) {
				// K = Pk*~Hk_x * tmp; // unecesarry transpose
				// Xk = Xk + K*(Zk - Hk_x*Xk);
				// Pk = Pk - K*Hk_x*Pk;
				
				K =  Pk * Hk_x * tmp;
				Xk = Xk + K * (Zk - Hk_x * Xk);
				Pk = Pk - K * Hk_x * Pk;

			}
		};

		void update_velocity(float vel) {
			Zk(0, 0) = 0.0f;
			Zk(0, 1) = vel;
			Zk(0, 2) = 0.0f;
			
			auto tmp = Hk_dx*Pk*Hk_dx + Rk;

			bool good_invert;
			tmp = tmp.invert(&good_invert, 0.001f);

			if ( good_invert ) {
				K = Pk * Hk_dx * tmp;
				Xk = Xk + K * (Zk - Hk_dx * Xk);
				Pk = Pk - K * Hk_dx * Pk;
			}
		};

		void update_posvel(float pos, float vel) {
			Zk(0, 0) = pos;
			Zk(0, 1) = vel;
			Zk(0, 2) = 0.0f;

			auto tmp = Hk_xdx*Pk*Hk_xdx + Rk;

			bool good_invert;
			tmp = tmp.invert(&good_invert, 0.001f);

			if ( good_invert ) {
				K = Pk*Hk_xdx * tmp;
				Xk = Xk + K*(Zk - Hk_xdx*Xk);
				Pk = Pk - K*Hk_xdx*Pk;
			}
		};

		void update_posacc(float pos, float acc) {
			Zk(0, 0) = pos;
			Zk(0, 1) = 0.0f;
			Zk(0, 2) = acc;

			auto tmp = Hk_xddx*Pk*Hk_xddx + Rk;
			
			bool good_invert;
			tmp = tmp.invert(&good_invert, 0.001f);

			if ( good_invert ) {
				K = Pk*Hk_xddx * tmp;
				Xk = Xk + K*(Zk - Hk_xddx*Xk);
				Pk = Pk - K*Hk_xddx*Pk;
			}
		};

		void get_covariances(float &cov_x, float &cov_dx, float &cov_ddx) {
			cov_x = Pk(0, 0);
			cov_dx = Pk(1, 1);
			cov_ddx = Pk(2, 2);
		}
		
		void get_states(float &x, float &dx, float &ddx) {
			x = Xk(0, 0);
			dx = Xk(0, 1);
			ddx = Xk(0, 2);
		}
	
	};

	vec3<float> rotaitonal_velocity;
	quat<float> rotation;
	
	vec3<float> acceleration_l;
	vec3<float> acceleration_i;
	vec3<float> acceleration_b;
	vec3<float> velocity;
	vec3<float> position;

	vec3<float> covariance_position;
	vec3<float> covariance_velocity;
	vec3<float> covariance_accelleration;

	namespace timing {
	
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
		
		float pressure;
		float alt;
		float temp;

		uint32_t process_time = 0;
		uint32_t read_time = 0;
	}

	kalman_1dof kalman_x;
	kalman_1dof kalman_y;
	kalman_1dof kalman_z;

	lsm6dso32 imu(spi0, pin_cs_imu);
	bmp581 baro(spi0, pin_cs_baro);
	lis2mdl mag(spi0, pin_cs_mag);

	// initialize sensors and configure data frequencies / sensitivity
	
	bool init() {

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
		if ( !baro.init() ) { boot_panic("Barometer failed to initialize"); return 0; }
		printf("Done\n\n");

		
		printf("Configuring barometer\n");
		if ( !baro.set_baro_odr(0) ) { boot_panic("Failed to configure Barometer"); return 0; }
		if ( !baro.set_osr(0b100, 0) ) { boot_panic("Failed to configure Barometer"); return 0; }
		printf("Done\n\n");	
		
		printf("============================================================================\n");

		return 1;
	};

	void update() {

		// ============================================================================
		// barometer
		
		if ( baro.data_ready() ) {

		}
		
		// ============================================================================
		// IMU

		// get new data from IMU
		// imu.read_accel_and_gyro(&raw::accel.x, &raw::accel.y, &raw::accel.z, &raw::gyro.x, &raw::gyro.y, &raw::gyro.z);

		uint32_t t_read_start = time_us_32();

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
					raw::fifo_gyro[raw::fifo_gyro_pos++] = vec3<int16_t>( (raw::fifo_frame_buf[i].data[1]) | (raw::fifo_frame_buf[i].data[2]<<8), 
																		 -(raw::fifo_frame_buf[i].data[3]) | (raw::fifo_frame_buf[i].data[4]<<8),
																		  (raw::fifo_frame_buf[i].data[5]) | (raw::fifo_frame_buf[i].data[6]<<8));
					break;
				}

				// accelerometer data
				case(2): {
					if ( raw::fifo_accel_pos > 64 ) { break; }
					raw::fifo_accel[raw::fifo_accel_pos++] = vec3<int16_t>( (raw::fifo_frame_buf[i].data[1]) | (raw::fifo_frame_buf[i].data[2]<<8), 
																		   -(raw::fifo_frame_buf[i].data[3]) | (raw::fifo_frame_buf[i].data[4]<<8),
																		    (raw::fifo_frame_buf[i].data[5]) | (raw::fifo_frame_buf[i].data[6]<<8));
					break;
				}

				default: {
					break;
				}

			}

		}

		raw::read_time = time_us_32() - t_read_start;
		
		t_read_start = time_us_32();

		// convert raw values to real numbers
		// acceleration_l = (vec3<float>)raw::accel * 0.0009765625f;
		// rotaitonal_velocity = (vec3<float>)raw::gyro * 0.00106526443f;

		uint32_t process_t_start = time_us_32();

		for ( int i = 0; i < raw::fifo_gyro_pos; i++ ) {

			constexpr float gyro_read_dt = 1.f/416.f;

			raw::gyro = raw::fifo_gyro[i];

			if ( get_vehicle_state() != state_boot && get_vehicle_state() != state_idle ) {

				// sensor debiasing
				// TODO:
				// save bias to IMU
				if ( get_vehicle_state() == state_nav_init ) {

					if ( !flags::nav::gyro_debiased ) {

						raw::gyro_bias += vec3<int32_t>(raw::gyro.x, raw::gyro.y, raw::gyro.z);
						timing::gyro_bias_count++;

						if ( timing::gyro_bias_count >= gyro_bias_count ) {
							flags::nav::gyro_debiased = true;
							raw::gyro_bias /= gyro_bias_count;
						}

					}

				}

				// gyro integration
				if ( flags::nav::gyro_debiased ) {

					rotaitonal_velocity = (vec3<float>)(raw::gyro-raw::gyro_bias) * 0.00106526443f;
					float rotational_velocity_magnitute = rotaitonal_velocity.len();
					
					// if there is rotation
					if ( rotational_velocity_magnitute > 0.00001f ) {

						quat<float> q = quat<float>().from_axis_angle(rotational_velocity_magnitute * gyro_read_dt, rotaitonal_velocity/rotational_velocity_magnitute);
						rotation *= q;
						rotation = rotation.normalize();

					}

				}

			}

		}

		for ( int i = 0; i < raw::fifo_accel_pos; i++ ) {

			constexpr float accel_read_dt = 1.f/104.f;

			// kalman filter stuff probably
			raw::accel = raw::fifo_accel[i];
			acceleration_l = (vec3<float>)raw::accel * 0.009765625f;
			acceleration_i = rotation.rotateVec(acceleration_l);

			if ( get_vehicle_state() != state_boot && get_vehicle_state() != state_idle ) {

				kalman_x.predict(acceleration_i.x, accel_read_dt);
				kalman_y.predict(acceleration_i.y, accel_read_dt);
				kalman_z.predict(acceleration_i.z, accel_read_dt);

				kalman_x.get_states(position.x, velocity.x, acceleration_b.x);
				kalman_y.get_states(position.y, velocity.y, acceleration_b.y);
				kalman_z.get_states(position.z, velocity.z, acceleration_b.z);
				
				kalman_x.get_covariances(covariance_position.x, covariance_velocity.x, covariance_accelleration.x);
				kalman_y.get_covariances(covariance_position.y, covariance_velocity.y, covariance_accelleration.y);
				kalman_z.get_covariances(covariance_position.z, covariance_velocity.z, covariance_accelleration.z);

			}

			if ( get_vehicle_state() == state_nav_init || get_vehicle_state() == state_launch_idle || get_vehicle_state() == state_launch_detect ) {

				float angle = acosf(clamp(acceleration_i.dot(vec3<float>(1.0, 0.0, 0.0)), -1.0f, 1.0f));
				vec3<float> axis = acceleration_i.cross(vec3<float>(1.0, 0.0, 0.0));
				rotation = quat<float>().from_axis_angle(0.0001f * angle, axis) * rotation;
				rotation = rotation.normalize();

			}

		}

		raw::fifo_gyro_pos = 0;
		raw::fifo_accel_pos = 0;

		raw::process_time = time_us_32() - t_read_start;

	}

}

