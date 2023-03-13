#include "core.h"

#include "drivers/pins.h"
#include "drivers/bmp581.h"
#include "drivers/lis2mdl.h"
#include "drivers/lsm6dso32.h"

#pragma once

namespace nav {

	const float altLUT[] = { 0.        , 0.41629385, 0.47499169, 0.5130931 , 0.54196597,
       0.56547576, 0.58543972, 0.6028679 , 0.61838369, 0.63240071,
       0.64520838, 0.65701764, 0.66798728, 0.67824003, 0.68787284,
       0.69696371, 0.70557638, 0.71376365, 0.72156982, 0.72903237,
       0.73618338, 0.7430505 , 0.74965775, 0.75602613, 0.76217412,
       0.76811807, 0.77387253, 0.77945047, 0.78486357, 0.79012234,
       0.79523626, 0.80021397, 0.80506332, 0.80979148, 0.81440501,
       0.81890995, 0.82331185, 0.82761583, 0.83182663, 0.83594864,
       0.83998594, 0.84394232, 0.84782133, 0.85162627, 0.85536021,
       0.85902606, 0.86262654, 0.86616419, 0.8696414 , 0.87306045,
       0.87642346, 0.87973244, 0.88298929, 0.88619582, 0.88935373,
       0.89246464, 0.89553009, 0.89855152, 0.90153034, 0.90446786,
       0.90736533, 0.91022397, 0.91304491, 0.91582924, 0.91857802,
       0.92129224, 0.92397285, 0.92662078, 0.9292369 , 0.93182205,
       0.93437704, 0.93690264, 0.93939961, 0.94186865, 0.94431045,
       0.94672568, 0.94911498, 0.95147895, 0.9538182 , 0.95613328,
       0.95842476, 0.96069316, 0.962939  , 0.96516277, 0.96736495,
       0.96954601, 0.97170638, 0.97384651, 0.97596682, 0.9780677 ,
       0.98014956, 0.98221277, 0.98425771, 0.98628472, 0.98829416,
       0.99028637, 0.99226167, 0.99422038, 0.99616281, 0.99808925,
       1.        , 1.00189534, 1.00377555, 1.00564089, 1.00749162,
       1.00932801, 1.01115028, 1.01295869, 1.01475346, 1.01653483,
       1.01830301 };

	float fast_pow(float P) {
		if ( P < 0.0f ) { return 0.0f; }
		int idx = int(P * 100.f);
		float diff = P * 100.f - idx;
		if (idx >= 110) {
			return 1.01830301;
		}
		return altLUT[idx] * (1 - diff) + diff * altLUT[idx + 1];
	}

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

			Xk = (Fk*Xk)+BUk;
			Pk = Fk*Pk*~Fk + Qk;

			// Xk = (Fk*Xk)+BUk;
			// Pk = Fk*Pk*Fk + Qk;
		};

		void update_position(float pos) {
			Zk(0, 0) = pos;
			
			// temporary inverted matrix
			auto tmp = Hk_x*Pk*~Hk_x + Rk; // unnecesarry transpose
			// auto tmp = Hk_x*Pk*Hk_x + Rk;

			// returned true if inversion was successful? 
			bool good_invert;
			tmp = tmp.invert(&good_invert, 0.001f);
			
			if ( good_invert ) {
				K = Pk*~Hk_x * tmp; // unecesarry transpose
				Xk = Xk + K*(Zk - Hk_x*Xk);
				Pk = Pk - K*Hk_x*Pk;
				
				// K =  Pk * Hk_x * tmp;
				// Xk = Xk + K * (Zk - Hk_x * Xk);
				// Pk = Pk - K * Hk_x * Pk;

			}
		};

		void update_velocity(float vel) {
			Zk(0, 1) = vel;
			
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

	float mass;

	const vec3<float> gravity(9.816, 0.0, 0.0);

	quat<float> rotation;
	vec3<float> rotational_velocity;
	vec3<float> rotational_velocity_bias;
	
	vec3<float> acceleration_l;
	vec3<float> acceleration_i;
	
	vec3<float> position;
	vec3<float> velocity;

	vec3<float> mag_field;

	vec3<float> acceleration_b;
	vec3<float> covariance_position;
	vec3<float> covariance_velocity;
	vec3<float> covariance_acceleration;

	float pressure;
	float altitude;
	float altitude_asl;
	float temperature;

	float pad_altitude;

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
		
		uint32_t pressure;
		int32_t temperature;
		
		uint32_t process_time = 0;
		uint32_t read_time = 0;
		uint32_t total_time = 0;

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

		kalman_x.set_Q(400.0f, 1.f, 1.f);
    	kalman_x.set_R(2500.0f, 1.0f, 1.0f);

		kalman_y.set_Q(1.f, 1.f, 1.f);
		kalman_y.set_R(1.f, 1.f, 1.f);

		kalman_z.set_Q(1.f, 1.f, 1.f);
		kalman_z.set_R(1.f, 1.f, 1.f);

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
		
		// printf("Configuring barometer\n");
		// t_start = time_us_64();
		// sts = 0;
		// while ( time_us_64() < t_start + 5000 ) {
		// 	if ( baro.set_baro_odr(0x12) ) { sts = 1; break; }
		// }
		// if ( !sts ) { boot_panic("Failed to configure Barometer");  }
		
		// t_start = time_us_64();
		// sts = 0;
		// while ( time_us_64() < t_start + 5000 ) {
		// 	if ( baro.set_osr(0b10, 0b0) ) { sts = 1; break; }
		// }
		// if ( !sts ) { boot_panic("Failed to configure Barometer");  }

		printf("Done\n\n");	

		printf("============================================================================\n");

		return 1;
	};

	void update() {

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

			altitude_asl = ( 1.f - fast_pow(pressure/101325.f) ) * 44330.f;

			if ( !flags::nav::baro_debiased && altitude_asl < 500.f) {

				pad_altitude += altitude_asl;
				timing::baro_bias_count++;

				if ( timing::baro_bias_count >= baro_bias_count ) {
					flags::nav::baro_debiased = true;
					pad_altitude /= (float)baro_bias_count;
				}

			} else {

				altitude = altitude_asl - pad_altitude;
				if ( get_vehicle_state() == state_nav_init && altitude > 2 ) { flags::nav::baro_debiased = false; timing::baro_bias_count = 0; }

				kalman_x.update_position(altitude);

			}
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

		raw::read_time = time_us_32() - t_read_start;
		uint32_t t_process_start = time_us_32();

		for ( int i = 0; i < raw::fifo_gyro_pos; i++ ) {

			constexpr float gyro_read_dt = 1.f/416.f;

			raw::gyro = raw::fifo_gyro[i];

			if ( get_vehicle_state() != state_boot && get_vehicle_state() != state_idle ) {

				// sensor debiasing
				if ( get_vehicle_state() == state_nav_init ) {

					if ( !flags::nav::gyro_debiased ) {
						rotational_velocity = ((vec3<float>)(raw::gyro)) * 0.00106526443f;
						float rotational_velocity_magnitude = rotational_velocity.len();
						
						if ( rotational_velocity_magnitude < 0.025 ) {
							rotational_velocity_bias += rotational_velocity;
							timing::gyro_bias_count++;

							if ( timing::gyro_bias_count >= gyro_bias_count ) {
								flags::nav::gyro_debiased = true;
								rotational_velocity_bias /= float(gyro_bias_count);
							}
						} else { 
							if ( rotational_velocity_magnitude > 0.08 ) {
								timing::gyro_bias_count = 0;
								rotational_velocity_bias = vec3();
							} 
						}
						
					}

				}

				// gyro integration
				if ( flags::nav::gyro_debiased ) {

					rotational_velocity = ((vec3<float>)(raw::gyro)) * 0.00106526443f;
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

			float gyro_deviation_from_0 = rotational_velocity.len() - landing_detect_ori_threshold;
			flags::state::gyro_within_landed_threshold = 	( get_vehicle_state() == state_landing_terminal ) & \
															( gyro_deviation_from_0 < landing_detect_ori_threshold ) & \
															( gyro_deviation_from_0 > -landing_detect_ori_threshold );

		}

		for ( int i = 0; i < raw::fifo_accel_pos; i++ ) {

			constexpr float accel_read_dt = 1.f/104.f;

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
				flags::nav::orientation_converged = ((1.45f < angle) & (angle < 1.68f));

				// complementary filter with accelerometer
				vec3<float> axis = acceleration_i.cross(vec3<float>(1.0, 0.0, 0.0));
				rotation = quat<float>().from_axis_angle(0.0001f * angle, axis) * rotation;
				rotation = rotation.normalize();

			}

			flags::state::accel_over_ld_threshold = (get_vehicle_state() == state_launch_detect) & ( acceleration_l.x > launch_detect_accel_threshold );
			flags::state::accel_under_burnout_threshold = (get_vehicle_state() == state_powered_ascent) & ( acceleration_l.x < burnout_detect_accel_threshold );
			flags::state::accel_over_landing_threshold = (get_vehicle_state() == state_landing_start) & ( acceleration_l.x > landing_burn_detect_accel_threshold );

			float deviation_from_g = (acceleration_l.len()-9.816);
			flags::state::accel_within_landed_threshold = 	( get_vehicle_state() == state_landing_terminal ) & \
															( deviation_from_g < landing_detect_accel_threshold ) & \
															( deviation_from_g > -landing_burn_detect_accel_threshold );

		}

		raw::fifo_gyro_pos = 0;
		raw::fifo_accel_pos = 0;

		raw::process_time = time_us_32() - t_process_start;
		raw::total_time = time_us_32() - t_read_start;

	}

}

