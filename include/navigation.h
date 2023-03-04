#include "core.h"

#include "drivers/pins.h"
#include "drivers/bmp581.h"
#include "drivers/lis2mdl.h"
#include "drivers/lsm6dso32.h"

#pragma once

namespace nav {

	constexpr float altLUT[] = {
	0.0, 0.4163035225020723, 0.47500105669954806, 0.513102173548425, 0.5419747652136768, 0.5654843012007094, 0.5854480239934159, 0.6028759801095918, 0.6183915635249146, 0.6324083901999832, 0.6452158727914808,
	0.6570249503112686, 0.6679944199562967, 0.6782470104389435, 0.687879665033116, 0.6969703794715113, 0.7055828986576926, 0.7137700328491678, 0.7215760553867444, 0.7290384725888854, 0.736189354183879,
	0.7430563496440957, 0.7496634758219397, 0.7560317352829258, 0.7621796074210809, 0.7681234426808453, 0.7738777820699412, 0.7794556184190263, 0.7848686117500598, 0.7901272681484279, 0.7952410893559155,
	0.8002186986836737, 0.8050679476291622, 0.8097960066590109, 0.8144094429134651, 0.8189142870422581, 0.8233160909564224, 0.8276199779465346, 0.8318306863536598, 0.8359526077687964, 0.8399898205678998,
	0.8439461194534683, 0.847825041563268, 0.8516298896167076, 0.8553637524955242, 0.8590295235945855, 0.8626299172282288, 0.8661674833356582, 0.8696446206939265, 0.8730635888176794, 0.8764265187001391,
	0.8797354225289209, 0.882992202492581, 0.8861986587787228, 0.8893564968516312, 0.8924673340863913, 0.8955327058269845, 0.898554070927706, 0.9015328168302063, 0.9044702642223588, 0.9073676713198583,
	0.9102262378068442, 0.913047108467813, 0.9158313765395681, 0.9185800868088617, 0.9212942384786738, 0.9239747878236838, 0.9266226506533788, 0.929238704599384, 0.9318237912419433, 0.9343787180890195,
	0.9369042604201759, 0.9394011630062418, 0.9418701417147317, 0.9443118850100618, 0.9467270553567773, 0.9491162905332656, 0.9514802048627635, 0.9538193903678645, 0.9561344178541967, 0.9584258379284554,
	0.9606941819555306, 0.962939962959081, 0.9651636764695385, 0.9673658013232062, 0.9695468004158188, 0.9717071214136601, 0.9738471974250937, 0.9759674476351379, 0.9780682779055134, 0.9801500813424088,
	0.9822132388340362, 0.9842581195599004, 0.986285081473557, 0.9882944717605117, 0.9902866272727878, 0.9922618749415852, 0.9942205321693495, 0.9961629072024801, 0.9980892994858215};
	
	// untested
	float x_pow_lut(float x) {
		
		int xi = x*1000000;

		if ( xi < 0 ) { return 0.0f; }
		if ( xi > 999999 ) { return 1.f; }

		int x_partial = xi%10000;
		xi /= 10000;

		return altLUT[xi]+(altLUT[xi+1]-altLUT[xi])/float(10000-x_partial);

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

			// Xk = (Fk*Xk)+BUk;
			// Pk = Fk*Pk*~Fk + Qk;

			Xk = (Fk*Xk)+BUk;
			Pk = Fk*Pk*Fk + Qk;
		};

		void update_position(float pos) {
			Zk(0, 0) = pos;
			
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

	const vec3<float> gravity(9.816, 0.0, 0.0);

	vec3<float> rotational_velocity;
	quat<float> rotation;
	
	vec3<float> acceleration_l;
	vec3<float> acceleration_i;
	vec3<float> acceleration_b;
	vec3<float> velocity;
	vec3<float> position;

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

		kalman_x.set_Q(400.0f, 250.f, 1.f);
    	kalman_x.set_R(750.0f, 25.0f, 1.0f);

		kalman_y.set_Q(0.f, 0.f, 0.f);
		kalman_y.set_R(0.f, 0.f, 0.f);

		kalman_z.set_Q(0.f, 0.f, 0.f);
		kalman_z.set_R(0.f, 0.f, 0.f);

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
		
		baro.read_all_data(&raw::temperature, &raw::pressure);

		pressure = (float)(raw::pressure / 64.0);
		temperature = float(raw::temperature)/65536.0f;
		
		// H = 44330 * [1 - (P/p0)^(1/5.255) ]

		//H = altitude (m)
		//P = measured pressure (Pa) from the sensor
		//p0 = reference pressure at sea level (e.g. 1013.25hPa)

		altitude_asl = ( 1.f - pow(pressure/101325.f, 1.f/5.255f) ) * 44330.f;

		if ( !flags::nav::baro_debiased ) {

			pad_altitude += altitude_asl;
			timing::baro_bias_count++;

			if ( timing::baro_bias_count >= baro_bias_count ) {
				flags::nav::baro_debiased = true;
				pad_altitude /= (float)baro_bias_count;
			}

		} else {
			altitude = altitude_asl - pad_altitude;
			kalman_x.update_position(altitude);
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
		
		uint32_t t_process_start = time_us_32();

		// convert raw values to real numbers
		// acceleration_l = (vec3<float>)raw::accel * 0.0009765625f;
		// rotational_velocity = (vec3<float>)raw::gyro * 0.00106526443f;

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

					rotational_velocity = (vec3<float>)(raw::gyro-raw::gyro_bias) * 0.00106526443f;
					float rotational_velocity_magnitute = rotational_velocity.len();
					
					// if there is rotation
					if ( rotational_velocity_magnitute > 0.00001f ) {

						quat<float> q = quat<float>().from_axis_angle(rotational_velocity_magnitute * gyro_read_dt, rotational_velocity/rotational_velocity_magnitute);
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

				float angle = acosf(clamp(acceleration_i.dot(vec3<float>(1.0, 0.0, 0.0)), -1.0f, 1.0f));
				vec3<float> axis = acceleration_i.cross(vec3<float>(1.0, 0.0, 0.0));
				rotation = quat<float>().from_axis_angle(0.0001f * angle, axis) * rotation;
				rotation = rotation.normalize();

			}

		}

		raw::fifo_gyro_pos = 0;
		raw::fifo_accel_pos = 0;

		raw::process_time = time_us_32() - t_process_start;
		raw::total_time = time_us_32() - t_read_start;

	}

}

