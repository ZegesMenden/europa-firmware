#pragma once

#include "mmath.h"

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
