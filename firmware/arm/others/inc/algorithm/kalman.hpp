/**
 * This file was implemented based on design ideas and algorithms referenced from:
 * yssickjgd/damiao_mc02_bsp
 * https://github.com/yssickjgd/damiao_mc02_bsp
 */

#pragma once

#include <matrix.hpp>


struct kalman_r1_s
{
	float x;
	float P;  // estimation error covariance
	const float Q;  // process noise covariance
	const float R;  // measurement noise covariance
	float K;  // Kalman gain
	float Update(const float measurement_)
	{
		P += Q;  // prediction step: increase uncertainty
		K = P / (P + R);  // compute Kalman gain
		x += K * (measurement_ - x);  // update estimate with measurement
		P *= 1.f - K;  // update uncertainty
		return x;
	}
	void Reset(const float x_new_ = 0.f)
	{
		x = x_new_;  // reset state estimate
		K = 0.f;  // reset Kalman gain
	}
};

template <typename model_, uint8_t state_dim_, uint8_t input_dim_, uint8_t measure_dim_>
class KalmanFilter
{
public:
	KalmanFilter(const Matrixf<input_dim_, input_dim_> &mat_Q_, const Matrixf<measure_dim_, measure_dim_> &mat_R_,
		const Matrixf<state_dim_, state_dim_> &mat_P_, const Matrixf<state_dim_, 1> &vec_x_,
		const Matrixf<input_dim_, 1> &vec_u_)
		: mat_Q(mat_Q_), mat_R(mat_R_), mat_P(mat_P_), vec_x(vec_x_), vec_u(vec_u_),
		vec_x_prior(matrixf::zeros<state_dim_, 1>()), mat_P_prior(matrixf::zeros<state_dim_, state_dim_>()),
		mat_K(matrixf::zeros<state_dim_, measure_dim_>()), vec_z(matrixf::zeros<measure_dim_, 1>())
	{}

	void Reset(const Matrixf<input_dim_, input_dim_> &mat_Q_, const Matrixf<measure_dim_, measure_dim_> &mat_R_,
		const Matrixf<state_dim_, state_dim_> &mat_P_, const Matrixf<state_dim_, 1> &vec_x_,
		const Matrixf<input_dim_, 1> &vec_u_)
	{
		mat_Q = mat_Q_;
		mat_R = mat_R_;
		mat_P = mat_P_;
		vec_x = vec_x_;
		vec_u = vec_u_;
	}

	/**
	 * @brief EKF filter predict step
	 * @param dt_ time step
	 */
	void Predict(const float dt_)
	{
		// predict state
		vec_x_prior = model_::F(vec_x, vec_u, dt_);
		// compute Jacobian
		Matrixf<state_dim_, state_dim_> mat_F_x = model_::JacobianF2X(vec_x, vec_u, dt_);
		Matrixf<state_dim_, input_dim_> mat_F_w = model_::JacobianF2W(vec_x, vec_u, dt_);
		// predict error covariance
		mat_P_prior = mat_F_x * mat_P * mat_F_x.trans()
			+ mat_F_w * mat_Q * mat_F_w.trans();
		// update state and covariance for next iteration
		vec_x = vec_x_prior;
		mat_P = mat_P_prior;
	}

	/**
	 * @brief EKF filter update step
	 * @note prediction should be up to the measurement time
	 */
	void Update(const float dt_)
	{
		// compute Jacobian
		Matrixf<measure_dim_, state_dim_> mat_H_x = model_::JacobianH2X(vec_x_prior, dt_);
		Matrixf<measure_dim_, measure_dim_> mat_H_v = model_::JacobianH2V(vec_x_prior, dt_);
		// compute Kalman gain
		mat_K = mat_P_prior * mat_H_x.trans()
			* matrixf::inv(mat_H_x * mat_P_prior * mat_H_x.trans() + mat_H_v * mat_R * mat_H_v.trans());
		// update state estimate
		vec_x = vec_x_prior + mat_K * (vec_z - model_::H(vec_x_prior, dt_));
		// update error covariance using Joseph form for better numerical stability
		Matrixf<state_dim_, state_dim_> mat_tmp = matrixf::eye<state_dim_, state_dim_>() - mat_K * mat_H_x;
		mat_P = mat_tmp * mat_P_prior * mat_tmp.trans()
			+ mat_K * mat_H_v * mat_R * mat_H_v.trans() * mat_K.trans();
	}

	Matrixf<input_dim_, input_dim_> mat_Q;  // process noise covariance
	Matrixf<measure_dim_, measure_dim_> mat_R;  // measurement noise covariance

	Matrixf<state_dim_, state_dim_> mat_P;  // estimation error covariance
	Matrixf<state_dim_, 1> vec_x;  // state estimate
	Matrixf<input_dim_, 1> vec_u;  // control input
	Matrixf<state_dim_, 1> vec_x_prior;  // prior state estimate
	Matrixf<state_dim_, state_dim_> mat_P_prior;  // prior estimation error covariance
	Matrixf<state_dim_, measure_dim_> mat_K;  // Kalman gain

	Matrixf<measure_dim_, 1> vec_z;  // measurement
};
