// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Copyright (C) 2025 Li Rui
// Licensed under the MIT License.
#include "Ekf.h"

namespace rm
{
	ExtendedKalmanFilter::ExtendedKalmanFilter(
		const NonlinearFunc& f, const NonlinearFunc& h, const JacobianFunc& Jf,
		const JacobianFunc& Jh, const Eigen::MatrixXf& Q, const Eigen::MatrixXf& R,
		const Eigen::MatrixXf& P0)
		: f(f),
		h(h),
		Jf(Jf),
		Jh(Jh),
		Q(Q),
		R(R),
		P_post(P0),
		n(Q.rows()),
		I(Eigen::MatrixXf::Identity(n, n)),
		x_pri(n),
		x_post(n)
	{
	}

	void ExtendedKalmanFilter::setState(const Eigen::VectorXf& x0) {
		x_post = x0;
	}

	Eigen::MatrixXf ExtendedKalmanFilter::predict()
	{
		x_pri = f(x_post);
		F = Jf(x_post);
		P_pri = F * P_post * F.transpose() + Q;
		x_post = x_pri;
		P_post = P_pri;
		return x_pri;
	}

	Eigen::MatrixXf ExtendedKalmanFilter::update(const Eigen::VectorXf& z)
	{
		H = Jh(x_pri);
		K = P_pri * H.transpose() * (H * P_pri * H.transpose() + R).inverse();
		x_post = x_pri + K * (z - h(x_pri));
		P_post = (I - K * H) * P_pri;
		return x_post;
	}

}