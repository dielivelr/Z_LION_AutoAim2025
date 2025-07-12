// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Copyright (C) 2025 Li Rui
// Licensed under the MIT License.
#ifndef _KCF_H_
#define _KCF_H_

#include <Eigen/Dense>
#include <functional>
#include <iostream>

namespace rm
{
    class ExtendedKalmanFilter
    {
    public:
        ExtendedKalmanFilter() = default;
        using NonlinearFunc = std::function<Eigen::VectorXf(const Eigen::VectorXf&)>;
        using JacobianFunc = std::function<Eigen::MatrixXf(const Eigen::VectorXf&)>;

        explicit ExtendedKalmanFilter(
            const NonlinearFunc& f, const NonlinearFunc& h, const JacobianFunc& Jf,
            const JacobianFunc& Jh, const Eigen::MatrixXf& Q, const Eigen::MatrixXf& R,
            const Eigen::MatrixXf& P0);
        void setState(const Eigen::VectorXf& x0);
        Eigen::MatrixXf predict();
        Eigen::MatrixXf update(const Eigen::VectorXf& z);
    private:
        // Process nonlinear vector function
        NonlinearFunc f;
        // Observation nonlinear vector function
        NonlinearFunc h;
        // Jacobian of f()
        JacobianFunc Jf;
        Eigen::MatrixXf F;
        // Jacobian of h()
        JacobianFunc Jh;
        Eigen::MatrixXf H;
        // Process noise covariance matrix
        Eigen::MatrixXf Q;
        // Measurement noise covariance matrix
        Eigen::MatrixXf R;
        // Priori error estimate covariance matrix
        Eigen::MatrixXf P_pri;
        // Posteriori error estimate covariance matrix
        Eigen::MatrixXf P_post;
        // Kalman gain
        Eigen::MatrixXf K;
        // System dimensions
        int n;
        // N-size identity
        Eigen::MatrixXf I;
        // Priori state
        Eigen::VectorXf x_pri;
        // Posteriori state
        Eigen::VectorXf x_post;
    };
};
#endif // !KCF
