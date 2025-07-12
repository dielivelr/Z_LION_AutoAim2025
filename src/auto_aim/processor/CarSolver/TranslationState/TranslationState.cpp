#include "TranslationState.h"

namespace rm
{
    Eigen::VectorXf TranslationStateBase::get_last_state() const
    {
        return target_state;
    }
    EkfTranslationState::EkfTranslationState(const std::vector<double>& q_vs, const std::vector<double>& r_vs)
    {
        // 平移目标我们不考虑Y方向
        auto f0 = [this](const Eigen::VectorXf& x) {
            Eigen::VectorXf x_new = x;
            x_new(0) += x(3) * d_t;
            x_new(2) += x(4) * d_t;
            return x_new;
            };

        // J_f - Jacobian of process function
        auto j_f0 = [this](const Eigen::VectorXf& x) {
            Eigen::Matrix<float, 5, 5> f;
            // clang-format off
            f <<
                1, 0, 0, d_t, 0,
                0, 1, 0, 0, 0,
                0, 0, 1, 0, d_t,
                0, 0, 0, 1, 0,
                0, 0, 0, 0, 1;
            // clang-format on
            return f;
            };

        // h - Observation function
        auto h0 = [this](const Eigen::VectorXf& x) {
            Eigen::VectorXf z(3);
            z(0) = x(0);
            z(1) = x(1);
            z(2) = x(2);
            return z;
            };

        // J_h - Jacobian of observation function
        auto j_h0 = [this](const Eigen::VectorXf& x) {
            Eigen::Matrix<float, 3, 5> h;

            h <<
                1, 0, 0, 0, 0,
                0, 1, 0, 0, 0,
                0, 0, 1, 0, 0;

            return h;
            };

        Eigen::DiagonalMatrix<float, 5> q0;
        q0.diagonal() << q_vs[0], q_vs[1], q_vs[2], q_vs[3], q_vs[4];

        //xa ya za yaw
        Eigen::DiagonalMatrix<float, 3> r0;
        r0.diagonal() << r_vs[0], r_vs[1], r_vs[2];

        // P - error estimate covariance matrix
        Eigen::DiagonalMatrix<float, 5> p0;
        p0.setIdentity();

        tracker__ = std::make_unique<ExtendedKalmanFilter>(f0, h0, j_f0, j_h0, q0, r0, p0);
    }
    Eigen::VectorXf EkfTranslationState::update(Eigen::VectorXf measurement, double new_d_t)
    {
        this->d_t = new_d_t;
        Eigen::VectorXf pre_state = tracker__->predict();
        this->target_state = tracker__->update(measurement);
        return get_last_state();
    }
    Eigen::VectorXf EkfTranslationState::reset(Eigen::VectorXf state)
    {
        tracker__->setState(state);
        this->target_state = state;
        return get_last_state();
    }
    Eigen::VectorXf EkfTranslationState::reset_maintain_speed(Eigen::VectorXf measurement, double new_d_t)
    {
        // 考虑此时滤波器的更新
        this->d_t = new_d_t;
        Eigen::VectorXf pre_state = tracker__->predict();
        Eigen::VectorXf new_state(5);
        new_state << measurement(0), measurement(1), measurement(2), pre_state(3), pre_state(4);

        tracker__->setState(new_state);
        this->target_state = new_state;
        return get_last_state();
    }
    ;
};

