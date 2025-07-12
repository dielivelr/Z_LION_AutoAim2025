#include"ExtendTranslationState.h"

namespace rm
{
    ExtendTranslationState::ExtendTranslationState(const std::vector<double>& q_vs, const std::vector<double>& r_vs)
    {
        // 预测过程协方差
        ekf.Q(0, 0) = q_vs[0];
        ekf.Q(1, 1) = q_vs[1];
        ekf.Q(2, 2) = q_vs[2];
        ekf.Q(3, 3) = q_vs[3];
        ekf.Q(4, 4) = q_vs[4];
        // 观测过程协方差
        ekf.R(0, 0) = r_vs[0];
        ekf.R(1, 1) = r_vs[1];
        ekf.R(2, 2) = r_vs[2];
    };

    Eigen::VectorXf ExtendTranslationState::get_last_state() const
    {
        // 单位化
        Eigen::VectorXf last_state = this->target_state;
        last_state(0) = last_state(0) * 100;
        last_state(1) = last_state(1) * 100;
        last_state(2) = last_state(2) * 100;
        last_state(3) = last_state(3) * 100;
        last_state(4) = last_state(4) * 100;
        return last_state;
    };

    Eigen::VectorXf ExtendTranslationState::update(Eigen::VectorXf measurement, double new_d_t)
    {
        TransPredict predictfunc;
        TransMeasure measure;
        predictfunc.delta_t = new_d_t;      // 设置距离上次预测的时间
        ekf.predict(predictfunc);           // 更新预测器，此时预测器里的是预测值

        // 单位转换
        measurement(0) = measurement(0) / 100.0;
        measurement(1) = measurement(1) / 100.0;
        measurement(2) = measurement(2) / 100.0;
        Eigen::Matrix<float, 5, 1> Xr;
        Xr << measurement(0), measurement(1), measurement(2), 0, 0;
        Eigen::Matrix<float, 3, 1> Yr;
        measure(Xr.data(), Yr.data());
        Eigen::Matrix<float, 5, 1> Xe = ekf.update(measure,Yr);   // 更新滤波器，输入真实的球面坐标 Yr
        this->target_state = Xe.cast<float>();

        return get_last_state();
    };

    Eigen::VectorXf ExtendTranslationState::reset(Eigen::VectorXf state)
    {
        // 单位转换
        state(0) = state(0) / 100.0;
        state(1) = state(1) / 100.0;
        state(2) = state(2) / 100.0;
        state(3) = state(3) / 100.0;
        state(4) = state(4) / 100.0;

        Eigen::Matrix<float, 5, 1> Xr;
        Xr << state(0), state(1), state(2), state(3), state(4);
        ekf.init(Xr);
        this->target_state = Xr.cast<float>();;
        return get_last_state();
    };

    Eigen::VectorXf ExtendTranslationState::reset_maintain_speed(Eigen::VectorXf measurement, double new_d_t)
    {

        TransPredict predictfunc;
        predictfunc.delta_t = new_d_t;  // 设置距离上次预测的时间
        ekf.predict(predictfunc);  // 更新预测器，此时预测器里的是预测值

        // 单位转换
        measurement(0) = measurement(0) / 100.0;
        measurement(1) = measurement(1) / 100.0;
        measurement(2) = measurement(2) / 100.0;

        Eigen::Matrix<float, 5, 1> Xr;
        Xr << measurement(0), measurement(1), measurement(2), target_state(3), target_state(4);
        ekf.init(Xr);
        this->target_state = Xr.cast<float>();
        return get_last_state();
    };
};