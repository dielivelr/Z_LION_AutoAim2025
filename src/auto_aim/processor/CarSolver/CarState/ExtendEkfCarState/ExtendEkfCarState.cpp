#include"ExtendEkfCarState.h"

namespace rm
{
    ExtendEkfCarState::ExtendEkfCarState(const std::vector<double>& q_vs, const std::vector<double>& r_vs)
        :CarStateBase()
    {
        // 预测过程协方差
        ekf.Q(0, 0) = q_vs[0];
        ekf.Q(1, 1) = q_vs[1];
        ekf.Q(2, 2) = q_vs[2];
        ekf.Q(3, 3) = q_vs[3];
        ekf.Q(4, 4) = q_vs[4];
        ekf.Q(5, 5) = q_vs[5];
        ekf.Q(6, 6) = q_vs[6];
        ekf.Q(7, 7) = q_vs[7];
        // 观测过程协方差
        ekf.R(0, 0) = r_vs[0];
        ekf.R(1, 1) = r_vs[1];
        ekf.R(2, 2) = r_vs[2];
        ekf.R(3, 3) = r_vs[3];
    };

    Eigen::VectorXf ExtendEkfCarState::get_last_state() const
    {
        Eigen::VectorXf state = target_state;
        state(0) = target_state(0) * 100;
        state(1) = target_state(2) * 100;
        state(2) = target_state(1) * 100;
        state(4) = target_state(4) * 100;
        state(5) = target_state(6) * 100;
        state(6) = target_state(5) * 100;
        return state;
    };

    Eigen::VectorXf ExtendEkfCarState::reset(Eigen::VectorXf state)
    {
        Eigen::VectorXf new_state = state;
        new_state(0) = state(0) / 100;
        new_state(1) = state(2) / 100;
        new_state(2) = state(1) / 100;
        new_state(4) = state(4) / 100;
        new_state(5) = state(6) / 100;
        new_state(6) = state(5) / 100;

        Eigen::Matrix<float, 8, 1> Xr;
        Xr << new_state(0), new_state(1), new_state(2), new_state(3), 
            new_state(4), new_state(5), new_state(6), new_state(7);
        ekf.init(Xr);
        this->target_state = Xr.cast<float>();;
        return get_last_state();
    };

    Eigen::VectorXf ExtendEkfCarState::update(Eigen::VectorXf measurement, double new_d_t)
    {
        CarPredict predictfunc;
        CarMeasure measure;
        predictfunc.delta_t = new_d_t; // 设置距离上次预测的时间
        measure.r = HIGH_ARMOR_STATE ? high_armor_radius : low_armor_radius;
        measure.r = measure.r / 100.0;

        ekf.predict(predictfunc);   // 更新预测器，此时预测器里的是预测值

        Eigen::VectorXf new_state = measurement;
        new_state(0) = measurement(0) / 100;
        new_state(1) = measurement(2) / 100;
        new_state(2) = (HIGH_ARMOR_STATE ? measurement(1) : measurement(1) + high_low_height_differ) / 100;

        Eigen::Matrix<float, 8, 1> Xr;
        Xr << new_state(0), new_state(1), new_state(2), new_state(3), 0, 0, 0, 0;
        Eigen::Matrix<float, 4, 1> Yr;
        // measure(Xr.data(), Yr.data());
        xyza_to_pyda(Xr.data(), Yr.data());
        //Yr << new_state(0), new_state(1), new_state(2), new_state(3);

        Eigen::Matrix<float, 8, 1> Xe = ekf.update(measure, Yr);   // 更新滤波器，输入真实的球面坐标 Yr
        this->target_state = Xe.cast<float>();
        return get_last_state();
    };

    Eigen::VectorXf ExtendEkfCarState::reset_maintain_speed(Eigen::VectorXf measurement, double new_d_t)
    {
        CarPredict predictfunc;
        CarMeasure measure;

        measure.r = HIGH_ARMOR_STATE ? high_armor_radius : low_armor_radius;
        measure.r = measure.r / 100.0;

        predictfunc.delta_t = new_d_t; // 设置距离上次预测的时间
        Eigen::Matrix<float, 8, 1> pre_state = ekf.predict(predictfunc);   // 更新预测器，此时预测器里的是预测值
        Eigen::VectorXf last_state = target_state;

        Eigen::VectorXf new_state = measurement;
        new_state(0) = measurement(0) / 100;
        new_state(1) = measurement(2) / 100;
        new_state(2) = (HIGH_ARMOR_STATE ? measurement(1) : measurement(1) + high_low_height_differ) / 100;

        Eigen::Matrix<float, 8, 1> Xr;
        Xr << new_state(0), new_state(1), new_state(2), new_state(3), 0, 0, 0, 0;
        Eigen::Matrix<float, 4, 1> Yr;
        //measure(Xr.data(), Yr.data());
        xyza_to_pyda(Xr.data(), Yr.data());
        //Yr << new_state(0), new_state(1), new_state(2), new_state(3);

        // 保持xyz的更新,而yaw与vyaw使用上一次测量值
        Eigen::Matrix<float, 8, 1> Xe = ekf.update(measure, Yr);
        Xe(3) = measurement(3);
        Xe(7) = last_state(7);

        this->target_state = Xe.cast<float>();

        // init初始化
        ekf.init(Xe);

        return get_last_state();
    };

};