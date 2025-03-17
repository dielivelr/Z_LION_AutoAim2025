#include"CarState.h"

namespace rm
{
    CarStateBase::CarStateBase()
    {
        high_armor_radius = 25.0;
        low_armor_radius = 25.0;
        high_low_height_differ = 0;
    }
    bool CarStateBase::angle_jump(float angle) const
    {
        // std::cout << "angle_jump differ: " << abs(angle - last_yaw) * 180 / CV_PI << std::endl;
        bool ret = abs(angle - last_yaw) > CV_PI / 3.5; // 0.4
        return ret;
    }
    void CarStateBase::set_last_yaw(float angle)
    {
        this->last_yaw = angle;
    }
    float CarStateBase::get_last_yaw() const
    {
        return last_yaw;
    }
    float CarStateBase::get_observe_armor_radius() const
    {
        return HIGH_ARMOR_STATE ? high_armor_radius : low_armor_radius;
    }
    float CarStateBase::get_high_low_height_differ() const
    {
        return high_low_height_differ;
    }
    void CarStateBase::get_high_low_radius(float& high_radius, float& low_radius) const
    {
        high_radius = this->high_armor_radius;
        low_radius = this->low_armor_radius;
    }
    void CarStateBase::set_radius(float high_armor_radius, float low_armor_radius)
    {
        this->high_armor_radius = high_armor_radius;
        this->low_armor_radius = low_armor_radius;
    }
    void CarStateBase::set_high_low_height_differ(float high_low_height_differ)
    {
        this->high_low_height_differ = high_low_height_differ;
    }
    void CarStateBase::set_high_armor_state(bool high_armor_state)
    {
        this->HIGH_ARMOR_STATE = high_armor_state;
    }
    void CarStateBase::inversion_high_armor_state()
    {
        this->HIGH_ARMOR_STATE = !this->HIGH_ARMOR_STATE;
    }
    bool CarStateBase::get_high_armor_state() const
    {
        return this->HIGH_ARMOR_STATE;
    }


    MIddleCarState::MIddleCarState(const std::vector<double>& q_vs, const std::vector<double>& r_vs)
        :CarStateBase()
    {

        d_t = 0.01;
        auto f0 = [this](const Eigen::VectorXf& x) {
            Eigen::VectorXf x_new = x;
            x_new(0) += x(4) * d_t;
            x_new(1) += x(5) * d_t;
            x_new(2) += x(6) * d_t;
            x_new(3) += x(7) * d_t;
            return x_new;
            };

        // J_f - Jacobian of process function
        auto j_f0 = [this](const Eigen::VectorXf& x) {
            Eigen::Matrix<float, 8, 8> f;
            // clang-format off
            f <<
                1, 0, 0, 0, d_t, 0, 0, 0,
                0, 1, 0, 0, 0, d_t, 0, 0,
                0, 0, 1, 0, 0, 0, d_t, 0,
                0, 0, 0, 1, 0, 0, 0, d_t,
                0, 0, 0, 0, 1, 0, 0, 0,
                0, 0, 0, 0, 0, 1, 0, 0,
                0, 0, 0, 0, 0, 0, 1, 0,
                0, 0, 0, 0, 0, 0, 0, 1;
            // clang-format on
            return f;
            };

        // h - Observation function
        auto h0 = [this](const Eigen::VectorXf& x) {
            Eigen::VectorXf z(4);
            z(0) = x(0);
            z(1) = x(1);
            z(2) = x(2);
            z(3) = x(3);
            return z;
            };

        // J_h - Jacobian of observation function
        auto j_h0 = [this](const Eigen::VectorXf& x) {
            Eigen::Matrix<float, 4, 8> h;

            h <<
                1, 0, 0, 0, 0, 0, 0, 0,
                0, 1, 0, 0, 0, 0, 0, 0,
                0, 0, 1, 0, 0, 0, 0, 0,
                0, 0, 0, 1, 0, 0, 0, 0;

            return h;
            };

        Eigen::DiagonalMatrix<float, 8> q0;
        q0.diagonal() << q_vs[0], q_vs[1], q_vs[2], q_vs[3], q_vs[4], q_vs[5],
            q_vs[6], q_vs[7];

        //xa ya za yaw
        Eigen::DiagonalMatrix<float, 4> r0;
        r0.diagonal() << r_vs[0], r_vs[1], r_vs[2], r_vs[3];

        // P - error estimate covariance matrix
        Eigen::DiagonalMatrix<float, 8> p0;
        p0.setIdentity();

        tracker__ = std::make_unique<ExtendedKalmanFilter>(f0, h0, j_f0, j_h0, q0, r0, p0);
    }
    Eigen::VectorXf MIddleCarState::get_last_state() const
    {
        Eigen::VectorXf state = target_state;
        state(0) = state(0) * 100;
        state(1) = state(1) * 100;
        state(2) = state(2) * 100;
        state(4) = state(4) * 100;
        state(5) = state(5) * 100;
        state(6) = state(6) * 100;
        return state;
    }

    Eigen::VectorXf MIddleCarState::update(Eigen::VectorXf measurement, double new_d_t)
    {
        this->d_t = new_d_t;
        Eigen::VectorXf pre_state = tracker__->predict();

        measurement(0) = measurement(0) / 100;
        measurement(1) = measurement(1) / 100;
        measurement(2) = measurement(2) / 100;

        this->target_state = tracker__->update(measurement);
        return get_last_state();
    };

    Eigen::VectorXf MIddleCarState::reset(Eigen::VectorXf state)
    {
        state(0) = state(0) / 100;
        state(1) = state(1) / 100;
        state(2) = state(2) / 100;
        state(4) = state(4) / 100;
        state(5) = state(5) / 100;
        state(6) = state(6) / 100;

        tracker__->setState(state);

        this->target_state = state;
        //return get_last_state();

        return get_last_state();
    };

    Eigen::VectorXf MIddleCarState::reset_maintain_speed(Eigen::VectorXf measurement, double new_d_t)
    {
        measurement(0) = measurement(0) / 100;
        measurement(1) = measurement(1) / 100;
        measurement(2) = measurement(2) / 100;


        // 考虑此时滤波器的更新
        this->d_t = new_d_t;
        Eigen::VectorXf pre_state = tracker__->predict();

        Eigen::VectorXf new_state(8);

        // Eigen::VectorXf state_xyz(4);
        // state_xyz << measurement(0), measurement(1), measurement(2), 0; // 无需考虑转速
        // // 更新xyz
        // Eigen::VectorXf pre_state_xyz = tracker__->update(state_xyz);    
        // new_state << measurement(0), measurement(1), measurement(2), measurement(3) * 180 / CV_PI, 
        //     pre_state_xyz(4), pre_state_xyz(5), pre_state_xyz(6), pre_state(7); // 转速取pre_state

        new_state << measurement(0), measurement(1), measurement(2), measurement(3),
            pre_state(4), pre_state(5), pre_state(6), pre_state(7);


        tracker__->setState(new_state);

        // return get_last_state();

        this->target_state = new_state;

        return get_last_state();
    }

    EkfCarState::EkfCarState(const std::vector<double>& q_v, const std::vector<double>& r_v)
        :CarStateBase()
    {
        auto f = [this](const Eigen::VectorXf& x) {
            Eigen::VectorXf x_new = x;
            x_new(0) += x(4) * d_t;
            x_new(1) += x(5) * d_t;
            x_new(2) += x(6) * d_t;
            x_new(3) += x(7) * d_t;
            return x_new;
            };
        // J_f - Jacobian of process function
        // 初始化一些滤波值
        auto j_f = [this](const Eigen::VectorXf&) {
            Eigen::MatrixXf f(8, 8);
            // clang-format off
            f << 1, 0, 0, 0, d_t, 0, 0, 0,
                0, 1, 0, 0, 0, d_t, 0, 0,
                0, 0, 1, 0, 0, 0, d_t, 0,
                0, 0, 0, 1, 0, 0, 0, d_t,
                0, 0, 0, 0, 1, 0, 0, 0,
                0, 0, 0, 0, 0, 1, 0, 0,
                0, 0, 0, 0, 0, 0, 1, 0,
                0, 0, 0, 0, 0, 0, 0, 1;
            // clang-format on
            return f;
            };
        // h - Observation function
        auto h = [this](const Eigen::VectorXf& x) {
            Eigen::VectorXf z(4);
            float xc = x(0), yc = x(1), yaw = CV_PI / 2 + x(3);
            float r = HIGH_ARMOR_STATE ? high_armor_radius : low_armor_radius;
            r = r / 100;
            z(0) = xc - r * cos(yaw);  // xa
            z(1) = yc - r * sin(yaw);  // ya
            z(2) = x(2);               // za
            z(3) = x(3);               // yaw
            return z;
            };
        // J_h - Jacobian of observation function
        auto j_h = [this](const Eigen::VectorXf& x) {
            Eigen::MatrixXf h(4, 8);
            float yaw = CV_PI / 2 + x(3);
            float r = HIGH_ARMOR_STATE ? high_armor_radius : low_armor_radius;
            r = r / 100;
            // clang-format off
            //    xc   yc   zc   yaw         vxc  vyc  vzc  vyaw r
            h << 1, 0, 0, r* sin(yaw), 0, 0, 0, 0,
                0, 1, 0, -r * cos(yaw), 0, 0, 0, 0,
                0, 0, 1, 0, 0, 0, 0, 0,
                0, 0, 0, 1, 0, 0, 0, 0;
            // clang-format on
            return h;
            };
        // Q - process noise covariance matrix 过程噪声协方差矩阵
        // 1e-2, 1e-2, 1e-2, 2e-2, 5e-2, 5e-2, 1e-4, 4e-2
        Eigen::DiagonalMatrix<float, 8> q;
        q.diagonal() << q_v[0], q_v[1], q_v[2], q_v[3], q_v[4], q_v[5], q_v[6], q_v[7];
        // R - measurement noise covariance matrix
         // 1e-1, 1e-1, 1e-1, 2e-1
        Eigen::DiagonalMatrix<float, 4> r;
        r.diagonal() << r_v[0], r_v[1], r_v[2], r_v[3];
        // P - error estimate covariance matrix
        Eigen::DiagonalMatrix<float, 8> p0;
        p0.setIdentity();
        tracker__ = std::make_unique<ExtendedKalmanFilter>(f, h, j_f, j_h, q, r, p0);
    }
    Eigen::VectorXf EkfCarState::get_last_state() const
    {
        Eigen::VectorXf state = target_state;
        state(0) = target_state(0) * 100;
        state(1) = target_state(2) * 100;
        state(2) = target_state(1) * 100;
        state(4) = target_state(4) * 100;
        state(5) = target_state(6) * 100;
        state(6) = target_state(5) * 100;
        return state;
    }
    Eigen::VectorXf EkfCarState::update(Eigen::VectorXf measurement, double new_d_t)
    {
        this->d_t = new_d_t;
        Eigen::VectorXf pre_state = tracker__->predict();

        Eigen::VectorXf new_state = measurement;
        new_state(0) = measurement(0) / 100;
        new_state(1) = measurement(2) / 100;
        new_state(2) = (HIGH_ARMOR_STATE ? measurement(1) : measurement(1) + high_low_height_differ) / 100;

        this->target_state = tracker__->update(new_state);
        return get_last_state();
    }
    Eigen::VectorXf EkfCarState::reset(Eigen::VectorXf state)
    {
        Eigen::VectorXf new_state = state;
        new_state(0) = state(0) / 100;
        new_state(1) = state(2) / 100;
        new_state(2) = state(1) / 100;
        new_state(4) = state(4) / 100;
        new_state(5) = state(6) / 100;
        new_state(6) = state(5) / 100;
        tracker__->setState(new_state);
        this->target_state = new_state;
        return get_last_state();
    }
    Eigen::VectorXf EkfCarState::reset_maintain_speed(Eigen::VectorXf measurement, double new_d_t)
    {
        this->d_t = new_d_t;
        Eigen::VectorXf pre_state = tracker__->predict();
        Eigen::VectorXf last_state = target_state;

        Eigen::VectorXf state = measurement;
        state(0) = measurement(0) / 100;
        state(1) = measurement(2) / 100;
        state(2) = (HIGH_ARMOR_STATE ? measurement(1) : measurement(1) + high_low_height_differ) / 100;
        state(3) = measurement(3);
        this->target_state = tracker__->update(state);

        Eigen::VectorXf new_state = target_state;
        new_state(3) = measurement(3);
        new_state(7) = last_state(7);

        tracker__->setState(new_state);
        this->target_state = new_state;
        return get_last_state();
    }
    ;

}