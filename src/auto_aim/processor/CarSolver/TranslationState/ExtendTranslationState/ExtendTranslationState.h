
#ifndef _EXTENDTRANSLATIONSTATE_H_
#define _EXTENDTRANSLATIONSTATE_H_

#include"../TranslationState.h"
#include"../../AdaptiveEKF.h"
#include <vector>
#include <cmath>
#include <ceres/ceres.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

namespace rm
{
    struct TransPredict {
        /*
         * 此处定义匀速直线运动模型
         * x y z vx vz
         */
        template<class T>
        void operator()(const T x0[5], T x1[5]) {
            x1[0] = x0[0] + delta_t * x0[3];
            x1[1] = x0[1];
            x1[2] = x0[2] + delta_t * x0[4];
            x1[3] = x0[3];
            x1[4] = x0[4];
        };
    
        float delta_t;
    };
    
    template<class T>
    void xyz2pyd(T xyz[3], T pyd[3]) {
        /*
         * 世界坐标系下: x y z
         * 绝对坐标系下: x z y 
         * 工具函数：将 xyz 转化为 pitch、yaw、distance
         */
        pyd[0] = ceres::atan2(xyz[1], ceres::sqrt(xyz[0]*xyz[0]+xyz[2]*xyz[2]));  // pitch
        pyd[1] = ceres::atan2(xyz[2], xyz[0]);  // yaw
        pyd[2] = ceres::sqrt(xyz[0]*xyz[0]+xyz[2]*xyz[2]+xyz[1]*xyz[1]);  // distance
    };
    
    struct TransMeasure {
        /*
         * 工具函数的类封装
         */
        template<class T>
        void operator()(const T x[5], T y[3]) {
            T x_[3] = {x[0], x[1], x[2]};
            xyz2pyd(x_, y);
        }
    };

    // 基于扩展卡尔曼滤波的平移观测器
    class ExtendTranslationState : public TranslationStateBase
    {
    public:
		ExtendTranslationState() = default;
		ExtendTranslationState(const std::vector<double>& q_vs, const std::vector<double>& r_vs);

        Eigen::VectorXf get_last_state() const override;

		Eigen::VectorXf update(Eigen::VectorXf measurement, double new_d_t) override;

		Eigen::VectorXf reset(Eigen::VectorXf state) override;

		Eigen::VectorXf reset_maintain_speed(Eigen::VectorXf measurement, double new_d_t) override;
	private:
        float d_t = 0.01;
        AdaptiveEKF<5, 3> ekf;
    };
};

#endif
