#ifndef _EXTENDCARSTATE_H_
#define _EXTENDCARSTATE_H_

#include"../CarState.h"
#include"../../AdaptiveEKF.h"
#include <vector>
#include <cmath>
#include <ceres/ceres.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

namespace rm
{
    struct CarPredict {
        /*
         * 此处定义匀速直线运动模型
         * x y z yaw vx vy vz vyaw
         */
        template<class T>
        void operator()(const T x0[8], T x1[8]) {
            x1[0] = x0[0] + delta_t * x0[4];
            x1[1] = x0[1] + delta_t * x0[5];
            x1[2] = x0[2] + delta_t * x0[6];
            x1[3] = x0[3] + delta_t * x0[7];
            x1[4] = x0[4];
            x1[5] = x0[5];
            x1[6] = x0[6]; 
            x1[7] = x0[7];
        };
    
        float delta_t;
    };

    template<class T>
    void xyza_to_pyda(T xyza[8], T pyda[4]) {
        /*
        * 工具函数：将 xyz 转化为 pitch、yaw、distance
        */
        pyda[0] = ceres::atan2(xyza[2], ceres::sqrt(xyza[0]*xyza[0]+xyza[1]*xyza[1]));  // pitch
        pyda[1] = ceres::atan2(xyza[1], xyza[0]);  // yaw
        pyda[2] = ceres::sqrt(xyza[0]*xyza[0]+xyza[1]*xyza[1]+xyza[2]*xyza[2]);  // distance
        pyda[3] = xyza[3];
    };
        
    struct CarMeasure {
        /*
         * 工具函数的类封装
         */  
        template<class T>
        void operator()(const T x[8], T y[4]) {
           
            const T xyz[3] = {
                x[0] - r * ceres::cos(T(CV_PI / 2) + x[3]),
                x[1] - r * ceres::sin(T(CV_PI / 2) + x[3]),
                x[2]
            };
            T pyd[3];
            pyd[0] = ceres::atan2(xyz[2], ceres::sqrt(xyz[0]*xyz[0]+xyz[1]*xyz[1]));  // pitch
            pyd[1] = ceres::atan2(xyz[1], xyz[0]);  // yaw
            pyd[2] = ceres::sqrt(xyz[0]*xyz[0]+xyz[1]*xyz[1]+xyz[2]*xyz[2]);  // distance
            for(int i = 0;i < 3;i++)
                y[i] = pyd[i];
            y[3] = x[3];
        };
        float r;
    };

    /*
	* 拓展卡尔曼整车观测器
	*/
	class ExtendEkfCarState : public CarStateBase
	{
	public:
        ExtendEkfCarState() = default;
        ExtendEkfCarState(const std::vector<double>& q_vs, const std::vector<double>& r_vs);
	public: // 提供整车观测的接口
		// 获取最新状态
		Eigen::VectorXf get_last_state() const override;
		// 状态更新 观测采用角度值,滤波器采用弧度值
		Eigen::VectorXf update(Eigen::VectorXf measurement, double new_d_t) override;
		// 状态重置
		Eigen::VectorXf reset(Eigen::VectorXf state) override;
		// 保持速度不变而切换位置
		Eigen::VectorXf reset_maintain_speed(Eigen::VectorXf measurement, double new_d_t) override;
	private: // 滤波器
        AdaptiveEKF<8, 4> ekf; // 卡尔曼滤波器
	};

};

#endif
