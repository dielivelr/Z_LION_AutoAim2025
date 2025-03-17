#ifndef _TRANSLATIONSTATE_H_
#define _TRANSLATIONSTATE_H_
#include"../Ekf/Ekf.h"
#include <memory>
#include<iostream>
#include<vector>
namespace rm
{
	class TranslationStateBase
	{
	public:
		TranslationStateBase() = default;
		virtual ~TranslationStateBase() {};
	public:
		virtual Eigen::VectorXf get_last_state() const;
		// 平移观测器状态更新 观测采用角度值,滤波器采用弧度值
		virtual	Eigen::VectorXf update(Eigen::VectorXf measurement, double new_d_t) = 0;
		// 平移观测器状态重置
		virtual Eigen::VectorXf reset(Eigen::VectorXf state) = 0;
		// 平移观测器保持速度不变而切换位置
		virtual Eigen::VectorXf reset_maintain_speed(Eigen::VectorXf measurement, double new_d_t) = 0;
	protected:
		Eigen::VectorXf target_state; // 目标状态,面向内部,角度单位为角度
	};

	class EkfTranslationState : public TranslationStateBase
	{
	public:
		EkfTranslationState() = default;
		EkfTranslationState(const std::vector<double>& q_vs, const std::vector<double>& r_vs);
		// 平移观测器状态更新 观测采用角度值,滤波器采用弧度值
		Eigen::VectorXf update(Eigen::VectorXf measurement, double new_d_t) override;
		// 平移观测器状态重置
		Eigen::VectorXf reset(Eigen::VectorXf state) override;
		// 平移观测器保持速度不变而切换位置
		Eigen::VectorXf reset_maintain_speed(Eigen::VectorXf measurement, double new_d_t) override;
	private:
		float d_t = 0.01;
		std::unique_ptr<ExtendedKalmanFilter> tracker__; // 卡尔曼滤波器
	};
};

#endif // !_TRANSLATIONSTATE_H_
