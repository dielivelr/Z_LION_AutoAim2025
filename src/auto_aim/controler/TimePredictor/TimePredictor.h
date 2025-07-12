#ifndef _TIMEPREDICTOR_H_
#define _TIMEPREDICTOR_H_
#include"../PitchCompensator/PitchCompensator.h"
#include<Eigen/Dense>
#include<opencv2/opencv.hpp>
#include<iostream>
namespace rm
{
	/*
	* 时间预测器
	*/
	class TimePredictor
	{
	public:
		TimePredictor() {};
		// 面向用户使用rpm为单位 min_track_rpm * 6 = min_track_rotational_speed * 180 / CV_PI
		TimePredictor(float shoot_delay, float bullet_speed, float serial_delay);
	public:
		// 简易时间预测,直接使用点到线
		float simple_time_predict(Eigen::VectorXf state, float program_timer_differ);
		// 基于补偿角的预测
		float time_predict(PitchCompensator* PitchCompensator__,Eigen::VectorXf state, float program_timer_differ);
		// 直接对输入的位置进行映射
		float time_predict(PitchCompensator* PitchCompensator__, cv::Point3f last_absolute_aim_target, float program_timer_differ);
	private:
		float bullet_speed; // 射速
		float shoot_delay; // 发弹延迟
		float serial_delay; // 通讯延迟
	};
}
#endif // !_TIMEPREDICTOR_H_
