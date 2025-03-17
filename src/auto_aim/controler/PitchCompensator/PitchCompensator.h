#ifndef _PITCHCOMPENSATOR_H_
#define _PITCHCOMPENSATOR_H_

#include<Eigen/Dense>
#include<opencv2/opencv.hpp>
namespace rm
{
	/*
	* 计算pitch抬升角
	* 负责计算出抬升后的瞄准三维点
	*/
	class PitchCompensator
	{
	public:
		PitchCompensator() = default;
		PitchCompensator(const std::function<float(const float&)>& horizontal_distance_correct_func,
			float bullet_speed, float pitch_compensation_amplitude);
		PitchCompensator(float bullet_speed, float pitch_compensation_amplitude);
	public:
		cv::Point3f pitch_equation_solve(cv::Point3f target_point);
		
		cv::Point3f solve_pitch_fly_delay_amplitude(float& fly_delay, cv::Point3f target_point);
		// 参考天大的迭代飞行延迟
		cv::Point3f solve_pitch_fly_delay(float& fly_delay, cv::Point3f target_point);
	private:
		// 水平距离修正函数
		std::function<float(const float&)> horizontal_distance_correct_func;
	private:
		float bullet_speed; // 弹速
		const float GRAVITY = 9.78; // 重力补偿
		float pitch_compensation_amplitude; // pitch补偿占比
	};
}

#endif // !PITCHCOMPENSATOR
