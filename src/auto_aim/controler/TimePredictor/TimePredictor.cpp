#include"TimePredictor.h"

namespace rm
{
	TimePredictor::TimePredictor(float shoot_delay, float bullet_speed, float serial_delay)
		:shoot_delay(shoot_delay), bullet_speed(bullet_speed),
		serial_delay(serial_delay)
	{
	}
	float TimePredictor::simple_time_predict(Eigen::VectorXf state, float program_timer_differ)
	{
		float average_radius = (state(8) + state(9)) / 2;
		float average_height = state(11) == 1 ? state(1) - state(10) / 2 : state(1) + state(10) / 2;

		// 飞行时间
		float distance = cv::norm(cv::Point3f(state(0), average_height, state(2) - average_radius));

		float fly_delay = distance  / this->bullet_speed;

		double pred_time = fly_delay + this->serial_delay + this->shoot_delay + program_timer_differ;

		// 当前状态是在image_timer_differ预测之后的状态,需要将该状态减去
		return pred_time;
	}
	float TimePredictor::time_predict(PitchCompensator* PitchCompensator__,
		Eigen::VectorXf state, float program_timer_differ)
	{
		float average_radius = (state(8) + state(9)) / 2;
		float average_height = state(11) == 1 ? state(1) - state(10) / 2 : state(1) + state(10) / 2;

		cv::Point3f rough_estimate_target(state(0), average_height , state(2) - average_radius);

		float fly_delay = 0.0;
		PitchCompensator__->solve_pitch_fly_delay(fly_delay, rough_estimate_target);
		double pred_time = fly_delay + this->serial_delay + this->shoot_delay + program_timer_differ;
		
		return pred_time;
	}
	float TimePredictor::time_predict(PitchCompensator* PitchCompensator__,
		cv::Point3f last_absolute_aim_target, float program_timer_differ)
	{
		float fly_delay = 0.0;
		PitchCompensator__->solve_pitch_fly_delay(fly_delay, last_absolute_aim_target);
		// 预测时间 = 飞行时间 + 串口通讯延迟 + 发弹延迟 + 程序延迟
		double pred_time = fly_delay + this->serial_delay + this->shoot_delay + program_timer_differ;

		return pred_time;
	}
	;
}