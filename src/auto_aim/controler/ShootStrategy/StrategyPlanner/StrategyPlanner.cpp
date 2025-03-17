#include"StrategyPlanner.h"

namespace rm
{
	StrategyPlanner::StrategyPlanner(float min_shoot_track_rpm,
		float min_shoot_middle_rpm,
		float min_shoot_precise_rpm,
		float min_shoot_track_distance,
		float max_shoot_track_distance)
		:min_shoot_track_rpm(min_shoot_track_rpm),
		min_shoot_middle_rpm(min_shoot_middle_rpm),
		min_shoot_precise_rpm(min_shoot_precise_rpm),
		min_shoot_track_distance(min_shoot_track_distance),
		max_shoot_track_distance(max_shoot_track_distance)
	{
	}
	StrategyPlanner::STATE StrategyPlanner::state_update(Eigen::VectorXf car_state, const int& grade)
	{

		// 可自定义你的状态更新!

		float rpm_abs = abs(car_state(7) * 30 / CV_PI);
		float distance = cv::norm(cv::Point2f(car_state(0), car_state(2)));




		////////////////////////////////////////////////////////////////////////////
		// 英雄自瞄
		//if (rpm_abs < 7.0) {
		//	return STATE::shoot_translation; // 只要转速过低,就采取平移击打
		//}
		//else if (rpm_abs < 20.0) // 瞄准每个装甲板中心击打
		//{
		//	return STATE::shoot_middle;
		//}
		//else {
		//	return STATE::shoot_hero;
		//}

		////////////////////////////////////////////////////////////////////////////

		////////////////////////////////////////////////////////////////////////////
		// 步兵自瞄
		if (rpm_abs < min_shoot_track_rpm) {
			return STATE::shoot_translation; // 只要转速过低,就采取平移击打
		};

		if (rpm_abs > min_shoot_precise_rpm) {
			return STATE::shoot_precise;
		}
		else if (rpm_abs > min_shoot_middle_rpm) { // 大于即瞄中心
			return STATE::shoot_middle;
		}
		else if (distance > max_shoot_track_distance || distance < min_shoot_track_distance) { // 距离超过跟随模式射程,设置为瞄准中心
			return STATE::shoot_middle;
		}
		else if (grade < 3) { // 等级小于3,瞄准中心
			return STATE::shoot_middle;
		}
		else {
			return STATE::shoot_track; // 进入跟随模式
		};
		////////////////////////////////////////////////////////////////////////////
	};


}