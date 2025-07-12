#ifndef _STRATEGYPLANNER_H_
#define _STRATEGYPLANNER_H_
#include<Eigen/Dense>
#include<opencv2/opencv.hpp>
namespace rm
{
	/*
	* 决策规划器
	* 定义决策功能
	* 基于不同决策,对状态信息进行处理
	*/
	class StrategyPlanner
	{
	public:
		// 状态机模型
		enum STATE
		{
			shoot_default = 0,			// 默认模式
			shoot_track,		// 跟随模式
			shoot_middle,		// 中心击打模式
			shoot_precise,		// 精准击打模式
			shoot_translation,	// 平动击打模式
			shoot_vague,		// 模糊击打模式
			shoot_hero,			// 英雄射击模式
		};
	public:
		StrategyPlanner() = default;
		StrategyPlanner(float min_shoot_track_rpm,
			float min_shoot_middle_rpm,
			float min_shoot_precise_rpm,
			float min_shoot_track_distance,
			float max_shoot_track_distance
			);
		// 策略更新
		STATE state_update(Eigen::VectorXf car_state, const int& grade);
	private:
		float min_shoot_track_rpm; // 最小跟随rpm
		float min_shoot_middle_rpm;	// 最小瞄准中心rpm
		float min_shoot_precise_rpm;// 最小精准击打rpm
		float min_shoot_track_distance; // 跟随模式允许最小射程
		float max_shoot_track_distance; // 跟随模式允许最大射程
	};
}

#endif // !_STRATEGYPLANNER_H_
