#ifndef _CONTROLER_H_
#define _CONTROLER_H_


#include"PitchCompensator/PitchCompensator.h"
#include"RangeSetter/RangeSetter.h"
#include"ShootStrategy/ShootStrategy.h"
#include"ShootStrategy/StrategyPlanner/StrategyPlanner.h"
#include"TimePredictor/TimePredictor.h"
#include"TransControler.h"
#include"../processor/CarSolver/CarSolver.h"
#include"../processor/Coordinate/Coordinate.h"

namespace rm
{
	/*
	* 开火逻辑
	* 由于逻辑相对简单,采用状态机模型.*在射击坐标系下判断*
	* 状态机:瞄准中心 / 跟随击打 / 间断开火 / 连续开火 / 取消开火
	* 瞄准中心: 设定刷新频率,只有期望到达最优解或超过最优解时,才传输1
	* 跟随击打: 只要瞄准,就能开火,返回1
	* 平动击打: 不考虑装甲板旋转的情况
	* 精准击打: 每个装甲板尽可能在转到中心时开火
	* 间断开火:设置一个间隔,相邻射击只能发生在间隔之外
	* 连续开火:一直返回1
	* 取消开火:一直返回0
	*
	* 返回通讯所需的数据: 
			pitch,
			yaw, 
			yaw_speed,
			canshoot,
			move_state,
			ifget,
			enemy_kind,
			enemy_x,
			enemy_y,
	*/
	class Controler
	{
		Coordinate* Coordinate__;
		PitchCompensator* PitchCompensator__;
		RangeSetter* RangeSetter__;
		TimePredictor* TimePredictor__;
		ShootTrackStrategyBase* ShootTrackStrategy__;
		ShootMiddleStrategy* ShootMiddleStrategy__;
		HeroShootStrategy* HeroShootStrategy__;
		PrecisionShootStrategy* PrecisionShootStrategy__;
		ShootTranslationStrategy* ShootTranslationStrategy__;
		ShootMiddleVagueStrategy* ShootMiddleVagueStrategy__;
		StrategyPlanner* StrategyPlanner__;
		ShootDelayer ShootDelayer_; // 更新目标时进行延迟
	public:
		Controler() = default;
		Controler(
			Coordinate* Coordinate__,
			PitchCompensator* PitchCompensator__,
			RangeSetter* RangeSetter__,
			TimePredictor* TimePredictor__,
			ShootTrackStrategyBase* ShootTrackStrategy__,
			ShootMiddleStrategy* ShootMiddleStrategy__,
			HeroShootStrategy* HeroShootStrategy__,
			PrecisionShootStrategy* PrecisionShootStrategy__,
			ShootTranslationStrategy* ShootTranslationStrategy__,
			ShootMiddleVagueStrategy* ShootMiddleVagueStrategy__,
			StrategyPlanner* StrategyPlanner__,
			float delay_shooting_param,
			StrategyPlanner::STATE state = StrategyPlanner::STATE::shoot_default);
	public:
		// 基于迭代的控制层,计算出理论发射延迟最优解,它会在迭代到第三次收敛
		TransControler iteration_control(int target_kind, float program_timer_differ, Eigen::VectorXf car_state,
			Eigen::VectorXf trans_state, const float& car_yaw, const float& car_pitch, const float& car_yaw_speed, const int& grade);

	public: // debug

		StrategyPlanner::STATE get_controler_state() const;

		// 画瞄准目标,若能够射击,画超酷叉
		cv::Mat draw_aim_targets(cv::Mat src, bool canshoot,  const float& car_yaw, const float& car_pitch,
			cv::Scalar color, int radius = 30, int thickness = 5);
		
		cv::Point3f debug_absolute_aim_target;
		
		Eigen::VectorXf last_pred_state; // 预测后的状态
		Eigen::VectorXf last_true_state; // 当前状态
	private: // 获取速度数据
		// 根据状态机状态更新
		TransControler update_by_state(
			const std::vector<cv::Point3f>& pred_car_four_points_shoot,
			const std::vector<cv::Point3f>& pred_car_four_points_absolute,
			const Eigen::VectorXf& pred_state, const float& car_yaw, const float& car_pitch, const float& car_yaw_speed);

		// 计算平移速度
		float solve_yaw_trans_speed(cv::Point3f aim_target_absolute,
			const float& send_yaw, const Eigen::VectorXf& state, const float& car_yaw, const float& car_pitch, const float& car_yaw_speed);

		// 计算旋转速度
		float solve_yaw_rotate_speed(cv::Point3f aim_target_absolute,
			const float& send_yaw, const Eigen::VectorXf& state, const float& car_yaw, const float& car_pitch, const float& car_yaw_speed);

		// 绕点旋转
		cv::Point2f rotate_point(const cv::Point2f& center, const cv::Point2f& point, float angle);

	private:
		void clear();
		bool lock = 0; // 锁定模式,不进行更换
		StrategyPlanner::STATE state; // 当前状态
	
	private:
		cv::Point3f last_absolute_aim_target;
		bool last_iteration;
	private: // 缓冲帧
		int last_target_kind; // 上一个击打目标
	};
};

#endif // _CONTROLER_H_
