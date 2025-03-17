#ifndef _PROCESSOR_H_
#define _PROCESSOR_H_
#include"ArmorSolver/ArmorSolver.h"
#include"CarSolver/CarSolver.h"
#include"TargetPlanner/TargetPlanner.h"
#include"TransProcessor.h"
namespace rm
{

	class ProcessorBase
	{
		ArmorSolver* ArmorSolver__;
		TargetPlanner* TargetPlanner__;
	public:
		ProcessorBase() = default;
		ProcessorBase(ArmorSolver* ArmorSolver__, TargetPlanner* TargetPlanner__, float vxz_truncation_rotate_rpm, float max_detect_distance);
	public: 

		TransProcessor process(std::vector<ArmorPos>& armor_poses, float new_d_t, const float& car_yaw, const float& car_pitch);
		void clear();

	public: // debug
		cv::Mat draw_car(NoneSolver* NoneSolver__, FaceAngleSolver* FaceAngleSolver__, Coordinate* Coordinate__,
			cv::Mat src, const Eigen::VectorXf& state, bool armor_small_size,
			const float& car_yaw, const float& car_pitch, cv::Scalar color, int thickness = 4);
		std::vector<std::vector<cv::Point2f>> get_draw_car_points(NoneSolver* NoneSolver__, FaceAngleSolver* FaceAngleSolver__, Coordinate* Coordinate__,
			const Eigen::VectorXf& state, bool armor_small_size,
			const float& car_yaw, const float& car_pitch);

	private:
		virtual std::vector<Eigen::VectorXf> init(const std::vector<ArmorPos>& armor_poses,
			float new_d_t, const float& car_yaw,float init_new_rotate_speed) = 0;

		virtual std::vector<Eigen::VectorXf> update(const std::vector<ArmorPos>& armor_poses,
			float new_d_t, const float& car_yaw) = 0;

		virtual TransProcessor data_to_transprocessor(const std::vector<Eigen::VectorXf>& States, int target_kind) = 0;

	protected: // 转速记录
		float record_rotate_speed[3] = { 0 }; // 英雄 哨兵 步兵
		float record_last_rotate_speed; // 记录的当前观测的转速
		float vxz_truncation_rotate_speed;
		float max_detect_distance; // 最远识别距离
		bool first_time;
		bool buffer_state;
	private:
		float record_car_vx, record_car_vz, record_trans_vx, record_trans_vz;
	};


	/*
	* 解算类
	*/
	class Processor : public ProcessorBase
	{
		CarStateBase* CarState__;
		NoneSolver* NoneSolver__;
		SingleSolver* SingleSolver__;
		DoubleSolver* DoubleSolver__;
	public:
		Processor() = default;
		Processor(CarStateBase* CarState__,ArmorSolver* ArmorSolver__, TargetPlanner* TargetPlanner__,
			NoneSolver* NoneSolver__, SingleSolver* SingleSolver__, DoubleSolver* DoubleSolver__,
			float vxz_truncation_rotate_rpm,float max_detect_distance);
		virtual ~Processor() {};

	
	private:
		std::vector<Eigen::VectorXf> init(const std::vector<ArmorPos>& armor_poses,
			float new_d_t, const float& car_yaw, float init_new_rotate_speed) override;

		std::vector<Eigen::VectorXf> update(const std::vector<ArmorPos>& armor_poses,
			float new_d_t, const float& car_yaw) override;

		TransProcessor data_to_transprocessor(const std::vector<Eigen::VectorXf>& States, int target_kind) override;
	private:
		// 依据目标数目获取数量
		std::vector<Eigen::VectorXf> run_solve_car_strategy(CarSolver::Operation operation, const std::vector<ArmorPos>& armor_poses,
			float new_d_t, const float& car_yaw);
	};
}


#endif // !_PROCESSOR_H_
