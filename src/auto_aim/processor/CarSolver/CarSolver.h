#ifndef _CARSOLVER_H_
#define _CARSOLVER_H_
#include"CarState/CarState.h"
#include"TranslationState/TranslationState.h"
#include"DataObserver/DataObserver.h"
namespace rm
{
	
	/*
	* 整车解算工具类
	*/
	class CarSolver
	{
	protected:
		CarStateBase* CarState__;
		TranslationStateBase* TranslationState__;
	public:
		enum Operation // 操作数
		{
			update = 0,
			init,
			buffer
		};
	public:
		CarSolver() = default;
		CarSolver(CarStateBase* CarState__, TranslationStateBase* TranslationState__);
		virtual ~CarSolver() {};
	public:// debug
		// 拟合整车四装甲板
		static std::vector<cv::Point3f> fit_four_armors(cv::Point3f car_pos, float car_face_angle,
			float high_radius, float low_radius, float high_low_height_differ, bool high_armor_state);
	public:// debug
		std::vector<cv::Point3f> fit_four_armors(cv::Point3f car_pos, float car_face_angle);
	public:
		// 解算决策
		virtual std::vector<Eigen::VectorXf> solve_car_strategy(Operation operation,
			const std::vector<ArmorPos>& armor_poses, float new_d_t, const float& car_yaw) = 0;
	protected:
		// 获取车中心		
		virtual cv::Point3f get_car_pos(const ArmorPos& armor_pos) const;
	};

	/*
	* 无装甲板解算类
	* 自预测
	*/
	class NoneSolver : public CarSolver
	{
	public:
		NoneSolver() = default;
		NoneSolver(CarStateBase* CarState__, TranslationStateBase* TranslationState__);
	public:
		std::vector<Eigen::VectorXf> solve_car_strategy(Operation operation,
			const std::vector<ArmorPos>& armor_poses, float new_d_t, const float& car_yaw) override;
	private:
		// 自更新
		std::vector<Eigen::VectorXf> self_update(float new_d_t);
	};

	/*
	* 单装甲板解算类
	* 解决出现单装甲板时的状态更新方案
	* 当灯条跳变时,选择不进行速度更新,后续可优化为均进行更新
	*/
	class SingleSolver : public CarSolver
	{
		NoneSolver* NoneSolver__;
	public:
		SingleSolver() = default;
		SingleSolver(NoneSolver* NoneSolver__, CarStateBase* CarState__, TranslationStateBase* TranslationState__, bool use_car_middle_pred = false);
	public:
		std::vector<Eigen::VectorXf> solve_car_strategy(Operation operation,
			const std::vector<ArmorPos>& armor_poses, float new_d_t, const float& car_yaw) override;
		void set_record_rotate_speed(float record_rotate_speed);
		// 直接更新,保持装甲板状态
		std::vector<Eigen::VectorXf> solve_car_maintain_high_armor_state(Operation operation, 
			ArmorPos armor_pos, float new_d_t, const float& car_yaw);
	private:
		// init
		std::vector<Eigen::VectorXf> init(ArmorPos armor_pos, 
			float new_d_t, const float& car_yaw,const float& rotate_speed = 0);
		std::vector<Eigen::VectorXf> buffer(ArmorPos armor_pos, float new_d_t, const float& car_yaw);

		// 对一个装甲板做检测朝向角跳变,如果存在更新装甲板状态,否则正常更新
		std::vector<Eigen::VectorXf> detect_update_state_angle_jump(ArmorPos armor_pos, float new_d_t);

	private:
		float record_rotate_speed;
		bool use_car_middle_pred; // 使用车中心更新车的状态
	};

	/*
	* 双装甲板解算类
	* 解决出现双装甲板时的状态更新方案
	* 由于能够检测到两个装甲板,除了更新半径和高度差,还可以根据两个朝向角的信息进行更新
	*/
	class DoubleSolver : public CarSolver
	{
		// 调用其他解算类的方法
		SingleSolver* SingleSolver__;
		NoneSolver* NoneSolver__;
		AngleOptimizer* AngleOptimizer__;
		RadiusOptimizer* RadiusOptimizer__;
	public:
		DoubleSolver() = default;
		DoubleSolver(SingleSolver* SingleSolver__, NoneSolver* NoneSolver__, 
			CarStateBase* CarState__, TranslationStateBase* TranslationState__
			, AngleOptimizer* AngleOptimizer__,int radius_optimizer_queue_length = 10);
		~DoubleSolver();
	public:
		// 运行相应策略
		std::vector<Eigen::VectorXf> solve_car_strategy(Operation operation,
			const std::vector<ArmorPos>& armor_poses, float new_d_t, const float& car_yaw) override;
	private:
		// 反向延伸得半径与高度差并返回高装甲板
		ArmorPos calculate_car_size(float& high_armor_radius, float& low_armor_radius, float& high_low_height_differ,
			const ArmorPos& left_armor_pos, const ArmorPos& right_armor_pos);
		// 取交点
		cv::Point2f get_intersection_point(float k1, float k2, cv::Point2f point1, cv::Point2f point2);
	};

}


#endif // !_CARSOLVER_H_
