#ifndef _CATSTATE_H_
#define _CATSTATE_H_
#include"../../ArmorPos/ArmorPos.h"
#include"../Ekf/Ekf.h"
namespace rm
{
	class CarStateBase
	{
	public:
		CarStateBase();
		virtual ~CarStateBase() {};
	public: // 整车观测接口
		virtual Eigen::VectorXf get_last_state() const = 0;
		virtual Eigen::VectorXf update(Eigen::VectorXf measurement, double new_d_t) = 0;
		virtual Eigen::VectorXf reset(Eigen::VectorXf state) = 0;
		virtual Eigen::VectorXf reset_maintain_speed(Eigen::VectorXf measurement, double new_d_t) = 0;
		//// 直接设置目标状态
		//virtual void set_target_state(Eigen::VectorXf target_state);
	public: // 提供车状态部分的接口
		// 朝向角跳变判断
		bool angle_jump(float angle) const;
		// 更新上一帧朝向角
		void set_last_yaw(float angle);
		// 读上一帧朝向角
		float get_last_yaw() const;
		// 获取车的当前观测角对应的半径
		float get_observe_armor_radius() const;
		// 获取车的当前观测角对应的高度差
		float get_high_low_height_differ() const;
		// 获取高低轴半径
		void get_high_low_radius(float& high_radius, float& low_radius) const;
		// 设置半径和高度差
		void set_radius(float high_armor_radius, float low_armor_radius);
		void set_high_low_height_differ(float high_low_height_differ);
	public: // 对高低状态进行操作
		void set_high_armor_state(bool high_armor_state); // 设置状态
		void inversion_high_armor_state(); // 状态取反
		bool get_high_armor_state() const; // 获取当前状态
	protected: // 滤波器
		Eigen::VectorXf target_state; // 目标状态
		float d_t = 0.01;
		bool HIGH_ARMOR_STATE; // 是否处于观测高装甲板状态
		float high_armor_radius; // 高装甲板对应半径
		float low_armor_radius; // 低装甲板对应半径
		float high_low_height_differ; // 高装甲板减低装甲板的高度差,由于绝对坐标系Y方向向下,必为负数
	protected: // record
		double last_yaw; // 上一帧朝向角,也是观测装甲板对应的朝向角
	};

	/*
	* 车中心更新类
	* 状态机模型,根据当前装甲板高低判断当前状态,来优化模型
	* 车更新时,原则上使用高处装甲板的位置进行更新
	* 对于平移向量,可以在子类中拓展
	*/
	class MIddleCarState : public CarStateBase
	{
	public:
		MIddleCarState() = default;
		MIddleCarState(const std::vector<double>& q_vs, const std::vector<double>& r_vs);
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
		std::unique_ptr<ExtendedKalmanFilter> tracker__; // 卡尔曼滤波器
	};

	/*
	* ekf观测器
	* 输入值为装甲板位置
	*/
	class EkfCarState : public CarStateBase
	{
	public:
		EkfCarState() = default;
		EkfCarState(const std::vector<double>& q_vs, const std::vector<double>& r_vs);
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
		std::unique_ptr<ExtendedKalmanFilter> tracker__; // 卡尔曼滤波器
	};

}

#endif // !_CATSTATE_H_
