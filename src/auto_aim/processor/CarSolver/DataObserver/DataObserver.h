#ifndef _DUALANGLEOBSERVER_H_
#define _DUALANGLEOBSERVER_H_
#include"../../ArmorSolver/FaceAngleSolver/FaceAngleSolver.h"
#include<iostream>
#include<vector>

namespace rm 
{
	/*
	* 朝向角优化器
	* 双朝向角有时会出现一个角度的检测跳变
	* 两个朝向角精度可能不够,进行精度修正
	*/
	class AngleOptimizer
	{
	public:
		AngleOptimizer();
		AngleOptimizer(float min_angle_error, float min_update_angle = 40, float max_update_angle = 50);
		// 观测,并返回观测后的输出序列
		// 保证左侧数据为
		std::vector<float> observe(const std::vector<float>& inputs,const float& car_yaw);
		// 修正朝向角度正负
		void correct_face_angle(ArmorPos& left_armor_pos, ArmorPos& right_armor_pos, const float& car_yaw);
	private:
		float min_angle_error; // 最小角度误差,超过该误差,则选择角度绝对值较大值为观测量,单位为角度
		float min_update_angle, max_update_angle; // 标记在一定范围内才更新半径
	};

	/*
	* 半径优化器
	* 半径的精度可能不够,使用队列不断更新迭代出的半径取均值
	*/
	class RadiusOptimizer
	{
	public:
		RadiusOptimizer() { this->queue_length = 10; };
		RadiusOptimizer(int queue_length);
		// 观测,并返回观测后的输出序列
		// 保证左侧数据为
		void observe(float& high_armor_radius,float& low_armor_radius, float& height_low_differ);
		void clear();
	private:
		int queue_length; // 队列长度
		// 高/低装甲板队列
		std::vector<float> high_armor_radius_queue;
		std::vector<float> low_armor_radius_queue;
		std::vector<float> height_low_differ_queue;
	};


}

#endif // !_DUALANGLEOBSERVER_H_
