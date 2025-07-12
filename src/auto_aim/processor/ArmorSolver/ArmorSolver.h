#ifndef _ARMORSOLVER_H_
#define _ARMORSOLVER_H_
#include"../ArmorPos/ArmorPos.h"
#include"../Coordinate/Coordinate.h"
#include"FaceAngleSolver/FaceAngleSolver.h"
namespace rm
{
	/*
	* 装甲板解算类
	* 将Detector层传来的数据转为绝对坐标系下信息
	* 只负责检测,不负责筛选
	* 
	* 朝向角修正:远距离朝向角可能取反,根据转速方向判断当前转到相机坐标系下朝向角应该根据角度变化
	* 发生朝向角跳变必须同时保证角度跳变和位置跳变
	*/
	class ArmorSolver
	{
		Coordinate* Coordinate__;
		FaceAngleSolver* FaceAngleSolver__;
	public:
		ArmorSolver() {};
		ArmorSolver(Coordinate* Coordinate__, FaceAngleSolver* FaceAngleSolver__);
		// 仅解算出装甲板位置信息
		bool solve_armor_pnp(const float& car_yaw, const float& car_pitch, ArmorPos& armorpos);
		// 解算朝向信息
		void solve_armor_face(ArmorPos& armorpos, const float& car_yaw, const float& car_pitch);
		// 直接解算全部装甲板位姿
		bool solve_armor_pos(const float& car_yaw, const float& car_pitch, ArmorPos& armorpos);
	public: // 用于debug
		cv::Mat draw_face_angle(cv::Mat src,const ArmorPos& armor_pos, const float& car_yaw, cv::Scalar color);
	};
}


#endif // !_ARMORSOLVER_H_
