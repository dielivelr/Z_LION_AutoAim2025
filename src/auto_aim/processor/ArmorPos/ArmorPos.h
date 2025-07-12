#ifndef _ARMORPOS_H_
#define _ARMORPOS_H_
#include<Eigen/Dense>
#include<opencv2/opencv.hpp>
#include<iostream>
#include<vector>
namespace rm
{
	/*
	* 装甲板类
	* 在结算层描述的装甲板仅关注位置和姿态
	* 由于会转换到绝对坐标系下进行计算,我们仅需考虑装甲板的绝对坐标系即可
	*/
	class ArmorPos
	{
	public:
		ArmorPos() {
			small_armor = true;
			light_loss = false;
			kind = 0;
			pixel_four_point = {};
			absolute_four_point = {};
			absolute_middle_point = cv::Point3f();
			face_angle = 0;
		};
		// 图像四点
		std::vector<cv::Point2f> pixel_four_point;
		// 绝对坐标系下四点坐标
		std::vector<cv::Point3f> absolute_four_point;
		// 绝对坐标系下装甲板中心位置
		cv::Point3f absolute_middle_point;
		// 绝对朝向角,全部采用弧度制
		float face_angle;
		// 小大装甲板
		bool small_armor;
		// 类型
		int kind;
		// 缺失灯条的装甲板
		bool light_loss;
	};


}

#endif // !_ARMORPOS_H_
