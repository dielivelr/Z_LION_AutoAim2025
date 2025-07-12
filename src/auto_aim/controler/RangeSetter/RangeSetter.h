#ifndef _RANGESETTER_H_
#define _RANGESETTER_H_
#include<opencv2/opencv.hpp>
namespace rm
{
	/*
	* 范围规划器
	* 规划射击范围
	* 通讯值设限
	*/
	class RangeSetter
	{
	public:
		RangeSetter() = default;
		RangeSetter(float max_shoot_x, float max_shoot_y, float max_shoot_middle_x, float max_send_yaw, float max_send_pitch,
			float max_send_yaw_speed, float trans_control_compensation_left, float trans_control_compensation_right,
			float rotate_control_compensation_left, float rotate_control_compensation_right);
	public:
		void set_armor_size_small(bool armor_size_small);
		bool get_armor_size_small() const;
		// 判断yaw轴是否击打命中
		bool if_yaw_shoot_hit(cv::Point3f target_point);
		bool if_yaw_shoot_hit(cv::Point3f target_point,float face_angle);
		// 中心击打判断yaw轴是否击打命中
		bool if_middle_yaw_shoot_hit(cv::Point3f target_point);
		bool if_middle_yaw_shoot_hit(cv::Point3f target_point, float face_angle);
		// 判断pitch轴是否击打命中
		bool if_pitch_shoot_hit(cv::Point3f target_point);
		// 限幅
		void send_data_range(float& send_pitch, float& send_yaw, float& send_yaw_speed);
		// 角度增幅
		float yaw_angle_increase(float send_yaw, float yaw_trans_speed, float yaw_rotate_speed, bool translation_right);
		// 获取当前max_shoot_x
		float get_max_shoot_x() const;
	private:
		bool armor_size_small; // 当前击打目标的装甲板大小
		float max_shoot_x, max_shoot_y; // 最大击打距离x,y
		float max_shoot_middle_x; // 最小中心击打x 
		float max_send_pitch, max_send_yaw, max_send_yaw_speed; // 传输值的最大,最小值,面向用户输入为角度值
		// 可能云台左右转动性能不同,设置不同的增幅参数
		float trans_control_compensation_left, trans_control_compensation_right; // 平移增幅
		float rotate_control_compensation_left, rotate_control_compensation_right; // 旋转增幅
	};
};
#endif // !_RANGESETTER_H_
