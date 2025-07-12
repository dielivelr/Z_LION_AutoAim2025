#include"RangeSetter.h"

namespace rm
{
	RangeSetter::RangeSetter(float max_shoot_x, float max_shoot_y, float max_shoot_middle_x, float max_send_yaw, float max_send_pitch,
		float max_send_yaw_speed, float trans_control_compensation_left, float trans_control_compensation_right,
		float rotate_control_compensation_left, float rotate_control_compensation_right)
		:max_shoot_x(max_shoot_x), max_shoot_y(max_shoot_y), max_shoot_middle_x(max_shoot_middle_x),
		max_send_yaw(max_send_yaw), max_send_pitch(max_send_pitch), 
		max_send_yaw_speed(max_send_yaw_speed * CV_PI / 180),
		trans_control_compensation_left(trans_control_compensation_left), 
		trans_control_compensation_right(trans_control_compensation_right),
		rotate_control_compensation_left(rotate_control_compensation_left),
		rotate_control_compensation_right(rotate_control_compensation_right)
	{
	}
	void RangeSetter::set_armor_size_small(bool armor_size_small)
	{
		this->armor_size_small = armor_size_small;
	}
	bool RangeSetter::get_armor_size_small() const
	{
		return armor_size_small;
	}
	bool RangeSetter::if_yaw_shoot_hit(cv::Point3f target_point)
	{
		float shoot_x = armor_size_small ? max_shoot_x : max_shoot_x * 1.667;
		return abs(target_point.x) < shoot_x;
	}
	bool RangeSetter::if_yaw_shoot_hit(cv::Point3f target_point, float face_angle)
	{
		float shoot_x = armor_size_small ? max_shoot_x : max_shoot_x * 1.667;
		shoot_x = shoot_x * abs(cos(face_angle));
		return abs(target_point.x) < shoot_x;
	}
	bool RangeSetter::if_middle_yaw_shoot_hit(cv::Point3f target_point)
	{
		float shoot_x = armor_size_small ? max_shoot_middle_x : max_shoot_middle_x * 1.667;
		return abs(target_point.x) < this->max_shoot_middle_x;
	}
	bool RangeSetter::if_middle_yaw_shoot_hit(cv::Point3f target_point, float face_angle)
	{
		float shoot_x = armor_size_small ? max_shoot_middle_x : max_shoot_middle_x * 1.667;
		shoot_x = shoot_x * abs(cos(face_angle));
		return abs(target_point.x) < this->max_shoot_middle_x;
	}
	bool RangeSetter::if_pitch_shoot_hit(cv::Point3f target_point)
	{
		return abs(target_point.y) < this->max_shoot_y;
	}
	void RangeSetter::send_data_range(float& send_pitch, float& send_yaw, float& send_yaw_speed)
	{
		if (abs(send_pitch) > max_send_pitch) send_pitch = send_pitch < 0 ? -max_send_pitch : max_send_pitch;
		if (abs(send_yaw) > max_send_yaw) send_yaw = send_yaw < 0 ? -max_send_yaw : max_send_yaw;
		if (abs(send_yaw_speed) > max_send_yaw_speed) send_yaw_speed = send_yaw_speed < 0 ? -max_send_yaw_speed : max_send_yaw_speed;
	}
	float RangeSetter::yaw_angle_increase(float send_yaw, float yaw_trans_speed, float yaw_rotate_speed, bool translation_right)
	{
		// 单位统一
		yaw_trans_speed = yaw_trans_speed * 180 / CV_PI;
		yaw_rotate_speed = yaw_rotate_speed * 180 / CV_PI;
		float add_yaw = translation_right ? yaw_trans_speed * this->trans_control_compensation_right +
			yaw_rotate_speed * this->rotate_control_compensation_right
			:
			yaw_trans_speed * this->trans_control_compensation_left +
			yaw_rotate_speed * this->rotate_control_compensation_left;
		return send_yaw + add_yaw;
	}
	float RangeSetter::get_max_shoot_x() const
	{
		return armor_size_small ? max_shoot_middle_x : max_shoot_middle_x * 1.667;
	}
	;
}