#ifndef _COORDINATE_H_
#define _COORDINATE_H_
#include"../ArmorPos/ArmorPos.h"

// 坐标转换相关
namespace rm 
{
	/******************
	* 大小装甲板尺寸参数
	******************/
	static const float small_armor_length = 13.5;
	static const float small_armor_width = 5.5;
	static const float big_armor_length = 22.5;
	static const float big_armor_width = 5.5;
	static const float armor_pitch = -15.0 / 180 * CV_PI;

	static const std::vector<cv::Point3f> small_armor_world_points
		= { 
		cv::Point3f(0, -small_armor_length / 2, small_armor_width / 2),
		cv::Point3f(0, -small_armor_length / 2, -small_armor_width / 2),
		cv::Point3f(0, small_armor_length / 2, -small_armor_width / 2),
		cv::Point3f(0, small_armor_length / 2, small_armor_width / 2),
	};
	static const std::vector<cv::Point3f> big_armor_world_points
		= {
		cv::Point3f(0, -big_armor_length / 2, big_armor_width / 2),
		cv::Point3f(0, -big_armor_length / 2, -big_armor_width / 2),
		cv::Point3f(0, big_armor_length / 2, -big_armor_width / 2),
		cv::Point3f(0, big_armor_length / 2, big_armor_width / 2),
	};
	


	/*
	* 坐标系类
	* 负责坐标系转换
	*/
	class Coordinate
	{
	public:
		Coordinate() = default;
		Coordinate(cv::Mat intrinsic, cv::Mat distcoeffs,
			const float& shaft_differ_x, const float& shaft_differ_y, 
			const float& shaft_differ_z, bool SOLVEPNP_IPPE,
			const float& sot_shaft_differ_x,
			const float& sot_shaft_differ_y,
			const float& sot_shaft_differ_z,
			const float& sot_shifting_yaw,
			const float& sot_shifting_pitch,
			const float& wheel_to_center_distance);
		// 转换为ArmorPos,根据自定义消息,这样转换相对高效
		bool trans_aromr_pos(const float& car_yaw, const float& car_pitch, ArmorPos& armorpos);
	public:
		// 为外部提供简要的接口
		cv::Point2f abs2pixel(cv::Point3f position, const float& car_yaw, const float& car_pitch);

		// 绝对坐标系转相机坐标系
		cv::Point3f abs2cam(cv::Point3f position, const float& car_yaw, const float& car_pitch);

		// 相机坐标系转绝对坐标系
		cv::Point3f cam2abs(cv::Point3f position, const float& car_yaw, const float& car_pitch);

		// 旋转角
		cv::Point3f rotate_angle(const cv::Point3f& position,const float& rotate_yaw, const float& rotate_pitch);

		// 坐标系变换
		cv::Point3f coordinate_transformation(const cv::Point3f& position, const float& car_yaw, const float& car_pitch,
			const float& shaft_differ_x, const float& shaft_differ_y, const float& shaft_differ_z);

		// 反向坐标系变换
		cv::Point3f reverse_coordinate_transformation(const cv::Point3f& position, const float& car_yaw, const float& car_pitch,
			const float& shaft_differ_x, const float& shaft_differ_y, const float& shaft_differ_z);

		// 获取欧拉角
		// 单位为弧度
		void pos2angles(float& yaw, float& pitch, const cv::Point3f& position);

	public:
		// 绝对坐标系转到射击坐标系
		cv::Point3f abs2sot(const cv::Point3f& abs_pos, const float& car_yaw, const float& car_pitch , float yaw_speed = 0);

		// 射击坐标系转到绝对坐标系
		cv::Point3f sot2abs(const cv::Point3f& sot_pos, const float& car_yaw, const float& car_pitch, float yaw_speed = 0);

	private:

		// 绝对坐标系转相机坐标系
		cv::Point3f abs2cam(cv::Point3f position, const float& car_yaw, const float& car_pitch,
			const float& shaft_differ_x, const float& shaft_differ_y, const float& shaft_differ_z);

		// PnP解算
		bool pnp(cv::Mat& Rvec, cv::Mat& Tvec, cv::Mat intrinsic, cv::Mat distcoeffs,
			const std::vector<cv::Point2f>& four_pixel_points
			, bool armor_small_size, bool SOLVEPNP_IPPE);

		// 矩阵计算
		cv::Mat get_rot_mat_x(float x);
		cv::Mat get_rot_mat_y(float y);
		cv::Mat get_rot_mat_z(float z);
		cv::Mat get_rot_mat_xyz(float x, float y, float z);
		cv::Mat get_tans_mat(float x, float y, float z);

		// 相机坐标系转绝对坐标系
		cv::Point3f cam2abs(cv::Point3f position, const float& car_yaw, const float& car_pitch,
			const float& shaft_differ_x, const float& shaft_differ_y, const float& shaft_differ_z);

		// 相机坐标系转像素坐标系
		cv::Point2f cam2pixel(cv::Point3f position, cv::Mat intrinsic, cv::Mat distcoeffs);

		// 相机坐标系转像素坐标系
		cv::Point2f abs2pixel(cv::Point3f position, const float& car_yaw, const float& car_pitch,
			const float& shaft_differ_x, const float& shaft_differ_y, const float& shaft_differ_z,
			cv::Mat intrinsic, cv::Mat distcoeffs);

	private:
		float shaft_differ_x, shaft_differ_y, shaft_differ_z; // 中心点偏移量
		cv::Mat intrinsic; // 相机内参
		cv::Mat distcoeffs; // 相机外参
		bool SOLVEPNP_IPPE = true; // 是否采用四点共面策略
	private: // 射击坐标系转换数据
		float sot_shaft_differ_x;
		float sot_shaft_differ_y;
		float sot_shaft_differ_z;
		float sot_shifting_yaw;
		float sot_shifting_pitch;
		float wheel_to_center_distance; // 摩擦轮到中心距离
	};
}
#endif // !COORDINATE_H

