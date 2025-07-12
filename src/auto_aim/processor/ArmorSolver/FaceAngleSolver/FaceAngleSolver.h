#ifndef _FACEANGLESOLVER_H_
#define _FACEANGLESOLVER_H_
#include"../../Coordinate/Coordinate.h"
#include"../../ArmorPos/ArmorPos.h"
#include<iostream>
#include<vector>


namespace rm {

	/*
	* 朝向角解算基类
	*/
	class FaceAngleSolver
	{
	protected:
		Coordinate* Coordinate__;
	public:
		FaceAngleSolver() {};
		FaceAngleSolver(Coordinate* Coordinate__);
		~FaceAngleSolver() {};
		// 解算出绝对坐标系下的朝向角位置
		virtual float absolute_angle_solve(const ArmorPos& armor_pos, const float& car_yaw, const float& car_pitch) = 0;
		// 绝对朝向角与相对朝向角相互转换
		static float switch_absolute_camera_angle(float angle, const float& car_yaw);
		// 弧度转角度
		static float conversion_arc_to_angle(float angle);
		// 根据装甲板的角度,获取装甲板四点
		std::vector<cv::Point3f> get_armor_intersection(const ArmorPos& armor_pos, float absolute_angle, bool armor_small_size, float armor_pitch);
		// 装甲板旋转
		std::vector<cv::Point3f> rotate_armor(cv::Point3f armor_center, float yaw_angle,
			bool armor_small_size, float armor_pitch = rm::armor_pitch);
		// 设置敌方颜色
		void set_enemy_color(bool enemy_blue);
	protected:
		bool enemy_blue;
		// 按照yaw轴旋转装甲板
		std::vector<cv::Point3f> rotate_armor_yaw(cv::Point3f armor_center, float yaw_angle,
			bool armor_small_size);
		// 按照pitch轴旋转装甲板
		std::vector<cv::Point3f> rotate_pitch(const std::vector<cv::Point3f>& points,
			cv::Point3f pt1, cv::Point3f pt2, float pitch_angle);
		// 绕点旋转
		cv::Point3f rotate_point(const cv::Point3f& point, const cv::Mat& rot_y, const cv::Point3f& center);
	};

	/*
	* 基于三点发迭代的朝向角算法
	* 借鉴上交的23年开源
	*/
	class IterationAngleSolver : public FaceAngleSolver
	{
	public:
		IterationAngleSolver(int orientationangle_iteration, Coordinate* Coordinate__,float increase_param_blue, float increase_param_red);
		float absolute_angle_solve(const ArmorPos& armor_pos, const float& car_yaw, const float& car_pitch) override;

	private:
		// 相似度函数
		virtual float cost(const std::vector<cv::Point2f>& armor_four_pixel_points,
			const std::vector<cv::Point2f>& points2f) = 0;
		float balance_cost(const std::vector<cv::Point2f>& armor_four_pixel_points,
			const std::vector<cv::Point2f>& points2f);
		// 增幅
		float increase_angle(float face_angle,float car_yaw,float increase_param); 
		// 修正朝向角
		float correct_face_angle(float face_angle,float face_angle_compute, float car_yaw);
	private:
		int orientationangle_iteration; // 迭代数
		float increase_param_blue; // 需求增幅朝向角(角度)与距离(cm)比值的参数,用于获取
		float increase_param_red;
		
	};


	/*
	* 基于角度和位移差计算相似度
	* 参考上交分享
	*/
	class DiagonalAngleSolver : public IterationAngleSolver
	{
	public:
		DiagonalAngleSolver(float inclined,float pixel_error,
		int orientationangle_iteration, Coordinate* Coordinate__, float increase_param_blue, float increase_param_red);

	private:
		float cost(const std::vector<cv::Point2f>& armor_four_pixel_points,
			const std::vector<cv::Point2f>& points2f) override;
	private:
		// 误差权重参数
		float inclined;
		float pixel_error;
	};

};
#endif // !_FACEANGLESOLVER_H_
