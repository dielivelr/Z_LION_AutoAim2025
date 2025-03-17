#ifndef _ARMOR_H_
#define _ARMOR_H_
#include<opencv2/opencv.hpp>
#include"../Light/Light.h"
namespace rm
{
	/**********************
	* 装甲板类
	* 由于装甲板规则上相对固定,使用具体类来实现
	***********************/
	class Armor
	{
	public:
		Armor() {};
		Armor(const Light& left_light, const Light& right_light);
		Armor(const cv::Point2f* pts, int label);
	public: // 特征
		//装甲板特征
		float height_ratio; // 灯条高度比
		float aspect_ratio; // Armor长宽比
		float center_angle; // 两个灯条连线与x轴的夹角
	public:
		Light left_light, right_light;
		// 装甲板颜色
		enum armor_color
		{
			Blue,	// 蓝色
			Red,	// 红色
			Bad		// 即灰色或紫色
		};
		armor_color color;
		// 标签类型
		enum label_type
		{
			negative = 0,
			one,
			sentinel,
			three,
			four,
			five
		};
		label_type label = negative;
		// 装甲板大小
		bool armor_small;
		// 装甲板四点
		// 1 2
		// 0 3
		std::vector<cv::Point2f> middle_four_points;
		//std::vector<cv::Point2f> strip_least_squares_four_points;
	};

	/**********************
	* 装甲板筛选
	* 是否符合装甲板形状
	***********************/
	bool screen_armor(const Armor& armor, float armor_min_height_ratio, float small_armor_min_aspect_ratio,
		float small_armor_max_aspect_ratio, float big_armor_min_aspect_ratio,
		float big_armor_max_aspect_ratio, float max_center_angle);

	/**********************
	* 大小装甲板筛选
	* 是否符合装甲板形状
	* 小: true
	* 大: false
	***********************/
	bool armor_size_discrimine(const Armor& armor,float big_armor_min_aspect_ratio);

	/**********************
	* 画出装甲板
	***********************/
	cv::Mat draw_armor(cv::Mat src, const Armor& armor, cv::Scalar color =
		cv::Scalar(0, 255, 0), int thickness = 3);

	/**********************
	* 画出装甲板类型
	* 包括大小和类型
	***********************/
	cv::Mat draw_armor_label_size(cv::Mat src, const Armor& armor, cv::Scalar color =
		cv::Scalar(0, 255, 0), int thickness = 3);

	/**********************
	* 画出交叉装甲板
	* 默认白色,超酷的!
	***********************/
	cv::Mat draw_right_armor(cv::Mat src, const Armor& armor, cv::Scalar color =
		cv::Scalar(255, 255, 255), int thickness = 3);
	cv::Mat draw_right_armor(cv::Mat src, const std::vector<cv::Point2f>& middle_four_points, cv::Scalar color =
		cv::Scalar(255, 255, 255), int thickness = 3);
};



#endif // !_ARMOR_H_
