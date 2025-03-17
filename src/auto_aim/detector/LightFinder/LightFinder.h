#ifndef _LIGHTFINDER_H_
#define _LIGHTFINDER_H_
#include<iostream>
#include<vector>
#include<opencv2/opencv.hpp>
#include"../Light/Light.h"
namespace rm
{
	/*
	* 灯条检索基类
	* 通过处理过的图像提取轮廓信息,并筛去形状特征的不符合者
	*/
	class LightFinder
	{
	public:
		LightFinder();
		LightFinder(float max_ratio_w_h, float max_angle, bool SIMPLE);
		virtual std::vector<Light> find_lights(const cv::Mat& src, const cv::Mat& origen_img) = 0; // 寻找灯条序列
		virtual ~LightFinder() {};
	protected: // 进给子类暴露应用接口
		bool screen_light(const Light& light);
		// 提取轮廓
		std::vector<std::vector<cv::Point>> find_counters(const cv::Mat& src);
		// 直接获取最小矩形
		std::vector<cv::RotatedRect> find_retaterects(const cv::Mat& src);
		// 清空父轮廓与过小轮廓
		bool remove_in_advance(const std::vector<cv::Point>& contour, const cv::Vec4i& hierarchy);
	private:
		// 灯条筛选:面积,长宽比,角度
		bool screen_light(float max_ratio_w_h, float max_angle, const Light& light);
		// 获取倾斜矩形
		std::vector<cv::RotatedRect> find_retaterects(const cv::Mat& src,bool SIMPLE);
		// 提取轮廓
		std::vector<std::vector<cv::Point>> find_counters(const cv::Mat& src, bool SIMPLE);
	private:
		float max_ratio_w_h; // 最大宽高比
		float max_angle; // 最大角度
		bool SIMPLE; // 是否采取最大轮廓提取
	};

	/*
	* 面前初筛,提取轮廓并转化为灯条序列
	*/
	class UsualLightFinder : public LightFinder
	{
	public:
		UsualLightFinder(float max_ratio_w_h, float max_angle, bool SIMPLE);
		std::vector<Light> find_lights_all_enemy(const cv::Mat& src); // 无视颜色,寻找所有灯条序列
		std::vector<Light> find_lights(const cv::Mat& src, const cv::Mat& origen_img) override; // 增加敌我区分

		void set_enemy_color(bool enemy_blue);
	
	private:
		// 最小二乘优化
		Light strip_least_squares(Light r_rect, cv::Rect b_rect,const std::vector<cv::Point>& counter);

		// 颜色区分
		bool color_separation(const Light& light,
			const cv::Mat& pic, const std::vector<cv::Point>& contour);
		// 颜色识别
		bool enemy_blue; // 敌方颜色为蓝色
	};

	/*
	* 面向roi提取最优中心灯条
	* 用于融合式神经网络
	*/
	class MiddleLightFinder : public LightFinder
	{
		UsualLightFinder* UsualLightFinder__; // 组合方式调用UsualLightFinder的find_lights接口
	public:
		MiddleLightFinder(UsualLightFinder* UsualLightFinder__,float ret_dis_error);
		std::vector<Light> find_lights(const cv::Mat& src, const cv::Mat& origen_img) override; // 寻找(最优)灯条
		// Rect超界检测
		cv::Rect rect_edge_detection(cv::Point point, int width, int height,
			int image_width, int image_height);
		bool light_center_in_rect_middle(Light light, cv::Rect rect);
	private:
		// 获取最低距离灯条
		Light get_min_dis_light(const cv::Point2f& img_center, const std::vector<Light>& lights);
		// 灯条位置区域检测
		bool light_center_in_rect_middle(Light light, cv::Rect rect, float ret_dis_error);
	private:
		// 灯条允许范围参数
		float ret_dis_error;
	};

}

#endif // !LIGHTFINDER_H
