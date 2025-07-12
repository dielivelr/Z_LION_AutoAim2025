#ifndef _MACHINE_H_
#define _MACHINE_H_
#include<opencv2/opencv.hpp>
#include<iostream>
#include<vector>

namespace rm 
{

	/*
	* 图像处理类
	*/
	class Machine
	{
	public:
		// 实现对图像的处理
		virtual cv::Mat machine(cv::Mat src) = 0; 

		virtual ~Machine() {};
	protected:
		//普通二值化
		inline cv::Mat threshold(cv::Mat src, double pic_min_threshold);
	};

	/***********
	* 图像相减获取二值图像
	**************/
	class DifferMachine : public Machine
	{
	public:
		DifferMachine(double pic_min_threshold, bool blue_enemy);
		cv::Mat machine(cv::Mat src) override;
	private:
		double pic_min_threshold; // 二值化参数
		bool blue_enemy; // 敌方颜色
	};

	/***********
	* 灰度图二值化
	**************/
	class GreyMachine : public Machine
	{
	public:
		GreyMachine(double pic_min_threshold);
		cv::Mat machine(cv::Mat src) override;
	private:
		double pic_min_threshold;  // 二值化参数
	};

	/***********
	* 自动二值化
	* 用于数字识别提取
	**************/
	class AutoMachine : public Machine
	{
	public:
		cv::Mat machine(cv::Mat src) override;
	private:
		// 自动二值化
		inline cv::Mat auto_threshold(cv::Mat src);
	};
}


#endif // !MACHINE_H
