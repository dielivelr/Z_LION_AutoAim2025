#ifndef _LIGHT_H_
#define _LIGHT_H_
#include<opencv2/opencv.hpp>
namespace rm
{
	/******************
	* 灯条类,继承RotatedRect类,能够获取其灯条宽高
	******************/
	class Light : public cv::RotatedRect
	{
	public:
		Light() {};
		explicit Light(const cv::RotatedRect& light);
		bool operator==(const Light& other) const;
		int width, height;
		// 抽象灯条为上下两点
		cv::Point2f top;
		cv::Point2f bottom;
	};

	/******************
	* 灯条排序
	* 图像x轴正方向
	******************/
	void sort_lights_x(std::vector<Light>& lights);

	/******************
	* 画出灯条
	******************/
	cv::Mat draw_light(cv::Mat src, const Light& light,cv::Scalar color =
		cv::Scalar(255,0,0), int thickness = 3);


}
#endif // !_LIGHT_H
