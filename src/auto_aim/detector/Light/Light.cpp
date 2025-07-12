#include "Light.h"

namespace rm
{
	Light::Light(const cv::RotatedRect& light) : cv::RotatedRect(light)
	{
		// 分别计算两点斜率
		cv::Point2f points_[4];
		points(points_);
		double K1 = abs((points_[0].y - points_[1].y) / (points_[0].x - points_[1].x));
		double K2 = abs((points_[1].y - points_[2].y) / (points_[1].x - points_[2].x));
		if (K1 < K2) {
			// 更加贴近x轴的是宽,否则是高
			width = cv::norm(points_[0] - points_[1]);
			height = cv::norm(points_[1] - points_[2]);
		}
		else {
			height = cv::norm((points_[0]) - points_[1]);
			width = cv::norm(points_[1] - points_[2]);
		};

		// 更新,将灯条映射为一条线段
		std::sort(points_, points_ + 4, [&](cv::Point2f& i, cv::Point2f& j)->bool {
			return i.y < j.y;
			});
		top = (points_[0] + points_[1]) / 2;
		bottom = (points_[2] + points_[3]) / 2;
	};

	bool Light::operator==(const Light& other) const
	{
		return other.center.x == center.x && other.center.y == center.y;
	};

	void sort_lights_x(std::vector<Light>& lights)
	{
		std::sort(lights.begin(), lights.end(), [&](Light& i, Light& j)->bool {
			return i.center.x < j.center.x;
			});
	};

	cv::Mat draw_light(cv::Mat src, const Light& light, cv::Scalar color, int thickness)
	{
		cv::Point2f FourPoints[4];
		light.points(FourPoints);
		cv::line(src, FourPoints[0], FourPoints[1], color, thickness);
		cv::line(src, FourPoints[1], FourPoints[2], color, thickness);
		cv::line(src, FourPoints[2], FourPoints[3], color, thickness);
		cv::line(src, FourPoints[3], FourPoints[0], color, thickness);
		return src;
	};
};