#include "LightFinder.h"
namespace rm
{
	LightFinder::LightFinder()
		:max_ratio_w_h(1.1), max_angle(35.0), SIMPLE(true)
	{
	}
	LightFinder::LightFinder(float max_ratio_w_h, float max_angle, bool SIMPLE)
		:max_ratio_w_h(max_ratio_w_h), max_angle(max_angle), SIMPLE(SIMPLE)
	{
	}
	bool LightFinder::screen_light(float max_ratio_w_h, float max_angle, const Light& light)
	{
		float angle = light.angle > 45 ? 90 - light.angle : light.angle;
		if (angle < max_angle)
			if ((float)light.width / light.height < max_ratio_w_h)
				return true;
		return false;
	};

	std::vector<cv::RotatedRect> LightFinder::find_retaterects(const cv::Mat& src,bool SIMPLE)
	{
		std::vector<cv::RotatedRect> rects;
		std::vector<std::vector<cv::Point>> contours = find_counters(src, SIMPLE);
		for (int i = 0; i < contours.size(); i++) {
			rects.push_back(cv::minAreaRect(contours[i]));
			//
			//if (this->remove_in_advance(contours[i], hierarchys[i])) {
			//	rects.push_back(cv::minAreaRect(contours[i]));
			//};
		};
		return rects;
	}
	std::vector<std::vector<cv::Point>> LightFinder::find_counters(const cv::Mat& src, bool SIMPLE)
	{
		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::Vec4i> hierarchys;
		cv::findContours(src, contours, hierarchys, cv::RETR_EXTERNAL, SIMPLE ? 
			cv::CHAIN_APPROX_SIMPLE : cv::CHAIN_APPROX_NONE); 
		// cv::CHAIN_APPROX_NONE CHAIN_APPROX_SIMPLE
		return contours;
	}
	;

	bool LightFinder::screen_light(const Light& light)
	{
		return screen_light(this->max_ratio_w_h,this->max_angle, light);
	}

	std::vector<std::vector<cv::Point>> LightFinder::find_counters(const cv::Mat& src)
	{
		return find_counters(src, this->SIMPLE);
	};

	std::vector<cv::RotatedRect> LightFinder::find_retaterects(const cv::Mat& src)
	{
		return find_retaterects(src, this->SIMPLE);
	};

	bool LightFinder::remove_in_advance(const std::vector<cv::Point>& contour, const cv::Vec4i& hierarchy)
	{
		if (contour.size() <= 10 || hierarchy[3] >= 0 || contour.empty()) {//不能有父轮廓
			return false;
		}
		return true;
	};

	UsualLightFinder::UsualLightFinder(float max_ratio_w_h, float max_angle, bool SIMPLE)
		:LightFinder(max_ratio_w_h, max_angle, SIMPLE)
	{
	}
	std::vector<Light> UsualLightFinder::find_lights_all_enemy(const cv::Mat& src)
	{
		std::vector<std::vector<cv::Point>> counters = find_counters(src);
		std::vector<rm::Light> Lights;
		for (const auto& counter : counters) {
			// 基于24君瞄的最小二乘
			auto b_rect = cv::boundingRect(counter);
			Light r_rect(cv::minAreaRect(counter));
			r_rect = strip_least_squares(r_rect, b_rect, counter);

			// 灯条筛选
			if (this->screen_light(r_rect)) // 时间开销小
				Lights.push_back(r_rect);
		};
		return Lights;
	};

	std::vector<Light> UsualLightFinder::find_lights(const cv::Mat& src,
		const cv::Mat& origen_img)
	{
		std::vector<std::vector<cv::Point>> counters = find_counters(src);
		std::vector<rm::Light> Lights;
		for (const auto& counter : counters) {
			// 基于24君瞄的最小二乘 暂不使用
			auto b_rect = cv::boundingRect(counter);
			Light r_rect(cv::minAreaRect(counter));

			//r_rect = strip_least_squares(r_rect, b_rect, counter);
			//r_rect.strip_least_squares_top = r_rect.top;
			//r_rect.strip_least_squares_bottom = r_rect.bottom;

			// 灯条筛选
			if (this->screen_light(r_rect)) // 时间开销小
				if (color_separation(r_rect, origen_img, counter)) // 时间开销大
					Lights.push_back(r_rect);
		};
		return Lights;
	}

	void UsualLightFinder::set_enemy_color(bool enemy_blue)
	{
		this->enemy_blue = enemy_blue;
	};

	Light UsualLightFinder::strip_least_squares(Light r_rect, cv::Rect b_rect, const std::vector<cv::Point>& counter)
	{
		//cv::Mat mask = cv::Mat::zeros(b_rect.size(), CV_8UC1);
		//std::vector<cv::Point> mask_contour;
		//for (const auto& p : counter) {
		//	mask_contour.emplace_back(p - cv::Point(b_rect.x, b_rect.y));
		//}
		//cv::fillPoly(mask, { mask_contour }, 255);
		//std::vector<cv::Point> points;
		//cv::findNonZero(mask, points);
		//// points / rotated rect area
		//bool is_fill_rotated_rect =
		//	points.size() / (r_rect.size.width * r_rect.size.height) > 0.8; //l.min_fill_ratio
		//cv::Vec4f return_param;
		//cv::fitLine(points, return_param, cv::DIST_L2, 0, 0.01, 0.01);
		//double angle_k;
		//if (int(return_param[0] * 100) == 100 || int(return_param[1] * 100) == 0) {
		//	r_rect.strip_least_squares_top = cv::Point2f(b_rect.x + b_rect.width / 2, b_rect.y);
		//	r_rect.strip_least_squares_bottom = cv::Point2f(b_rect.x + b_rect.width / 2, b_rect.y + b_rect.height);
		//	angle_k = 0;
		//}
		//else {
		//	auto k = return_param[1] / return_param[0];
		//	auto b = (return_param[3] + b_rect.y) - k * (return_param[2] + b_rect.x);
		//	r_rect.strip_least_squares_top = cv::Point2f((b_rect.y - b) / k, b_rect.y);
		//	r_rect.strip_least_squares_bottom = cv::Point2f((b_rect.y + b_rect.height - b) / k, b_rect.y + b_rect.height);
		//	angle_k = std::atan(k) / CV_PI * 180 - 90;
		//	if (angle_k > 90) {
		//		angle_k = 180 - angle_k;
		//	}
		//};
		return r_rect;
	}
	;

	bool UsualLightFinder::color_separation(const Light& light, const cv::Mat& pic,
		const std::vector<cv::Point>& contour)
	{
		int w = pic.cols;
		int h = pic.rows;
		cv::Rect rect = light.boundingRect();
		int x = rect.x;
		int y = rect.y;
		int lengthw = rect.width;
		int lengthh = rect.height / 8;
		int max_x = rect.x + rect.width / 2 + lengthw;
		int max_y = rect.y + rect.height / 2 + lengthh;
		int min_x = rect.x + rect.width / 2 - lengthw;
		int min_y = rect.y + rect.height / 2 - lengthh;
		max_x = max_x > w ? w : max_x;
		max_y = max_y > h ? h : max_y;
		min_x = min_x < 0 ? 0 : min_x;
		min_y = min_y < 0 ? 0 : min_y;

		rect = cv::Rect(min_x, min_y, max_x - min_x, max_y - min_y);
		auto roi = pic(rect);
		// Iterate through the ROIs
		int sum_r = 0, sum_b = 0;
		for (int i = 0; i < roi.rows; i++) {
			for (int j = 0; j < roi.cols; j++) {
				if (cv::pointPolygonTest(contour, cv::Point2f(j + rect.x, i + rect.y), false) >= 0) {
					sum_r += roi.at<cv::Vec3b>(i, j)[2];
					sum_b += roi.at<cv::Vec3b>(i, j)[0];
				}
			}
		};
		return (sum_r < sum_b && enemy_blue) || (sum_r > sum_b && !enemy_blue);
	};

	MiddleLightFinder::MiddleLightFinder(UsualLightFinder* UsualLightFinder__, float ret_dis_error)
		:UsualLightFinder__(UsualLightFinder__), ret_dis_error(ret_dis_error), LightFinder()
	{
	}
	std::vector<Light> MiddleLightFinder::find_lights(const cv::Mat& src, const cv::Mat& origen_img)
	{
		// std::vector<Light> lights = UsualLightFinder__->find_lights(src, origen_img); 
		std::vector<Light> lights = UsualLightFinder__->find_lights_all_enemy(src);
		if (lights.empty()) return {};
		return { get_min_dis_light(cv::Point2f(src.cols / 2, src.rows / 2) , lights) };
	}
	Light MiddleLightFinder::get_min_dis_light(const cv::Point2f& img_center, const std::vector<Light>& lights)
	{
		double min_dis = 1000000;
		rm::Light target;
		for (auto& x : lights) {
			double dis = cv::norm(x.center - img_center);
			if (dis < min_dis) {
				min_dis = dis;
				target = x;
			};
		};
		return target;
	};

	cv::Rect MiddleLightFinder::rect_edge_detection(cv::Point point, int width, int height,
		int image_width, int image_height)
	{
		point.x -= width / 2;
		point.y -= height / 2;
		if (point.x < 0) point.x = 0;
		if (point.x + width > image_width) width = image_width - point.x;
		if (point.y < 0) point.y = 0;
		if (point.y + height > image_height) height = image_height - point.y;
		return cv::Rect(point.x, point.y, width, height);
	}
	bool MiddleLightFinder::light_center_in_rect_middle(Light light, cv::Rect rect)
	{
		return light_center_in_rect_middle(light, rect, ret_dis_error);
	}
	;

	bool MiddleLightFinder::light_center_in_rect_middle(Light light, cv::Rect rect, float ret_dis_error)
	{
		cv::Point2f center(rect.width / 2, rect.height / 2);
		// 距离必须是对角线的一定范围以内
		bool ret_dis = cv::norm(light.center - center) <
			sqrt(rect.width * rect.width + rect.height * rect.height) / ret_dis_error;
		return ret_dis;
	};
}
