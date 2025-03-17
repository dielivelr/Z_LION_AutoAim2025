#include"Armor.h"

rm::Armor::Armor(const Light& left_light, const Light& right_light)
	:left_light(left_light), right_light(right_light)
{
	middle_four_points = { left_light.bottom, left_light.top,
		right_light.top, right_light.bottom };
	//strip_least_squares_four_points = { left_light.strip_least_squares_bottom, left_light.strip_least_squares_top,
	//	right_light.strip_least_squares_top, right_light.strip_least_squares_bottom };
	// 提取特征量
	height_ratio = (float)MIN(left_light.height, right_light.height) / MAX(left_light.height, right_light.height);
	float height_middle = float(left_light.height + right_light.height) / 2;
	aspect_ratio = cv::norm(left_light.center - right_light.center) / height_middle;
	cv::Point2f diff = left_light.center - right_light.center;
	center_angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;
	if(center_angle > 45) center_angle = 90 - center_angle; 
}
rm::Armor::Armor(const cv::Point2f* pts, int label)
{
	middle_four_points = { pts[1],pts[0],pts[3],pts[2] };
	//strip_least_squares_four_points = middle_four_points;
	// 将神经网络label映射为装甲板类型
	armor_small = !(label == 0 || label == 5);
	if (label == 0 || label == 5)
	{
		this->label = label_type::one;
	}
	else if (label == 1 || label == 6) {
		this->label = label_type::sentinel;
	}
	else if (label == 10) {
		this->label = label_type::negative;
	}
	else {
		this->label = label_type::three;
	};

	//if(this->label == label_type::sentinel){
	//	this->label = label_type::three;
	//};
}
;

bool rm::screen_armor(const Armor& armor, float armor_min_height_ratio, float small_armor_min_aspect_ratio, float small_armor_max_aspect_ratio, float big_armor_min_aspect_ratio, float big_armor_max_aspect_ratio, float max_center_angle)
{
	//std::cout<<"armor.center_angle:"<<armor.center_angle<<std::endl;
	if (armor.center_angle < max_center_angle)
	{
		//std::cout<<"armor.height_ratio > armor_min_height_ratio"<<armor.height_ratio<<std::endl;
		if (armor.height_ratio > armor_min_height_ratio)
		{
			//std::cout<<"armor.aspect_ratio "<<armor.aspect_ratio<<std::endl;
			if (armor.aspect_ratio > small_armor_min_aspect_ratio &&
				armor.aspect_ratio < small_armor_max_aspect_ratio)
			{
				return true;
			}
			else if (armor.aspect_ratio > big_armor_min_aspect_ratio &&
				armor.aspect_ratio < big_armor_max_aspect_ratio)
			{
				return true;
			}
			
		}
	}
	return false;
}

bool rm::armor_size_discrimine(const Armor& armor, float big_armor_min_aspect_ratio)
{
	return armor.aspect_ratio < big_armor_min_aspect_ratio;
}

cv::Mat rm::draw_armor(cv::Mat src, const Armor& armor, cv::Scalar color, int thickness)
{
	std::vector<cv::Point2f> FourPoints = armor.middle_four_points;
	cv::line(src, FourPoints[0], FourPoints[1], color, thickness);
	cv::line(src, FourPoints[1], FourPoints[2], color, thickness);
	cv::line(src, FourPoints[2], FourPoints[3], color, thickness);
	cv::line(src, FourPoints[3], FourPoints[0], color, thickness);
	return src;
}

cv::Mat rm::draw_armor_label_size(cv::Mat src, const Armor& armor, 
	cv::Scalar color, int thickness)
{
	std::string str_armor_size = armor.armor_small ? "small" : "big";
	std::string str_armor_label;

	// 碰到negative,直接返回
	switch (armor.label) 
	{
	case Armor::sentinel:
		str_armor_label = "sentinel";
		break;
	case Armor::one:
		str_armor_label = "one";
		break;
	case Armor::three:
		str_armor_label = "three";
		break;
	case Armor::four:
		str_armor_label = "four";
		break;
	case Armor::five:
		str_armor_label = "five";
		break;
	case Armor::negative:
		return src;
	default:
		break;
	};

	cv::putText(src, str_armor_label, armor.middle_four_points[3], 2, 2, color, thickness);
	cv::putText(src, str_armor_size, armor.middle_four_points[3] + 
		cv::Point2f(0,30), 2, 2, color, thickness);
	return src;
}

cv::Mat rm::draw_right_armor(cv::Mat src, const Armor& armor, cv::Scalar color, int thickness)
{
	draw_right_armor(src, armor.middle_four_points, color, thickness);
	return src;
}

cv::Mat rm::draw_right_armor(cv::Mat src, const std::vector<cv::Point2f>& middle_four_points, cv::Scalar color, int thickness)
{
	cv::line(src, middle_four_points[0], middle_four_points[2], color, thickness);
	cv::line(src, middle_four_points[2], middle_four_points[3], color, thickness);
	cv::line(src, middle_four_points[3], middle_four_points[1], color, thickness);
	cv::line(src, middle_four_points[1], middle_four_points[0], color, thickness);
	return src;
}
