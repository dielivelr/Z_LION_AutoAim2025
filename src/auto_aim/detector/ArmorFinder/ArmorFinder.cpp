#include "ArmorFinder.h"

namespace rm
{
	ArmorFinder::ArmorFinder(float armor_min_height_ratio, float small_armor_min_aspect_ratio,
		float small_armor_max_aspect_ratio, float big_armor_min_aspect_ratio, 
		float big_armor_max_aspect_ratio, float max_center_angle)
		:armor_min_height_ratio(armor_min_height_ratio), small_armor_min_aspect_ratio(small_armor_min_aspect_ratio),
		small_armor_max_aspect_ratio(small_armor_max_aspect_ratio), big_armor_min_aspect_ratio(big_armor_min_aspect_ratio),
		big_armor_max_aspect_ratio(big_armor_max_aspect_ratio), max_center_angle(max_center_angle)
	{
	}
	std::vector<rm::Armor> ArmorFinder::find_armors(std::vector<rm::Light> Lights)
	{
		//std::cout<<"Light_size:"<<Lights.size()<<std::endl;
		std::vector<rm::Armor> Armors;
		int size = Lights.size();
		rm::sort_lights_x(Lights);// 保证是排过顺序的
		for (int i = 0; i < size; i++) {
			for (int j = i + 1; j < size; j++) {
				rm::Armor armor(Lights[i], Lights[j]);
				if (rm::screen_armor(
					armor,
					armor_min_height_ratio,
					small_armor_min_aspect_ratio,
					small_armor_max_aspect_ratio,
					big_armor_min_aspect_ratio,
					big_armor_max_aspect_ratio,
					max_center_angle
				)) {
					Armors.push_back(armor);
				};
			};
		};
		//std::cout<<"Armor_size:"<<Armors.size()<<std::endl;
		remove_pos_noise(Armors, Lights); // 去除噪声项
		// 将赋值装甲板大小
		for (auto& armor : Armors)
			armor.armor_small = rm::armor_size_discrimine(
				armor, big_armor_min_aspect_ratio);
		//std::cout<<"Armor_size1:"<<Armors.size()<<std::endl;
		return Armors;
	}

	std::vector<rm::Armor> ArmorFinder::remove_lights_coincide(std::vector<rm::Armor> Armors)
	{
		std::vector<rm::Armor> OutArmors;
		// 排除左灯调或者右灯条项
		int size = Armors.size();
		for (int i = 0; i < size; i++) {
			bool ret = true;
			for (int j = i + 1; j < size; j++) {
				if (
					Armors[i].left_light == Armors[j].right_light ||
					Armors[i].left_light == Armors[j].right_light ||
					Armors[i].left_light == Armors[j].right_light ||
					Armors[i].left_light == Armors[j].right_light
					) {
					ret = false;
					break;
				}
			};
			if (ret) OutArmors.push_back(Armors[i]);
		};
		return OutArmors;
	}

	void ArmorFinder::remove_pos_noise(std::vector<rm::Armor>& Armors,
		const std::vector<rm::Light>& Lights)
	{
		if (Armors.size() == 1 || Armors.empty())return;
		std::vector<rm::Armor> outputs;
		bool state;
		for (const auto& x : Armors) {
			rm::Light L = x.left_light, R = x.right_light;
			state = true;
			for (auto y : Lights) {
				if (y.center.x == L.center.x && y.center.y == L.center.y)continue;
				if (y.center.x == R.center.x && y.center.y == R.center.y)continue;
				cv::Point2f points[4];
				y.points(points);
				std::sort(points, points + 4, [&](cv::Point2f& i, cv::Point2f& j)->bool {
					return i.y < j.y;
					});
				if (y.center.x > L.center.x && y.center.x < R.center.x)
				{
					// 灯条不能在该装甲板的区间内
					std::vector<cv::Point2f> OutFourPoints = x.middle_four_points;
					double MAX_Y = MAX(OutFourPoints[0].y, OutFourPoints[3].y);
					double MIN_Y = MIN(OutFourPoints[1].y, OutFourPoints[2].y);
					if (!(points[0].y > MAX_Y || points[3].y < MIN_Y)) state = false;
				}
			}
			if (state)outputs.push_back(x);
		}
		Armors = outputs;
	};
}
