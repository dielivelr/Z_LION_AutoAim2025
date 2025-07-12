#ifndef _ARMORFINDER_H_
#define _ARMORFINDER_H_
#include"../Armor/Armor.h"
namespace rm
{
	/*
	* 装甲板检测类
	* 灯条匹配+整板筛选
	* 比赛中大小装甲板尺寸相对固定,设计为具体类
	*/
	class ArmorFinder
	{
	public:
		ArmorFinder() {};
		ArmorFinder(float armor_min_height_ratio,float small_armor_min_aspect_ratio, float small_armor_max_aspect_ratio,
			float big_armor_min_aspect_ratio, float big_armor_max_aspect_ratio, float max_center_angle);
		// 寻找可能装甲板
		std::vector<Armor> find_armors(std::vector<Light> Lights);
	private:
		// 删除所有灯条重合项
		std::vector<Armor> remove_lights_coincide(std::vector<Armor> Armors);
		// 删除跨越项
		void remove_pos_noise(std::vector<Armor>& Armors, const std::vector<Light>& Lights);
	private:
		float armor_min_height_ratio; // 装甲板最小高度比
		float small_armor_min_aspect_ratio; // 小装甲板最小长宽比
		float small_armor_max_aspect_ratio; // 小装甲板最大长宽比
		float big_armor_min_aspect_ratio; // 大装甲板最小长宽比
		float big_armor_max_aspect_ratio;// 大装甲板最大长宽比
		float max_center_angle;// 最大中心角

	};
}


#endif // !ARMORFINDER_H
