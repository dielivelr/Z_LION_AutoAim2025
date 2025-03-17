#ifndef _TRAINSDETECTOR_H_
#define _TRAINSDETECTOR_H_
#include"Armor/Armor.h"
namespace rm 
{
	class SendArmor
	{
	public:
		SendArmor(const Armor& armor) {
			this->label = static_cast<int>(armor.label);
			if (label > 2) // 联盟赛步兵只会出现3号装甲板
			{
				label = 3;
			};
			for (int i = 0; i < 4; i++) {
				middle_four_points[i] = armor.middle_four_points[i];
			};
			light_loss = false;
		};
		int label; // 标签类
		cv::Point2f middle_four_points[4];
		bool light_loss; 
	};

}

#endif // !_TRAINSDETECTOR_H_
