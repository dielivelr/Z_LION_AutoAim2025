#ifndef _TRANSPROCESSOR_H_
#define _TRANSPROCESSOR_H_
#include<Eigen/Dense>
namespace rm
{
	/*
	* 记录状态值和测序测量起始时间
	*/
	class TransProcessor
	{
	public:
		TransProcessor() = default;
		TransProcessor(const std::vector<Eigen::VectorXf>& States, 
			float high_armor_radius, float low_armor_radius, float high_low_height_differ,
			bool high_armor_state, int target_kind) {
			Eigen::VectorXf car_state_(12);
			Eigen::VectorXf trans_state_(5);
			if (States[0].size() >= 8 && States[1].size() >= 5)
			{
				for (int i = 0; i < 8; i++)
					car_state_(i) = States[0](i);
				for (int i = 0; i < 5; i++)
					trans_state_(i) = States[1](i);
			};
			car_state_(8) = high_armor_radius;
			car_state_(9) = low_armor_radius;
			car_state_(10) = high_low_height_differ;
			car_state_(11) = high_armor_state;

			this->car_state = car_state_;
			this->trans_state = trans_state_;
			this->target_kind = target_kind;
		};
		Eigen::VectorXf car_state;	// 整车状态 x y z yaw vx vy vz vyaw high_r low_r h_differ high_armor_state
		Eigen::VectorXf trans_state;// 平移状态 x y z vx vz
		int target_kind = 0;		// 击打目标 
		// 判断是否为空
		bool empty() { return target_kind == 0; };
	};

}
#endif // !_TRAINSDETECTOR_H_
