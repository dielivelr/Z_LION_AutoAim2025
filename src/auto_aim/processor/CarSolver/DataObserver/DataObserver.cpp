#include"DataObserver.h"

namespace rm
{
	AngleOptimizer::AngleOptimizer()
	{
	}
	AngleOptimizer::AngleOptimizer(float min_angle_error, float min_update_angle, float max_update_angle)
		:min_angle_error(min_angle_error), min_update_angle(min_update_angle * CV_PI / 180), max_update_angle(max_update_angle * CV_PI / 180)
	{
	}
	std::vector<float> AngleOptimizer::observe(const std::vector<float>& inputs, const float& car_yaw)
	{
		if (inputs.size() != 2) return {};
		// 左正右负
		float camera_yaw_left = FaceAngleSolver::switch_absolute_camera_angle(inputs[0], car_yaw);
		float camera_yaw_right = FaceAngleSolver::switch_absolute_camera_angle(inputs[1], car_yaw);

		// 若存在装甲板不符合实际的情况,仅返回其中一个，都不符合实际,返回空
		if (camera_yaw_left <= 0 && camera_yaw_right < 0) return { inputs[1] };
		else if (camera_yaw_left > 0 && camera_yaw_right >= 0) return { inputs[0] };
		else if (camera_yaw_left <= 0 && camera_yaw_right >= 0) return {};

		if (camera_yaw_left <= min_update_angle || camera_yaw_left >= max_update_angle) return { inputs[0] };
		if (-camera_yaw_right <= min_update_angle || -camera_yaw_right >= max_update_angle) return { inputs[1] };

		return { inputs[0],inputs[1] };
	}
	void AngleOptimizer::correct_face_angle(ArmorPos& left_armor_pos, ArmorPos& right_armor_pos, const float& car_yaw)
	{
		// 左正右负
		float camera_yaw_left = FaceAngleSolver::switch_absolute_camera_angle(left_armor_pos.face_angle, car_yaw);
		float camera_yaw_right = FaceAngleSolver::switch_absolute_camera_angle(right_armor_pos.face_angle, car_yaw);

		// 相机坐标系下的朝向角左侧必定为正,右侧必定为负
		// 如果不符合就取反
		if (camera_yaw_left < 0) {
			camera_yaw_left = -camera_yaw_left;
		};
		if (camera_yaw_right > 0) {
			camera_yaw_right = -camera_yaw_right;
		};
		left_armor_pos.face_angle = FaceAngleSolver::switch_absolute_camera_angle(camera_yaw_left, car_yaw);
		right_armor_pos.face_angle = FaceAngleSolver::switch_absolute_camera_angle(camera_yaw_right, car_yaw);
	}
	;


	RadiusOptimizer::RadiusOptimizer(int queue_length)
		:queue_length(queue_length)
	{
	}
	void RadiusOptimizer::observe(float& high_armor_radius, float& low_armor_radius, float& height_low_differ)
	{
		//// 容量满就让最后一个出队
		if (high_armor_radius_queue.size() >= this->queue_length) {
			high_armor_radius_queue.erase(high_armor_radius_queue.begin());
			low_armor_radius_queue.erase(low_armor_radius_queue.begin());
			height_low_differ_queue.erase(height_low_differ_queue.begin());
		};

		// 放入
		high_armor_radius_queue.push_back(high_armor_radius);
		low_armor_radius_queue.push_back(low_armor_radius);
		height_low_differ_queue.push_back(height_low_differ);

		// 取平均值
		float avange_high_armor_radius = 0, avange_low_armor_radius = 0, min_height_low_differ = 0;
		for (const auto& x : high_armor_radius_queue)
			avange_high_armor_radius += x;
		for (const auto& x : low_armor_radius_queue)
			avange_low_armor_radius += x;
		for (const auto& x : height_low_differ_queue)
			min_height_low_differ += x;

		avange_high_armor_radius = avange_high_armor_radius / high_armor_radius_queue.size();
		avange_low_armor_radius = avange_low_armor_radius / low_armor_radius_queue.size();
		min_height_low_differ = min_height_low_differ / height_low_differ_queue.size();

		high_armor_radius = avange_high_armor_radius;
		low_armor_radius = avange_low_armor_radius;
		height_low_differ = min_height_low_differ;
	}
	void RadiusOptimizer::clear()
	{
		high_armor_radius_queue.clear(); high_armor_radius_queue.shrink_to_fit();
		low_armor_radius_queue.clear(); low_armor_radius_queue.shrink_to_fit();
		height_low_differ_queue.clear(); height_low_differ_queue.shrink_to_fit();
	}
	;
}