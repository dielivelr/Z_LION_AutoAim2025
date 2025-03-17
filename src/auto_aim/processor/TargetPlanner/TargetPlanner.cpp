#include "TargetPlanner.h"

namespace rm
{
	int TargetPlanner::get_target_kind() const
	{
		return this->target_kind;
	}
	std::vector<ArmorPos> TargetPlanner::get_target_kind_armor_poses(const std::vector<ArmorPos>& armor_poses)
	{
		std::vector<ArmorPos> target_armor_poses;
		for (const auto& x : armor_poses)
			if (this->target_kind == x.kind)
				target_armor_poses.push_back(x);
		return target_armor_poses;
	}
	std::vector<std::vector<ArmorPos>> TargetPlanner::screen_armors(const std::vector<ArmorPos>& armor_poses)
	{
		std::vector<std::vector<ArmorPos>> armor_poses_vector(5);
		for (const auto& armor : armor_poses) {
			if (armor.kind - 1 >= 0) { // 非negative
				armor_poses_vector[armor.kind - 1].push_back(armor);
			};
		};
		return armor_poses_vector;
	}
	;
	
	DefaultPlanner::DefaultPlanner(int buffer_size)
		:buffer_size(buffer_size), TargetPlanner()
	{
		clear();
	}
	std::vector<ArmorPos> DefaultPlanner::plan(const std::vector<ArmorPos>& armor_poses)
	{
		if (armor_poses.empty()) { // 判断缓冲帧,失效clear
			if (target_kind != 0)
				if (--buffer <= 0) clear();
			return {};
		};
	
		if (target_kind == 0) { // 当前无目标,取距离最近目标
			ArmorPos min_dis_armor;
			double min_distance = 100000000;
			for (const auto& armor_pos : armor_poses) {
				double dis = cv::norm(min_dis_armor.absolute_middle_point);
				if (min_distance > dis) {
					min_distance = dis;
					min_dis_armor = armor_pos;
				};
			};
			target_kind = min_dis_armor.kind;
			return this->get_target_kind_armor_poses(armor_poses);
		}
		else { // 找寻原始目标,没有则重复操作
			
			auto armors = this->get_target_kind_armor_poses(armor_poses);
			if (armors.empty())
			{
				if (--buffer <= 0) 
					clear();
				return {};
			}
			else {
				buffer = buffer_size;
				return armors;
			}
		};
	
	}
	void DefaultPlanner::clear()
	{
		buffer = this->buffer_size;
		target_kind = 0;
	};
	BufferPlanner::BufferPlanner(int buffer_size, int _kind_buffer_size_)
		:buffer_size(buffer_size), _kind_buffer_size_(_kind_buffer_size_), TargetPlanner()
	{
		clear();
	}
	std::vector<ArmorPos> BufferPlanner::plan(const std::vector<ArmorPos>& armor_poses)
	{
		if (armor_poses.empty()) { // 判断缓冲帧,失效clear
			if (target_kind != 0)
				if (--buffer <= 0) clear();
			return {};
		};

		if (target_kind == 0) { // 当前无目标,先筛选出识别次数连续超过_kind_buffer_size次的目标,之后取距离最近目标
			
			std::vector<ArmorPos> armor_poses_ = update_kind_buffer(armor_poses);
			if (armor_poses_.empty()) { 
				return {};
			};
			ArmorPos min_dis_armor;
			double min_distance = 100000000;
			for (const auto& armor_pos : armor_poses_) {
				double dis = cv::norm(min_dis_armor.absolute_middle_point);
				if (min_distance > dis) {
					min_distance = dis;
					min_dis_armor = armor_pos;
				};
			};
			target_kind = min_dis_armor.kind;
			return this->get_target_kind_armor_poses(armor_poses_);
		}
		else { // 找寻原始目标,没有则重复操作

			auto armors = this->get_target_kind_armor_poses(armor_poses);
			if (armors.empty())
			{
				if (--buffer <= 0)
					clear();
				return {};
			}
			else {
				buffer = buffer_size;
				return armors;
			}
		};
	}
	std::vector<ArmorPos> BufferPlanner::update_kind_buffer(const std::vector<ArmorPos>& armor_poses)
	{
		std::vector<ArmorPos> out_armor_poses;
		bool record_last_target_ret[3] = { 0 }; // 记录本次识别是否每种类型都有
		for (const auto& armor_pos : armor_poses) {
			if (armor_pos.kind > 0) {
				record_vector[armor_pos.kind - 1] = record_vector[armor_pos.kind - 1] + 1;
				record_last_target_ret[armor_pos.kind - 1] = true; 
			};
		};

		// 如果某一种类型没有,清空
		for (int i = 0; i < 3; i++)
			if (!record_last_target_ret[i])
				record_vector[i] = 0;

		// 一旦某一种类型达到_kind_buffer_size_,就放入
		for (const auto& armor_pos : armor_poses) {
			if (armor_pos.kind > 0) {
				if (record_vector[armor_pos.kind - 1] >= this->_kind_buffer_size_)
					out_armor_poses.push_back(armor_pos);
			};
		};

		return out_armor_poses;
	}
	void BufferPlanner::clear()
	{
		buffer = this->buffer_size;
		target_kind = 0;
		for (int i = 0; i < 3; i++)
		{
			record_vector[i] = 0;
		};
	}
}
