#ifndef _TARGETPLANNER_H_
#define _TARGETPLANNER_H_
#include"../ArmorPos/ArmorPos.h"

namespace rm
{
	/*
	* 击打目标规划器
	* 对于每个目标,我们拥有装甲板的位置信息,由于使用迭代法计算朝向角有比较大的时间损耗,仅已知装甲板类别,图像位置以及三维坐标
	* 基类规定了当前的既定击打目标
	* 3V3中,仅存在3种装甲板,未来可拓展为更多
	*/
	class TargetPlanner
	{
	public:
		TargetPlanner() {};
		virtual ~TargetPlanner() {};
	protected: // 仅子类可以修改
		//  在本项目中.其与SendArmor的映射关系相同
		//	negative = 0,
		//	one,
		//	sentinel,
		//	three
		int target_kind = 0;
	public:
		// 获取目标类型
		int get_target_kind() const;
		// 目标规划,返回对应的同种类别
		virtual std::vector<ArmorPos> plan(const std::vector<ArmorPos>& armor_poses) = 0;
	protected:
		// 根据装甲板目标获取当前所有装甲板
		std::vector<ArmorPos> get_target_kind_armor_poses(const std::vector<ArmorPos>& armor_poses);
		// 按照类型分类放入不同数组中
		std::vector<std::vector<ArmorPos>> screen_armors(const std::vector<ArmorPos>& armor_poses);
	};

	/*
	* 距离规划器
	* 对优先识别到的近距离目标进行打击
	* 当不存在先前目标,则减少缓冲帧,到零clear,存在即回调
	*/
	class DefaultPlanner : public TargetPlanner
	{
	public:
		DefaultPlanner() = default;
		DefaultPlanner(int buffer_size);
	public:
		std::vector<ArmorPos> plan(const std::vector<ArmorPos>& armor_poses) override;
	private:
		void clear();
		// 缓冲
		int buffer;
		// 缓冲尺寸
		const int buffer_size;
	};

	/*
	* 增加类型缓冲的Planner
	* 同一种类型必须出现n次才会进入下一层的距离判断
	*/
	class BufferPlanner : public TargetPlanner
	{
	public:
		BufferPlanner() = default;
		BufferPlanner(int buffer_size,int _kind_buffer_size_ = 3);
	public:
		std::vector<ArmorPos> plan(const std::vector<ArmorPos>& armor_poses) override;
	private:
		// 更新记录数组
		std::vector<ArmorPos> update_kind_buffer(const std::vector<ArmorPos>& armor_poses);

		void clear();
		// 缓冲
		int buffer;
		// 缓冲尺寸
		const int buffer_size;
		// 
		int record_vector[3];
		// 
		const int _kind_buffer_size_;
	};
};

#endif // !_TARGETPLANNER_H_
