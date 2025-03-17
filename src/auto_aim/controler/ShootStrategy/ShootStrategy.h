#ifndef _SHOOTSTRATEGY_H_
#define _SHOOTSTRATEGY_H_
#include"../RangeSetter/RangeSetter.h"
#include"../../processor/Coordinate/Coordinate.h"
#include<iostream>
#include<Eigen/Dense>
#include<opencv2/opencv.hpp>
#include <cmath>
namespace rm
{
	/************************
	* 时间打弹缓冲器
	************************/
	class ShootBuffer
	{
	public:
		ShootBuffer() :min_time_stamp(0.5) { last_time = 0; };
		ShootBuffer(double min_time_stamp)
			:min_time_stamp(min_time_stamp)
		{
			last_time = 0;
		};
		// 更新,开始last_time为0,必更新
		bool drop(double time = get_now_time()) {
			double time_stamp = time - last_time;
			if (time_stamp > min_time_stamp) {
				last_time = time;
				return true;
			}
			return false;
		};
		// 获取当前时间
		static double get_now_time();

	private:
		// 最近通过时间,也是参考时间
		double last_time;
		// 最小时间戳
		const double min_time_stamp;
	};

	/************************
	* 击打延迟器
	************************/
	class ShootDelayer
	{
	public:
		ShootDelayer() = default;
	public:
		void reset(); // 重置
		bool over(); // 判断结束
		double set_shoot_delay_time(double delay_time);
	private:
		double last_time;
		double _shoot_delay_time_ = 0; // 设定延迟时间
	};


	/*
	* 射击策略类,根据敌方的状态
	* 输入预测时间后的目标位置
	* 获取瞄准目标和击打时机
	*/
	class ShootStrategy
	{
	protected:
		RangeSetter* RangeSetter__;
	public:
		ShootStrategy() = default;
		ShootStrategy(RangeSetter* RangeSetter__);
		virtual ~ShootStrategy() {};
	public:
		// 输入预测后的敌方射击坐标系下的四点,获取当前的瞄准目标,击打目标不为用户提供
		// 返回能够开火的延迟时间,-1表示不能击打
		virtual float calculate_target(cv::Point3f& aim_target,
			std::vector<cv::Point3f> car_four_points, const Eigen::VectorXf& state, const float& car_yaw) = 0;
	protected:
		float shoot_time = -1; // 与能够射击的时间戳的时间差
		// 四点按照距离排序
		void sort_car_four_points(std::vector<cv::Point3f>& car_four_points);
		// 获取车中心二维坐标
		cv::Point2f get_car_middle(const std::vector<cv::Point3f>& car_four_points);
		// 计算角ABC的角度
		float calculate_angle_abs(const cv::Point2f& A, const cv::Point2f& B, const cv::Point2f& C);
		// 获取最近的左右点
		void get_left_right_points(cv::Point3f& left_point, cv::Point3f& right_point,
			const std::vector<cv::Point3f>& sorted_car_four_points);
		// 射击判断
		float shoot_judge(const cv::Point3f& shoot_target);
	};



	/*
	* 射击中心策略
	* 永远倾向于瞄准车的中心
	* 按照装甲板
	*/
	class ShootMiddleStrategy : public ShootStrategy
	{
	public:
		ShootMiddleStrategy() = default;
		ShootMiddleStrategy(RangeSetter* RangeSetter__);
		virtual ~ShootMiddleStrategy() {};
	public:
		// 用于清空历史信息
		virtual void clear() {};

		float calculate_target(cv::Point3f& aim_target, std::vector<cv::Point3f> car_four_points, const Eigen::VectorXf& state, const float& car_yaw) override;

		//选择目标
		virtual cv::Point3f choose_target(std::vector<cv::Point3f> car_four_points, const Eigen::VectorXf& state);
	};


	/*
	* 精确击打策略
	* 继承射击中心策略
	* 思想核心:每次转过枪口的装甲板打且仅击打一次
	*/
	class PrecisionShootStrategy : public ShootMiddleStrategy
	{
	public:
		PrecisionShootStrategy() = default;
		PrecisionShootStrategy(RangeSetter* RangeSetter__);
	public:
		float calculate_target(cv::Point3f& aim_target, std::vector<cv::Point3f> car_four_points, const Eigen::VectorXf& state, const float& car_yaw) override;
		// 当目标切换或为空,重置
		void clear() override;
		// 放入
		void push_image_timer_differ(float image_timer_differ);
	private:
		// 击打逻辑
		// 对于射击装甲板,如果枪口落后,则放弃该目标
		// 枪口未落后,认为枪口与车中心相对静止,根据装甲板到枪口射击时的时间差作为解
		float judging_shoot_logic(cv::Point3f shoot_target, cv::Point2f car_middle2f, const Eigen::VectorXf& state);
	public:
		// 获取击打朝向角
		static float get_shoot_face_angle(cv::Point3f shoot_target, cv::Point2f car_middle2f);
		// 计算夹角
		static float angleABC(cv::Point2f A, cv::Point2f B, cv::Point2f C);
		// 计算圆与x=0的交点
		static std::vector<cv::Point2f> find_intersections(cv::Point2f center, float radius);
	private:
		float get_image_timer_differ_average() const;
		// 相机侦差作为判断能否跳过帧的逻辑判断
		std::vector<float> image_timer_differ_queue;
		int image_timer_differ_queue_max_size = 10;
	private:
		bool first_time;
		bool be_shooted; // 是否已被击打
		float last_face_angle; // 标记的绝对朝向角
	};

	/*
	* 自适应精确击打策略
	* 思想核心:根据最高稳定射频计算出发弹时间区间,一旦装甲板足够靠近发弹时间就延迟后在计算出的区间内连续发弹
	*/
	class AdaptivePrecisionShootStrategy : public ShootMiddleStrategy
	{
	public:
		AdaptivePrecisionShootStrategy() = default;
		AdaptivePrecisionShootStrategy(float max_stability_shoot_frequency, RangeSetter* RangeSetter__);

		void clear() override;
		// 装入时间差
		void update_image_timer_differ(float image_timer_differ);

		// 计算持续时间
		// 基于射频,装甲板尺寸,转速
		void calculate_sustain_shoot_time(float armor_size, const Eigen::VectorXf& state);

		float calculate_target(cv::Point3f& aim_target, std::vector<cv::Point3f> car_four_points, const Eigen::VectorXf& state, const float& car_yaw) override;

	private:
		// 开火逻辑:
		// 检测当前离最优击打的时间,时间小于帧差就延迟击打,且进入发射状态
		// 在开火持续时间内,默认直接开火
		float judging_shoot_logic(cv::Point3f shoot_target, cv::Point2f car_middle2f, const Eigen::VectorXf& state);
		
	private:
		bool shoot_state;
		// 起始击打时间
		double start_shoot_time;
		// 开火持续时间
		double shoot_sustain_time;
	private:
		// 最大稳定射频
		float max_stability_shoot_frequency;
		float get_image_timer_differ_average() const;
		// 时间戳序列,用于取算术平均
		int image_timer_differ_queue_max_size = 5;
		std::vector<float> image_timer_differ_queue;
	private:
		bool first_time;
		bool be_shooted; // 是否已被击打
		float last_face_angle; // 标记的绝对朝向角
	};

	class ShootTrackStrategyBase : public ShootStrategy
	{
	public:
		ShootTrackStrategyBase() = default;
		ShootTrackStrategyBase(float switch_reverse_amplification_weight);
		ShootTrackStrategyBase(RangeSetter* RangeSetter__, float switch_reverse_amplification_weight);

		// 规划击打目标
		virtual cv::Point3f choose_absolute_target(std::vector<cv::Point3f> car_four_absolute_points,
			std::vector<cv::Point3f> car_four_shoot_points,
			const Eigen::VectorXf& state, const float& car_yaw) = 0;
		
		bool get_aim_preview() const;

		// 获取切换增幅
		float get_switch_reverse_amplification_weight() const;

	protected:
		bool aim_preview; // 预瞄
		float switch_reverse_amplification_weight; // 切换增幅
		// 计算旋转后的点
		// rotate_angle为弧度值,长度单位为厘米
		cv::Point2f calculate_rotate_point2f(const cv::Point2f& car_middle2f, float radius, float rotate_angle);
	};

	/*
	* 跟随击打策略
	* 当前策略为手动更新,关键是朝向角参数
	* 提供可扩展的朝向角计算函数功能
	*/
	class ShootTrackStrategy : public ShootTrackStrategyBase
	{
	public:
		ShootTrackStrategy() = default;
		ShootTrackStrategy(bool ignore_min_shoot_x,float switch_reverse_amplification_weight,
			const std::function<float(const Eigen::VectorXf&)>& track_face_angle_func,
			RangeSetter* RangeSetter__);
		//ShootTrackStrategy(bool ignore_min_shoot_x, const float& static_max_stability_track_rpm,
		//	const float& track_rotate_speed_acceleration_buffer_k,
		//	RangeSetter* RangeSetter__);
	public: // 用于debug
		float last_track_face_angle = 0;
	public:
		// 则返回绝对坐标系
		cv::Point3f choose_absolute_target(std::vector<cv::Point3f> car_four_absolute_points,
			std::vector<cv::Point3f> car_four_shoot_points,
			const Eigen::VectorXf& state, const float& car_yaw) override;
		
		float calculate_target(cv::Point3f& aim_target, std::vector<cv::Point3f> car_four_points,
			const Eigen::VectorXf& state, const float& car_yaw) override;
		// 获取当前是否为预瞄
		bool get_aim_preview() const;
		// 获取切换增幅
		float get_switch_reverse_amplification_weight() const;
	private:
		// 计算跟随朝向角函数,输出为绝对值,默认采用简单暴力函数
		std::function<float(const Eigen::VectorXf&)> track_face_angle_func;
		// 自适应计算跟随角度
		float adaptive_calculate_track_face_angle(const Eigen::VectorXf& state);
		float static_max_stability_track_rotate_speed; // 电机最大稳定跟随速度
		float track_rotate_speed_acceleration_buffer_k;  // 电机加速缓冲斜率
		float calculate_rotate_cost(float track_face_angle, float rotate_speed_abs,
			float static_max_stability_track_rotate_speed);
	private:
		bool ignore_min_shoot_x; // 只要非track,就开火

	};

	/*
	* 间断开火策略
	*/
	class ShootStepStrategy : public ShootStrategy
	{
		ShootBuffer* ShootBuffer__;
	public:
		ShootStepStrategy() = default;
		ShootStepStrategy(float setting_shoot_interval);
		~ShootStepStrategy();
	public:
		float calculate_target(cv::Point3f& aim_target, std::vector<cv::Point3f> car_four_points,
			const Eigen::VectorXf& state, const float& car_yaw) override;
	};

	/*
	* 持续射击策略
	*/
	class ShootSustainStrategy : public ShootStrategy
	{
	public:
		ShootSustainStrategy();
	public:
		float calculate_target(cv::Point3f& aim_target, std::vector<cv::Point3f> car_four_points,
			const Eigen::VectorXf& state, const float& car_yaw) override;
	};

	/*
	* 持续射击策略
	*/
	class ShootCancelStrategy : public ShootStrategy
	{
	public:
		ShootCancelStrategy();
	public:
		float calculate_target(cv::Point3f& aim_target, std::vector<cv::Point3f> car_four_points,
			const Eigen::VectorXf& state, const float& car_yaw) override;
	};

	/*
	* 平动击打模式,需要考虑装甲板优先击打识别到的目标
	*
	*/
	class ShootTranslationStrategy : public ShootStrategy
	{
	public:
		ShootTranslationStrategy() = default;
		ShootTranslationStrategy(RangeSetter* RangeSetter__);
	public:
		void clear();
		// 平动目标选择器,需要结构上需要优化
		// 由于我们一直知道目标的四点以及相关角度,根据绝对坐标系四点和car_yaw计算击打目标朝向角度
		// 这里的四点一般是使用平移观测器更新的
		cv::Point3f translation_target_choose(Coordinate* Coordinate__, std::vector<cv::Point3f> shoot_car_four_points,
			const float& car_yaw, const float& car_pitch);

		float calculate_target(cv::Point3f& aim_target, std::vector<cv::Point3f> car_points,
			const Eigen::VectorXf& state, const float& car_yaw) override;
	private:
		// 计算偏转角
		float calculate_angle(const cv::Point2f& A, const cv::Point2f& B, const cv::Point2f& C);
	private:
		float last_abs_shoot_yaw; // 上一个击打目标的绝对角 
	};

	/*
	* 基于时间间隔与仅击打一侧装甲板的中心击打策略
	* 用于英雄
	*/
	class HeroShootStrategy : public ShootMiddleStrategy
	{
		std::unique_ptr<ShootBuffer> ShootBuffer__;
	public:
		HeroShootStrategy() = default;
		HeroShootStrategy(float min_shoot_interval, RangeSetter* RangeSetter__);
	public:
		float calculate_target(cv::Point3f& aim_target, std::vector<cv::Point3f> car_four_points, const Eigen::VectorXf& state, const float& car_yaw) override;
		cv::Point3f choose_target(std::vector<cv::Point3f> car_four_points, const Eigen::VectorXf& state) override;
	};

	/*
	* 模糊击打策略
	* 瞄准中心,在一定朝向角范围内一直射击
	* TODO:可以改为以时间为参数,乘以转速得到自适应的角度
	*/
	class ShootMiddleVagueStrategy : public ShootStrategy
	{
	public:
		ShootMiddleVagueStrategy() = default;
		ShootMiddleVagueStrategy(float shooting_time,RangeSetter* RangeSetter__);
	public:
		void get_aim_preview(std::vector<cv::Point3f> car_four_absolute_points,
			std::vector<cv::Point3f> car_four_shoot_points,
			const Eigen::VectorXf& state);
		float calculate_target(cv::Point3f& aim_target, std::vector<cv::Point3f> car_four_points,
			const Eigen::VectorXf& state, const float& car_yaw) override;
	private:
		float shooting_time; // 击打时间区间
		bool aim_preview;
	};

	/*
	* 缓冲火控模式
	* 用于哨兵识别到目标后,稳定锁定目标来拟合
	* 只会瞄准真实目标,且不会开火
	*/
	class ShootBufferStrategy : public ShootStrategy
	{
	public:
		ShootBufferStrategy();

		float calculate_target(cv::Point3f& aim_target, std::vector<cv::Point3f> car_four_points,
			const Eigen::VectorXf& state, const float& car_yaw) override;

		void set_car_true_middle(cv::Point3f car_true_middle);

	private:
		cv::Point3f car_true_middle;
	};

	/*
	* 自适应跟随击打
	* 仅设置跟随角,其他参数2由解算自身计算。该算法不能用于转速过慢的目标
	* 自适应跟随击打的开火判断完全独立
	* TODO: 需要保证子弹落点拟合到整车观测的装甲板上
	*/
	class AutoTrackStrategy : public ShootTrackStrategyBase
	{
	public:
		AutoTrackStrategy() = default;
		AutoTrackStrategy(RangeSetter* RangeSetter__, 
			float static_track_angle, float switch_reverse_amplification_weight,float max_shoot_x_ratio);
		/*
		* 将敌方旋转过程划分为: 超越跟随区域 - 通过跟随区域 - 超越跟随区域 - 通过跟随区域 ...
		* 当敌方处于超越跟随区域,瞄准预瞄角并在能够开火时设置aim_preview=false,持续开火,直到判断需要预瞄,停止开火
		* 判断需要预瞄: 
			1. 枪口落后敌方装甲板(这里可能存在问题,因为开始加速时可能会落后)
			2. 超过预瞄角
		  判断开火逻辑:
			1. aim_preview=true,基于装甲板朝向计算允许min_shoot_x,一旦能够开火,aim_preview=false
			2. 一旦判断需要预瞄,aim_preview=true,否则保持aim_preview=false一直射击
		* 
		*/

		cv::Point3f choose_absolute_target(std::vector<cv::Point3f> car_four_absolute_points,
			std::vector<cv::Point3f> car_four_shoot_points, const Eigen::VectorXf& state, const float& car_yaw) override;
		
		float calculate_target(cv::Point3f& aim_target, std::vector<cv::Point3f> car_four_points,
			const Eigen::VectorXf& state, const float& car_yaw) override;

	private:
		// 预瞄函数
		cv::Point3f preview_func(std::vector<cv::Point3f> car_four_absolute_points,
			std::vector<cv::Point3f> car_four_shoot_points,cv::Point2f car_absolute_middle2f,float radius,
			float rotational_speed,float track_face_angle);

	private:
		float static_track_face_angle; // 静态跟随角
		float max_shoot_x_ratio; // 缩放比例
		bool advance_aim_preview = false; // 提前预瞄
		bool advance_shoot = false; // 提前开火
	};


};


#endif // !_SHOOTSTRATEGY_H_
