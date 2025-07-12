#include "Controler.h"

namespace rm
{
	Controler::Controler(
		Coordinate* Coordinate__,
		PitchCompensator* PitchCompensator__,
		RangeSetter* RangeSetter__,
		TimePredictor* TimePredictor__,
		ShootTrackStrategyBase* ShootTrackStrategy__,
		ShootMiddleStrategy* ShootMiddleStrategy__,
		HeroShootStrategy* HeroShootStrategy__,
		PrecisionShootStrategy* PrecisionShootStrategy__,
		ShootTranslationStrategy* ShootTranslationStrategy__,
		ShootMiddleVagueStrategy* ShootMiddleVagueStrategy__,
		StrategyPlanner* StrategyPlanner__,
		float delay_shooting_param,
		StrategyPlanner::STATE state)
		:
		Coordinate__(Coordinate__),
		PitchCompensator__(PitchCompensator__),
		RangeSetter__(RangeSetter__),
		TimePredictor__(TimePredictor__),
		ShootTrackStrategy__(ShootTrackStrategy__),
		ShootMiddleStrategy__(ShootMiddleStrategy__),
		HeroShootStrategy__(HeroShootStrategy__),
		PrecisionShootStrategy__(PrecisionShootStrategy__),
		ShootTranslationStrategy__(ShootTranslationStrategy__),
		ShootMiddleVagueStrategy__(ShootMiddleVagueStrategy__),
		StrategyPlanner__(StrategyPlanner__)
	{
		// 一旦设定模式,进入锁定状态
		if (state != StrategyPlanner::STATE::shoot_default) this->lock = true;
		ShootDelayer_.set_shoot_delay_time(delay_shooting_param);
		this->state = state;
	};
	TransControler Controler::iteration_control(int target_kind, float program_timer_differ, Eigen::VectorXf car_state, 
		Eigen::VectorXf trans_state, const float& car_yaw, const float& car_pitch, const float& car_yaw_speed, const int& grade)
	{
		TransControler trans_control_data;
		
		if (target_kind == 0) {	// 此时说明无目标
			// 清空,这里可能存在击打目标切换但是ShootMiddleStrategy__未清空的情况,但是对击打效果逻辑上影响不大
			clear();
			return trans_control_data;
		};

		if (target_kind != last_target_kind) { // 类型不同,重置击打延迟器
			ShootDelayer_.reset();
		};

		last_target_kind = target_kind;

		// 基于敌方目标为大小装甲板设定击打范围器
		RangeSetter__->set_armor_size_small(last_target_kind != 1);

		// 基于RPM的策略判断
		if (!lock) {
			this->state = StrategyPlanner__->state_update(car_state,grade);
		};
		
		// 迭代前将射击目标设置为车正中心
		float average_radius = (car_state(8) + car_state(9)) / 2;
		float average_height = car_state(11) == 1 ? car_state(1) - car_state(10) / 2 : car_state(1) + car_state(10) / 2;
		this->last_absolute_aim_target = cv::Point3f(car_state(0), average_height, car_state(2) - average_radius);

		TransControler TransControler_;
		float pred_time;
		int iteration = 4;
		for (int i = 0; i < iteration; i++) // 迭代5次
		{
			last_iteration = i == iteration - 1;

			Eigen::VectorXf car_state_copy = car_state;
			this->last_true_state = car_state_copy;
			// 计算延迟时间
			pred_time = TimePredictor__->time_predict(PitchCompensator__, this->last_absolute_aim_target, program_timer_differ);

			// 更新所有目标至射击坐标系
			std::vector<cv::Point3f> car_four_points_absolute;
			if (state == StrategyPlanner::STATE::shoot_translation) {
				car_state_copy(0) = car_state_copy(0) + trans_state(3) * pred_time;
				car_state_copy(1) = car_state_copy(1);
				car_state_copy(2) = car_state_copy(2) + trans_state(4) * pred_time;
				car_state_copy(4) = trans_state(3);
				car_state_copy(6) = trans_state(4);
			}
			else {
				car_state_copy(0) += car_state_copy(4) * pred_time;
				car_state_copy(1) += car_state_copy(5) * pred_time;
				car_state_copy(2) += car_state_copy(6) * pred_time;
				car_state_copy(3) += car_state_copy(7) * pred_time;
			};
			this->last_pred_state = car_state_copy;
			car_four_points_absolute =
				CarSolver::fit_four_armors(cv::Point3f(car_state_copy(0), car_state_copy(1), car_state_copy(2)), car_state_copy(3),
					car_state_copy(8), car_state_copy(9), car_state_copy(10), car_state_copy(11));

			// 转射击坐标系
			std::vector<cv::Point3f> car_four_points_shoot(4);
			for (int i = 0; i < 4; i++) {
				car_four_points_shoot[i] = Coordinate__->abs2sot(car_four_points_absolute[i], car_yaw, car_pitch, car_yaw_speed);
			};

			TransControler_ = update_by_state(car_four_points_shoot, car_four_points_absolute,
				car_state_copy, car_yaw, car_pitch, car_yaw_speed);
		};

		TransControler_.rpm = car_state(7) * 30 / CV_PI;
		TransControler_.enemy_kind = last_target_kind;

		cv::Point3f car_state_cam = Coordinate__->abs2cam(cv::Point3f(car_state(0), car_state(1), car_state(2)),car_yaw,car_pitch);
		TransControler_.enemy_x = car_state_cam.x;
		TransControler_.enemy_y = car_state_cam.z;

		if (!ShootDelayer_.over())
			TransControler_.shoot_time = -1.0;

		return TransControler_;
	}

	StrategyPlanner::STATE Controler::get_controler_state() const
	{
		return this->state;
	}
	cv::Mat Controler::draw_aim_targets(cv::Mat src, bool canshoot, const float& car_yaw, const float& car_pitch, cv::Scalar color, int radius, int thickness)
	{
		cv::Point2f target = Coordinate__->abs2pixel(this->debug_absolute_aim_target, car_yaw, car_pitch);
		cv::circle(src, target, radius, color, thickness);
		if (canshoot) { // 画叉!
			cv::line(src, cv::Point2f(target.x - 10, target.y + 10),
				cv::Point2f(target.x - 50, target.y + 50), cv::Scalar(0, 0, 255), 7);
			cv::line(src, cv::Point2f(target.x - 10, target.y - 10),
				cv::Point2f(target.x - 50, target.y - 50), cv::Scalar(0, 0, 255), 7);
			cv::line(src, cv::Point2f(target.x + 10, target.y - 10),
				cv::Point2f(target.x + 50, target.y - 50), cv::Scalar(0, 0, 255), 7);
			cv::line(src, cv::Point2f(target.x + 10, target.y + 10),
				cv::Point2f(target.x + 50, target.y + 50), cv::Scalar(0, 0, 255), 7);
		};
		return src;
	}
	;

	TransControler Controler::update_by_state(const std::vector<cv::Point3f>& pred_car_four_points_shoot,
		const std::vector<cv::Point3f>& pred_car_four_points_absolute,
		const Eigen::VectorXf& pred_state,const float& car_yaw, const float& car_pitch, const float& car_yaw_speed)
	{
		bool aim_preview = false; // 记录预瞄时的状态
		cv::Point3f aim_target; // 瞄准目标
		float shoot_time = -1; // 发弹时机
		switch (state)
		{
		case StrategyPlanner::STATE::shoot_default:
			return TransControler();
		case StrategyPlanner::STATE::shoot_track:
		{
			cv::Point3f absolute_shoot_target = ShootTrackStrategy__->choose_absolute_target(
				pred_car_four_points_absolute, pred_car_four_points_shoot, pred_state, car_yaw);
			aim_target = Coordinate__->abs2sot(absolute_shoot_target, car_yaw, car_pitch, car_yaw_speed);
			shoot_time = ShootTrackStrategy__->calculate_target(aim_target, pred_car_four_points_shoot, pred_state, car_yaw);
			// shoot_time = 0.0;
			aim_preview = ShootTrackStrategy__->get_aim_preview();
		};
		break;
		case StrategyPlanner::STATE::shoot_middle:
		{
			if(last_iteration) { // 处于迭代,仅仅计算最后一次目标的延迟
				shoot_time = ShootMiddleStrategy__->calculate_target(aim_target, pred_car_four_points_shoot, pred_state, car_yaw);
			}
			else{ 
				aim_target = ShootMiddleStrategy__->choose_target(pred_car_four_points_shoot, pred_state);
				shoot_time = -1.0;
			};
		};
		break;
		case StrategyPlanner::STATE::shoot_hero:
		{
			if (last_iteration) { // 处于迭代,仅仅计算最后一次目标的延迟
				shoot_time = HeroShootStrategy__->calculate_target(aim_target, pred_car_four_points_shoot, pred_state, car_yaw);
			}
			else {
				aim_target = HeroShootStrategy__->choose_target(pred_car_four_points_shoot, pred_state);
				shoot_time = -1.0;
			};
		};
		break;
		case StrategyPlanner::STATE::shoot_precise:
		{
			if (last_iteration) { // 处于迭代,仅仅计算最后一次目标的延迟
				shoot_time = PrecisionShootStrategy__->calculate_target(aim_target, pred_car_four_points_shoot, pred_state, car_yaw);
			}
			else {
				aim_target = PrecisionShootStrategy__->choose_target(pred_car_four_points_shoot, pred_state);
				shoot_time = -1.0;
			};
		};
		break;
		case StrategyPlanner::STATE::shoot_translation:
		{ 
			// 选取击打目标
			aim_target = ShootTranslationStrategy__->translation_target_choose(
				Coordinate__, pred_car_four_points_shoot, car_yaw,car_pitch);
			shoot_time = ShootTranslationStrategy__->calculate_target(aim_target, { aim_target }, pred_state,car_yaw);
			shoot_time = 0.0; // 一直开火
		};
		break;
		case StrategyPlanner::STATE::shoot_vague:
		{
			ShootMiddleVagueStrategy__->get_aim_preview(pred_car_four_points_absolute, pred_car_four_points_shoot, pred_state);
			shoot_time = ShootMiddleVagueStrategy__->calculate_target(aim_target, pred_car_four_points_shoot, pred_state, car_yaw);
		};
		break;
		default:
			return TransControler();
		};

		// 计算pitch抬升角

		cv::Point3f absolute_aim_target = Coordinate__->sot2abs(aim_target, car_yaw, car_pitch);

		float fly_delay = 0.0;
		absolute_aim_target = PitchCompensator__->solve_pitch_fly_delay_amplitude(fly_delay, absolute_aim_target);

		this->debug_absolute_aim_target = absolute_aim_target;
		aim_target = Coordinate__->abs2sot(absolute_aim_target, car_yaw, car_pitch, car_yaw_speed);

		// 基于抬升点重新判断
		//shoot_time = RangeSetter__->if_pitch_shoot_hit(aim_target) ? shoot_time : -1.0;

		// 计算控制数据
		float send_pitch, send_yaw;
		send_pitch = send_yaw = 0;
		Coordinate__->pos2angles(send_yaw, send_pitch, aim_target);

		// 计算速度
		// 只有在跟随击打下,才考虑旋转速度的增幅
		float yaw_trans_speed = solve_yaw_trans_speed(absolute_aim_target, send_yaw, pred_state, car_yaw, car_pitch, car_yaw_speed);
		float yaw_rotate_speed = solve_yaw_rotate_speed(absolute_aim_target, send_yaw, pred_state, car_yaw, car_pitch, car_yaw_speed);

		// 控制数据转为弧度
		send_yaw = send_yaw * 180 / CV_PI;
		send_pitch = send_pitch * 180 / CV_PI;

		// 增幅
		cv::Point3f speed_vector(pred_state(4), pred_state(5), pred_state(6));
		speed_vector = Coordinate__->abs2sot(speed_vector, car_yaw, car_pitch, car_yaw_speed);
		// 处于预瞄状态,则反向增幅 / 不增幅
		if (state == StrategyPlanner::STATE::shoot_track) { // 只有在跟随击打下才使用选择旋转增幅
			send_yaw = RangeSetter__->yaw_angle_increase(send_yaw, yaw_trans_speed, aim_preview ?
				-yaw_rotate_speed * ShootTrackStrategy__->get_switch_reverse_amplification_weight() : yaw_rotate_speed, speed_vector.x > 0);
		}
		else {
			send_yaw = RangeSetter__->yaw_angle_increase(send_yaw, yaw_trans_speed, 0, speed_vector.x > 0);
		};
		

		// 限幅
		// 
		float send_yaw_speed = yaw_trans_speed + yaw_rotate_speed;
		RangeSetter__->send_data_range(send_pitch, send_yaw, send_yaw_speed);

		TransControler trans_controler;
		trans_controler.yaw = send_yaw;
		trans_controler.pitch = send_pitch;
		trans_controler.yaw_speed = send_yaw_speed;
		trans_controler.shoot_time = shoot_time;
		trans_controler.move_state = aim_preview ? 2 : 1;
		trans_controler.ifget = (send_yaw != 0 || send_pitch != 0);

		// 将目标的用于迭代
		this->last_absolute_aim_target = absolute_aim_target;

		return trans_controler;
	};

	float Controler::solve_yaw_trans_speed(cv::Point3f aim_target_absolute,
		const float& send_yaw,const Eigen::VectorXf& state, const float& car_yaw, const float& car_pitch, const float& car_yaw_speed)
	{
		float p_t = 0.0001;
		Eigen::VectorXf p_vector(3);
		p_vector(0) = state(4) * p_t;
		p_vector(1) = state(5) * p_t;
		p_vector(2) = state(6) * p_t;
		// 二维选定为瞄准中心x,z
		cv::Point2f car_middle2d(state(0), state(2));
		cv::Point2f aim_target_absolute2f(aim_target_absolute.x, aim_target_absolute.z);

		// 获取p_t时间后的位置
		cv::Point3f p_aim_target_absolute(aim_target_absolute.x + p_vector(0), aim_target_absolute.y + p_vector(1),
			aim_target_absolute.z + p_vector(2));

		// 转到射击坐标系
		cv::Point3f p_aim_target_shoot = Coordinate__->abs2sot(p_aim_target_absolute, car_yaw, car_pitch, car_yaw_speed);
		
		return (atan(p_aim_target_shoot.x / p_aim_target_shoot.z) - send_yaw) / p_t;
	}
	float Controler::solve_yaw_rotate_speed(cv::Point3f aim_target_absolute, const float& send_yaw, 
		const Eigen::VectorXf& state, const float& car_yaw, const float& car_pitch, const float& car_yaw_speed)
	{
		float p_t = 0.0001;

		Eigen::VectorXf p_vector(3);
		p_vector(0) = 0;
		p_vector(1) = 0;
		p_vector(2) = 0;

		// 二维选定为瞄准中心x,z
		cv::Point2f car_middle2d(state(0), state(2));
		cv::Point2f aim_target_absolute2f(aim_target_absolute.x, aim_target_absolute.z);
		cv::Point2f p_mvector = rotate_point(car_middle2d, aim_target_absolute2f, state(7) * p_t) - aim_target_absolute2f;
		p_vector(0) += p_mvector.x;
		p_vector(2) += p_mvector.y;
		// 获取p_t时间后的位置
		cv::Point3f p_aim_target_absolute(aim_target_absolute.x + p_vector(0), aim_target_absolute.y + p_vector(1),
			aim_target_absolute.z + p_vector(2));

		// 转到射击坐标系
		cv::Point3f p_aim_target_shoot = Coordinate__->abs2sot(p_aim_target_absolute, car_yaw, car_pitch, car_yaw_speed);

		return (atan(p_aim_target_shoot.x / p_aim_target_shoot.z) - send_yaw) / p_t;
	}
	cv::Point2f Controler::rotate_point(const cv::Point2f& center, const cv::Point2f& point, float angle)
	{
		float cosA = cos(angle);
		float sinA = sin(angle);
		cv::Point2f translated_point = point - center;
		cv::Point2f rotated_point(
			translated_point.x * cosA - translated_point.y * sinA,
			translated_point.x * sinA + translated_point.y * cosA
		);
		rotated_point += center;
		return rotated_point;
	}



	void Controler::clear()
	{
		ShootMiddleStrategy__->clear();
		ShootTranslationStrategy__->clear();
		if(!lock)
			this->state = StrategyPlanner::STATE::shoot_default;
		last_iteration = false;
		last_target_kind = 0;
	}
}
