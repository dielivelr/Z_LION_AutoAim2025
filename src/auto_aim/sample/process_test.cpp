#include"debug.h"
#ifdef PROCESS_TEST

#include"../detector/Video/Video.h"
#include"../detector/ArmorFinder/ArmorFinder.h"
#include"../detector/LightFinder/LightFinder.h"
#include"../detector/Machine/Machine.h"
#include"../detector/Detector.h"
#include"../processor/Processor.h"
#include"../controler/Controler.h"
#include"../../auto_aim_debug/src/ImageShower/ImageShower.h"
#include"../../auto_aim_debug/src/Color/Color.h"
#include"../../auto_aim_debug/src/Tools/Tools.h"

using namespace rm;

using STATE = rm::StrategyPlanner::STATE;


int main()
{
	std::vector<std::string> video_path = {
		"../auto_aim_publisher/videos/sample.avi",
	};
	Video video(video_path);

	// Machine
	Machine* Machine__ = new AutoMachine();
	
	// UsualLightFinder
	bool enemy_blue = true;
	float max_ratio_w_h = 1.1;
	float max_angle = 35.0;
	bool SIMPLE = false;
	UsualLightFinder* UsualLightFinder__
		= new UsualLightFinder(max_ratio_w_h, max_angle, SIMPLE);

	// MiddleLightFinder
	float ret_dis_error = 5.0;
	MiddleLightFinder* MiddleLightFinder__
		= new MiddleLightFinder(UsualLightFinder__, ret_dis_error);

	// ArmorFinder
	float armor_min_height_ratio = 0.1;
	float small_armor_min_aspect_ratio = 0.6;
	float small_armor_max_aspect_ratio = 3.2;
	float big_armor_min_aspect_ratio = 3.2;
	float big_armor_max_aspect_ratio = 8.0;
	float max_center_angle = 25.0;
	ArmorFinder* ArmorFinder__
		= new ArmorFinder(armor_min_height_ratio, small_armor_min_aspect_ratio,
			small_armor_max_aspect_ratio, big_armor_min_aspect_ratio,
			big_armor_max_aspect_ratio, max_center_angle);

	// YoloModel
	std::string model_path = "./models/yolo.onnx";
	int image_size = 256;
	YoloModel* YoloModel__ = new YoloModel(model_path, image_size);

	// Detector
	int img_width = 1440;
	int img_height = 1080;
	float _width_expand_ = 1.414;
	float _height_expand_ = 1.414;

	std::unique_ptr<Detector> Detector__
		= std::make_unique<FusionDetector>(Machine__, YoloModel__, ArmorFinder__
			, MiddleLightFinder__, _width_expand_, _height_expand_, img_width, img_height);


	// Coordinate
	cv::Mat intrinsic = (cv::Mat_<double>(3, 3) << 2395.426254180501, 0.0, 694.9358835586881,
		0.0, 2395.806816563093, 470.1413626685076,
		0.0, 0.0, 1.0);
	cv::Mat distcoeffs = (cv::Mat_<double>(1, 5) <<
		-0.01256079948602146, -0.8310156617739106, -0.004397133054070004, -0.009633099373784157, 7.968233659224768);

	float shaft_differ_x = 0.0;
	float shaft_differ_y = -4.56;
	float shaft_differ_z = 14.396;
	bool SOLVEPNP_IPPE = true;
	float sot_shaft_differ_x = 0.0;
	float sot_shaft_differ_y = 0.0;
	float sot_shaft_differ_z = 0.0;
	float sot_shifting_yaw = 0.0;
	float sot_shifting_pitch = 0.0;
	float wheel_to_center_distance = 0;
	Coordinate* Coordinate__ = new Coordinate(intrinsic, distcoeffs, shaft_differ_x, shaft_differ_y, 
		shaft_differ_z, SOLVEPNP_IPPE, 
		sot_shaft_differ_x, sot_shaft_differ_y, sot_shaft_differ_z, sot_shifting_yaw, sot_shifting_pitch, wheel_to_center_distance);

	// DiagonalAngleSolver
	float inclined = 0.4;
	float pixel_error = 2.0;
	int orientationangle_iteration = 18;
	float increase_param_blue = 0.5;
	float increase_param_red = 0.5;
	FaceAngleSolver* FaceAngleSolver__ = new DiagonalAngleSolver(
		inclined, pixel_error, orientationangle_iteration, Coordinate__, increase_param_blue, increase_param_red);

	//ArmorSolver
	ArmorSolver* ArmorSolver__
		= new ArmorSolver(Coordinate__, FaceAngleSolver__);

	// TargetPlanner
	int buffer_size = 18;
	int _kind_buffer_size_ = 3;
	TargetPlanner* TargetPlanner__ = new DefaultPlanner(buffer_size); // BufferPlanner DefaultPlanner

	// TranslationState
	std::vector<double> trans_q_vs = { 0.1, 0.1,0.1, 0.01, 0.01 };
	std::vector<double> trans_r_vs = { 0.3, 0.2, 1.0 };
	TranslationStateBase* TranslationState__ = new EkfTranslationState(trans_q_vs, trans_r_vs);

	// CarState
	std::vector<double> q_vs = { 0.1, 0.01, 0.1, 0.02, 0.05, 0.05, 0.001, 0.1 ,0.001 };
	std::vector<double> r_vs = { 1.0, 1.0, 1.0, 0.2 };

	bool use_middle_car_state = false;
	CarStateBase* CarState__;
	if (use_middle_car_state) {
		CarState__ = new MIddleCarState(q_vs, r_vs); // MIddleCarState
	}
	else {
		CarState__ = new EkfCarState(q_vs, r_vs); // EkfCarState
	};

	// CarSolver
	NoneSolver* NoneSolver__ = new NoneSolver(CarState__, TranslationState__);

	SingleSolver* SingleSolver__ = new SingleSolver(NoneSolver__, CarState__, TranslationState__, use_middle_car_state);
	

	// AngleOptimizer
	float min_angle_error = 1.8;
	float min_update_angle = 40;
	float max_update_angle = 50;
	AngleOptimizer* AngleOptimizer__ = new AngleOptimizer(
		min_angle_error, min_update_angle, max_update_angle);

	// DoubleSolver
	int radius_optimizer_queue_length = 10;
	DoubleSolver* DoubleSolver__ = new DoubleSolver(SingleSolver__, NoneSolver__,
		CarState__, TranslationState__,
		AngleOptimizer__, radius_optimizer_queue_length);

	// Processor
	float vxz_truncation_rotate_rpm = 100000;
	float max_detect_distance = 100000;
	std::unique_ptr<ProcessorBase> Processor__
		= std::make_unique<Processor>(CarState__, 
			ArmorSolver__, TargetPlanner__, NoneSolver__, SingleSolver__, DoubleSolver__,
			vxz_truncation_rotate_rpm, max_detect_distance);
	// PitchCompensator

	// 水平距离修正函数
	auto horizontal_distance_correct_func = [](const float& horizontal_distance) {
		return horizontal_distance;
		};

	float bullet_speed = 1145.0;
	float pitch_compensation_amplitude = 1.0;
	PitchCompensator* PitchCompensator__ = new PitchCompensator(horizontal_distance_correct_func,bullet_speed, pitch_compensation_amplitude);
	// RangeSetter
	float max_shoot_x = 6.75;
	float max_shoot_y = 4.5; 
	float max_shoot_middle_x = 3.5;
	float max_send_yaw = 10.0; 
	float max_send_pitch = 4.0;
	float max_send_yaw_speed = 3000.0;
	float trans_control_compensation_left = 0.0;
	float trans_control_compensation_right = 0.0;
	float rotate_control_compensation_left = 0.0;
	float rotate_control_compensation_right = 0.0;
	RangeSetter* RangeSetter__ = new RangeSetter(max_shoot_x, max_shoot_y, max_shoot_middle_x, max_send_yaw, max_send_pitch
		, max_send_yaw_speed, trans_control_compensation_left, trans_control_compensation_right,
		rotate_control_compensation_left, rotate_control_compensation_right);

	// TimePredictor
	float shoot_delay = 0.11;
	float serial_delay = 0.005;
	TimePredictor* TimePredictor__ = new TimePredictor(shoot_delay,bullet_speed,serial_delay);

	// ShootTrackStrategy
	bool ignore_min_shoot_x = true;

	// 跟随击打朝向角区间映射函数
	auto track_face_angle_func = [](const Eigen::VectorXf& car_state) {
		float rpm = abs(car_state(7)) * 30 / CV_PI;
		float track_angle;
		if (rpm > 60) {
			track_angle = 10;
		}
		else if (rpm > 43) {
			track_angle = 20;
		}
		else if (rpm > 30) {
			track_angle = 30;
		}
		else {
			track_angle = 37;
		};
		return track_angle / 180 * CV_PI;
	};

	float switch_reverse_amplification_weight = 0.8;

	ShootTrackStrategyBase* ShootTrackStrategy__ = new ShootTrackStrategy(ignore_min_shoot_x, switch_reverse_amplification_weight,
		track_face_angle_func,  RangeSetter__);

	// ShootMiddleStrategy
	ShootMiddleStrategy* ShootMiddleStrategy__ = new ShootMiddleStrategy(RangeSetter__);
	/*float min_shoot_interval = 1.5;
	ShootMiddleStrategy* ShootMiddleStrategy__ = new HeroShootStrategy(min_shoot_interval, RangeSetter__);*/

	// ShootMiddleStrategy
	float min_shoot_interval = 0.40;
	HeroShootStrategy* HeroShootStrategy__ = new HeroShootStrategy(min_shoot_interval,RangeSetter__);

	// PrecisionShootStrategy
	PrecisionShootStrategy* PrecisionShootStrategy__ = new PrecisionShootStrategy(RangeSetter__);

	// ShootTranslationStrategy
	double longest_buffer_time = 0.5;
	ShootTranslationStrategy* ShootTranslationStrategy__ = new ShootTranslationStrategy(RangeSetter__);

	// ShootMiddleVagueStrategy
	float shooting_time = 0.04;
	ShootMiddleVagueStrategy* ShootMiddleVagueStrategy__ = new ShootMiddleVagueStrategy(shooting_time, RangeSetter__);

	// StrategyPlanner
	float min_shoot_track_rpm = 7;
	float min_shoot_middle_rpm = 70;
	float min_shoot_precise_rpm = 90;
	float min_shoot_track_distance = 100;
	float max_shoot_track_distance = 600;
	
	StrategyPlanner* StrategyPlanner__ = new StrategyPlanner(
		min_shoot_track_rpm, min_shoot_middle_rpm, min_shoot_precise_rpm, min_shoot_track_distance, max_shoot_track_distance
	);


	// Controler
	
	// 0 默认模式
	// 1 跟随模式
	// 2 中心击打模式
	// 3 精准击打模式
	// 4 平动击打模式
	// 5 模糊击打模式
	// 6 英雄射击模式
	int controler_state_id = 0;

	STATE controler_state = static_cast<STATE>(controler_state_id);
	float delay_shooting_param = 0.5;
	std::unique_ptr<Controler> Controler__
		= std::make_unique<Controler>(Coordinate__,PitchCompensator__,RangeSetter__,
			TimePredictor__,ShootTrackStrategy__,ShootMiddleStrategy__, HeroShootStrategy__, PrecisionShootStrategy__,
			ShootTranslationStrategy__, ShootMiddleVagueStrategy__, StrategyPlanner__, delay_shooting_param,controler_state);

	//主循环
	while (1) {

		double image_header = get_now_time();
		cv::Mat src = video.ReadPic();
		if (src.empty()) {
			break;
		};

		UsualLightFinder__->set_enemy_color(enemy_blue);
		if(YoloModel__) YoloModel__->set_enemy_color(enemy_blue);

		std::vector<SendArmor> send_armors = Detector__->detect(src);

		///////////////////////////////////////////////

		// 模拟转化
		float car_yaw = 0.0;
		float car_pitch= -0.083;// 
		float car_yaw_speed = 0.0;
		int grade = 7;

		std::vector<ArmorPos> trans_armor_poses;
		for (const auto& send_armor : send_armors) {
			ArmorPos armor_pos;
			armor_pos.pixel_four_point = {
				send_armor.middle_four_points[0],
				send_armor.middle_four_points[1],
				send_armor.middle_four_points[2],
				send_armor.middle_four_points[3],
			};

			armor_pos.kind = 3;//send_armor.label;
			armor_pos.light_loss = send_armor.light_loss;
			if (send_armor.label == 1) armor_pos.small_armor = false;
			trans_armor_poses.push_back(armor_pos);
		};

		FaceAngleSolver__->set_enemy_color(enemy_blue);

		// 解算
		float new_d_t = 0.01;
		TransProcessor trans_processor = Processor__->process(trans_armor_poses, new_d_t, car_yaw, car_pitch);

		///////////////////////////////////////////////

		PrecisionShootStrategy__->push_image_timer_differ(new_d_t);

		// 计算程序执行时间
		double now_time = get_now_time();
		double program_timer_differ = now_time - image_header;

		// 控制
		TransControler trans_controler
			= Controler__->iteration_control(
			trans_processor.target_kind, program_timer_differ, trans_processor.car_state, trans_processor.trans_state,
			car_yaw, car_pitch, car_yaw_speed, grade);

		if (!trans_processor.empty()){

			Eigen::VectorXf last_true_state = Controler__->last_true_state;
			Eigen::VectorXf last_pred_state = Controler__->last_pred_state;

			// 画出全部装甲板
			src = Processor__->draw_car(NoneSolver__, FaceAngleSolver__, Coordinate__, src, last_true_state, trans_processor.target_kind != 1,
				car_yaw, car_pitch, rm::Green, 5);

			for (const auto& armor_pos : trans_armor_poses) {
				src = ArmorSolver__->draw_face_angle(src, armor_pos, car_yaw, rm::White);
			};

			src = Processor__->draw_car(NoneSolver__, FaceAngleSolver__, Coordinate__, src, last_pred_state, trans_processor.target_kind != 1,
				car_yaw, car_pitch, rm::Red, 5);

			// 画出预测点和瞄点
			src = Controler__->draw_aim_targets(src, trans_controler.shoot_time >= 0, car_yaw, car_pitch, Orange, 50, 5);
			// src = Controler__->draw_pred_four_points(src, car_yaw, car_pitch, Red);
			cv::putText(src, "rpm: " + std::to_string(last_true_state(7) * 30 / CV_PI)
				, cv::Point(0, 200), 3, 1, Green, 3);
			cv::putText(src, "VX: " + std::to_string(last_pred_state(4))
				, cv::Point(0, 250), 3, 1, Green, 3);
			cv::putText(src, "VZ: " + std::to_string(last_pred_state(6))
				, cv::Point(0, 300), 3, 1, Green, 3);
			// 画出状态
			cv::putText(src, "yaw: " + std::to_string(trans_controler.yaw)
				, cv::Point(0, 50), 3, 1, Green, 3);
			cv::putText(src, "yaw_speed: " + std::to_string(trans_controler.yaw_speed)
				, cv::Point(0, 100), 3, 1, Green, 3);
			cv::putText(src, "pitch: " + std::to_string(trans_controler.pitch)
				, cv::Point(0, 150), 3, 1, Green, 3);
			cv::putText(src, "target_kind: " + std::to_string(TargetPlanner__->get_target_kind())
				, cv::Point(0, 350), 3, 1, Green, 3);

		};

		std::string shoot_model;
		auto controler_state = Controler__->get_controler_state();

		switch (controler_state)
		{
		case STATE::shoot_default:
			shoot_model = "shoot_default";
			break;
		case STATE::shoot_track:
			shoot_model = "shoot_track";
			break;
		case STATE::shoot_middle:
			shoot_model = "shoot_middle";
			break;
		case STATE::shoot_precise:
			shoot_model = "shoot_precise";
			break;
		case STATE::shoot_translation:
			shoot_model = "shoot_translation";
			break;
		case STATE::shoot_vague:
			shoot_model = "shoot_vague";
			break;
		case STATE::shoot_hero:
			shoot_model = "shoot_hero";
			break;
		default:
			break;
		};
		cv::putText(src, "shoot_model: " + shoot_model
			, cv::Point(0, 400), 3, 1, Green, 3);

		//cv::resize(src, src, cv::Size(1080, 810));

		cv::imshow("src", src);
		cv::waitKey(1);

	};

	if (Machine__ != NULL)delete Machine__;
	if (UsualLightFinder__ != NULL)delete UsualLightFinder__;
	if (ArmorFinder__ != NULL)delete ArmorFinder__;
	if (YoloModel__ != NULL)delete YoloModel__;
	if (MiddleLightFinder__ != NULL)delete MiddleLightFinder__;
	if (Coordinate__ != NULL)delete Coordinate__;
	if (FaceAngleSolver__ != NULL)delete FaceAngleSolver__;
	if (ArmorSolver__ != NULL)delete ArmorSolver__;
	if (TargetPlanner__ != NULL)delete TargetPlanner__;
	if (CarState__ != NULL)delete CarState__;
	if (NoneSolver__ != NULL)delete NoneSolver__;
	if (SingleSolver__ != NULL)delete SingleSolver__;
	if (AngleOptimizer__ != NULL)delete AngleOptimizer__;
	if (DoubleSolver__ != NULL)delete DoubleSolver__;
	if (PitchCompensator__ != NULL)delete PitchCompensator__;
	if (RangeSetter__ != NULL)delete RangeSetter__;
	if (TimePredictor__ != NULL)delete TimePredictor__;
	if (ShootTrackStrategy__ != NULL)delete ShootTrackStrategy__;
	if (ShootMiddleStrategy__ != NULL)delete ShootMiddleStrategy__;
	if (ShootTranslationStrategy__ != NULL)delete ShootTranslationStrategy__;
	if (ShootMiddleVagueStrategy__ != NULL)delete ShootMiddleVagueStrategy__;
	if (StrategyPlanner__ != NULL)delete StrategyPlanner__;
	if (HeroShootStrategy__ != NULL)delete HeroShootStrategy__;
	return 0;
};


#endif // CONTROL_TEST