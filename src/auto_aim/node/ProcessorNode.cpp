// // 单线程调试
// // 调试无需考虑敌我方
#include "rclcpp/rclcpp.hpp"
#include "auto_aim_interfaces/msg/debug_processor.hpp"       
#include "auto_aim_interfaces/msg/trans_detector.hpp"    
#include "auto_aim_interfaces/msg/serial_send_data.hpp"     
#include "../processor/Processor.h"
#include "../controler/Controler.h"
#include "../processor/CarSolver/CarState/ExtendEkfCarState/ExtendEkfCarState.h"
#include "../processor/CarSolver/TranslationState/ExtendTranslationState/ExtendTranslationState.h"
#include "std_msgs/msg/header.hpp"
#include"cv_bridge/cv_bridge.h"
using namespace rm;
using STATE = rm::StrategyPlanner::STATE;
using Point2f = auto_aim_interfaces::msg::Point2f;
using SendArmor = auto_aim_interfaces::msg::SendArmor;
using DebugArmor = auto_aim_interfaces::msg::DebugArmor;
using DebugCar = auto_aim_interfaces::msg::DebugCar;
using DebugFaceAngle = auto_aim_interfaces::msg::DebugFaceAngle;
class ProcessorNode : public rclcpp::Node {
public:
    ProcessorNode()
        : Node("auto_aim_processor_node") {

        this->last_timer = this->now();

        // 是否进入调试模式
        this->debug = this->declare_parameter("debug",true);
       
        /////////////////////////////////////////////////////

        // 相机的内外参
        auto intrinsic_vector = this->declare_parameter(
            "Coordinate.intrinsic", std::vector<double>{2395.426254180501, 0.0, 694.9358835586881,
                0.0, 2395.806816563093, 470.1413626685076,
                0.0, 0.0, 1.0});
        auto distcoeffs_vector = this->declare_parameter(
            "Coordinate.distcoeffs", std::vector<double>{-0.01277453310025184, -0.5972643209615739, 0.0004892134380423771, 0.0006469244898998684, -6.186713717883507});

        cv::Mat intrinsic = (cv::Mat_<double>(3, 3) << intrinsic_vector[0], intrinsic_vector[1], intrinsic_vector[2],
            intrinsic_vector[3], intrinsic_vector[4], intrinsic_vector[5],
            intrinsic_vector[6], intrinsic_vector[7], intrinsic_vector[8]);
        cv::Mat distcoeffs = (cv::Mat_<double>(1, 5) <<
            distcoeffs_vector[0], distcoeffs_vector[1], distcoeffs_vector[2], distcoeffs_vector[3], distcoeffs_vector[4]);
        
        // 绝对坐标轴中心偏移向量
        float shaft_differ_x = this->declare_parameter("Coordinate.shaft_differ_x", 0.0);
        float shaft_differ_y = this->declare_parameter("Coordinate.shaft_differ_y", -4.56);
        float shaft_differ_z = this->declare_parameter("Coordinate.shaft_differ_z", 14.396);

        // 射击坐标系偏移量(射击坐标系和相机坐标系相对固定)
        float sot_shaft_differ_x = declare_parameter("Coordinate.sot_shaft_differ_x", 0.0);
		float sot_shaft_differ_y = declare_parameter("Coordinate.sot_shaft_differ_y", 0.0);
		float sot_shaft_differ_z = declare_parameter("Coordinate.sot_shaft_differ_z", 0.005);
		float sot_shifting_yaw = declare_parameter("Coordinate.sot_shifting_yaw", -0.5);
		float sot_shifting_pitch = declare_parameter("Coordinate.sot_shifting_pitch", 5.5);
        float wheel_to_center_distance = declare_parameter("Coordinate.wheel_to_center_distance", 0.0);

        // 是否采用四点共面来优化
        bool SOLVEPNP_IPPE = this->declare_parameter("Coordinate.SOLVEPNP_IPPE", true);
        Coordinate__ = new Coordinate(intrinsic, distcoeffs, shaft_differ_x, shaft_differ_y, 
		shaft_differ_z, SOLVEPNP_IPPE, sot_shaft_differ_x, sot_shaft_differ_y, sot_shaft_differ_z, 
        sot_shifting_yaw / 180 * CV_PI , sot_shifting_pitch / 180 * CV_PI,wheel_to_center_distance / 180 * CV_PI);

        // DiagonalAngleSolver
        float inclined = this->declare_parameter("DiagonalAngleSolver.inclined", 0.392);
        float pixel_error = this->declare_parameter("DiagonalAngleSolver.pixel_error",  1.8);
        int orientationangle_iteration = this->declare_parameter("DiagonalAngleSolver.orientationangle_iteration", 18);
        float increase_param_blue = this->declare_parameter("DiagonalAngleSolver.increase_param_blue", 2.5);
        float increase_param_red = this->declare_parameter("DiagonalAngleSolver.increase_param_red", 2.5);
        FaceAngleSolver__ = new DiagonalAngleSolver(inclined, pixel_error, 
            orientationangle_iteration, Coordinate__,increase_param_blue,increase_param_red);
        
        ArmorSolver__
            = new ArmorSolver(Coordinate__, FaceAngleSolver__);
        
        // TargetPlanner
        int buffer_size = this->declare_parameter("TargetPlanner.buffer_size", 18);
        int _kind_buffer_size_ = this->declare_parameter("TargetPlanner._kind_buffer_size_", 3);
	    TargetPlanner__ = new DefaultPlanner(buffer_size); // BufferPlanner DefaultPlanner

        // TranslationState
        auto trans_q_vs = this->declare_parameter(
            "TranslationState.trans_q_vs", std::vector<double>{ 0.1, 0.1,0.1, 0.001, 0.001});
        auto trans_r_vs = this->declare_parameter(
            "TranslationState.trans_r_vs", std::vector<double>{1.0, 1.0, 10.0 });
        TranslationState__ = new ExtendTranslationState(trans_q_vs, trans_r_vs); // ExtendTranslationState EkfTranslationState

        // CarState
        auto q_vs = this->declare_parameter(
            "CarState.q_vs", std::vector<double>{0.01, 0.01, 0.01, 0.02, 0.05, 0.05, 0.01, 0.05});
        auto r_vs = this->declare_parameter(
            "CarState.r_vs", std::vector<double>{0.1, 0.1, 0.1, 0.2});
        
        bool use_middle_car_state = this->declare_parameter("CarState.use_middle_car_state", false);
        if (use_middle_car_state) {
            CarState__ = new MIddleCarState(q_vs, r_vs); // MIddleCarState
        }
        else {
            CarState__ = new ExtendEkfCarState(q_vs, r_vs); // EkfCarState ExtendEkfCarState
        };

        // CarSolver
        NoneSolver__ = new NoneSolver(CarState__,TranslationState__);
        
        SingleSolver__ = new SingleSolver(NoneSolver__, CarState__, TranslationState__, use_middle_car_state);

        // AngleOptimizer
        float min_angle_error = this->declare_parameter("AngleOptimizer.min_angle_error", 2.5);
        float min_update_angle = this->declare_parameter("AngleOptimizer.min_update_angle", 42.0);
        float max_update_angle = this->declare_parameter("AngleOptimizer.max_update_angle", 52.0);
        AngleOptimizer__ = new AngleOptimizer(
            min_angle_error, min_update_angle, max_update_angle);
        
        // DoubleSolver__ 
        // 半径优化队列长度
        int radius_optimizer_queue_length = this->declare_parameter("DoubleSolver.radius_optimizer_queue_length", 10);
        DoubleSolver__  = new DoubleSolver(SingleSolver__, NoneSolver__, CarState__,TranslationState__,
            AngleOptimizer__, radius_optimizer_queue_length);

        // Processor
        float vxz_truncation_rotate_rpm = this->declare_parameter("Processor.vxz_truncation_rotate_rpm", 80.0);
        float max_detect_distance = this->declare_parameter("Processor.max_detect_distance", 450.0);
        Processor__
            = std::make_unique<Processor>(CarState__,ArmorSolver__,TargetPlanner__,NoneSolver__,SingleSolver__,DoubleSolver__,
                vxz_truncation_rotate_rpm,max_detect_distance);
        
        /////////////////////////////////////////////////////

        float bullet_speed = this->declare_parameter("bullet_speed", 1240.0);

        // PitchCompensator
        // 水平距离修正函数
        auto horizontal_distance_correct_func = [](const float& horizontal_distance) {
            return horizontal_distance;
            };

		float pitch_compensation_amplitude = declare_parameter("PitchCompensator.pitch_compensation_amplitude", 1.0);
		PitchCompensator__ = new PitchCompensator(
            horizontal_distance_correct_func,bullet_speed, pitch_compensation_amplitude);

		// RangeSetter
		float max_shoot_x = declare_parameter("RangeSetter.max_shoot_x", 6.75);
		float max_shoot_y = declare_parameter("RangeSetter.max_shoot_y", 5.5);
        float max_shoot_middle_x = declare_parameter("RangeSetter.max_shoot_middle_x", 5.5);
		float max_send_yaw = declare_parameter("RangeSetter.max_send_yaw", 10.0);
		float max_send_pitch = declare_parameter("RangeSetter.max_send_pitch", 4.0);
		float max_send_yaw_speed = declare_parameter("RangeSetter.max_send_yaw_speed", 4000.0);
        float trans_control_compensation_left = declare_parameter("RangeSetter.trans_control_compensation_left", 0.0);
        float trans_control_compensation_right = declare_parameter("RangeSetter.trans_control_compensation_right", 0.0);
		float rotate_control_compensation_left = declare_parameter("RangeSetter.rotate_control_compensation_left", 0.0);
        float rotate_control_compensation_right = declare_parameter("RangeSetter.rotate_control_compensation_right", 0.0);
		RangeSetter__ = new RangeSetter(max_shoot_x, max_shoot_y, max_shoot_middle_x, max_send_yaw, max_send_pitch
            , max_send_yaw_speed, trans_control_compensation_left, trans_control_compensation_right,
            rotate_control_compensation_left, rotate_control_compensation_right);

		// TimePredictor
		float shoot_delay = declare_parameter("TimePredictor.shoot_delay", 0.091);// 0.11
		float serial_delay = declare_parameter("TimePredictor.serial_delay", 0.005);
		TimePredictor__ = new TimePredictor(shoot_delay,bullet_speed,serial_delay);

		// ShootTrackStrategy

        bool ignore_min_shoot_x = declare_parameter("ShootTrackStrategy.ignore_min_shoot_x", true);
        float switch_reverse_amplification_weight = declare_parameter("ShootTrackStrategy.switch_reverse_amplification_weight", 0.8);
        // 跟随击打朝向角区间映射函数

	    auto track_face_angle_func = [](const Eigen::VectorXf& car_state) {
            float rpm = abs(car_state(7)) * 30 / CV_PI;
            float track_angle;
            if (rpm > 60) {
                track_angle = 8;
            }
             else if (rpm > 45) {
                track_angle = 14;
            }
            else if (rpm > 30) {
                track_angle = 25;
            }
            else if (rpm > 20) {
                track_angle = 34;
            }
            else {
                track_angle = 37;
            };
            return track_angle / 180 * CV_PI;
        };
        ShootTrackStrategy__ = new ShootTrackStrategy(
            ignore_min_shoot_x,switch_reverse_amplification_weight,track_face_angle_func,RangeSetter__);

		// ShootMiddleStrategy
		ShootMiddleStrategy__ = new ShootMiddleStrategy(RangeSetter__);

        // HeroShootStrategy
        float min_shoot_interval =declare_parameter("HeroShootStrategy.min_shoot_interval", 0.4);
        HeroShootStrategy__ = new HeroShootStrategy(min_shoot_interval,RangeSetter__);

        // PrecisionShootStrategy
        PrecisionShootStrategy__ = new PrecisionShootStrategy(RangeSetter__);

		// ShootTranslationStrategy
		ShootTranslationStrategy__ = new ShootTranslationStrategy(RangeSetter__);

        // ShootMiddleVagueStrategy
        float shooting_time = declare_parameter("ShootMiddleVagueStrategy.shooting_time", 0.04);
        ShootMiddleVagueStrategy__ = new ShootMiddleVagueStrategy(shooting_time, RangeSetter__);

        // StrategyPlanner
        float min_shoot_track_rpm = declare_parameter("StrategyPlanner.min_shoot_track_rpm", 15.0);
		float min_shoot_middle_rpm = declare_parameter("StrategyPlanner.min_shoot_middle_rpm", 50.0);
        float min_shoot_precise_rpm = declare_parameter("StrategyPlanner.min_shoot_precise_rpm", 80.0);
        float min_shoot_track_distance = declare_parameter("StrategyPlanner.min_shoot_track_distance", 100.0);
        float max_shoot_track_distance = declare_parameter("StrategyPlanner.max_shoot_track_distance", 300.0);

        StrategyPlanner__ = new StrategyPlanner(
            min_shoot_track_rpm, min_shoot_middle_rpm,min_shoot_precise_rpm,min_shoot_track_distance,max_shoot_track_distance
        );

		// Controler
        int controler_state_id = declare_parameter("Controler.shoot_state", 0);
        float delay_shooting_param = declare_parameter("Controler.delay_shooting_param", 0.5);
		STATE controler_state = static_cast<STATE>(controler_state_id);
        Controler__
            = std::make_unique<Controler>(Coordinate__,PitchCompensator__,RangeSetter__,
                TimePredictor__,ShootTrackStrategy__,ShootMiddleStrategy__, HeroShootStrategy__,PrecisionShootStrategy__,
                ShootTranslationStrategy__,ShootMiddleVagueStrategy__,
                StrategyPlanner__,delay_shooting_param,controler_state);

        trans_detector_sub_ = this->create_subscription<auto_aim_interfaces::msg::TransDetector>(
            "trans_detector_topic", 10,
            std::bind(&ProcessorNode::trans_detector_callback, this, std::placeholders::_1));

        send_data_publisher_ = this->create_publisher<auto_aim_interfaces::msg::SerialSendData>(
            "serial_send_data_topic", 10);

        debug_processor_publisher_ = this->create_publisher<auto_aim_interfaces::msg::DebugProcessor>(
            "debug_processor_topic", 10);

        RCLCPP_INFO(this->get_logger(), "ProcessorNode callback is all right!!!");

    };

private:
    void trans_detector_callback(
        const auto_aim_interfaces::msg::TransDetector::SharedPtr trans_detector_msg) {

        FaceAngleSolver__->set_enemy_color(trans_detector_msg->enemy_blue);

        rclcpp::Time image_timer = trans_detector_msg->header.stamp;
        std::vector<ArmorPos> trans_armor_poses;
        for(const auto& sendarmor : trans_detector_msg->sendarmors)
        {
            ArmorPos armor_pos;
            armor_pos.kind = sendarmor.label;
            armor_pos.small_armor = (sendarmor.label != 1);
            armor_pos.pixel_four_point = {
                cv::Point2f(sendarmor.middle_four_points[0].x,sendarmor.middle_four_points[0].y),
                cv::Point2f(sendarmor.middle_four_points[1].x,sendarmor.middle_four_points[1].y),
                cv::Point2f(sendarmor.middle_four_points[2].x,sendarmor.middle_four_points[2].y),
                cv::Point2f(sendarmor.middle_four_points[3].x,sendarmor.middle_four_points[3].y),
            };
            trans_armor_poses.push_back(armor_pos);
        };

        float car_yaw  = trans_detector_msg->car_yaw;
        float car_pitch = trans_detector_msg->car_pitch;
        float car_yaw_speed = 0; //trans_detector_msg->car_yaw_speed / 180 * CV_PI;
        int grade = trans_detector_msg->grade;

        float image_timer_differ = (image_timer - last_timer).seconds();
        last_timer = image_timer;
        PrecisionShootStrategy__->push_image_timer_differ(image_timer_differ);

        // 解算
        TransProcessor trans_processor = Processor__->process(
            trans_armor_poses, image_timer_differ, car_yaw, car_pitch);
       
        auto now_time = this->now();
        float program_timer_differ = (now_time - image_timer).seconds();

        // 火控
        TransControler trans_controler = Controler__->iteration_control(
                trans_processor.target_kind, program_timer_differ, trans_processor.car_state, trans_processor.trans_state,
                car_yaw, car_pitch,car_yaw_speed, grade);

        // 发送
        auto message = auto_aim_interfaces::msg::SerialSendData();
        message.pitch = trans_controler.pitch;
        message.yaw = trans_controler.yaw;
        message.yaw_speed = trans_controler.yaw_speed;
        message.shoot_time = trans_controler.shoot_time;
        message.move_state = trans_controler.move_state;
        message.ifget = trans_controler.ifget;
        message.rpm = trans_controler.rpm;
        message.enemy_kind = trans_controler.enemy_kind;
        message.enemy_x = trans_controler.enemy_x;
        message.enemy_y = trans_controler.enemy_y;

        send_data_publisher_->publish(message);
        
        if(debug)
        {
            auto debug_message = auto_aim_interfaces::msg::DebugProcessor();
            if(trans_processor.target_kind > 0){

                Eigen::VectorXf last_pred_state = Controler__->last_pred_state;
                Eigen::VectorXf last_true_state = Controler__->last_true_state;

                // true_car
                auto true_car_points = Processor__->get_draw_car_points(NoneSolver__,
			        FaceAngleSolver__, Coordinate__, last_true_state, trans_processor.target_kind != 1, car_yaw, car_pitch);
                for(int i = 0;i < 4;i++){
                    for(int j = 0;j < 4;j++){
                        debug_message.true_car.armors[i].middle_four_points[j].x = true_car_points[i][j].x;
                        debug_message.true_car.armors[i].middle_four_points[j].y = true_car_points[i][j].y;
                    };
                };
                cv::Point2f true_car_middle2f = Coordinate__->abs2pixel(cv::Point3f(last_true_state(0),last_true_state(1),last_true_state(2)), car_yaw, car_pitch);
                debug_message.true_car.car_middle.x = true_car_middle2f.x;
                debug_message.true_car.car_middle.y = true_car_middle2f.y;

                // pred_car
                auto pred_car_points = Processor__->get_draw_car_points(NoneSolver__,
			        FaceAngleSolver__, Coordinate__, last_pred_state, trans_processor.target_kind != 1, car_yaw, car_pitch);
                for(int i = 0;i < 4;i++){
                    for(int j = 0;j < 4;j++){
                        debug_message.pred_car.armors[i].middle_four_points[j].x = pred_car_points[i][j].x;
                        debug_message.pred_car.armors[i].middle_four_points[j].y = pred_car_points[i][j].y;
                    };
                };
                cv::Point2f pred_car_middle2f = Coordinate__->abs2pixel(
                    cv::Point3f(last_pred_state(0),last_pred_state(1),last_pred_state(2)),
                    car_yaw, car_pitch);
                debug_message.pred_car.car_middle.x = pred_car_middle2f.x;
                debug_message.pred_car.car_middle.y = pred_car_middle2f.y;

                // 
                debug_message.pitch = message.pitch;
                debug_message.yaw = message.yaw;
                debug_message.yaw_speed = message.yaw_speed;
                debug_message.enemy_rpm = last_pred_state(7) * 30.0 / CV_PI;
                debug_message.enemy_vx = last_pred_state(4);
                debug_message.enemy_vz = last_pred_state(6);
                switch (trans_processor.target_kind)
                {
                case 0:
                    debug_message.target_kind = "enemy";
                    break;
                case 1:
                    debug_message.target_kind = "hero";
                    break;
                case 2:
                    debug_message.target_kind = "sentinel";
                    break;
                case 3:
                    debug_message.target_kind = "three";
                    break;
                case 4:
                    debug_message.target_kind = "four";
                    break;
                case 5:
                    debug_message.target_kind = "five";
                    break;
                default:
                    break;
                };

                // shoot_strategy
                std::string shoot_strategy;
		        auto controler_state = Controler__->get_controler_state();
                switch (controler_state)
                {
                case STATE::shoot_default:
                    shoot_strategy = "defult";
                    break;
                case STATE::shoot_track:
                    shoot_strategy = "shoot_track";
                    break;
                case STATE::shoot_middle:
                    shoot_strategy = "shoot_middle";
                    break;
                case STATE::shoot_precise:
                    shoot_strategy = "shoot_precise";
                    break;
                case STATE::shoot_translation:
                    shoot_strategy = "shoot_translation";
                    break;
                case STATE::shoot_vague:
                    shoot_strategy = "shoot_vague";
                    break;
                case STATE::shoot_hero:
                    shoot_strategy = "shoot_hero";
                    break;
                default:
                    break;
                };
                debug_message.shoot_strategy = shoot_strategy;

                // face_angles
                for (const auto& armor_pos : trans_armor_poses) {
                    DebugFaceAngle debug_face_angle;
					float angle = FaceAngleSolver__->switch_absolute_camera_angle(armor_pos.face_angle, car_yaw) * 180 / CV_PI;
                    debug_face_angle.cam_face_angle = angle;
                    debug_face_angle.point.x = armor_pos.pixel_four_point[2].x;
                    debug_face_angle.point.y = armor_pos.pixel_four_point[2].y;
                    debug_message.face_angles.push_back(debug_face_angle);
                };

                // aim_direction
                cv::Point3f shoot_point(0,0,100);
                shoot_point = Coordinate__->sot2abs(shoot_point,car_yaw,car_pitch);
                cv::Point2f shoot_point2f = Coordinate__->abs2pixel(shoot_point,car_yaw,car_pitch);
                debug_message.aim_direction.x = shoot_point2f.x;
                debug_message.aim_direction.y = shoot_point2f.y;
                
                // aim_target
                cv::Point2f target = Coordinate__->abs2pixel(Controler__->debug_absolute_aim_target, car_yaw, car_pitch);
                debug_message.aim_target.x = target.x;
                debug_message.aim_target.y = target.y;

                // canshoot
                debug_message.canshoot = message.shoot_time != -1;

                debug_message.empty = false;
            }
            else{
                debug_message.empty = true;
            };
            debug_processor_publisher_->publish(debug_message);
        };
    };

private: 
    bool debug;
private:
    Coordinate* Coordinate__;
    FaceAngleSolver* FaceAngleSolver__;
    ArmorSolver* ArmorSolver__;
    TargetPlanner* TargetPlanner__;
    CarStateBase* CarState__;
    TranslationStateBase* TranslationState__;
    NoneSolver* NoneSolver__;
    SingleSolver* SingleSolver__;
    AngleOptimizer* AngleOptimizer__;
    DoubleSolver* DoubleSolver__;
    std::unique_ptr<ProcessorBase> Processor__;
    PitchCompensator* PitchCompensator__;
	RangeSetter* RangeSetter__;
	TimePredictor* TimePredictor__;
	ShootTrackStrategyBase* ShootTrackStrategy__;
	ShootMiddleStrategy* ShootMiddleStrategy__;
    HeroShootStrategy* HeroShootStrategy__;
    PrecisionShootStrategy* PrecisionShootStrategy__;
	ShootTranslationStrategy* ShootTranslationStrategy__;
	ShootMiddleVagueStrategy* ShootMiddleVagueStrategy__;
    StrategyPlanner* StrategyPlanner__;
	std::unique_ptr<Controler> Controler__;
public:
    ~ProcessorNode()
    {
        if(Coordinate__ != NULL) delete Coordinate__;
        if(FaceAngleSolver__ != NULL) delete FaceAngleSolver__;
        if(ArmorSolver__ != NULL) delete ArmorSolver__;
        if(TargetPlanner__ != NULL) delete TargetPlanner__;
        if(CarState__ != NULL) delete CarState__;
        if(TranslationState__ != NULL) delete TranslationState__;
        if(NoneSolver__ != NULL) delete NoneSolver__;
        if(SingleSolver__ != NULL) delete SingleSolver__;
        if(AngleOptimizer__ != NULL) delete AngleOptimizer__;
        if(DoubleSolver__ != NULL) delete DoubleSolver__;
        if(PitchCompensator__ != NULL) delete PitchCompensator__;
		if(RangeSetter__ != NULL) delete RangeSetter__;
		if(TimePredictor__ != NULL) delete TimePredictor__;
		if(ShootTrackStrategy__ != NULL) delete ShootTrackStrategy__;
		if(ShootMiddleStrategy__ != NULL) delete ShootMiddleStrategy__;
        if(PrecisionShootStrategy__ != NULL) delete PrecisionShootStrategy__;
		if(ShootTranslationStrategy__ != NULL) delete ShootTranslationStrategy__;
		if(ShootMiddleVagueStrategy__ != NULL) delete ShootMiddleVagueStrategy__;
        if(StrategyPlanner__ != NULL) delete StrategyPlanner__;
        if(HeroShootStrategy__ != NULL) delete HeroShootStrategy__;
    };
private: // 缓冲变量
    rclcpp::Time last_timer;
private:
    rclcpp::Subscription<auto_aim_interfaces::msg::TransDetector>::SharedPtr trans_detector_sub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::SerialSendData>::SharedPtr send_data_publisher_;
    rclcpp::Publisher<auto_aim_interfaces::msg::DebugProcessor>::SharedPtr debug_processor_publisher_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ProcessorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}