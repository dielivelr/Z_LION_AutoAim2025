#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "auto_aim_interfaces/msg/debug_processor.hpp" // 包含自定义消息头文件

using namespace std::chrono_literals;


class AutoAimPublisher : public rclcpp::Node {
public:
    using Point2f = auto_aim_interfaces::msg::Point2f;
    using DebugArmor = auto_aim_interfaces::msg::DebugArmor;
    using DebugCar = auto_aim_interfaces::msg::DebugCar;
    using DebugFaceAngle = auto_aim_interfaces::msg::DebugFaceAngle;

    AutoAimPublisher()
        : Node("send_debug_processor_node") {

        publisher_ = this->create_publisher<auto_aim_interfaces::msg::DebugProcessor>("debug_processor_topic", 10);

        // 创建定时器，每秒发布一次消息
        timer_ = this->create_wall_timer(
            100ms, std::bind(&AutoAimPublisher::timer_callback, this));
    }

private:
    void timer_callback() {
        auto message = auto_aim_interfaces::msg::DebugProcessor();
        
        // DebugCar
        DebugCar true_car,pred_car;
        DebugArmor armor;
        Point2f point;
        point.x = 500;
        point.y = 500;
        for(int i = 0; i< 4 ;i++)
            armor.middle_four_points[i] = point;
        
        Point2f car_middle;
        car_middle.x = 400;
        car_middle.y = 400;
        for(int i = 0; i< 4 ;i++)
            true_car.armors[i] = armor;
        true_car.car_middle = car_middle;

        point.x = 300;
        point.y = 300;
        for(int i = 0; i< 4 ;i++)
            armor.middle_four_points[i] = point;
        car_middle.x = 200;
        car_middle.y = 200;
        for(int i = 0; i< 4 ;i++)
            pred_car.armors[i] = armor;
        pred_car.car_middle = car_middle;

        message.true_car = true_car;
        message.pred_car = pred_car;

        // Text
        message.pitch = -1.0;
        message.yaw = 1.0;
        message.yaw_speed = 43.2570402;
        message.enemy_rpm = 647457547.676;
        message.enemy_vx = 46456456;
        message.enemy_vz = 0.6575675675;
        message.target_kind = "Hero";
        message.shoot_strategy = "===OK===";
        // DebugFaceAngle
        DebugFaceAngle face_angle;
        point.x = 100;
        point.y = 100;
        face_angle.point = point;
        face_angle.cam_face_angle = 48;
        message.face_angles.push_back(face_angle);

        // Point2f
        Point2f aim_direction,aim_target;
        aim_direction.x = 540;
        aim_direction.y = 720;
        aim_target.x = 50;
        aim_target.y = 50;
        message.aim_direction = aim_direction;
        message.aim_target = aim_target;

        // 
        message.canshoot = true;

        // 发布消息
        RCLCPP_INFO(this->get_logger(), "Publishing");
        publisher_->publish(message);
    }

    rclcpp::Publisher<auto_aim_interfaces::msg::DebugProcessor>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutoAimPublisher>());
    rclcpp::shutdown();
    return 0;
}