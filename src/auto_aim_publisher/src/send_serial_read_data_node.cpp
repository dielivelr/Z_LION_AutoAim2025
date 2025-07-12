#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "auto_aim_interfaces/msg/serial_read_data.hpp" // 包含自定义消息头文件
#include <iostream>
#include <string>
using namespace std::chrono_literals;

static double get_now_time()
{
    auto now = std::chrono::system_clock::now();
    auto durationSinceEpoch = now.time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(durationSinceEpoch);
    auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(durationSinceEpoch - seconds);
    double fractional_seconds = microseconds.count() * 1e-6;
    return seconds.count() + fractional_seconds;
};

class AutoAimPublisher : public rclcpp::Node {
public:

    AutoAimPublisher()
        : Node("send_serial_read_data_node") {
        // 创建发布者，指定话题名称和QoS配置
        publisher_ = this->create_publisher<auto_aim_interfaces::msg::SerialReadData>("serial_read_data_topic", 10);
        
        // 创建定时器，每秒发布一次消息
        timer_ = this->create_wall_timer(
            5ms, std::bind(&AutoAimPublisher::timer_callback, this));
    };

    ~AutoAimPublisher()
    {
    }

private:
    void timer_callback() {
        auto message = auto_aim_interfaces::msg::SerialReadData();
        message.enemy_color = 2;
        message.car_yaw = 0.542;
        message.car_pitch = -0.081;

        publisher_->publish(message);

        // RCLCPP_INFO(this->get_logger(), "Publishing");
    }

    rclcpp::Publisher<auto_aim_interfaces::msg::SerialReadData>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutoAimPublisher>());
    rclcpp::shutdown();
    return 0;
}