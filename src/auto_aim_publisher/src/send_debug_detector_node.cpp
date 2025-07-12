#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include <opencv2/opencv.hpp>
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include "auto_aim_interfaces/msg/debug_detector.hpp" // 包含自定义消息头文件
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
        : Node("send_debug_detector_node") {
        // 创建发布者，指定话题名称和QoS配置
        publisher_ = this->create_publisher<auto_aim_interfaces::msg::DebugDetector>("debug_detector_topic", 10);
        
        std::string path = "/home/rui/Projects/ruirui_love_v9/src/auto_aim_publisher/1.avi";
        video = new cv::VideoCapture(path);

        // 创建定时器，每秒发布一次消息
        timer_ = this->create_wall_timer(
            100ms, std::bind(&AutoAimPublisher::timer_callback, this));
    };

    ~AutoAimPublisher()
    {
        if( video!=NULL ) delete video;
    }

private:
    void timer_callback() {
        auto message = auto_aim_interfaces::msg::DebugDetector();

        // 填充消息数据
        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = "send_debug_detector_node";

        cv::Mat src; //= cv::imread("/home/rui/Projects/auto_aim/src/auto_aim_publisher/1.png");
        (*video) >> src;
        if(src.empty()){
            video->release();
            RCLCPP_INFO(this->get_logger(),"Video Over");
            return;
        };

        // cv::resize(src,src,cv::Size(1440,1080));
        double a = get_now_time();
        cv::resize(src,src,cv::Size(src.cols / 3 ,src.rows / 3));
        double s = get_now_time();
        RCLCPP_INFO(this->get_logger(), "resize: '%f'", 
                    s - a);
        cv_bridge::CvImage cv_image;
        cv_image.header = header;
        cv_image.encoding = "bgr8"; // mono8 rgb8
        cv_image.image = src;

        sensor_msgs::msg::Image image_msg;
        cv_image.toImageMsg(image_msg);
        message.image = image_msg;

        // 发布消息
        publisher_->publish(message);
        double e = get_now_time();
        RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", 
                    e - s);
    }

    rclcpp::Publisher<auto_aim_interfaces::msg::DebugDetector>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture* video;

};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutoAimPublisher>());
    rclcpp::shutdown();
    return 0;
}