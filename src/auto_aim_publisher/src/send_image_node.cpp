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
        : Node("send_image_node") {
        // 创建发布者，指定话题名称和QoS配置
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);
        
        std::string path = "src/auto_aim_publisher/videos/sample.avi";
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
        double s = get_now_time();
        // 填充消息数据
        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = "image_raw";

        cv::Mat src;
        (*video) >> src;
        if(src.empty()){
            video->release();
            RCLCPP_INFO(this->get_logger(),"Video Over");
            return;
        };

        cv_bridge::CvImage cv_image;
        cv_image.header = header;
        cv_image.encoding = "bgr8"; // mono8 rgb8
        cv_image.image = src;

        sensor_msgs::msg::Image image_msg;
        cv_image.toImageMsg(image_msg);
        // 发布消息
        publisher_->publish(image_msg);
        double e = get_now_time();
        // RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", 
        //     e - s + 0.001);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture* video;

};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutoAimPublisher>());
    rclcpp::shutdown();
    return 0;
}