#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "cv_bridge/cv_bridge.h"
#include "../detector/Detector.h"
#include "auto_aim_interfaces/msg/serial_read_data.hpp" 
#include "auto_aim_interfaces/msg/trans_detector.hpp"
#include "auto_aim_interfaces/msg/debug_detector.hpp" 

using namespace rm;

class DetectorNode : public rclcpp::Node {
public:
    DetectorNode()
        : Node("auto_aim_detector_node"), image_sub_(nullptr) {

        declare_parameter("debug",true);
        declare_parameter("set_enemy_color",1);
        declare_parameter("use_fusion_detector",false);
        declare_parameter("show_time_cost",true); 
        declare_parameter("Machine.pic_min_threshold",200.0); 
        declare_parameter("UsualLightFinder.max_ratio_w_h",1.1); 
        declare_parameter("UsualLightFinder.max_angle",35.0);
        declare_parameter("UsualLightFinder.SIMPLE",false);
        declare_parameter("MiddleLightFinder.ret_dis_error",5.0);
        declare_parameter("ArmorFinder.armor_min_height_ratio",0.6);
        declare_parameter("ArmorFinder.small_armor_min_aspect_ratio",0.8);
        declare_parameter("ArmorFinder.small_armor_max_aspect_ratio",3.2);
        declare_parameter("ArmorFinder.big_armor_min_aspect_ratio",3.2);
        declare_parameter("ArmorFinder.big_armor_max_aspect_ratio",5.0);
        declare_parameter("ArmorFinder.max_center_angle",15.0);
        declare_parameter("YoloModel.yolo_model_path","src/auto_aim/models/yolo.onnx");
        declare_parameter("YoloModel.yolo_model_image_size",256);
        declare_parameter("NumberFinder.number_model_path","src/auto_aim/models/fc2.onnx");
        declare_parameter("NumberFinder.number_min_confidence",0.5);
        declare_parameter("NumberFinder.num_img_roi_light_length",12);
        declare_parameter("Detector.img_width",1440);
        declare_parameter("Detector.img_height",1080);
        declare_parameter("FusionDetector._width_expand_",1.414);
        declare_parameter("FusionDetector._height_expand_",1.414);

        // 是否进入调试模式
        this->debug = get_parameter("debug").as_bool();
        // 是否使用融合识别
        this->use_fusion_detector = get_parameter("use_fusion_detector").as_bool();
        // 设置敌方颜色 0: 不设置 1: 敌方为蓝色 2: 敌方为红色
        this->set_enemy_color = get_parameter("set_enemy_color").as_int();

        this->show_time_cost = get_parameter("show_time_cost").as_bool();

               // Machine
        if(use_fusion_detector){
            double pic_min_threshold = get_parameter("Machine.pic_min_threshold").as_double();
            if(pic_min_threshold == 0){
                Machine__ = new AutoMachine(); 
            }
            else{
                Machine__ = new GreyMachine(pic_min_threshold);
            };
        }
        else{
            // 二值化参数
            double pic_min_threshold = get_parameter("Machine.pic_min_threshold").as_double();
	        Machine__ = new GreyMachine(pic_min_threshold);
        };

        // UsualLightFinder

        // 灯条高宽比阈值
        float max_ratio_w_h = get_parameter("UsualLightFinder.max_ratio_w_h").as_double();
        // 灯条最大角度
        float max_angle = get_parameter("UsualLightFinder.max_angle").as_double();
        // 是否启用简易提取外部轮廓
        bool SIMPLE = get_parameter("UsualLightFinder.SIMPLE").as_bool();
        UsualLightFinder__
		= new UsualLightFinder(max_ratio_w_h, max_angle, SIMPLE);
        
        // MiddleLightFinder

        if(use_fusion_detector)
        {
            // 灯条允许范围参数
            float ret_dis_error = get_parameter("MiddleLightFinder.ret_dis_error").as_double();
            MiddleLightFinder__
                = new MiddleLightFinder(UsualLightFinder__, ret_dis_error);
        };
        
        // ArmorFinder

        // 装甲板二维尺寸限制
        float armor_min_height_ratio = get_parameter("ArmorFinder.armor_min_height_ratio").as_double();
        float small_armor_min_aspect_ratio = get_parameter("ArmorFinder.small_armor_min_aspect_ratio").as_double();
        float small_armor_max_aspect_ratio = get_parameter("ArmorFinder.small_armor_max_aspect_ratio").as_double();
        float big_armor_min_aspect_ratio = get_parameter("ArmorFinder.big_armor_min_aspect_ratio").as_double();
        float big_armor_max_aspect_ratio = get_parameter("ArmorFinder.big_armor_max_aspect_ratio").as_double();
        float max_center_angle = get_parameter("ArmorFinder.max_center_angle").as_double();
        ArmorFinder__
            = new ArmorFinder(armor_min_height_ratio, small_armor_min_aspect_ratio,
                small_armor_max_aspect_ratio, big_armor_min_aspect_ratio,
                big_armor_max_aspect_ratio, max_center_angle);

        if(use_fusion_detector){
            // YoloModel

            // 模型路径
            std::string yolo_model_path = get_parameter("YoloModel.yolo_model_path").as_string();
            // 模型尺寸
            int yolo_model_image_size = get_parameter("YoloModel.yolo_model_image_size").as_int();
            YoloModel__ = new YoloModel(yolo_model_path, yolo_model_image_size);
        }
        else{
            // NumberFinder

            // 模型路径
            std::string number_model_path = get_parameter("NumberFinder.number_model_path").as_string();
            // 置信度
            double number_min_confidence = get_parameter("NumberFinder.number_min_confidence").as_double();
            // 灯条roi长度设置
            int num_img_roi_light_length = get_parameter("NumberFinder.num_img_roi_light_length").as_int();
            NumberFinder__
                = new NumberFinder(number_model_path, number_min_confidence, num_img_roi_light_length);
        };

        // Detector

        // 图像长宽
        int img_width = get_parameter("Detector.img_width").as_int();
        int img_height = get_parameter("Detector.img_height").as_int();
        if(use_fusion_detector){
            // 灯条roi长宽扩展参数
            float _width_expand_ = get_parameter("FusionDetector._width_expand_").as_double();
            float _height_expand_ = get_parameter("FusionDetector._height_expand_").as_double();
            Detector__
                = std::make_unique<FusionDetector>(Machine__,YoloModel__,ArmorFinder__
                    , MiddleLightFinder__, _width_expand_, _height_expand_, img_width, img_height);
        }
        else{
            Detector__
                = std::make_unique<TraditionDetector>(Machine__, UsualLightFinder__, ArmorFinder__,
                    NumberFinder__, img_width, img_height);
        };
        
        ret_serial_read_data = ret_image = false;

        read_data_sub_ = this->create_subscription<auto_aim_interfaces::msg::SerialReadData>(
            "serial_read_data_topic", 10,
            std::bind(&DetectorNode::serial_read_data_callback, this, std::placeholders::_1));

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_raw", 10,
            std::bind(&DetectorNode::image_callback, this, std::placeholders::_1));

        trans_detector_publisher_ = 
            this->create_publisher<auto_aim_interfaces::msg::TransDetector>("trans_detector_topic", 10);

        debug_detector_publisher_ = 
            this->create_publisher<auto_aim_interfaces::msg::DebugDetector>("debug_detector_topic", 10);

        RCLCPP_INFO(this->get_logger(), "DetectorNode callback is all right!!!");

    };

private: // 回调函数

    void call_back(const auto_aim_interfaces::msg::SerialReadData& serial_read_data_msg,
        const sensor_msgs::msg::Image& image_msg)
    {
        auto debug_record_start_time = this->now();

        bool enemy_blue;
        if(set_enemy_color != 0 && debug){
            enemy_blue = set_enemy_color == 1;
        }
        else
            enemy_blue = (serial_read_data_msg.enemy_color == 2);
        
        UsualLightFinder__->set_enemy_color(enemy_blue);
        if(use_fusion_detector) YoloModel__->set_enemy_color(enemy_blue);
       
        auto imageSharedPtr = std::make_shared<sensor_msgs::msg::Image>(image_msg);
        cv::Mat src = cv_bridge::toCvShare(imageSharedPtr, "bgr8")->image;

        std::vector<SendArmor> send_armors = Detector__->detect(src);

        auto message = this->trans_data_to_message(send_armors,image_msg.header);
        message.car_yaw = serial_read_data_msg.car_yaw;
        message.car_pitch = serial_read_data_msg.car_pitch;
        message.car_yaw_speed = serial_read_data_msg.car_yaw_speed;
        message.enemy_blue = enemy_blue;
        message.grade = serial_read_data_msg.grade;
        trans_detector_publisher_->publish(message);

        // debug
        if(debug){

            auto debug_record_end_time = this->now();

            if(show_time_cost)
                RCLCPP_INFO(this->get_logger(), "DetectorNode time cost: %f",
                    (debug_record_end_time - debug_record_start_time).seconds());

            auto_aim_interfaces::msg::DebugDetector debug_message;
            // 缩小
            cv::resize(src,src,cv::Size(src.cols / 4 ,src.rows / 4));
            cv_bridge::CvImage cv_image;
            cv_image.header = image_msg.header;
            cv_image.encoding = "bgr8"; // mono8 rgb8
            cv_image.image = src;
            sensor_msgs::msg::Image image_msg_;
            cv_image.toImageMsg(image_msg_);
            debug_message.image = image_msg_;
            debug_message.armors = message.sendarmors;
            debug_detector_publisher_->publish(debug_message);
        };
    };

    void serial_read_data_callback(
        const auto_aim_interfaces::msg::SerialReadData::SharedPtr serial_read_data_msg) 
    {
        this->serial_read_data_msg = *serial_read_data_msg;
        ret_serial_read_data = true;
        call_all();
    };

    void image_callback(const sensor_msgs::msg::Image::SharedPtr image_msg) 
    {
        this->image_msg = *image_msg;
        ret_image = true;
        call_all();
    };

    void call_all()
    {
        if(ret_serial_read_data && ret_image){
            ret_serial_read_data = false;
            ret_image = false;
            call_back(serial_read_data_msg,image_msg);
        };
    }
private:

    // 转换消息函数
    auto_aim_interfaces::msg::TransDetector trans_data_to_message(
        const std::vector<SendArmor>& send_armors,
        const std_msgs::msg::Header& header){
        auto message = auto_aim_interfaces::msg::TransDetector();
        message.header = header;
        for(const auto& send_armor : send_armors){
            auto msg_sendarmor = auto_aim_interfaces::msg::SendArmor();
            msg_sendarmor.label = send_armor.label;
            for(int i = 0;i < 4; i++ ){
                msg_sendarmor.middle_four_points[i].x = send_armor.middle_four_points[i].x;
                msg_sendarmor.middle_four_points[i].y = send_armor.middle_four_points[i].y;
            };
            msg_sendarmor.light_loss = send_armor.light_loss;
            message.sendarmors.push_back(msg_sendarmor);
        };
        return message;
    };

private: // 参数
    bool use_fusion_detector; // 是否采用融合算法
    bool debug;
    int set_enemy_color; // 设置敌方颜色
    bool show_time_cost;
private: // 组件
    Machine* Machine__;
    UsualLightFinder* UsualLightFinder__;
    MiddleLightFinder* MiddleLightFinder__;
    ArmorFinder* ArmorFinder__;
    YoloModel* YoloModel__;
    NumberFinder* NumberFinder__;
    std::unique_ptr<Detector> Detector__;
    
public:
    ~DetectorNode(){
        if(Machine__ != NULL) delete Machine__;
        if(UsualLightFinder__ != NULL) delete UsualLightFinder__;
        if(MiddleLightFinder__ != NULL) delete MiddleLightFinder__;
        if(ArmorFinder__ != NULL) delete ArmorFinder__;
        if(YoloModel__ != NULL) delete YoloModel__;
        if(NumberFinder__ != NULL) delete NumberFinder__;
    };
private:
    auto_aim_interfaces::msg::SerialReadData serial_read_data_msg;
    sensor_msgs::msg::Image image_msg;
    rclcpp::Subscription<auto_aim_interfaces::msg::SerialReadData>::SharedPtr read_data_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::TransDetector>::SharedPtr trans_detector_publisher_;
    rclcpp::Publisher<auto_aim_interfaces::msg::DebugDetector>::SharedPtr debug_detector_publisher_;
    bool ret_serial_read_data, ret_image; 
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
};