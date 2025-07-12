#include "../src/debugger/Debugger.h"

using namespace rm;

class DebugNode : public rclcpp::Node {
public:
    
    DebugNode()
        : Node("auto_aim_debug_node") {
        last_time = get_now_time();
        // set
        show = this->declare_parameter("show", false);
        show_time_cost = this->declare_parameter("show_time_cost", true);
        show_true_car = this->declare_parameter("show_true_car", true);
        show_pred_car = this->declare_parameter("show_pred_car", true);
        show_pitch = this->declare_parameter("show_pitch", true);
        show_yaw = this->declare_parameter("show_yaw", true);
        show_yaw_speed = this->declare_parameter("show_yaw_speed", true);
        show_enemy_rpm = this->declare_parameter("show_enemy_rpm", true);
        show_enemy_vx = this->declare_parameter("show_enemy_vx", true);
        show_enemy_vz = this->declare_parameter("show_enemy_vz", true);
        show_target_kind = this->declare_parameter("show_target_kind", true);
        show_shoot_strategy = this->declare_parameter("show_shoot_strategy", true);
        show_face_angles = this->declare_parameter("show_face_angles", true);
        show_aim_direction = this->declare_parameter("show_aim_direction", true);
        show_aim_target = this->declare_parameter("show_aim_target", true);
        show_canshoot = this->declare_parameter("show_canshoot", true);
        show_send_armors = this->declare_parameter("show_send_armors", true);

        // show_detect
        bool show_detect = this->declare_parameter("show_detect", false);
        if(show_detect){
            show_true_car = false;
            show_pred_car = false;
            show_pitch = false;
            show_yaw = false;
            show_yaw_speed = false;
            show_enemy_rpm = false;
            show_enemy_vx = false;
            show_enemy_vz = false;
            show_target_kind = false;
            show_shoot_strategy = false;
            show_face_angles = false;
            show_aim_direction = false;
            show_aim_target = false;
            show_canshoot = false;
            show_send_armors = true;
        };

        // show_process
        bool show_process = this->declare_parameter("show_process", false);
        if(show_process){
            show_true_car = true;
            show_pred_car = true;
            show_pitch = true;
            show_yaw = true;
            show_yaw_speed = true;
            show_enemy_rpm = true;
            show_enemy_vx = true;
            show_enemy_vz = true;
            show_target_kind = true;
            show_shoot_strategy = true;
            show_face_angles = true;
            show_aim_direction = true;
            show_aim_target = true;
            show_canshoot = true;
            show_send_armors = false;
        };

        // TextDrawer__
        TextDrawer__ = std::make_unique<TextDrawer>();

        // VideoDownLoader

        std::string load_video_root_path = 
            this->declare_parameter("load_video_root_path", "src/auto_aim_debug/debug_videos");
        float recording_time = this->declare_parameter("recording_time", -1.0);
        debug_image_width = this->declare_parameter("image_width", 1440);
        debug_image_height = this->declare_parameter("image_height", 1080);
		int fps = this->declare_parameter("fps", 30);
        bool avi = this->declare_parameter("avi", false);

        // 控制源图像更新
        bool open_origin_image_loader = this->declare_parameter("open_origin_image_loader", false);
        if(open_origin_image_loader)
            origin_image_loader = std::make_unique<VideoDownloader>(recording_time,
                load_video_root_path + "/origin",fps,debug_image_width,debug_image_height,avi);
        
        bool open_show_image_loader = this->declare_parameter("open_show_image_loader", true);
         // 控制展示图像更新
        if(open_show_image_loader)
            show_image_loader = std::make_unique<VideoDownloader>(recording_time,
                load_video_root_path + "/show",fps,debug_image_width,debug_image_height,avi);
        

        debug_detector_sub_ = this->create_subscription<auto_aim_interfaces::msg::DebugDetector>(
            "debug_detector_topic", 10,
            std::bind(&DebugNode::debug_detector_callback, this, std::placeholders::_1));
        
        debug_processor_sub_ = this->create_subscription<auto_aim_interfaces::msg::DebugProcessor>(
            "debug_processor_topic", 10,
            std::bind(&DebugNode::debug_processor_callback, this, std::placeholders::_1));

        debug_detector_ret = debug_processor_ret = false;
    };

private:
    // 总回调
    void callback(const auto_aim_interfaces::msg::DebugDetector& detector_msg,
                  const auto_aim_interfaces::msg::DebugProcessor& processor_msg) {

        auto imageSharedPtr = std::make_shared<sensor_msgs::msg::Image>(detector_msg.image);
        cv::Mat src;
        try
        {
            src = cv_bridge::toCvShare(imageSharedPtr, "bgr8")->image;
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        };
        cv::resize(src,src,cv::Size(debug_image_width,debug_image_height));
        if(origin_image_loader != NULL) origin_image_loader->load_in_video(src);

        if(!processor_msg.empty) // 
        {

            if(show_send_armors)
                for(const auto& x : detector_msg.armors){
                    draw_send_armor(src,x);
                };

            if(show_true_car)
                draw_debug_car(src,processor_msg.true_car,Green);

            if(show_pred_car)
                draw_debug_car(src,processor_msg.pred_car,Red);

            if(show_face_angles)
                for(const auto& x : processor_msg.face_angles){
                    draw_debug_face_angle(src,x);
                };

            if(show_pitch)
                TextDrawer__->draw(src,"pitch",processor_msg.pitch);

            if(show_yaw)
                TextDrawer__->draw(src,"yaw",processor_msg.yaw);
                
            if(show_yaw_speed)
                TextDrawer__->draw(src,"yaw_speed",processor_msg.yaw_speed);

            if(show_enemy_rpm)
                TextDrawer__->draw(src,"RPM",processor_msg.enemy_rpm);

            if(show_enemy_vx)
                TextDrawer__->draw(src,"vx",processor_msg.enemy_vx);

            if(show_enemy_vz)
                TextDrawer__->draw(src,"vz",processor_msg.enemy_vz);

            if(show_target_kind)
                TextDrawer__->draw(src,"target_kind",processor_msg.target_kind);

            if(show_shoot_strategy)
                TextDrawer__->draw(src,processor_msg.shoot_strategy);


            
            if(show_aim_direction)
                draw_point2f(src,processor_msg.aim_direction,Orange);
                
            if(show_aim_target)
                draw_point2f(src,processor_msg.aim_target,Purple);

            if(processor_msg.canshoot && show_canshoot)
                draw_shoot(src,processor_msg.aim_direction);

            
        };
        if(show){
            cv::imshow("DebugNode",src);
            cv::waitKey(1);
        };
        if(show_image_loader != NULL) show_image_loader->load_in_video(src);
        // 每次循环清空
        TextDrawer__->clear();
        if(show_time_cost)
        {
            double now_time = get_now_time();
            RCLCPP_INFO(this->get_logger(), "Time Cost: '%f'", 
                    now_time - last_time);
            last_time = now_time;
        };
    };

    void debug_detector_callback(const auto_aim_interfaces::msg::DebugDetector::SharedPtr detector_msg)
    {
        this->debug_detector_msg = *detector_msg;
        debug_detector_ret = true;
        call_all();
        
    };

    void debug_processor_callback(const auto_aim_interfaces::msg::DebugProcessor::SharedPtr processor_msg)
    {
        this->debug_processor_msg = *processor_msg;
        debug_processor_ret = true;
        call_all();
    };

    void call_all()
    {
        if(debug_detector_ret && debug_processor_ret){
            debug_detector_ret = false;
            debug_processor_ret = false;
            callback(debug_detector_msg,debug_processor_msg);

        };
    };
private:
    double last_time;

    // 源图像下载器
    std::unique_ptr<VideoDownloader> origin_image_loader;
    // 展示图像下载器
    std::unique_ptr<VideoDownloader> show_image_loader;

    std::unique_ptr<TextDrawer> TextDrawer__;
private:
    int debug_image_width;
    int debug_image_height;
    bool show; // 是否显示图像
    bool show_time_cost;
    bool show_true_car;
    bool show_pred_car;
    bool show_pitch;
    bool show_yaw;
    bool show_yaw_speed;
    bool show_enemy_rpm;
    bool show_enemy_vx;
    bool show_enemy_vz;
    bool show_target_kind;
    bool show_shoot_strategy;
    bool show_face_angles;
    bool show_aim_direction;
    bool show_aim_target;
    bool show_canshoot;
    bool show_send_armors;
private:
    rclcpp::Subscription<auto_aim_interfaces::msg::DebugDetector>::SharedPtr debug_detector_sub_;
    rclcpp::Subscription<auto_aim_interfaces::msg::DebugProcessor>::SharedPtr debug_processor_sub_;
    auto_aim_interfaces::msg::DebugDetector debug_detector_msg;
    auto_aim_interfaces::msg::DebugProcessor debug_processor_msg;
    bool debug_detector_ret,debug_processor_ret;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DebugNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}