#ifndef _DEBUGGER_H_
#define _DEBUGGER_H_

#include "rclcpp/rclcpp.hpp"
#include "auto_aim_interfaces/msg/debug_detector.hpp"     
#include "auto_aim_interfaces/msg/debug_processor.hpp"
#include "../Color/Color.h"
#include "../Tools/Tools.h"
#include "../ImageShower/ImageShower.h"
#include "../VideoDownloader/VideoDownloader.h"
#include "std_msgs/msg/header.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
using Point2f = auto_aim_interfaces::msg::Point2f;
using SendArmor = auto_aim_interfaces::msg::SendArmor;
using DebugArmor = auto_aim_interfaces::msg::DebugArmor;
using DebugCar = auto_aim_interfaces::msg::DebugCar;
using DebugFaceAngle = auto_aim_interfaces::msg::DebugFaceAngle;
namespace rm
{

    /*
    * 画DebugArmor
    */
    void draw_point2f(cv::Mat& src,const Point2f& point2f,const cv::Scalar& color,int radius = 20,int thickness = 5);

    /*
    * 画DebugArmor
    */
    void draw_debug_armor(cv::Mat& src,const DebugArmor& debug_armor,const cv::Scalar& color,int thickness = 5);

    /*
    * 画DebugCar
    */
    void draw_debug_car(cv::Mat& src,const DebugCar& debug_car,const cv::Scalar& color,
        int radius = 30,int thickness = 5);

    /*
    * 画DebugText
    */
    class TextDrawer
    {
    public:
        TextDrawer(
            cv::Point start_point = cv::Point(0,50),cv::Point displacement = cv::Point(0,50),
            const cv::Scalar& color = Green,int thickness = 3);
        void draw(cv::Mat& src,const std::string& test,cv::Point pos = cv::Point(0,0));
        void draw(cv::Mat& src,const std::string& name,int data,cv::Point pos = cv::Point(0,0));
        void draw(cv::Mat& src,const std::string& name,float data,cv::Point pos = cv::Point(0,0));
        void draw(cv::Mat& src,const std::string& name,const std::string& data,cv::Point pos = cv::Point(0,0));
        void clear();
    private:
        cv::Point get_draw_pos(cv::Point pos);
        cv::Scalar color;
        int thickness;
        int size; // 存放test数目
        cv::Point start_point; // 起始点
        cv::Point displacement; // 默认位移
    };
    
    /*
    * 画DebugFaceAngle
    */
    void draw_debug_face_angle(cv::Mat& src,const DebugFaceAngle& debug_face_angle,const cv::Scalar& color = White,
        int thickness = 3);
    
    /*
    * 画SendArmor
    * 不同在于SendArmor标记出类型
    */
    void draw_send_armor(cv::Mat& src,const SendArmor& send_armor, const cv::Scalar& color = White,int thickness = 5);

    /*
    * 画开火
    */
    void draw_shoot(cv::Mat& src,const Point2f& aim_direction, int thickness = 7);
};

#endif