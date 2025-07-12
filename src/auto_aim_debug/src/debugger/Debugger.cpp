#include"Debugger.h"
namespace rm
{
    void draw_point2f(cv::Mat& src,const Point2f& point2f,const cv::Scalar& color,int radius,int thickness)
    {
        cv::circle(src, cv::Point2f(point2f.x,point2f.y), radius, color, thickness);
    };

    void draw_debug_armor(cv::Mat& src,const DebugArmor& debug_armor,const cv::Scalar& color,int thickness)
    {
        for(int i = 0;i < 4;i++)
            cv::line(src, cv::Point2f(debug_armor.middle_four_points[i].x, debug_armor.middle_four_points[i].y),
                cv::Point2f(debug_armor.middle_four_points[(i + 1) % 4].x, debug_armor.middle_four_points[(i + 1) % 4].y)
                , color, thickness);
    };

    void draw_debug_car(cv::Mat& src,const DebugCar& debug_car,const cv::Scalar& color,int radius, int thickness)
    {
        for(int i = 0;i < 4;i++){
            auto debug_armor = debug_car.armors[i];
            for(int j = 0;j < 4;j++){
                cv::line(src, cv::Point2f(debug_armor.middle_four_points[j].x, debug_armor.middle_four_points[j].y),
                    cv::Point2f(debug_armor.middle_four_points[(j + 1) % 4].x, debug_armor.middle_four_points[(j + 1) % 4].y)
                    , color, thickness);
            };
        };
        cv::circle(src, cv::Point2f(debug_car.car_middle.x,debug_car.car_middle.y), radius, color, thickness);
    };

    TextDrawer::TextDrawer(cv::Point start_point,cv::Point displacement,const cv::Scalar& color,int thickness)
        :start_point(start_point),displacement(displacement),color(color),thickness(thickness)
    {
        clear();
    };

    void TextDrawer::draw(cv::Mat& src,const std::string& test,cv::Point pos)
    {
        cv::putText(src, test, get_draw_pos(pos), 3, 1, color, thickness);
    };

    void TextDrawer::draw(cv::Mat& src,const std::string& name,int data,cv::Point pos)
    {
        std::string test = name + ": " + std::to_string(data);
        draw(src,test,pos);
    };

    void TextDrawer::draw(cv::Mat& src,const std::string& name,float data,cv::Point pos)
    {
        data = int(data * 100) / 100.0f;
        std::string test = name + ": " + std::to_string(data);
        draw(src,test,pos);
    }

    void TextDrawer::draw(cv::Mat& src,const std::string& name,const std::string& data,cv::Point pos)
    {
        std::string test = name + ": " + data;
        draw(src,test,pos);
    };

    cv::Point TextDrawer::get_draw_pos(cv::Point pos){
        cv::Point point_;
        if(pos == cv::Point(0,0)){
            point_ = start_point + size * displacement;
            size++;
            return point_;
        }
        else{
            return pos;
        };
    };

    void TextDrawer::clear(){
        size = 0;
    };

    void draw_debug_face_angle(cv::Mat& src,const DebugFaceAngle& debug_face_angle,const cv::Scalar& color,
        int thickness)
    {
        float data = int(debug_face_angle.cam_face_angle * 100) / 100.0f;
        cv::putText(src, std::to_string(data), cv::Point2f(debug_face_angle.point.x,debug_face_angle.point.y)
            , 2, 1, color, thickness);
    };

    void draw_send_armor(cv::Mat& src,const SendArmor& send_armor, const cv::Scalar& color ,int thickness)
    {
        
        cv::line(src, cv::Point2f(send_armor.middle_four_points[0].x, send_armor.middle_four_points[0].y),
            cv::Point2f(send_armor.middle_four_points[1].x, send_armor.middle_four_points[1].y)
            , color, thickness);
        cv::line(src, cv::Point2f(send_armor.middle_four_points[1].x, send_armor.middle_four_points[1].y),
            cv::Point2f(send_armor.middle_four_points[3].x, send_armor.middle_four_points[3].y)
            , color, thickness);
        cv::line(src, cv::Point2f(send_armor.middle_four_points[3].x, send_armor.middle_four_points[3].y),
            cv::Point2f(send_armor.middle_four_points[2].x, send_armor.middle_four_points[2].y)
            , color, thickness);
        cv::line(src, cv::Point2f(send_armor.middle_four_points[2].x, send_armor.middle_four_points[2].y),
            cv::Point2f(send_armor.middle_four_points[0].x, send_armor.middle_four_points[0].y)
            , color, thickness);
        int label = send_armor.label;
        if(send_armor.light_loss){
            cv::putText(src, "ID: error", cv::Point2f(send_armor.middle_four_points[2].x, send_armor.middle_four_points[2].y)
            , 2, 1, color, thickness);
        }
        else{
            cv::putText(src, "ID: " + std::to_string(label), cv::Point2f(send_armor.middle_four_points[2].x, send_armor.middle_four_points[2].y)
            , 2, 1, color, thickness);
        };

    };

    void draw_shoot(cv::Mat& src,const Point2f& aim_direction, int thickness)
    {
        // 画叉!
        cv::Point2f target(aim_direction.x,aim_direction.y);
        cv::line(src, cv::Point2f(target.x - 10, target.y + 10),
            cv::Point2f(target.x - 50, target.y + 50), Red, thickness);
        cv::line(src, cv::Point2f(target.x - 10, target.y - 10),
            cv::Point2f(target.x - 50, target.y - 50), Red, thickness);
        cv::line(src, cv::Point2f(target.x + 10, target.y - 10),
            cv::Point2f(target.x + 50, target.y - 50), Red, thickness);
        cv::line(src, cv::Point2f(target.x + 10, target.y + 10),
            cv::Point2f(target.x + 50, target.y + 50), Red, thickness);
    };
};