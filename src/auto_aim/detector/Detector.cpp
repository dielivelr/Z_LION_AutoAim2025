#include"Detector.h"

namespace rm
{
	Detector::Detector()
		:img_width(1440), img_height(1080)
	{
	};

	Detector::Detector(int img_width, int img_height)
		:img_width(img_width), img_height(img_height)
	{
	};


    TraditionDetector::TraditionDetector(Machine* Machine__, UsualLightFinder* UsualLightFinder__,
        ArmorFinder* ArmorFinder__, NumberFinder* NumberFinder__,
        int img_width, int img_height)
        :Machine__(Machine__), UsualLightFinder__(UsualLightFinder__), ArmorFinder__(ArmorFinder__),
        NumberFinder__(NumberFinder__), Detector(img_width, img_height)
    {
    }

    std::vector<SendArmor> TraditionDetector::detect(cv::Mat src)
    {
        std::vector<SendArmor> SendArmors;
        cv::Mat drt = Machine__->machine(src);
        auto lights = UsualLightFinder__->find_lights(drt, src);
        auto armors = ArmorFinder__->find_armors(lights);
        armors = NumberFinder__->forward(src, armors);
        return SendArmors;
    };

  
    FusionDetector::FusionDetector(Machine* Machine__, YoloModel* YoloModel__,
        ArmorFinder* ArmorFinder__, MiddleLightFinder* MiddleLightFinder__,
        float _width_expand_, float _height_expand_,
        int img_width, int img_height)
        :Machine__(Machine__), YoloModel__(YoloModel__), ArmorFinder__(ArmorFinder__), MiddleLightFinder__(MiddleLightFinder__),
         _width_expand_(_width_expand_), _height_expand_(_height_expand_),
        Detector(img_width, img_height)
    {
    }
    std::vector<SendArmor> FusionDetector::detect(cv::Mat src)
    {
        std::vector<SendArmor> sendarmors;
        std::vector<Armor> armors = YoloModel__->find_armors(src);
        for (int i = 0, I = armors.size(); i < I; i++) {
            SendArmor send_armor(armors[i]);

            cv::Rect L_roi = capture_light_roi(true, armors[i].middle_four_points,this->_width_expand_, this->_height_expand_,
                src.cols,src.rows);
            cv::Rect R_roi = capture_light_roi(false, armors[i].middle_four_points, this->_width_expand_, this->_height_expand_,
                src.cols, src.rows);
            cv::Mat L_src = src(L_roi); cv::Mat L_drt = Machine__->machine(L_src);
            cv::Mat R_src = src(R_roi); cv::Mat R_drt = Machine__->machine(R_src);
            // 提取灯条
            std::vector<Light> left_lights = MiddleLightFinder__->find_lights(L_drt, L_src);
            std::vector<Light> right_lights = MiddleLightFinder__->find_lights(R_drt, R_src);
            // 跳过无灯条检测到的情况
            bool ret_light_loss = true;
            if (!left_lights.empty() && !right_lights.empty())
            {
                // 一旦出现灯条未在中心的情况
                if (MiddleLightFinder__->light_center_in_rect_middle(left_lights[0], L_roi) &&
                    MiddleLightFinder__->light_center_in_rect_middle(right_lights[0], R_roi)) {
                    // 位置还原
                    Light left_light = rm::Light(cv::RotatedRect(left_lights[0].center + cv::Point2f(L_roi.x, L_roi.y), left_lights[0].size
                        , left_lights[0].angle));
                    //left_light.strip_least_squares_top = left_lights[0].strip_least_squares_top + cv::Point2f(L_roi.x, L_roi.y);
                    //left_light.strip_least_squares_bottom = left_lights[0].strip_least_squares_bottom + cv::Point2f(L_roi.x, L_roi.y);

                    Light right_light = rm::Light(cv::RotatedRect(right_lights[0].center + cv::Point2f(R_roi.x, R_roi.y), right_lights[0].size
                        , right_lights[0].angle));
                    //right_light.strip_least_squares_top = right_lights[0].strip_least_squares_top + cv::Point2f(R_roi.x, R_roi.y);
                    //right_light.strip_least_squares_bottom = right_lights[0].strip_least_squares_bottom + cv::Point2f(R_roi.x, R_roi.y);

                    //left_light.strip_least_squares_top = left_light.top;
                    //left_light.strip_least_squares_bottom = left_light.bottom;
                    //right_light.strip_least_squares_top = right_light.top;
                    //right_light.strip_least_squares_bottom = right_light.bottom;

                    // 匹配筛选
                    std::vector<rm::Armor> combination_armor = ArmorFinder__->find_armors({ left_light , right_light });
                    if (!combination_armor.empty()) {
                        ret_light_loss = false;
                        combination_armor[0].label = armors[i].label;
                        combination_armor[0].armor_small = armors[i].label != 1;
                        send_armor = SendArmor(combination_armor[0]);
                    };
                };
            };
            // 对未通过的装甲板赋值
            if (ret_light_loss) {
                send_armor.light_loss = true;
            };
            sendarmors.push_back(send_armor);
        };

        return sendarmors;
    }
    cv::Rect FusionDetector::capture_light_roi(bool get_left_light,const std::vector<cv::Point2f>& middle_four_points, 
        float width_expand, float height_expand, int image_width, int image_height)
    {
        cv::Point point; int length;
        if(get_left_light)
        {
            point = (middle_four_points[0] + middle_four_points[1]) / 2;
            length = cv::norm(middle_four_points[0] - middle_four_points[1]);
        }
        else {
            point = (middle_four_points[2] + middle_four_points[3]) / 2;
            length = cv::norm(middle_four_points[2] - middle_four_points[3]);
        }
        return MiddleLightFinder__->rect_edge_detection(point, length * width_expand, length * height_expand,
            image_width, image_height);
    }
    ;

}