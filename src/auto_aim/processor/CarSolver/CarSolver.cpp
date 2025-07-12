#include "CarSolver.h"

namespace rm
{
    CarSolver::CarSolver(CarStateBase* CarState__,
        TranslationStateBase* TranslationState__)
        :CarState__(CarState__), TranslationState__(TranslationState__)
    {
    }

    std::vector<cv::Point3f> CarSolver::fit_four_armors(cv::Point3f car_pos, float car_face_angle,
        float high_radius, float low_radius, float high_low_height_differ, bool high_armor_state)
    {
        float now_armor_radius, next_armor_radius;
        if (high_armor_state) {
            now_armor_radius = high_radius;
            next_armor_radius = low_radius;
        }
        else {
            now_armor_radius = low_radius;
            next_armor_radius = high_radius;
        };

        // 转化为真实角求出向量
        cv::Point2f car_pos2f(car_pos.x, car_pos.z);
        float angle_next = car_face_angle;
        float angle_now = CV_PI / 2 + angle_next;
        cv::Point2f _vector_now(now_armor_radius * cos(angle_now), now_armor_radius * sin(angle_now));
        cv::Point2f _vector_next(next_armor_radius * cos(angle_next), next_armor_radius * sin(angle_next));

        // 装甲板排列,未按照顺序
        std::vector<cv::Point2f> armor_poses2d = {
            car_pos2f - _vector_now, car_pos2f + _vector_next,car_pos2f + _vector_now, car_pos2f - _vector_next
        };

        // 转化为三维点
        std::vector<cv::Point3f> armor_poses(4);
        for (int i = 0; i < 4; i++) {
            if (high_armor_state) {
                armor_poses[i] = i % 2 == 0 ? cv::Point3f(armor_poses2d[i].x, car_pos.y, armor_poses2d[i].y)
                    : cv::Point3f(armor_poses2d[i].x, car_pos.y - high_low_height_differ, armor_poses2d[i].y);
            }
            else {
                armor_poses[i] = i % 2 == 0 ? cv::Point3f(armor_poses2d[i].x, car_pos.y - high_low_height_differ, armor_poses2d[i].y)
                    : cv::Point3f(armor_poses2d[i].x, car_pos.y, armor_poses2d[i].y);
            };
        };

        return armor_poses;
    }

    std::vector<cv::Point3f> CarSolver::fit_four_armors(cv::Point3f car_pos, float car_face_angle)
    {
        // 获取车当前半径与高度差数据
        bool high_armor_state = CarState__->get_high_armor_state();
        float high_radius, low_radius;
        CarState__->get_high_low_radius(high_radius, low_radius);
        float high_low_height_differ = CarState__->get_high_low_height_differ();
        
        float now_armor_radius, next_armor_radius;
        if (high_armor_state) {
            now_armor_radius = high_radius;
            next_armor_radius = low_radius;
        }
        else {
            now_armor_radius = low_radius;
            next_armor_radius = high_radius;
        };

        // 转化为真实角求出向量
        cv::Point2f car_pos2f(car_pos.x, car_pos.z);
        float angle_next = car_face_angle;
        float angle_now = CV_PI / 2 + angle_next;
        cv::Point2f _vector_now(now_armor_radius * cos(angle_now), now_armor_radius * sin(angle_now));
        cv::Point2f _vector_next(next_armor_radius * cos(angle_next), next_armor_radius * sin(angle_next));

        // 装甲板排列,未按照顺序
        std::vector<cv::Point2f> armor_poses2d = {
            car_pos2f - _vector_now, car_pos2f + _vector_next,car_pos2f + _vector_now, car_pos2f - _vector_next
        };

        // 转化为三维点
        std::vector<cv::Point3f> armor_poses(4);
        for (int i = 0; i < 4; i++) {
            if (high_armor_state) {
                armor_poses[i] = i % 2 == 0 ? cv::Point3f(armor_poses2d[i].x, car_pos.y, armor_poses2d[i].y)
                    : cv::Point3f(armor_poses2d[i].x, car_pos.y - high_low_height_differ, armor_poses2d[i].y);
            }
            else {
                armor_poses[i] = i % 2 == 0 ? cv::Point3f(armor_poses2d[i].x, car_pos.y - high_low_height_differ, armor_poses2d[i].y)
                    : cv::Point3f(armor_poses2d[i].x, car_pos.y, armor_poses2d[i].y);
            };
        };
      
        return armor_poses;
    }

    cv::Point3f CarSolver::get_car_pos(const ArmorPos& armor_pos) const
    {
        // 读取车状态
        bool HIGH_ARMOR_STATE = CarState__->get_high_armor_state();
        float observe_armor_radius = CarState__->get_observe_armor_radius();
        float high_low_height_differ = CarState__->get_high_low_height_differ();

        // 延长求车的中心点
        // 延长会出现两个解,选择距离最远的解(车中心的距离一定大于检测到的装甲板位置)
        std::vector<cv::Point3f> absolute_four_points = armor_pos.absolute_four_point;
        cv::Point2f M2d(armor_pos.absolute_middle_point.x, armor_pos.absolute_middle_point.z);
        float angle_now;
        if (absolute_four_points[0].x - absolute_four_points[1].x == 0) {
            angle_now = CV_PI / 2;
        }
        else
        {
            angle_now = atan((absolute_four_points[0].z - absolute_four_points[1].z) / (absolute_four_points[0].x - absolute_four_points[1].x));
        };
        cv::Point2f _vector_now(observe_armor_radius * cos(angle_now), observe_armor_radius * sin(angle_now));
        M2d = cv::norm(M2d + _vector_now) > cv::norm(M2d - _vector_now) ?
            M2d + _vector_now : M2d - _vector_now;

        //std::cout << "get_car_pos : " << cv::Point3f(M2d.x, HIGH_ARMOR_STATE ? armor_pos.absolute_middle_point.y : armor_pos.absolute_middle_point.y + high_low_height_differ, M2d.y) << std::endl;
        // 高度判断,低处装甲板就与高度差相加并返回
        return cv::Point3f(M2d.x, HIGH_ARMOR_STATE ? armor_pos.absolute_middle_point.y : armor_pos.absolute_middle_point.y + high_low_height_differ, M2d.y);
    }

    
    SingleSolver::SingleSolver(NoneSolver* NoneSolver__, CarStateBase* CarState__, TranslationStateBase* TranslationState__, bool use_car_middle_pred)
        :CarSolver(CarState__, TranslationState__), use_car_middle_pred(use_car_middle_pred), NoneSolver__(NoneSolver__)
    {
        record_rotate_speed = 0;
    }
    std::vector<Eigen::VectorXf> SingleSolver::solve_car_strategy(Operation operation, const std::vector<ArmorPos>& armor_poses, float new_d_t, const float& car_yaw)
    {
        ArmorPos armor_pos = armor_poses[0];

        // 对朝向角在特殊区间内的目标使用自预测
        //float camera_yaw_abs = abs(armor_pos.face_angle + car_yaw);
        //if (operation != Operation::init && camera_yaw_abs > 15 && camera_yaw_abs < 25)
        //    if (camera_yaw_abs > 15 && camera_yaw_abs < 25) return NoneSolver__->solve_car_strategy(operation, armor_poses, new_d_t, car_yaw);

        switch (operation)
        {
        case rm::CarSolver::update:
            return detect_update_state_angle_jump(armor_pos, new_d_t);
        case rm::CarSolver::init:
            return init(armor_pos, new_d_t, car_yaw, record_rotate_speed);
        case rm::CarSolver::buffer:
            return buffer(armor_pos, new_d_t, car_yaw);
            break;
        default:
            break;
        }
        return std::vector<Eigen::VectorXf>();
    }
    void SingleSolver::set_record_rotate_speed(float record_rotate_speed)
    {
        this->record_rotate_speed = record_rotate_speed;
    }
    std::vector<Eigen::VectorXf> SingleSolver::init(ArmorPos armor_pos, 
        float new_d_t, const float& car_yaw, const float& rotate_speed)
    {
        Eigen::VectorXf car_state, trans_state;
        CarState__->set_high_armor_state(true);
        // 初始化使用车中心
        // 获取车中心
        cv::Point3f car_pos = get_car_pos(armor_pos);
        CarState__->set_last_yaw(armor_pos.face_angle);
        // 初始化
        Eigen::VectorXf car_state_(8);
        car_state_ << car_pos.x, car_pos.y, car_pos.z, armor_pos.face_angle, 0, 0, 0, rotate_speed;
        car_state = CarState__->reset(car_state_);

        // 平移
        Eigen::VectorXf trans_state_(5);
        trans_state_ << armor_pos.absolute_middle_point.x, armor_pos.absolute_middle_point.y,
            armor_pos.absolute_middle_point.z, 0, 0;
        trans_state = TranslationState__->reset(trans_state_);

        return { car_state, trans_state };
    }
    std::vector<Eigen::VectorXf> SingleSolver::buffer(ArmorPos armor_pos, float new_d_t, const float& car_yaw)
    {
        Eigen::VectorXf car_state, trans_state;
        bool angle_jump = CarState__->angle_jump(armor_pos.face_angle);
        CarState__->set_last_yaw(armor_pos.face_angle);
        if (angle_jump) { // 按照跳变更新 
            // 高低装甲板状态取反,才能获取正确的车半径
            CarState__->inversion_high_armor_state();
        };
        // 此次更新不进行速度更新
        // 除了init全部使用装甲板位置
        Eigen::VectorXf measurement(4);
        if (use_car_middle_pred) {
            cv::Point3f car_pos = get_car_pos(armor_pos);
            Eigen::VectorXf measurement(4);
            measurement << car_pos.x, car_pos.y, car_pos.z, armor_pos.face_angle;
        }
        else {
            Eigen::VectorXf measurement(4);
            measurement << armor_pos.absolute_middle_point.x,
                armor_pos.absolute_middle_point.y, armor_pos.absolute_middle_point.z, armor_pos.face_angle;
        };
        car_state = CarState__->reset_maintain_speed(measurement, new_d_t);

        Eigen::VectorXf trans_measurement(3);
        trans_measurement << armor_pos.absolute_middle_point.x, armor_pos.absolute_middle_point.y, armor_pos.absolute_middle_point.z;
        trans_state = TranslationState__->reset_maintain_speed(trans_measurement, new_d_t);
        return { car_state,trans_state };
    };
    std::vector<Eigen::VectorXf> SingleSolver::detect_update_state_angle_jump(ArmorPos armor_pos, float new_d_t)
    {
        Eigen::VectorXf car_state, trans_state;
        bool angle_jump = CarState__->angle_jump(armor_pos.face_angle);
        CarState__->set_last_yaw(armor_pos.face_angle);

        if (angle_jump) { // 按照跳变更新

            // 更新高低装甲板状态,观测角,此次仅更新位置不更新速度
            // 先取反,才能获取正确的车半径
            CarState__->inversion_high_armor_state();

            Eigen::VectorXf measurement(4);
            if (use_car_middle_pred) {
                cv::Point3f car_pos = get_car_pos(armor_pos);
                measurement << car_pos.x, car_pos.y, car_pos.z, armor_pos.face_angle;
            }
            else {
                measurement << armor_pos.absolute_middle_point.x,
                    armor_pos.absolute_middle_point.y, armor_pos.absolute_middle_point.z, armor_pos.face_angle;
            };

            car_state = CarState__->reset_maintain_speed(measurement, new_d_t);

            // 平移对象
            Eigen::VectorXf trans_measurement(3);
            trans_measurement << armor_pos.absolute_middle_point.x, armor_pos.absolute_middle_point.y, armor_pos.absolute_middle_point.z;
            trans_state = TranslationState__->reset_maintain_speed(trans_measurement, new_d_t);

        }
        else {
            // 全部更新

            Eigen::VectorXf measurement(4);
            if (use_car_middle_pred) {
                cv::Point3f car_pos = get_car_pos(armor_pos);
                measurement << car_pos.x, car_pos.y, car_pos.z, armor_pos.face_angle;
            }
            else {
                measurement << armor_pos.absolute_middle_point.x,
                    armor_pos.absolute_middle_point.y, armor_pos.absolute_middle_point.z, armor_pos.face_angle;
            };

            car_state = CarState__->update(measurement, new_d_t);

            // 更新平移对象
            Eigen::VectorXf trans_measurement(3);
            trans_measurement << armor_pos.absolute_middle_point.x, armor_pos.absolute_middle_point.y, armor_pos.absolute_middle_point.z;
            trans_state = TranslationState__->update(trans_measurement, new_d_t);
        };
        return { car_state,trans_state };
    }

    std::vector<Eigen::VectorXf> SingleSolver::solve_car_maintain_high_armor_state(
        Operation operation, ArmorPos armor_pos, float new_d_t, const float& car_yaw)
    {
        Eigen::VectorXf car_state, trans_state;
        bool angle_jump = CarState__->angle_jump(armor_pos.face_angle);
        
        if (operation == Operation::init) {
            Eigen::VectorXf car_state, trans_state;
            CarState__->set_high_armor_state(true);
            // 初始化使用车中心
            // 获取车中心
            cv::Point3f car_pos = get_car_pos(armor_pos);
            // 初始化
            Eigen::VectorXf car_state_(8);
            car_state_ << car_pos.x, car_pos.y, car_pos.z, armor_pos.face_angle, 0, 0, 0, record_rotate_speed;
            car_state = CarState__->reset(car_state_);
            // 平移
            Eigen::VectorXf trans_state_(5);
            trans_state_ << armor_pos.absolute_middle_point.x, armor_pos.absolute_middle_point.y,
                armor_pos.absolute_middle_point.z, 0, 0;
            trans_state = TranslationState__->reset(trans_state_);
        }
        else if (angle_jump || operation == Operation::buffer) { // 按照跳变更新

            //// 如果装甲板跳变但是朝向角
            //float last_yaw_cam_abs = abs(-CarState__->get_last_yaw() - car_yaw) * 180 / CV_PI;
            //if (last_yaw_cam_abs < 25.0) { // 说明yaw在中心跳变,返回

            //};

            // 更新高低装甲板状态,观测角,此次仅更新位置不更新速度
            // 先取反,才能获取正确的车半径
            Eigen::VectorXf measurement(4);
            if (use_car_middle_pred) {
                cv::Point3f car_pos = get_car_pos(armor_pos);
                measurement << car_pos.x, car_pos.y, car_pos.z, armor_pos.face_angle;
            }
            else {
                measurement << armor_pos.absolute_middle_point.x,
                    armor_pos.absolute_middle_point.y, armor_pos.absolute_middle_point.z, armor_pos.face_angle;
            };

            car_state = CarState__->reset_maintain_speed(measurement, new_d_t);

            // 平移对象
            Eigen::VectorXf trans_measurement(3);
            trans_measurement << armor_pos.absolute_middle_point.x, armor_pos.absolute_middle_point.y, armor_pos.absolute_middle_point.z;
            trans_state = TranslationState__->reset_maintain_speed(trans_measurement, new_d_t);
        }
        else {
            // 全部更新
            Eigen::VectorXf measurement(4);
            if (use_car_middle_pred) {
                cv::Point3f car_pos = get_car_pos(armor_pos);
                measurement << car_pos.x, car_pos.y, car_pos.z, armor_pos.face_angle;
            }
            else {
                measurement << armor_pos.absolute_middle_point.x,
                    armor_pos.absolute_middle_point.y, armor_pos.absolute_middle_point.z, armor_pos.face_angle;
            };

            car_state = CarState__->update(measurement, new_d_t);

            // 更新平移对象
            Eigen::VectorXf trans_measurement(3);
            trans_measurement << armor_pos.absolute_middle_point.x, armor_pos.absolute_middle_point.y, armor_pos.absolute_middle_point.z;
            trans_state = TranslationState__->update(trans_measurement, new_d_t);
        };
        CarState__->set_last_yaw(armor_pos.face_angle);
        return { car_state,trans_state };
    };

    DoubleSolver::DoubleSolver(SingleSolver* SingleSolver__, NoneSolver* NoneSolver__,
        CarStateBase* CarState__, TranslationStateBase* TranslationState__,
        AngleOptimizer* AngleOptimizer__, int radius_optimizer_queue_length)
        :NoneSolver__(NoneSolver__), SingleSolver__(SingleSolver__), CarSolver(CarState__, TranslationState__),
        AngleOptimizer__(AngleOptimizer__)
    {
        RadiusOptimizer__ = new RadiusOptimizer(radius_optimizer_queue_length);
    };
    DoubleSolver::~DoubleSolver()
    {
        if (RadiusOptimizer__ != NULL) delete RadiusOptimizer__;
    }

    std::vector<Eigen::VectorXf> DoubleSolver::solve_car_strategy(Operation operation,
        const std::vector<ArmorPos>& armor_poses, float new_d_t, const float& car_yaw)
    {
        // 分左右装甲板再解算
        ArmorPos left_armor_pos, right_armor_pos;
        // 用图像点对比二者在图像中的左右位置关系
        cv::Point2f center0(0, 0), center1(0, 0);
        for (int i = 0; i < 4; i++)
            center0 = center0 + armor_poses[0].pixel_four_point[i];
        for (int i = 0; i < 4; i++)
            center1 = center1 + armor_poses[1].pixel_four_point[i];
        center0.x = center0.x / 4;
        center0.y = center0.y / 4;
        center1.x = center1.x / 4;
        center1.y = center1.y / 4;

        if (center0.x < center1.x)
        {
            left_armor_pos = armor_poses[0];
            right_armor_pos = armor_poses[1];
        }
        else {
            left_armor_pos = armor_poses[1];
            right_armor_pos = armor_poses[0];
        };
        
        // 优化朝向角精度
        AngleOptimizer__->correct_face_angle(left_armor_pos, right_armor_pos, car_yaw);
        int light_get_armor_pos_size = 0;
        if (!left_armor_pos.light_loss) light_get_armor_pos_size++;
        if (!right_armor_pos.light_loss) light_get_armor_pos_size++;
        
        if (light_get_armor_pos_size == 0) { // 无有效装甲板,自更新
            return NoneSolver__->solve_car_strategy(operation, {}, new_d_t, car_yaw);
        }
        else if (light_get_armor_pos_size == 1) { // 单装甲板单独更新
            if (left_armor_pos.light_loss)
                return SingleSolver__->solve_car_strategy(operation, { right_armor_pos }, new_d_t, car_yaw);
            else
                return SingleSolver__->solve_car_strategy(operation, { left_armor_pos }, new_d_t, car_yaw);
        }
        else {
            float high_armor_radius, low_armor_radius, high_low_height_differ;
            ArmorPos high_armor_pos = 
                calculate_car_size(high_armor_radius, low_armor_radius, high_low_height_differ, left_armor_pos, right_armor_pos);
         
            // 按照单装甲板更新,每次保证放入为高装甲板,保证在双装甲板情况下也能持续更新
            CarState__->set_high_armor_state(true);
            
            // 英雄长短轴半径不同,其他兵种默认长短轴半径相同,且根据官方限幅,限制长短轴半径,去除不合理情况
            int kind = high_armor_pos.kind;
            if (kind != 1) {
                //float middle_r = (low_armor_radius + high_armor_radius) / 2;
                //low_armor_radius = middle_r;
                //high_armor_radius = middle_r;

                //float middle_r_limit = (kind == 2 ? 35 : 30);
                //if (middle_r > middle_r_limit) {
                //    low_armor_radius = middle_r_limit;
                //    high_armor_radius = middle_r_limit;
                //};

                float middle_r_limit = (kind == 2 ? 35 : 30);
                float middle_r_limit_min = 20;
                if (low_armor_radius > middle_r_limit) {
                    low_armor_radius = middle_r_limit;
                }
                else if (low_armor_radius < middle_r_limit_min)
                {
                    low_armor_radius = middle_r_limit_min;
                };

                if (high_armor_radius > middle_r_limit) {
                    high_armor_radius = middle_r_limit;
                }
                else if (high_armor_radius < middle_r_limit_min)
                {
                    high_armor_radius = middle_r_limit_min;
                };
            }
            else {
                high_armor_radius = high_armor_radius > 40 ? 40 : high_armor_radius;
                low_armor_radius = low_armor_radius > 40 ? 40 : low_armor_radius;
                float middle_r_limit_min = 20;
                if (high_armor_radius < middle_r_limit_min) {
                    high_armor_radius = middle_r_limit_min;
                };
                if (low_armor_radius < middle_r_limit_min) {
                    low_armor_radius = middle_r_limit_min;
                };
            };

            RadiusOptimizer__->observe(high_armor_radius, low_armor_radius, high_low_height_differ);
            CarState__->set_radius(high_armor_radius, low_armor_radius);
            CarState__->set_high_low_height_differ(high_low_height_differ);
            return SingleSolver__->solve_car_maintain_high_armor_state(operation, high_armor_pos, new_d_t,car_yaw);

            //std::vector<float> fit_angles = AngleOptimizer__->observe({ left_armor_pos.face_angle,right_armor_pos.face_angle }, car_yaw);
            //int angle_size = fit_angles.size();
            //if(angle_size == 0){
            //    return NoneSolver__->solve_car_strategy(operation, {}, new_d_t, car_yaw);
            //}
            //else if(angle_size == 1)
            //{
            //    return fit_angles[0] == left_armor_pos.face_angle ?
            //        SingleSolver__->solve_car_strategy(operation, { left_armor_pos }, new_d_t, car_yaw) :
            //        SingleSolver__->solve_car_strategy(operation, { right_armor_pos }, new_d_t, car_yaw);
            //}
            //else{
            //    float high_armor_radius, low_armor_radius, high_low_height_differ;
            //    ArmorPos high_armor_pos = 
            //        calculate_car_size(high_armor_radius, low_armor_radius, high_low_height_differ, left_armor_pos, right_armor_pos);
            //    float middle_r = (low_armor_radius + high_armor_radius) / 2;
            //    low_armor_radius = middle_r;
            //    high_armor_radius = middle_r;
            //    RadiusOptimizer__->observe(high_armor_radius, low_armor_radius, high_low_height_differ);
            //    // 按照单装甲板更新,每次保证放入为高装甲板,保证在双装甲板情况下也能持续更新
            //    CarState__->set_high_armor_state(true);
            //    CarState__->set_radius(high_armor_radius, low_armor_radius);
            //    CarState__->set_high_low_height_differ(high_low_height_differ);
            //    return SingleSolver__->solve_car_maintain_high_armor_state(operation, high_armor_pos, new_d_t,car_yaw);
            //};
                   
        };

    };
   
    ArmorPos DoubleSolver::calculate_car_size(float& high_armor_radius, float& low_armor_radius, float& high_low_height_differ,
        const ArmorPos& left_armor_pos, const ArmorPos& right_armor_pos)
    {
        // 使用绝对坐标系y值取高,低装甲板
        ArmorPos high_armor_pos, low_armor_pos;
        /*if (left_armor_pos.absolute_middle_point.y < right_armor_pos.absolute_middle_point.y) {
            high_armor_pos = left_armor_pos;
            low_armor_pos = right_armor_pos;
        }
        else {
            high_armor_pos = right_armor_pos;
            low_armor_pos = left_armor_pos;
        };*/

        // 使用图像位置取高,低装甲板,y越小,位置越高
        float left_armor_pos_y = 0; 
        for (const auto& x : left_armor_pos.pixel_four_point)
            left_armor_pos_y += x.y;
        left_armor_pos_y /= 4.0;

        float right_armor_pos_y = 0;
        for (const auto& x : right_armor_pos.pixel_four_point)
            right_armor_pos_y += x.y;
        right_armor_pos_y /= 4.0;

        if (left_armor_pos_y < right_armor_pos_y) {
            high_armor_pos = left_armor_pos;
            low_armor_pos = right_armor_pos;
        }
        else {
            high_armor_pos = right_armor_pos;
            low_armor_pos = left_armor_pos;
        };
        
        // 求交点
        float k_next = tan(high_armor_pos.face_angle);
        float k = -1.0 / k_next;
        cv::Point2f high_armor_pos2f = cv::Point2f(high_armor_pos.absolute_middle_point.x, high_armor_pos.absolute_middle_point.z);
        cv::Point2f low_armor_pos2f = cv::Point2f(low_armor_pos.absolute_middle_point.x, low_armor_pos.absolute_middle_point.z);
        cv::Point2f car_middle = get_intersection_point(k_next, k, low_armor_pos2f, high_armor_pos2f);

        // 得半径
        high_armor_radius = cv::norm(car_middle - high_armor_pos2f);
        low_armor_radius = cv::norm(car_middle - low_armor_pos2f);
        high_low_height_differ = high_armor_pos.absolute_middle_point.y - low_armor_pos.absolute_middle_point.y;
        return high_armor_pos;
    }
    ;
    cv::Point2f DoubleSolver::get_intersection_point(float k1, float k2, cv::Point2f point1, cv::Point2f point2) {
        float x1 = point1.x;
        float y1 = point1.y;
        float x2 = point2.x;
        float y2 = point2.y;
        float x = (k1 * x1 - y1 - k2 * x2 + y2) / (k1 - k2);
        float y = k1 * (x - x1) + y1;
        return cv::Point2f(x, y);
    }
    NoneSolver::NoneSolver(CarStateBase* CarState__, TranslationStateBase* TranslationState__)
        :CarSolver(CarState__, TranslationState__)
    {
    }
    std::vector<Eigen::VectorXf> NoneSolver::solve_car_strategy(Operation operation, const std::vector<ArmorPos>& armor_poses, float new_d_t, const float& car_yaw)
    {
        return operation == Operation::init ? std::vector<Eigen::VectorXf>() : self_update(new_d_t);
    }
    std::vector<Eigen::VectorXf> NoneSolver::self_update(float new_d_t)
    {
        Eigen::VectorXf car_state, trans_state;
        // 自更新
        Eigen::VectorXf last_state = CarState__->get_last_state();
        last_state(0) += last_state(4) * new_d_t;
        last_state(1) += last_state(5) * new_d_t;
        last_state(2) += last_state(6) * new_d_t;
        last_state(3) += last_state(7) * new_d_t;
        // 设置最新朝向角
        CarState__->set_last_yaw(last_state(3));
        car_state = CarState__->reset(last_state);

        // 更新平移
        Eigen::VectorXf last_trans_state = TranslationState__->get_last_state();
        last_trans_state(0) += last_trans_state(3) * new_d_t;
        last_trans_state(2) += last_trans_state(4) * new_d_t;

        Eigen::VectorXf trans_measurement(3);
        trans_measurement << last_trans_state(0), last_trans_state(1), last_trans_state(2);
        trans_state = TranslationState__->reset_maintain_speed(trans_measurement, new_d_t);

        return { car_state , trans_state };
    };
}
