#include "PitchCompensator.h"


namespace rm
{
    PitchCompensator::PitchCompensator(const std::function<float(const float&)>& horizontal_distance_correct_func, float bullet_speed, float pitch_compensation_amplitude)
        :bullet_speed(bullet_speed), pitch_compensation_amplitude(pitch_compensation_amplitude),
        horizontal_distance_correct_func(horizontal_distance_correct_func)
    {
    }
    PitchCompensator::PitchCompensator(float bullet_speed, float pitch_compensation_amplitude)
        :bullet_speed(bullet_speed), pitch_compensation_amplitude(pitch_compensation_amplitude)
    {
        auto fuc = [](const float& horizontal_distance) {
            return horizontal_distance;
        };
        horizontal_distance_correct_func = fuc;
    }
    cv::Point3f PitchCompensator::pitch_equation_solve(cv::Point3f target_point)
    {
        // 注意这里的弹速需要转换为m/s,因为要和G统一
        float last_height = target_point.y;
        float distance = cv::norm(target_point);
        bullet_speed /= 100.0;
        last_height /= 100.0;
        distance /= 100.0;
        float x = sqrt(distance * distance - last_height * last_height); // 计算水平距离
        // 解方程
        float a = GRAVITY * GRAVITY / 4;
        float b = -(bullet_speed * bullet_speed + GRAVITY * last_height);
        float c = x * x + last_height * last_height;
        float discriminant = b * b - 4 * a * c;
        if (discriminant < 0) return target_point;
        float t;
        float t1 = (-b + sqrt(discriminant)) / (2 * a);
        float t2 = (-b - sqrt(discriminant)) / (2 * a);
        if (t1 < 0 && t2 < 0) return target_point;
        else if (t1 < 0) t = t2;
        else if (t2 < 0) t = t1;
        else {
            t = MIN(t1, t2);
        };
        t = sqrt(t); // 开根号,得解
        float angle = acos(x / (t * bullet_speed));

        // 计算高度差
        float differ_height;
        differ_height = x * tan(angle);
        float dif = abs(differ_height - last_height) * 100;
        differ_height = dif * this->pitch_compensation_amplitude;

        target_point.y -= differ_height;

        return target_point;
    }

    cv::Point3f PitchCompensator::solve_pitch_fly_delay(float& fly_delay,cv::Point3f target_point)
    {
        double bullet_speed_m = bullet_speed / 100.0; // 统一单位
        double g = 9.8;
        double h = -target_point.y / 100.0; // 绝对坐标系y轴相反
        double d = sqrt(target_point.x * target_point.x + target_point.z * target_point.z) / 100.0;

        // 水平距离修正
        d = this->horizontal_distance_correct_func(d);

        double t = sqrt(d * d + h * h) / bullet_speed_m;
        float pitch = 0;
        for (int i = 0; i < 5; i++) {

            pitch = asin((h + 0.5 * g * t * t) / (bullet_speed_m * t));

            if (std::isnan(pitch)) {
                pitch = 0.0;
            };

            t = d / (bullet_speed_m * cos(pitch));
        };
        
        
        // 将pitch映射为高度y
        float new_y = -100.0 * d * tan(pitch);
        // this->pitch_compensation_amplitude
        float differ = (target_point.y - new_y) * this->pitch_compensation_amplitude;

        if(pitch != 0)
            target_point.y -= differ;

        // new pitch
        h = -target_point.y / 100.0;
        pitch = asin((h + 0.5 * g * t * t) / (bullet_speed_m * t));
        if (std::isnan(pitch)) {
            pitch = 0.0;
        };
        t = d / (bullet_speed_m * cos(pitch));

        fly_delay = t;
        return target_point;
    }
    ;

    cv::Point3f PitchCompensator::solve_pitch_fly_delay_amplitude(float& fly_delay, cv::Point3f target_point){
        if(this->pitch_compensation_amplitude == 0) return target_point;
        return solve_pitch_fly_delay(fly_delay,target_point);
    };
}
