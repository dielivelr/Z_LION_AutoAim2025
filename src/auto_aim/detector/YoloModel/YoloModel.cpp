#include"YoloModel.h"

namespace rm
{

    int YoloModel::argmax(const float* ptr, int len) {
        int max_arg = 0;
        for (int i = 1; i < len; i++) {
            if (ptr[i] > ptr[max_arg]) max_arg = i;
        }
        return max_arg;
    };

    float YoloModel::sigmoid(float x) {
        return 1 / (1 + std::exp(-x));
    }

    float YoloModel::min_4(const float pts1[4]) {
        float min = pts1[0];
        for (int i = 1; i < 4; i++) {
            if (min > pts1[i])
                min = pts1[i];
        };
        return min;
    };

    float YoloModel::max_4(const float pts1[4]) {
        float max = pts1[0];
        for (int i = 1; i < 4; i++) {
            if (max < pts1[i])
                max = pts1[i];
        };
        return max;
    };

    bool YoloModel::is_overlap(const float pts1[4], const float pts2[4]) {
        cv::Rect2f bbox1, bbox2;
        bbox1.x = pts1[0];
        bbox1.y = pts1[1];
        bbox1.width = pts1[2];
        bbox1.height = pts1[3];
        bbox2.x = pts2[0];
        bbox2.y = pts2[1];
        bbox2.width = pts2[2];
        bbox2.height = pts2[3];
        return (bbox1 & bbox2).area() > 0;
    }
    bool YoloModel::strip_filter(int label, int enemy_blue)
    {
        if (label >= 10) return false;
        if (enemy_blue) {
            return label < 5;
        }
        else {
            return label >= 5 && label < 10;
        };
    }
    ;

    cv::Mat YoloModel::letterbox(cv::Mat& src, int w, int h, int& padd_w_, int& padd_h_)
    {
        int in_w = src.cols;  // width
        int in_h = src.rows;  // height
        int tar_w = w;
        int tar_h = h;
        float r = std::min(float(tar_h) / in_h, float(tar_w) / in_w);
        int inside_w = round(in_w * r);
        int inside_h = round(in_h * r);
        padd_w_ = tar_w - inside_w;
        padd_h_ = tar_h - inside_h;

        cv::Mat resize_img;

        cv::resize(src, resize_img, cv::Size(inside_w, inside_h));

        padd_w_ = padd_w_ / 2;
        padd_h_ = padd_h_ / 2;

        int top = int(round(padd_h_ - 0.1));
        int bottom = int(round(padd_h_ + 0.1));
        int left = int(round(padd_w_ - 0.1));
        int right = int(round(padd_w_ + 0.1));

        cv::copyMakeBorder(
            resize_img, resize_img, top, bottom, left, right, 0, cv::Scalar(114, 114, 114));
        return resize_img;
    };

    cv::Mat YoloModel::letterbox(cv::Mat& src, int w, int h)
    {
        int _w, _h;
        return letterbox(src, w, h, _w, _h);
    };

    YoloModel::YoloModel(std::string model_path, int image_size)
        :image_size(image_size)
    {
        model = core.compile_model(model_path, "CPU"); // 默认采用cpu加载模型
        iq = model.create_infer_request();
        input_tensor_ = iq.get_input_tensor(0);
    };

    void YoloModel::set_enemy_color(bool enemy_blue)
    {
        this->enemy_blue = enemy_blue;
    };

    std::vector<Armor> YoloModel::find_armors(cv::Mat src)
    {
        // 神经网络推理
        std::vector<bbox_t> bbox_ts = forward(src);
        // 剔除边缘目标
        bbox_ts = screen_out_edge_targets(bbox_ts, src.cols, src.rows);
        std::vector<Armor> out_armors;
        for (const auto& x : bbox_ts) {
            out_armors.push_back(Armor(x.pts, x.label));
        };
        return out_armors;
    }
    ;

    std::vector<YoloModel::bbox_t> YoloModel::forward(cv::Mat src)
    {
        // 设定
        double image_width = src.cols;
        double image_height = src.rows;
        src = letterbox(src, image_size, image_size, padd_w_, padd_h_);
        // double start = get_now_time();
        auto input = iq.get_input_tensor(0);
        input.set_shape({ 1,3,static_cast<unsigned long long>(src.cols),static_cast<unsigned long long>(src.rows) });
        // 转换颜色空间
        cv::cvtColor(src, src, cv::COLOR_BGR2RGB);
        src.convertTo(src, CV_32F, 1.0 / 255.0);
        // 分离通道并复制数据到输出向量
        std::vector<cv::Mat> channels(3);
        cv::split(src, channels);
        float* input_data_host = input.data<float>();
        int image_area = src.rows * src.cols;
        std::copy(channels[0].begin<float>(), channels[0].end<float>(), input_data_host + image_area * 0);
        std::copy(channels[1].begin<float>(), channels[1].end<float>(), input_data_host + image_area * 1);
        std::copy(channels[2].begin<float>(), channels[2].end<float>(), input_data_host + image_area * 2);
        iq.infer(); // 推理过程，这可能是最耗时的部分
        auto output = iq.get_output_tensor(0);

        float confidence_threshold = 0.25;

        int output_numbox = output.get_shape()[1]; // TOPK_NUM = 25200
        int output_numprob = output.get_shape()[2]; // 49
        int modle_last_length = 13;
        int num_classes = output_numprob - modle_last_length; // 36
        float* output_buffer = output.data<float>();
        int TOPK_NUM = output_numbox;

        // 该四点模型采用 49: 四点分别是左上,左下,右下,右上
        // x0 y0 x1 y1 confince ltx lty lbx lby rbx rby rtx rty ==> 0 - 12
        // 13 - 48 分类,总共36种
        std::vector<bbox_t> rst;
        rst.reserve(TOPK_NUM);
        std::vector<uint8_t> removed(TOPK_NUM);
        for (int i = 0; i < TOPK_NUM; i++) {
            // 获取每一个i列数据的位信息
            auto* box_buffer = output_buffer + i * output_numprob;
            // 其中4为confince置信度
            if (box_buffer[4] < confidence_threshold) continue;
            if (removed[i]) continue;
            rst.emplace_back();
            auto& box = rst.back();
            // box_buffer + 5 位移到该地址
            memcpy(&box.pts, box_buffer + 5, 8 * sizeof(float));
            for (auto& pt : box.pts) {
                pt.x = (pt.x - padd_w_) / (image_size - 2 * padd_w_) * image_width;
                pt.y = (pt.y - padd_h_) / (image_size - 2 * padd_h_) * image_height;
            };
            box.confidence = sigmoid(box_buffer[4]);//prob * objness;
            // 类型的指针为 13之后
            float* pclass = box_buffer + modle_last_length;
            box.label = argmax(pclass, num_classes);
            for (int j = i + 1; j < TOPK_NUM; j++) {
                auto* box2_buffer = output_buffer + j * output_numprob;
                if (box2_buffer[4] < confidence_threshold) continue;
                if (removed[j]) continue;
                if (is_overlap(box_buffer, box2_buffer)) removed[j] = true;
            };
        };

        std::vector<bbox_t> out_rst;
        // label过滤
        for (const auto& rst_ : rst)
            if (strip_filter(rst_.label, this->enemy_blue))
                out_rst.push_back(rst_);
        return out_rst;
    };

    std::vector<YoloModel::bbox_t> YoloModel::screen_out_edge_targets(const std::vector<bbox_t>& bbox_ts,
        double image_width, double image_height)
    {
        std::vector<bbox_t> Out;
        for (const auto& x : bbox_ts) {
            if (
                MIN(x.pts[0].x, x.pts[1].x) <= 20 ||
                MIN(x.pts[0].y, x.pts[3].y) <= 20 ||
                MAX(x.pts[2].x, x.pts[3].x) >= image_width - 20 ||
                MAX(x.pts[2].y, x.pts[1].y) >= image_height - 20
                ) continue;
            Out.push_back(x);
        }
        return Out;
    };

}