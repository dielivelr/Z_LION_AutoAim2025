#ifndef _YOLOMODEL_H_
#define _YOLOMODEL_H_
#include"../Light/Light.h"
#include"../Armor/Armor.h"
#include <opencv2/opencv.hpp>
#include<openvino/openvino.hpp>
namespace rm
{

	class YoloModel
	{
	protected:
		// 0  3
		// 1  2
		// 左上,左下,右下,右上
		struct alignas(4) bbox_t {
			cv::Point2f pts[4];
			float confidence;
			int label;// 
		};
	public:
		YoloModel() {};
		YoloModel(std::string model_path, int image_size);
		void set_enemy_color(bool enemy_blue);
		virtual ~YoloModel() {};
	public:
		// 寻找装甲板
		virtual std::vector<Armor> find_armors(cv::Mat src);

	protected:
		// 仅向子类暴露bbox_t,对用户透明
		std::vector<bbox_t> forward(cv::Mat src);
		// 目标超界检测
		std::vector<bbox_t> screen_out_edge_targets(const std::vector<bbox_t>& bbox_ts,
			double image_width, double image_height);
		// 自适应尺寸
		cv::Mat letterbox(cv::Mat& src, int w, int h, int& padd_w_, int& padd_h_);
		cv::Mat letterbox(cv::Mat& src, int w, int h);
		// 取最值标签
		int argmax(const float* ptr, int len);
		// sigmoid函数
		float sigmoid(float x);
		// 最小值
		float min_4(const float pts1[4]);
		// 最大值
		float max_4(const float pts1[4]);
		// 是否超出重合区域
		bool is_overlap(const float pts1[4], const float pts2[4]);
	private:
		// 标签过滤,针对颜色类型
		bool strip_filter(int label,int enemy_blue);
	private:
		int image_size; // 采用放缩尺寸处理的图像大小参数
		// 拓展的图片宽高
		int padd_w_ = 0;
		int padd_h_ = 0;
		bool enemy_blue; // 敌方颜色
	private:
		// openvino处理项
		ov::Core core;
		ov::CompiledModel model;
		ov::InferRequest iq;
		ov::Tensor input_tensor_;
	};

};
#endif // !YOLOMODEL_H

