#ifndef _NUMBERFINDER_H_
#define _NUMBERFINDER_H_

#include"../Armor/Armor.h"
#include<vector>

/********************************
* 装甲板数字识别类
*********************************/
namespace rm
{
	/*************************
	* 数字识别神经网络类
	* 默认使用20, 28尺寸
	*************************/
	class NumNet
	{
	public:
		NumNet() :img_width(20), img_height(28) {};
		// 获取数字识别所需图像roi
		cv::Mat get_num_img_roi(const cv::Mat& pic, const Armor& armor,
			int num_img_roi_light_length = 12);
		// 模型前向推理
		bool forward(int& kind, double& forward_confince, cv::Mat src,
			cv::dnn::Net& model);
	private:
		int img_width, img_height;
	};

	class NumberFinder
	{
		rm::NumNet* NumNet__; // 若未来数字尺寸更改,更改构造函数即可
	public:
		NumberFinder() {};
		NumberFinder(cv::dnn::Net model, double min_confidence,int num_img_roi_light_length);
		NumberFinder(std::string model_path, double min_confidence, int num_img_roi_light_length);
		~NumberFinder();
		// 前向推理
		std::vector<rm::Armor> forward(cv::Mat src,std::vector<rm::Armor> armors);
	private:
		// 排除不可能类型
		bool exclude_impossible_kind(const rm::Armor& armor);
		// 将识别出的kind映射到Armor::label_type
		void kind_to_label_type(rm::Armor& armor, int kind);
		cv::dnn::Net model; // 模型
		double min_confidence; // 最低置信度
	private:
		int num_img_roi_light_length; // 数字图像长
	};
}


#endif // !_NUMBERFINDER_H_
