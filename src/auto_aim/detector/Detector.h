#ifndef _DETECTOR_H_
#define _DETECTOR_H_
#include"Light/Light.h"
#include"Armor/Armor.h"
#include"TransDetector.h"
#include"Machine/Machine.h"
#include"LightFinder/LightFinder.h"
#include"ArmorFinder/ArmorFinder.h"
#include"NumberFinder/NumberFinder.h"
#include"YoloModel/YoloModel.h"
namespace rm
{
	/*
	* 模板方法
	* 功能接口:输入图像,识别装甲板
	* 由于装甲板尺寸上只有大小两种类型,鉴于长期未改变,作为静态参数
	*/
	class Detector
	{
	public:
		Detector(); 
		Detector(int img_width, int img_height);
		virtual ~Detector() {};
		// 检测函数
		virtual std::vector<SendArmor> detect(cv::Mat src) = 0;
	protected:
		// 图像宽高
		const int img_width, img_height;
	};

	
	/***********************************/
	// 传统视觉检测机
	/***********************************/
	class TraditionDetector : public Detector
	{
		Machine* Machine__;
		UsualLightFinder* UsualLightFinder__;
		ArmorFinder* ArmorFinder__;
		NumberFinder* NumberFinder__;
	public:
		TraditionDetector() :Detector() {};
		TraditionDetector(Machine* Machine__, UsualLightFinder* UsualLightFinder__,
			ArmorFinder* ArmorFinder__, NumberFinder* NumberFinder__,
			int img_width, int img_height);
		std::vector<SendArmor> detect(cv::Mat src) override;
	};


	/***********************************/
	// 视觉融合检测机
	/***********************************/
	class FusionDetector : public Detector
	{
		Machine* Machine__;
		YoloModel* YoloModel__;
		ArmorFinder* ArmorFinder__;
		MiddleLightFinder* MiddleLightFinder__;
	public:
		FusionDetector() :Detector() {};
		FusionDetector(Machine* Machine__, YoloModel* YoloModel__, ArmorFinder* ArmorFinder__, 
			MiddleLightFinder* MiddleLightFinder__, float _width_expand_, float _height_expand_,
			int img_width, int img_height);
		std::vector<SendArmor> detect(cv::Mat src) override;
	private:
		// 截取roi
		cv::Rect capture_light_roi(bool get_left_light, const std::vector<cv::Point2f>& middle_four_points, 
			float width_expand, float height_expand, int image_width, int image_height);
		
	private:
		// 灯条roi拓展项
		float _width_expand_;
		float _height_expand_;
	};
	


}



#endif // !DETECTR_H
