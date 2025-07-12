#include"debug.h"
#ifdef DETECT_TEST

#include"../detector/Video/Video.h"
#include"../detector/ArmorFinder/ArmorFinder.h"
#include"../detector/LightFinder/LightFinder.h"
#include"../detector/Machine/Machine.h"
#include"../detector/Detector.h"
#include"../../auto_aim_debug/src/Color/Color.h"
#include"../../auto_aim_debug/src/Tools/Tools.h"

using namespace rm;


int main()
{
	std::vector<std::string> video_path = {
		"../auto_aim_publisher/videos/sample.avi",
	};
	Video video(video_path);

	bool use_fusiondetector = true;

	// Machine
	Machine* Machine__ = new AutoMachine();
	
	// UsualLightFinder
	bool enemy_blue = true;
	float max_ratio_w_h = 1.1;
	float max_angle = 35.0;
	bool SIMPLE = false;
	UsualLightFinder* UsualLightFinder__
		= new UsualLightFinder(max_ratio_w_h, max_angle, SIMPLE);

	// MiddleLightFinder
	float ret_dis_error = 5.0;
	MiddleLightFinder* MiddleLightFinder__
		= new MiddleLightFinder(UsualLightFinder__, ret_dis_error);

	// ArmorFinder
	float armor_min_height_ratio = 0.6;
	float small_armor_min_aspect_ratio = 0.8;
	float small_armor_max_aspect_ratio = 3.2;
	float big_armor_min_aspect_ratio = 3.2;
	float big_armor_max_aspect_ratio = 8.0;
	float max_center_angle = 15.0;
	ArmorFinder* ArmorFinder__
		= new ArmorFinder(armor_min_height_ratio, small_armor_min_aspect_ratio,
			small_armor_max_aspect_ratio, big_armor_min_aspect_ratio,
			big_armor_max_aspect_ratio, max_center_angle);

	// YoloModel
	std::string model_path = "./models/yolo.onnx";
	int image_size = 256;
	YoloModel* YoloModel__ = new YoloModel(model_path, image_size);

	// Detector
	int img_width = 1440;
	int img_height = 1080;
	float _width_expand_ = 1.414;
	float _height_expand_ = 1.414;

	std::unique_ptr<Detector> Detector__
		= std::make_unique<FusionDetector>(Machine__, YoloModel__, ArmorFinder__
			, MiddleLightFinder__, _width_expand_, _height_expand_, img_width, img_height);
	//Ö÷Ñ­»·
	while (1) {
		cv::Mat src = video.ReadPic();
		
		cv::resize(src, src, cv::Size(1440,1080));
		if (src.empty()) {
			break;
		};

		UsualLightFinder__->set_enemy_color(enemy_blue);
		if (YoloModel__) YoloModel__->set_enemy_color(enemy_blue);
		std::vector<SendArmor> send_armors = Detector__->detect(src);
		for (const auto& send_armor : send_armors)
		{

			cv::line(src, send_armor.middle_four_points[0],
				send_armor.middle_four_points[1],
				White, 5);
			cv::line(src, send_armor.middle_four_points[1],
				send_armor.middle_four_points[3],
				White, 5);
			cv::line(src, send_armor.middle_four_points[3],
				send_armor.middle_four_points[2],
				White, 5);
			cv::line(src, send_armor.middle_four_points[2],
				send_armor.middle_four_points[0],
				White, 5);
			
			if (send_armor.light_loss)
				cv::putText(src, "error", send_armor.middle_four_points[2],
					2, 2, White, 2);
			else
				cv::putText(src, std::to_string(send_armor.label), send_armor.middle_four_points[2],
					2, 2, White, 2);
		};
		//cv::resize(src, src, cv::Size(src.cols / 2, src.rows / 2));

		cv::imshow("src", src);
		cv::waitKey(1);

	};

	if (Machine__ != NULL)delete Machine__;
	if (UsualLightFinder__ != NULL)delete UsualLightFinder__;
	if (ArmorFinder__ != NULL)delete ArmorFinder__;
	if (YoloModel__ != NULL)delete YoloModel__;
	if (MiddleLightFinder__ != NULL)delete MiddleLightFinder__;
	return 0;
};


#endif // DETECT_TEST