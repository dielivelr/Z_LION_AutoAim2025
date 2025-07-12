#include "NumberFinder.h"

namespace rm
{
	cv::Mat rm::NumNet::get_num_img_roi(const cv::Mat& pic, const Armor& armor,
		int num_img_roi_light_length)
	{
		// Light length in image
		const int light_length = num_img_roi_light_length;
		// Image size after warp
		const int warp_height = 28;
		const int small_armor_width = 32;
		const int large_armor_width = 54;
		// Number ROI size
		const cv::Size roi_size(img_width, img_height);
		// Warp perspective transform
		cv::Point2f lights_vertices[4] = {
			armor.left_light.bottom, armor.left_light.top, armor.right_light.top,
			armor.right_light.bottom };

		const int top_light_y = (warp_height - light_length) / 2 - 1;
		const int bottom_light_y = top_light_y + light_length;
		const int warp_width = armor.armor_small ? small_armor_width : large_armor_width;
		cv::Point2f target_vertices[4] = {
			cv::Point(0, bottom_light_y),
			cv::Point(0, top_light_y),
			cv::Point(warp_width - 1, top_light_y),
			cv::Point(warp_width - 1, bottom_light_y),
		};
		cv::Mat number_image;
		auto rotation_matrix = cv::getPerspectiveTransform(lights_vertices, target_vertices);
		cv::warpPerspective(pic, number_image, rotation_matrix, cv::Size(warp_width, warp_height));
		// Get ROI
		number_image =
			number_image(cv::Rect(cv::Point((warp_width - roi_size.width) / 2, 0), roi_size));

		// machine
		cv::cvtColor(number_image, number_image, cv::COLOR_RGB2GRAY);
		cv::threshold(number_image, number_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

		

		return number_image;
	};

	bool rm::NumNet::forward(int& kind, double& forward_confince, cv::Mat src,
		cv::dnn::Net& model)
	{
		try {
			src = src / 255.0;
			cv::Mat blob;
			cv::dnn::blobFromImage(src, blob, 1., cv::Size(img_width, img_height)); // img_height, img_width
			//std::cout << blob << std::endl;
			model.setInput(blob);
			int max_ = -100000;
			int max_id = -1;
			cv::Mat output = model.forward();
			float max_prob = *std::max_element(output.begin<float>(), output.end<float>());
			cv::Mat softmax_prob;
			cv::exp(output - max_prob, softmax_prob);
			float sum = static_cast<float>(cv::sum(softmax_prob)[0]);
			softmax_prob /= sum;
			double confidence;
			cv::Point class_id_point;
			minMaxLoc(softmax_prob.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
			kind = class_id_point.x;
			forward_confince = confidence;
			return true;
		}
		catch (std::exception e) {
			return false;
		};
	}
	;


	NumberFinder::NumberFinder(cv::dnn::Net model, double min_confidence, int num_img_roi_light_length)
		:NumNet__(NumNet__), model(model), min_confidence(min_confidence), num_img_roi_light_length(num_img_roi_light_length)
	{
		NumNet__ = new NumNet();
	};

	NumberFinder::NumberFinder(std::string model_path,
		double min_confidence, int num_img_roi_light_length)
		:NumNet__(NumNet__), min_confidence(min_confidence), num_img_roi_light_length(num_img_roi_light_length)
	{
		this->model = cv::dnn::readNetFromONNX(model_path);
		NumNet__ = new NumNet();
	}
	NumberFinder::~NumberFinder()
	{
		if (NumNet__ != NULL) delete NumNet__;
	}
	;

	std::vector<rm::Armor> NumberFinder::forward(cv::Mat src, std::vector<rm::Armor> armors)
	{
		for (auto& armor : armors) {
			//std::cout << "armor armor_small : " <<armor.armor_small<< std::endl;
			cv::Mat num_img = NumNet__->get_num_img_roi(src,
				armor, num_img_roi_light_length);
			// cv::imshow("num_img",num_img);
			// cv::waitKey(1);
			int kind;
			double forward_confince;
			// 推理,按照类型映射和置信度对armor.label赋值
			if (!NumNet__->forward(kind, forward_confince, num_img, this->model))
			{
				/*Logger& Logger_ = Logger::getInstance();
				Logger_.log(LoggerEvent::Event::FATAL, "NumberFinder forward error!!!");*/
			}
			else {
				//std::cout << "forward_confince : " <<forward_confince<< std::endl;
				//std::cout << "kind : " <<kind<< std::endl;
				if (forward_confince < min_confidence) {
					armor.label = rm::Armor::label_type::negative;
				}
				else {
					kind_to_label_type(armor, kind);
					if (exclude_impossible_kind(armor))
						armor.label = rm::Armor::label_type::negative;
				};
			};
		};

		std::vector<rm::Armor> out_armors;
		for (const auto& armor : armors)
			if (armor.label != Armor::label_type::negative)
				out_armors.push_back(armor);
		return out_armors;
	};

	bool NumberFinder::exclude_impossible_kind(const rm::Armor& armor)
	{
		bool ret = armor.label != rm::Armor::one;
		return armor.armor_small ^ ret;
	};

	void NumberFinder::kind_to_label_type(rm::Armor& armor, int kind)
	{
		switch (kind)
		{
		case 0:
			armor.label = rm::Armor::label_type::one;
			break;
		case 6:
			armor.label = rm::Armor::label_type::sentinel;
			break;
		case 2:
			armor.label = rm::Armor::label_type::three;
			break;
		case 3:
			armor.label = rm::Armor::label_type::four;
			break;
		case 4:
			armor.label = rm::Armor::label_type::five;
			break;
		default:
			armor.label = rm::Armor::label_type::negative;
			break;
		};
	};
}