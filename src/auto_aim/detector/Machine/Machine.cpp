#include "Machine.h"

inline cv::Mat rm::Machine::threshold(cv::Mat src, double pic_min_threshold)
{
	cv::threshold(src, src, pic_min_threshold, 255, cv::THRESH_BINARY);
	return src;
}

rm::DifferMachine::DifferMachine(double pic_min_threshold, bool blue_enemy)
	:pic_min_threshold(pic_min_threshold), blue_enemy(blue_enemy)
{
}

cv::Mat rm::DifferMachine::machine(cv::Mat src)
{
	cv::Mat channels[3];
	cv::split(src, channels);
	if(blue_enemy)
	{
		src = channels[0] - channels[2];
		
	}
	else
	{
		
		src = channels[2] - channels[0];
	}
	return threshold(src, this->pic_min_threshold);
}

rm::GreyMachine::GreyMachine(double pic_min_threshold)
	:pic_min_threshold(pic_min_threshold)
{
}

cv::Mat rm::GreyMachine::machine(cv::Mat src)
{
	cv::cvtColor(src, src, cv::COLOR_BGR2GRAY);
	return threshold(src, this->pic_min_threshold);
}

cv::Mat rm::AutoMachine::machine(cv::Mat src)
{
	cv::cvtColor(src, src, cv::COLOR_RGB2GRAY);
	return auto_threshold(src);
}

inline cv::Mat rm::AutoMachine::auto_threshold(cv::Mat src)
{
	cv::threshold(src, src, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
	return src;
}
