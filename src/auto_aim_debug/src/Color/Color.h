#ifndef _COLOR_H_
#define _COLOR_H_

#include<opencv2/opencv.hpp>

namespace rm {

	const cv::Scalar Red = cv::Scalar(0, 0, 255);		
	const cv::Scalar Blue = cv::Scalar(255, 0, 0);			
	const cv::Scalar Green = cv::Scalar(0, 255, 0);		
	const cv::Scalar Yellow = cv::Scalar(0, 255, 255);		
	const cv::Scalar Brown = cv::Scalar(42, 42, 128);		
	const cv::Scalar White = cv::Scalar(255, 255, 255);		
	const cv::Scalar Black = cv::Scalar(0, 0, 0);			
	const cv::Scalar Purple = cv::Scalar(240, 32, 160);		
	const cv::Scalar Cyan = cv::Scalar(255, 255, 0);		
	const cv::Scalar Grey = cv::Scalar(128, 128, 128);		
	const cv::Scalar Orange = cv::Scalar(0, 165, 255);		
	const cv::Scalar Pink = cv::Scalar(255, 0, 255);		

}
#endif // !COLOR
