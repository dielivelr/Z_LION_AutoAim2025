#ifndef _IMAGESHOWER_H_
#define _IMAGESHOWER_H_
#include<opencv2/opencv.hpp>
#include<mutex>
namespace rm
{
	/************************
	* 图像放映机
	* 多线程运行
	************************/
	class ImageShower {
	public:
		ImageShower();
		~ImageShower();
	public:
		// 主线程
		void run();

		// 关闭线程
		void close();

		// 放入图像
		void set_image(cv::Mat src);
		
	private:
		cv::Mat last_image;
		bool close_image_show;
		std::mutex lock_image;
	};
};
#endif // !_IMAGESHOWER_H_
