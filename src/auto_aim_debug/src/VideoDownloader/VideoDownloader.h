#ifndef _VIDEODOWNLOADER_H_
#define _VIDEODOWNLOADER_H_
#include"../Tools/Tools.h"
#include<opencv2/opencv.hpp>
#include<iostream>
#include<string>
namespace rm
{
	/************************
	* 视频下载器
	************************/
	class VideoDownloader
	{
	public:
		VideoDownloader() = default;
	 	VideoDownloader(float recording_time, const std::string& video_load_path,
			int fps, int whole_width, int whole_height, bool avi = true);
		~VideoDownloader();
		// 下载图片
		void load_in_video(const cv::Mat& src);
	private:
		cv::VideoWriter* Writer;
		int writer_iterations; // 迭代数
		bool down_load_die;
	};
}
#endif // !_VIDEODOWNLOADER_H_
