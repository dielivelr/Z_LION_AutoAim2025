#ifndef _VIDEO_H_
#define _VIDEO_H_
#include<opencv2/opencv.hpp>
#include<iostream>
#include<vector>
namespace rm
{
	class Video
	{
	public:
		Video();
		~Video();
		Video(std::vector<std::string>& paths);
		Video(const int video_id);
		// 获取图像
		cv::Mat ReadPic();
	private:
		// 切换目标
		std::string next_path();
		// 开启相机
		bool open_video();
		// 关闭相机
		void close_video();
		// 路径
		const std::vector<std::string> paths;
		cv::VideoCapture* video;
		int id = -1;
	};
};
#endif // !_VIDEO_H_