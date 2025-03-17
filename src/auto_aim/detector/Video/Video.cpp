#include "Video.h"

rm::Video::Video()
	:paths(std::vector<std::string>())
{
	video = new cv::VideoCapture();
}
rm::Video::~Video()
{
	if (video != nullptr) delete video;
}
rm::Video::Video(std::vector<std::string>& paths)
	:paths(paths)
{
	video = new cv::VideoCapture();
}
rm::Video::Video(const int video_id)
{
	video = new cv::VideoCapture(video_id);
}
cv::Mat rm::Video::ReadPic()
{
open_video:
	if (open_video())
	{
		cv::Mat img;
		*video >> img; // 录入
		if (img.empty()) {
			// 当前视频为空
			close_video();
			goto open_video;
		}
		return img;
	}
	else {
		std::cout << "video is over!!!" << std::endl;
		return cv::Mat();
	}
}
std::string rm::Video::next_path()
{
	id++;
	if (id >= paths.size()) id = 0;
	return paths[id];
}

bool rm::Video::open_video()
{
	if (!video->isOpened()) {
		std::string path = next_path();
		if (path == "") {
			return false;
		}
		video->open(path);
	};
	return true;
}

void rm::Video::close_video()
{
	video->release();
}
