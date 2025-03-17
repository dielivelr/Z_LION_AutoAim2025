#include"VideoDownloader.h"
namespace rm{

    VideoDownloader::VideoDownloader(float recording_time, const std::string& video_load_path,
			int fps, int whole_width, int whole_height, bool avi)
        :down_load_die(false)
    {
        std::string name = std::to_string(float(get_now_time()));
        if (avi) {
            name = name + ".avi";
            this->Writer = new cv::VideoWriter(video_load_path + "/" + name, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                fps, cv::Size(whole_width, whole_height), true);
        }
        else
        {
            name = name + ".mp4";
            this->Writer = new cv::VideoWriter(video_load_path + "/" + name, cv::VideoWriter::fourcc('H', '2', '6', '4'),
                fps, cv::Size(whole_width, whole_height), true);
        }
        // 若recording_time = -1,设置为无限模式
        writer_iterations = recording_time == -1 ? -1 : recording_time * fps;
    };

    VideoDownloader::~VideoDownloader() {
        if (Writer != NULL) {
            delete Writer;
        }
    };

    void VideoDownloader::load_in_video(const cv::Mat& src)
    {
        if (this->down_load_die)return;
        if (src.empty()) {
            //std::cerr << "Error: Blank frame grabbed." << std::endl;
            return;
        }
        if (writer_iterations == 0) {
            std::cerr << "==============================" << std::endl;
            std::cerr << "Warn: writer_iterations IS FULL." << std::endl;
            std::cerr << "==============================" << std::endl;
            this->down_load_die = true;
            return;
        }
        this->Writer->write(src);
        if (writer_iterations != -1)
        {
            writer_iterations--;
            if (writer_iterations == 0) {
                this->Writer->release();
            }
        }
    };
}