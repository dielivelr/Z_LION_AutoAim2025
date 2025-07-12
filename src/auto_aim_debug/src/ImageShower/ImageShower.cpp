#include"ImageShower.h"

namespace rm
{
    ImageShower::ImageShower() {
			close_image_show = false;
	};
    ImageShower::~ImageShower() {
        close_image_show = true;
    };
    void ImageShower::close() {
		close_image_show = true;
	};

    void ImageShower::set_image(cv::Mat src)
    {
        lock_image.lock();
        last_image = src;
        lock_image.unlock();
    };

    void ImageShower::run()
    {
        cv::Mat show_image;
        while (1)
        {
            // 互斥访问结束标志位
            if (close_image_show) break;

            // 互斥访问图像
            lock_image.lock();
            show_image = last_image;
            lock_image.unlock();
            if (!show_image.empty())
            {
                cv::imshow("ImageShower", show_image);
                cv::waitKey(1);
            };

        };
    };
}