#include "Color.h"

void Color::rgb2Gray(const cv::Mat& rgb, cv::Mat& gray) {
    cv::cvtColor(rgb, gray, cv::COLOR_RGB2GRAY);
}

void Color::binarize(cv::Mat& grayFrame) {
//#define CANNY
#ifdef CANNY

    cv::Canny(grayFrame, grayFrame, 70, 150);

#else

    cv::threshold(grayFrame, grayFrame, 140, 255, cv::THRESH_BINARY); //threshold trial & error

    cv::Point anchor = cv::Point(-1, -1);
    cv::Mat kernel = cv::Mat(1, 3, CV_32F);
    kernel.at<float>(0, 0) = -1;
    kernel.at<float>(0, 1) = 0;
    kernel.at<float>(0, 2) = 1;

    //filter the binary image to obtain the edges
    cv::filter2D(grayFrame, grayFrame, -1, kernel, anchor, 0, cv::BORDER_DEFAULT);

#endif // CANNY

}