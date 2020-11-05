#include "Wrapper.h"
#include <opencv2/highgui/highgui.hpp>

cv::VideoCapture Wrapper::s_videoCapture;
cv::Mat Wrapper::s_frame;
cv::Mat Wrapper::s_grayFrame;

void Wrapper::init(const std::string& source) {
	s_videoCapture = cv::VideoCapture(source);
    if (!s_videoCapture.isOpened()) ErrorHandler::critical(ErrorHandler::ErrorCode::VideoCaptureError);
	Crop::init(s_videoCapture);
}

void Wrapper::run() {
    while (s_videoCapture.read(s_frame)) {

        auto t = new Timer("Loop");

        Crop::crop(s_frame);
        Color::rgb2Gray(s_frame, s_grayFrame);
        Blur::blur(s_grayFrame);
        Crop::ROI(s_grayFrame);
        Color::binarize(s_grayFrame);
        Crop::removeEdges(s_grayFrame);
        EdgeDetection::houghLinesP(s_grayFrame);
        EdgeDetection::classify();
        EdgeDetection::regression(s_grayFrame);
        EdgeDetection::average();

        delete t;

        //Debug & printing
        EdgeDetection::print(s_frame);
    }
}

