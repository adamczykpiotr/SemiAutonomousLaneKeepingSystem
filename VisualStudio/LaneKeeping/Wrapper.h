#pragma once
#include <opencv2/opencv.hpp>

#include "Crop.h"
#include "Blur.h"
#include "Color.h"
#include "EdgeDetection.h"
#include "ErrorHandler.h"

#include "Timer.h"

class Wrapper {

	static cv::VideoCapture s_videoCapture;
	static cv::Mat s_frame;
	static cv::Mat s_grayFrame;

public:
	static void init(const std::string& source);
	static void run();
};

