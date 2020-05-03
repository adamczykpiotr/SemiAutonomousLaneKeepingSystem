#pragma once
#include <opencv2/opencv.hpp>
#include "Timer.h"

class Color {
public:
	static void rgb2Gray(const cv::Mat& rgb, cv::Mat& gray);
	static void binarize(cv::Mat& grayFrame);
};