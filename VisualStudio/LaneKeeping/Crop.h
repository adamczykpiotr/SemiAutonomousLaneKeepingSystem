#pragma once
#include <opencv2/opencv.hpp>

class Crop {
	static const cv::Vec2f s_scale;
	static const cv::Vec2f s_offset;
	static const int s_centerOffset; //offset from image center to real center

	static cv::Size s_originalSize;
	static cv::Size s_croppedSize;

	static cv::Rect s_croppingRect;
	static cv::Mat s_mask;
	static std::array<cv::Point, 4> s_maskPoints;

	static int s_centerPosition;

public:
	static void init(const cv::VideoCapture& capture);

	static void crop(cv::Mat& frame);
	static void ROI(cv::Mat& grayframe);
	static void binarize(cv::Mat& grayFrame);
	static void removeEdges(cv::Mat& grayFrame);

	static int getFrameCenter();

	static cv::Size getCroppedSize();
	static int getTopOffset();
};

