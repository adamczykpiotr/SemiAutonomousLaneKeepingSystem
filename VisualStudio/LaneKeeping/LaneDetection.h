#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <array>

class LaneDetection {

	static cv::Mat s_frame;	//Current frame
	static int s_frameCenter;
	static int s_maxLineHeight;

	static cv::Mat s_mask;
	static std::vector<cv::Vec4i> s_lines;

	static std::vector<cv::Point> s_rightLinePoints;
	static std::vector<cv::Point> s_leftLinePoints;
	static std::array<cv::Point, 4> s_boundaries;

	static void createMask(const cv::Size& frameSize, double frameFormat);
	static inline void applyMask();
	static inline void blur();
	static inline void edgeDetection();
	static inline void houghLines();
	static void classifyLines();
	static void leastSquaresRegression();


public:
	static void prepare(const cv::Size& frameSize, double frameFormat);
	static void setFrame(const cv::Mat& frame);
	static void process(cv::Mat& frame);
	static void display(cv::Mat& frame);


};

