#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <array>

#define displayFrames

class LaneDetection {

	#define USE_HYSTHERESIS

	static cv::Mat s_frame;
	static int s_frameCenter;
	static int s_maxLineHeight;

	static const unsigned short s_hystheresisCount = 6;
	static std::array<std::array<cv::Point, 4>, s_hystheresisCount> s_hystheresisArray;
	static unsigned short s_hystheresisArrayCounter;
	static bool s_hystheresisArrayFilled;

	static cv::Mat s_mask;
	static std::vector<cv::Vec4i> s_lines;

	static std::vector<cv::Point> s_rightLinePoints;
	static std::vector<cv::Point> s_leftLinePoints;
	static std::array<cv::Point, 4> s_boundaries;

	static void createMask(const cv::Size& frameSize, double frameFormat);
	static inline void applyMask();
	static inline void changeContrast();
	static inline void blur();
	static inline void edgeDetection();
	static inline void houghLines();
	static void classifyLines();
	static void leastSquaresRegression();
	static inline void hystheresis(std::array<float, 4> xPositions);

	static void errorHanlder();

public:
	static void prepare(const cv::Size& frameSize, double frameFormat);
	static void setFrame(const cv::Mat& frame);
	static void process(cv::Mat& frame);
	static void display(cv::Mat& frame);

	static float getOffset();
};

