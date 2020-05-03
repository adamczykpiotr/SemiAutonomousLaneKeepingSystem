#pragma once
#include <vector>
#include <opencv2/opencv.hpp>

#include "Crop.h"

class EdgeDetection {

    static std::vector<cv::Vec4i> s_lines;
    static std::vector<cv::Point> s_rightLines;
    static std::vector<cv::Point> s_leftLines;
    static std::array<cv::Point, 4> s_boundaries; //????
    static cv::Vec6f s_laneCoefficients;

public:
    static void houghLinesP(cv::Mat& frame);
    static void classify();
    static void regression();

    static void print(const cv::Mat& frame);
        
};

