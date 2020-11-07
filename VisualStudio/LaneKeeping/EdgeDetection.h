#pragma once
#include <vector>
#include <opencv2/opencv.hpp>

#include "Crop.h"

class EdgeDetection {
    static std::vector<cv::Vec4i> s_lines;
    static std::vector<cv::Point> s_rightLines;
    static std::vector<cv::Point> s_leftLines;
    static cv::Vec4f s_lanePoints;


    static const uint8_t averageSampleCount = 5;
    static uint8_t averageSampleIndex;
    static std::array<cv::Vec4f, averageSampleCount> averageSamples;
    static bool averageAvailable;

public:
    static void houghLinesP(cv::Mat& frame);
    static void classify();
    static void regression(cv::Mat& frame);
    static void average();

    static int32_t getCenterOffset();

    static void print(const cv::Mat& frame);        
};

