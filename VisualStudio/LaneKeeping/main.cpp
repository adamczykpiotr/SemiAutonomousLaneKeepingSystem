#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include "Wrapper.h"

#include "LaneDetection.h"
#include "Timer.h"

int main(int argc, const char** argv) {

    std::string src = "C://Users//adamc//Desktop//pulpit//output.mp4";
    if (argc > 1 && strlen(argv[1]) > 0) src = std::string(argv[1]);

    Wrapper::init(src);
    Wrapper::run();

    /*

    cv::VideoCapture cap(src);
    if (!cap.isOpened()) return -1;
    //LEGACY

    //Set up lane detection parameters
    cv::Size frameSize = cv::Size(
        static_cast<int>( cap.get(cv::CAP_PROP_FRAME_WIDTH) ), 
        static_cast<int>( cap.get(cv::CAP_PROP_FRAME_HEIGHT) )
    );

    
    double frameFormat = cap.get(cv::CAP_PROP_FORMAT);
    LaneDetection::prepare(frameSize, frameFormat);

    cv::Mat frame;

    auto globalTimer = new Timer("Loop", static_cast<long long>( cap.get(cv::CAP_PROP_FRAME_COUNT) ) );

    while (cap.read(frame)) {
        auto t = new Timer("Loop");

        LaneDetection::process(frame);

        delete t;
    }
    delete globalTimer;
    */
 
}
