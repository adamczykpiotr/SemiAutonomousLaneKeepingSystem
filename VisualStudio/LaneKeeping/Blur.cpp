#include "Blur.h"

void Blur::blur(cv::Mat& frame) {
    const int size = 3;

#define PARALLEL_BLUR
#ifdef PARALLEL_BLUR

    cv::Mat in1(frame, cv::Rect(0, 0, frame.cols, frame.rows / 2 ));
    cv::Mat out1(frame, cv::Rect(0, 0, frame.cols, frame.rows / 2 ));

    cv::Mat in2(frame, cv::Rect(0, frame.rows / 2 , frame.cols, frame.rows / 2 ));
    cv::Mat out2(frame, cv::Rect(0, frame.rows/ 2, frame.cols, frame.rows / 2 ));

    std::thread t1([&] {
        cv::GaussianBlur(in1, out1, cv::Size(size, size), 0, 0);
    });
    std::thread t2([&] {
        cv::GaussianBlur(in2, out2, cv::Size(size, size), 0, 0);
    });
    t1.join();
    t2.join();

#else
    cv::GaussianBlur(frame, frame, cv::Size(size, size), 0, 0);
#endif

    cv::imshow("frame", frame);
    cv::waitKey(1);

}
