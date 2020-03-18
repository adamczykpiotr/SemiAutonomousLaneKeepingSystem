#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
	cv::Mat img = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 255));
	cv::imshow("640x480 Red image", img);
	cv::waitKey(0);

	return 0;
}
