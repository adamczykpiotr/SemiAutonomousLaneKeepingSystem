#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
	cv::Mat img = cv::Scalar(255,0,0);
	cv::imshow("Red", img);	
	cv::waitKey(1);
	std::cin.get();

	return 0;
}