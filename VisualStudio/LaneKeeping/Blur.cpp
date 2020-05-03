#include "Blur.h"

void Blur::blur(cv::Mat& frame) {	
	cv::GaussianBlur(frame, frame, cv::Size(3, 3), 0, 0);
}
