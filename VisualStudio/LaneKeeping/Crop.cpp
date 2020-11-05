#include "Crop.h"

//calib
const cv::Vec2f Crop::s_scale(0.9f, 0.38f);
const cv::Vec2f Crop::s_offset(0.05f, -0.05f);
const int Crop::s_centerOffset = -30;


cv::Size Crop::s_originalSize;
cv::Size Crop::s_croppedSize;
cv::Rect Crop::s_croppingRect;
cv::Mat Crop::s_mask;
std::array<cv::Point, 4> Crop::s_maskPoints;
int Crop::s_centerPosition;

void Crop::init(const cv::VideoCapture& capture) {
    s_originalSize = cv::Size(
        static_cast<int>( capture.get(cv::CAP_PROP_FRAME_WIDTH) ),
        static_cast<int>( capture.get(cv::CAP_PROP_FRAME_HEIGHT) )
    );

    s_croppingRect = cv::Rect(
        s_originalSize.width * s_offset[0],                        //width offset (left)
        (1.f - s_scale[1] + s_offset[1]) * s_originalSize.height,  //height offset (top)
        s_scale[0] * s_originalSize.width,                         //width
        s_scale[1] * s_originalSize.height                         //height
    );

    //create ROI mask to crop image
    s_croppedSize = s_originalSize;
    s_croppedSize.height *= s_scale[1];
    s_croppedSize.width *=  s_scale[0];

    //create trapezoidal mask
    s_mask = cv::Mat::zeros(s_croppedSize, CV_8U);                             /*     ______________________     */
    s_maskPoints[0] = cv::Point(0, s_croppedSize.height                  );    /*     |  /              \  |     */
    s_maskPoints[1] = cv::Point(0.48 * s_croppedSize.width, 0            );    /*     | /                \ |     */
    s_maskPoints[2] = cv::Point(0.52 * s_croppedSize.width, 0            );    /*     |/                  \|     */
    s_maskPoints[3] = cv::Point(s_croppedSize.width, s_croppedSize.height);    /*     ̅ ̅ ̅ ̅ ̅ ̅ ̅ ̅ ̅ ̅ ̅ ̅ ̅ ̅ ̅ ̅ ̅ ̅ ̅ ̅ ̅ ̅      */
   

    //Fill binary mask
    cv::fillConvexPoly(s_mask, s_maskPoints.data(), 4, cv::Scalar(255, 0, 0));

    //calculate real line center
    s_centerPosition = s_croppedSize.width / 2 + s_centerOffset;
}

void Crop::crop(cv::Mat& frame) {
    frame = frame(s_croppingRect); 
}

void Crop::ROI(cv::Mat& grayFrame) {
    cv::bitwise_and(grayFrame, s_mask, grayFrame);
}

void Crop::removeEdges(cv::Mat& grayFrame) {
   cv::line(grayFrame, s_maskPoints[0], s_maskPoints[1], cv::Scalar(0, 0, 0), 2, cv::LINE_4);
   cv::line(grayFrame, s_maskPoints[2], s_maskPoints[3], cv::Scalar(0, 0, 0), 2, cv::LINE_4); //line_4 vs line_8 (default) vs cv::FILLED?? Perf diff
}

int Crop::getFrameCenter() {
    return s_centerPosition;
}

//unused for now
cv::Size Crop::getCroppedSize() {
    return s_croppedSize;
}