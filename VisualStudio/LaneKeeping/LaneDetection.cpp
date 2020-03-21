#include "LaneDetection.h"

cv::Mat LaneDetection::s_frame;
int LaneDetection::s_frameCenter;
int LaneDetection::s_maxLineHeight;
std::array<std::array<cv::Point, 4>, LaneDetection::s_hystheresisCount> LaneDetection::s_hystheresisArray = { {} };
unsigned short LaneDetection::s_hystheresisArrayCounter = 0;
bool LaneDetection::s_hystheresisArrayFilled = false;
cv::Mat LaneDetection::s_mask;
std::vector<cv::Vec4i> LaneDetection::s_lines;
std::vector<cv::Point> LaneDetection::s_rightLinePoints;
std::vector<cv::Point> LaneDetection::s_leftLinePoints;
std::array<cv::Point, 4> LaneDetection::s_boundaries = {};

void LaneDetection::createMask(const cv::Size& frameSize, double frameFormat) {
    s_mask = cv::Mat::zeros(frameSize, frameFormat);

    const float hScale = 0.625;

    std::array<cv::Point, 4> points ({
        cv::Point(0.08 * frameSize.width,          frameSize.height),
        cv::Point(0.42 * frameSize.width, hScale * frameSize.height),
        cv::Point(0.56 * frameSize.width, hScale * frameSize.height),
        cv::Point(       frameSize.width,          frameSize.height)
    });

    //Create trapezoid mask
    cv::fillConvexPoly(s_mask, points.data(), 4, cv::Scalar(255, 0, 0));
}

inline void LaneDetection::applyMask() {
    cv::bitwise_and(s_frame, s_mask, s_frame);
}

inline void LaneDetection::changeContrast() {
    //cv::convertScaleAbs(s_frame, s_frame, 1.3, 0);
}

inline void LaneDetection::blur() {
    cv::GaussianBlur(s_frame, s_frame, cv::Size(7, 7), 0, 0); //7x7px trial & error
}

inline void LaneDetection::edgeDetection() {
    //TODO: ARM NEON optimization
    cv::cvtColor(s_frame, s_frame, cv::COLOR_RGB2GRAY);

    //binarize gray image
    //TODO: runtime calibration every x seconds
    cv::threshold(s_frame, s_frame, 140, 255, cv::THRESH_BINARY); //threshold trial & error

    /*
    Create the kernel [-1 0 1]
    This kernel is based on the one found in the
    Lane Departure Warning System by Mathworks
    */

    cv::Point anchor = cv::Point(-1, -1);
    cv::Mat kernel = cv::Mat(1, 3, CV_32F);
    kernel.at<float>(0, 0) = -1;
    kernel.at<float>(0, 1) = 0;
    kernel.at<float>(0, 2) = 1;

    //filter the binary image to obtain the edges
    cv::filter2D(s_frame, s_frame, -1, kernel, anchor, 0, cv::BORDER_DEFAULT);
}

inline void LaneDetection::houghLines() {
    s_lines.clear();

    //calibrate once every x seconds?
    HoughLinesP(s_frame, s_lines, 1, CV_PI / 180, 20, 20, 30); //rho & theta by trial & error
}

void LaneDetection::classifyLines() {
    s_rightLinePoints.clear();
    s_leftLinePoints.clear();

    const float minSlope = 0.3f;
    const float maxSlope = 1.5f;

    for (const auto& line : s_lines) {

        //slope = (y1 - y0) / (x1 - x0)
        float slope = static_cast<float>(line[3] - line[1]);
        slope /= ( static_cast<float>(line[2] - line[0]) + 0.00001f );

        //filter too horizontal slopes
        float absSlope = std::fabs(slope);
        if (absSlope < minSlope || absSlope > maxSlope) continue;

        if (slope > 0 && line[2] > s_frameCenter && line[0] > s_frameCenter) {
            s_rightLinePoints.push_back(cv::Point(line[0], line[1]));
            s_rightLinePoints.push_back(cv::Point(line[2], line[3]));
        } else if (slope < 0 && line[2] < s_frameCenter && line[0] < s_frameCenter) {
            s_leftLinePoints.push_back(cv::Point(line[0], line[1]));
            s_leftLinePoints.push_back(cv::Point(line[2], line[3]));
        }
    }
}

void LaneDetection::leastSquaresRegression() {

    std::array<float, 4> xPositions = { 0.f, 0.f, 0.f, 0.f };

    //fit left lane
    if (!s_leftLinePoints.empty()) {
        cv::Vec4d left_line;

        cv::fitLine(s_leftLinePoints , left_line, cv::DIST_L2, 0, 0.01, 0.01);
        float left_m = left_line[1] / left_line[0];
        cv::Point left_b = cv::Point(left_line[2], left_line[3]);

        xPositions[0] = (static_cast<float>(s_frame.rows - left_b.y) / left_m) + left_b.x; //lower
        xPositions[1] = (static_cast<float>(s_maxLineHeight - left_b.y) / left_m) + left_b.x; //upper
    }


    //fit right lane
    if (!s_rightLinePoints.empty()) {
        cv::Vec4d right_line;

        cv::fitLine(s_rightLinePoints, right_line, cv::DIST_L2, 0, 0.01, 0.01);
        float right_m = right_line[1] / right_line[0];
        cv::Point right_b = cv::Point(right_line[2], right_line[3]); // y = m*x + b

        xPositions[2] = (static_cast<float>(s_frame.rows - right_b.y) / right_m) + right_b.x; //lower
        xPositions[3] = (static_cast<float>(s_maxLineHeight - right_b.y) / right_m) + right_b.x; //upper
    }


    hystheresis(xPositions);
}

inline void LaneDetection::hystheresis(std::array<float, 4> xPositions) {

    s_hystheresisArray[s_hystheresisArrayCounter][0] = cv::Point(xPositions[0], s_frame.rows);
    s_hystheresisArray[s_hystheresisArrayCounter][1] = cv::Point(xPositions[1], s_maxLineHeight);
    s_hystheresisArray[s_hystheresisArrayCounter][2] = cv::Point(xPositions[2], s_frame.rows);
    s_hystheresisArray[s_hystheresisArrayCounter][3] = cv::Point(xPositions[3], s_maxLineHeight);

    if (s_hystheresisArrayFilled) {

        std::array<cv::Point, 4> previousRow;
        if (s_hystheresisArrayCounter == 0) {
            previousRow = s_hystheresisArray[s_hystheresisCount - 1];
        } else {
            previousRow = s_hystheresisArray[s_hystheresisArrayCounter - 1];
        }

        const int maxLowerDiff = 0.01f * s_frame.cols;
        const int maxUpperDiff = 0.004f * s_frame.cols;

        std::array<int, 4> maxDiff = { maxLowerDiff, maxUpperDiff, maxLowerDiff, maxUpperDiff };
        std::array<int, 4> avgXPositions = {};

        //unsigned short attempts = 0;

        //average over whole array (excluding 0-values)
        for (unsigned short i = 0; i < 4; i++) {
            unsigned short skipped = 0;
            int avg = 0;

            for (unsigned short j = 0; j < s_hystheresisCount; j++) {

                if (s_hystheresisArray[j][i].x == 0) skipped++;
                avg += s_hystheresisArray[j][i].x;
            }

            avgXPositions[i] = avg;
            if (s_hystheresisCount == skipped) {
                errorHanlder();
                return;
            }
            avgXPositions[i] /= (s_hystheresisCount - skipped);
            /*
            bool reCalculate = false;
            for (unsigned short j = 0; j < s_hystheresisCount; j++) {

                if (s_hystheresisArray[j][i].x != 0 && s_hystheresisArray[j][i].x * 5 < avg) {
                    reCalculate = true;
                    break;
                }
            }

            if (reCalculate && attempts < 5) {
                std::cout << "Recalculating...\n";
                i--;
                attempts++;
                continue;
            } else {
                attempts = 0;
            }
            */
            int diff = avgXPositions[i] - previousRow[i].x;
            if (diff > maxDiff[i]) {
                avgXPositions[i] = previousRow[i].x + maxDiff[i];
            } else if (diff < -1 * maxDiff[i]) {
                avgXPositions[i] = previousRow[i].x - maxDiff[i];
            }
        }

        s_hystheresisArray[s_hystheresisArrayCounter][0] = cv::Point(avgXPositions[0], s_frame.rows);
        s_hystheresisArray[s_hystheresisArrayCounter][1] = cv::Point(avgXPositions[1], s_maxLineHeight);
        s_hystheresisArray[s_hystheresisArrayCounter][2] = cv::Point(avgXPositions[2], s_frame.rows);
        s_hystheresisArray[s_hystheresisArrayCounter][3] = cv::Point(avgXPositions[3], s_maxLineHeight);
    }

    s_boundaries[0] = s_hystheresisArray[s_hystheresisArrayCounter][0];
    s_boundaries[1] = s_hystheresisArray[s_hystheresisArrayCounter][1];
    s_boundaries[3] = s_hystheresisArray[s_hystheresisArrayCounter][2];
    s_boundaries[2] = s_hystheresisArray[s_hystheresisArrayCounter][3];


    s_hystheresisArrayCounter++;
    if (s_hystheresisArrayCounter == s_hystheresisCount) {
        s_hystheresisArrayCounter = 0;
        s_hystheresisArrayFilled = true;
    }
}

void LaneDetection::errorHanlder() {
    std::cerr << "An error has occured!\n";
}

void LaneDetection::prepare(const cv::Size& frameSize, double frameFormat) {
    createMask(frameSize, frameFormat);
    s_frameCenter = frameSize.width / 2;
    s_maxLineHeight = static_cast<int>(0.66f * frameSize.height);
}

void LaneDetection::setFrame(const cv::Mat& frame) {
	s_frame = frame;
}

void LaneDetection::process(cv::Mat& frame) {
    setFrame(frame);
    changeContrast();
    blur(); //remove noise by blurring image
    edgeDetection(); //detect edges
    applyMask(); //crop ROI
    houghLines(); // use HoughLinesP


    cv::imshow("asd",s_frame);
    cv::waitKey(1);
    if (!s_lines.empty()) {
        classifyLines(); //Classify which lines are for left or right lane 
        leastSquaresRegression(); //calculate lane regression
        display(frame);
    } else {
        errorHanlder();
    }

}

void LaneDetection::display(cv::Mat& frame) {

    cv::Mat output;
    frame.copyTo(output);

    //create semi-transparent trapezoid
    cv::fillConvexPoly(output, s_boundaries.data(), 4, cv::Scalar(255, 255, 255), cv::LINE_AA, 0);
    cv::addWeighted(output, 0.4, frame, 0.6, 0, frame);

    //draw left & right lane
    cv::line(frame, s_boundaries[0], s_boundaries[1], cv::Scalar(255, 255, 255), 7, cv::LINE_AA);
    cv::line(frame, s_boundaries[2], s_boundaries[3], cv::Scalar(255, 255, 255), 7, cv::LINE_AA);
    
    //display processed frame
    cv::imshow("Lane detection", frame);
    cv::waitKey(1);
}

