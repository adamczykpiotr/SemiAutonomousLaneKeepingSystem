#include "EdgeDetection.h"

std::vector<cv::Vec4i> EdgeDetection::s_lines;
std::vector<cv::Point> EdgeDetection::s_rightLines;
std::vector<cv::Point> EdgeDetection::s_leftLines;
std::array<cv::Point, 4> EdgeDetection::s_boundaries;
cv::Vec6f EdgeDetection::s_laneCoefficients;

void EdgeDetection::houghLinesP(cv::Mat& frame){
    s_lines.clear();

    // CV_8UC1 mat
    //calibrate once every x seconds?
    //InputArray image, OutputArray lines, double rho, double theta, int threshold,double minLineLength = 0, double maxLineGap = 0 );
    cv::HoughLinesP(frame, s_lines, 1, CV_PI / 180, 20, 50, 5); //rho & theta by trial & error
}

void EdgeDetection::classify() {
    if (s_lines.empty()) return;

    s_rightLines.clear();
    s_leftLines.clear();

    const float minSlope = 0.3f;
    const float maxSlope = 1.5f;

    const int frameCenter = Crop::getFrameCenter();

    for (const auto& line : s_lines) {

        //slope = (y1 - y0) / (x1 - x0)
        float slope = static_cast<float>(line[3] - line[1]);
        slope /= (static_cast<float>(line[2] - line[0]) + 0.00001f);

        //filter too horizontal slopes
        float absSlope = std::fabs(slope);
        if (absSlope < minSlope || absSlope > maxSlope) continue;

        if (slope > 0 && line[2] > frameCenter && line[0] > frameCenter) {
            s_rightLines.push_back(cv::Point(line[0], line[1]));
            s_rightLines.push_back(cv::Point(line[2], line[3]));
        } else if (slope < 0 && line[2] < frameCenter && line[0] < frameCenter) {
            s_leftLines.push_back(cv::Point(line[0], line[1]));
            s_leftLines.push_back(cv::Point(line[2], line[3]));
        }
    }
}

void EdgeDetection::regression() {

    std::array<float, 4> xPositions = { 0.f, 0.f, 0.f, 0.f };

    auto size = Crop::getCroppedSize();

    cv::Vec4f leftLane;
    cv::Vec4f rightLane;

    //fit left lane
    if (!s_leftLines.empty()) {
        cv::fitLine(s_leftLines, leftLane, cv::DIST_L2, 0, 0.01, 0.01);

        float a = leftLane[1] / leftLane[0];
        
        s_laneCoefficients[0] = a;
        s_laneCoefficients[1] = leftLane[2];
        s_laneCoefficients[2] = leftLane[3];
    }


    //fit right lane
    if (!s_rightLines.empty()) {
        cv::fitLine(s_rightLines, rightLane, cv::DIST_L2, 0, 0.01, 0.01);
         
        float a = rightLane[1] / rightLane[0];

        s_laneCoefficients[3] = a;
        s_laneCoefficients[4] = rightLane[2];
        s_laneCoefficients[5] = rightLane[3];
    }
}



float f(float x, float a, float x0, float y0) {
    return a * (x - x0) + y0;
}


void EdgeDetection::print(const cv::Mat& frame) {
       
    cv::Mat output(frame);

    for (const auto& point : s_leftLines) {
        cv::circle(output, point, 5, cv::Scalar(255, 0, 255), 5);
    }

    if (s_leftLines.size() > 1) {
        for (int i = 0; i < s_leftLines.size() - 1; i += 2) {
            cv::line(output, s_leftLines[i], s_leftLines[i + 1], cv::Scalar(255, 0, 255), 3, cv::LINE_AA);
        }
    }

    for (const auto& point : s_rightLines) {
        cv::circle(output, point, 5, cv::Scalar(255, 255, 0), 5);
    }

    if (s_rightLines.size() > 1) {
        for (int i = 0; i < s_rightLines.size() - 1; i += 2) {
            cv::line(output, s_rightLines[i], s_rightLines[i + 1], cv::Scalar(255, 255, 0), 3, cv::LINE_AA);
        }

    }



    //center line
    auto size = Crop::getCroppedSize();
    auto center = Crop::getFrameCenter();
    cv::line(output, cv::Point(center, 0), cv::Point(center, size.height), cv::Scalar(0, 255, 0), 1, cv::LINE_AA);

    auto z = Crop::getCroppedSize();

    /*
    auto a1 = s_laneCoefficients[0];
    auto a2 = s_laneCoefficients[3];

    auto x01 = s_laneCoefficients[1];
    auto x02 = s_laneCoefficients[4];

    auto y01 = s_laneCoefficients[2];
    auto y02 = s_laneCoefficients[5];


    //cross point
    float xCross = (y02 - y01 + a1 * x01 - a2 * x02) / (a1 - a2);

    std::cout << f(xCross, a1, x01, y01) << "\t";
    std::cout << f(xCross, a2, x02, y02) << "\n";
    */

    cv::Point a = cv::Point(0, f(0, s_laneCoefficients[0], s_laneCoefficients[1], s_laneCoefficients[2]));
    cv::Point b = cv::Point(z.width, f(z.width, s_laneCoefficients[0], s_laneCoefficients[1], s_laneCoefficients[2]));

    cv::Point c = cv::Point(0, f(0, s_laneCoefficients[3], s_laneCoefficients[4], s_laneCoefficients[5]));
    cv::Point d = cv::Point(z.width, f(z.width, s_laneCoefficients[3], s_laneCoefficients[4], s_laneCoefficients[5]));

    cv::line(output, a, b, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
    cv::line(output, c, d, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);


    //cv::Point y[4] = { a,b,c,d };
    //cv::fillConvexPoly(output, y, 4, cv::Scalar(255, 255, 255), cv::LINE_AA, 0);


    //display processed frame
    cv::imshow("Lines", output);
    cv::waitKey(1);
}
