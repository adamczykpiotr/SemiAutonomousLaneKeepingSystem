#include "EdgeDetection.h"

std::vector<cv::Vec4i> EdgeDetection::s_lines;
std::vector<cv::Point> EdgeDetection::s_rightLines;
std::vector<cv::Point> EdgeDetection::s_leftLines;
cv::Vec4f EdgeDetection::s_lanePoints;

uint8_t EdgeDetection::averageSampleIndex = 0;
bool EdgeDetection::averageAvailable = false;
std::array<cv::Vec4f, EdgeDetection::averageSampleCount> EdgeDetection::averageSamples;

void EdgeDetection::houghLinesP(cv::Mat& frame){
    s_lines.clear();

    // CV_8UC1 mat
    //calibrate once every x seconds?
    //InputArray image, OutputArray lines, double rho, double theta, int threshold,double minLineLength = 0, double maxLineGap = 0 );
    cv::HoughLinesP(frame, s_lines, 1, CV_PI / 180, 20, 50, 30); //convert px values to % of width
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

void EdgeDetection::regression(cv::Mat& frame) {

    auto interpolate = [](float x, float a, float x0, float y0) {
        return static_cast<int>(a * (x - x0) + y0);
    };
    auto size = Crop::getCroppedSize();

    //fit left lane
    if (!s_leftLines.empty()) {
        cv::Vec4f leftLane;
        cv::fitLine(s_leftLines, leftLane, cv::DIST_L2, 0, 0.01, 0.01);

        float a = leftLane[1] / leftLane[0];
        //s_lanePoints[0] = a;
        s_lanePoints[0] = interpolate(0, a, leftLane[2], leftLane[3]); //y for x=0
        s_lanePoints[1] = interpolate(size.width, a, leftLane[2], leftLane[3]); //y for x=frame.width
    }

    //fit right lane
    if (!s_rightLines.empty()) {
        cv::Vec4f rightLane;
        cv::fitLine(s_rightLines, rightLane, cv::DIST_L2, 0, 0.01, 0.01);

        float a = rightLane[1] / rightLane[0];
        //s_lanePoints[3] = a;
        s_lanePoints[2] = interpolate(0, a, rightLane[2], rightLane[3]);
        s_lanePoints[3] = interpolate(size.width, a, rightLane[2], rightLane[3]);
    }
}

void EdgeDetection::average() {

    for (uint8_t i = 0; i < 4; i++) averageSamples[averageSampleIndex].val[i] = s_lanePoints[i];
    
    if (averageAvailable) {
        const cv::Vec2f maxSteps = { 0.1, 0.1 };

        //get rolling average of all coefficients
        float average;
        for (uint8_t i = 0; i < 4; i++) {
            average = 0;

            for (int j = 0; j < averageSamples.size(); j++) {
                if (averageSampleIndex == j) continue;
                average += averageSamples[j].val[i];
            }
            average /= (averageSampleCount - 1);
            average = 0.9 * average + 0.1 * averageSamples[averageSampleIndex].val[i];

            //hystheresis
            //float diff = average - s_lanePoints[i];
            //float maxStep = maxSteps[i % 2];

            averageSamples[averageSampleIndex].val[i] = average;
            s_lanePoints[i] = average;
        }
    }

    averageSampleIndex++;
    if (averageSampleIndex == averageSampleCount) {
        averageAvailable = true;
        averageSampleIndex = 0;
    }
}


void EdgeDetection::print(const cv::Mat& frame) {
       
    cv::Mat output(frame);

    auto size = Crop::getCroppedSize();
    for (int i = 0; i < averageSampleCount; i+=2) {
        cv::line(output, cv::Point(0, averageSamples[i].val[0]), cv::Point(size.width, averageSamples[i].val[1]), cv::Scalar(255, 0, 255), 3, cv::LINE_AA);
        cv::line(output, cv::Point(0, averageSamples[i].val[2]), cv::Point(size.width, averageSamples[i].val[3]), cv::Scalar(0, 255, 255), 3, cv::LINE_AA);
    }


    //center line
    auto center = Crop::getFrameCenter();
    cv::line(output, cv::Point(center, 0), cv::Point(center, size.height), cv::Scalar(0, 255, 0), 1, cv::LINE_AA);

    //lanes
    auto interpolate = [](float x, float a, float x0, float y0) {
        return static_cast<int>( a * (x - x0) + y0 ); 
    };
    cv::Point a = cv::Point(0, s_lanePoints[0]);
    cv::Point b = cv::Point(size.width, s_lanePoints[1]);

    cv::Point c = cv::Point(0, s_lanePoints[2]);
    cv::Point d = cv::Point(size.width, s_lanePoints[3]);

    cv::line(output, a, b, cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
    cv::line(output, c, d, cv::Scalar(255, 255, 255), 3, cv::LINE_AA);   /*

#define showPoints
#ifdef showPoints

    for (const auto& point : s_leftLines) {
        cv::circle(output, point, 5, cv::Scalar(255, 0, 255), 1);
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

#endif
*/
   
    //display processed frame
    cv::imshow("Lines", output);
    cv::waitKey(1);
}