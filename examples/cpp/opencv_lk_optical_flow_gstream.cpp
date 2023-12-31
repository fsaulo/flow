#include <iostream>
#include <chrono>
#include <thread>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>

#define PI 3.14159265358979323846

int main(int argc, char* argv[])
{

    const std::string keys =
        "{ h help |      | print this help message }"
        "{ @image | vtest.avi | path to image file }";

    cv::CommandLineParser parser(argc, argv, keys);
    std::string pipeline = parser.get<std::string>("@image");
    if (!parser.check())
    {
        parser.printErrors();
        return 0;
    }

    cv::VideoCapture video(pipeline, cv::CAP_GSTREAMER);
    if (!video.isOpened())
    {
        std::cout << "Failed to open the video file!\n";
        return -1;
    }

    cv::Mat prevFrame, currFrame;
    cv::Mat flow;

    std::vector<cv::Point2f> prevPoints, currPoints;
    std::vector<uchar> status;
    std::vector<float> error;

    video >> prevFrame;

    float target_width = 640;
    float imratio = target_width / prevFrame.size().width;
    cv::Size size = cv::Size((int) target_width, int(prevFrame.size().height * imratio));

    cv::resize(prevFrame, prevFrame, size, 0, 0, cv::INTER_LINEAR);
    cv::cvtColor(prevFrame, prevFrame, cv::COLOR_BGR2GRAY);
    cv::goodFeaturesToTrack(prevFrame, prevPoints, 500, 0.01, 10);

    double cx = 320.5;
    double cy = 240.5;
    double focalLength = 277.191356;

    // cv::Mat priorPosition = (cv::Mat_<double>(2, 1) << 0, 0, 0);
    cv::Point2d priorPosition(0, 0);
    cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << 0, 0, 0, 0, 0);
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) <<
        focalLength, 0, cx,
        0, focalLength, cy,
        0, 0, 1);

    double maxVelocityThreshold = 10;
    double prevX = 250;
    double prevY = 250;
    double x, y;

    bool pause = false;
    bool filterByStatus = true;
    bool filterByVelocity = false;

    x = y = prevX;
    std::vector<cv::Point2f> filteredPrevPts, filteredCurrPts;
    cv::Mat trajectory = cv::Mat::zeros(480, 640, CV_8UC3);

    int loop_counter = 0;
    auto start = std::chrono::high_resolution_clock::now();

    while (video.read(currFrame))
    {
        try {
            cv::resize(currFrame, currFrame, size, 0, 0, cv::INTER_LINEAR);
            cv::cvtColor(currFrame, currFrame, cv::COLOR_BGR2GRAY);
            cv::calcOpticalFlowPyrLK(prevFrame, currFrame, prevPoints, currPoints, status, error);

            double sumMagnitude = 0.0;
            int count = 0;
            for (size_t i = 0; i < status.size(); ++i) {
                double magnitude = cv::norm(prevPoints[i] - currPoints[i]);
                sumMagnitude += magnitude;
                count++;
            }

            double averageMagnitude = sumMagnitude / count;
            for (size_t i = 0; i < status.size(); ++i) {
                if (!status[i]) {
                    continue;
                }

                double velocity = cv::norm(prevPoints[i] - currPoints[i]);
                if (velocity > averageMagnitude + maxVelocityThreshold && filterByVelocity) {
                    continue;
                }

                filteredPrevPts.push_back(prevPoints[i]);
                filteredCurrPts.push_back(currPoints[i]);
            }

            if (filteredCurrPts.size() < 4 || filteredPrevPts.size() != filteredPrevPts.size()) {
                continue;
            }

            std::vector<cv::Mat> rotationMatrix, translationVector, normalVector;
            cv::Mat homography = cv::findHomography(filteredPrevPts, filteredCurrPts, cv::RANSAC);
            cv::decomposeHomographyMat(homography, cameraMatrix, rotationMatrix, translationVector, normalVector);

            cv::Mat rvec_decomp = rotationMatrix[0](cv::Rect(0, 0, 2, 2)).clone();
            cv::Point2d tvec_decomp(translationVector[0].at<double>(0), translationVector[0].at<double>(1));
            cv::Mat mat = (rvec_decomp * cv::Mat(priorPosition) + cv::Mat(tvec_decomp));
            cv::Point2d estimatedPosition(mat.at<double>(0), mat.at<double>(1));

            int scaling = 10;
            double x_candidate = estimatedPosition.x;
            double y_candidate = estimatedPosition.y;
            if (!std::isnan(x_candidate) && x_candidate < 1e6 &&
                !std::isnan(y_candidate) && y_candidate < 1e6) {
                priorPosition = estimatedPosition;
                x = x_candidate * scaling + 250;
                y = y_candidate * scaling + 250;
            }
        } catch (const cv::Exception& e) {
            std::cout << e.what()<< std::endl;
        }


        cv::Mat flowDisplay = currFrame.clone();
        cv::Mat coloredFlowDisplay;
        cv::cvtColor(flowDisplay, coloredFlowDisplay, cv::COLOR_GRAY2BGR);
        for (size_t i = 0; i < prevPoints.size(); ++i) {
            if (status[i]) {
                cv::arrowedLine(coloredFlowDisplay, prevPoints[i], currPoints[i], cv::Scalar(0, 0, 255), 2);
                cv::arrowedLine(coloredFlowDisplay, filteredPrevPts[i], filteredCurrPts[i], cv::Scalar(30, 150, 0), 3);
            }
        }

        filteredPrevPts.clear();
        filteredCurrPts.clear();

        cv::line(trajectory, cv::Point(prevX, prevY), cv::Point(x, y), cv::Scalar(0, 0, 255), 2);
        prevX = x;
        prevY = y;

        if (!pause) {
            cv::imshow("Optical Flow", coloredFlowDisplay);
            cv::imshow("Trajectory", trajectory);
        }

        int keyboard = cv::waitKey(1);
        if (keyboard == 'q' || keyboard == 27)
            break;
        if (keyboard == 'p')
            pause = !pause;

        // Update frames and feature points
        std::swap(prevFrame, currFrame);
        std::swap(prevPoints, currPoints);
        cv::goodFeaturesToTrack(prevFrame, prevPoints, 500, 0.01, 10);

        loop_counter++;
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = end - start;
        if (duration.count() >= 1.0) {
            std::cout << "[INFO]: thread frequency = " << loop_counter / duration.count() << " Hz" << std::endl;
            loop_counter = 0;
            start = std::chrono::high_resolution_clock::now();
        }
    }

    cv::destroyAllWindows();
    video.release();

    return 0;
}
