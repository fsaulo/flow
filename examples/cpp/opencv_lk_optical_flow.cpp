#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>

int main(int argc, char* argv[])
{

    const std::string keys =
        "{ h help |      | print this help message }"
        "{ @image | vtest.avi | path to image file }";

    cv::CommandLineParser parser(argc, argv, keys);
    std::string filename = cv::samples::findFile(parser.get<std::string>("@image"));
    if (!parser.check())
    {
        parser.printErrors();
        return 0;
    }

    cv::VideoCapture video(filename);
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

    float target_width = 840;
    float imratio = target_width / prevFrame.size().width;
    cv::Size size = cv::Size((int) target_width, int(prevFrame.size().height * imratio));

    cv::resize(prevFrame, prevFrame, size, 0, 0, cv::INTER_LINEAR);
    cv::cvtColor(prevFrame, prevFrame, cv::COLOR_BGR2GRAY);
    cv::goodFeaturesToTrack(prevFrame, prevPoints, 500, 0.01, 10);


    while (video.read(currFrame))
    {
        cv::resize(currFrame, currFrame, size, 0, 0, cv::INTER_LINEAR);
        cv::cvtColor(currFrame, currFrame, cv::COLOR_BGR2GRAY);
        cv::calcOpticalFlowPyrLK(prevFrame, currFrame, prevPoints, currPoints, status, error);
        cv::Mat flowDisplay = currFrame.clone();

        // Visualize optical flow
        for (size_t i = 0; i < prevPoints.size(); ++i)
        {
            if (status[i])
            {
                cv::arrowedLine(flowDisplay, prevPoints[i], currPoints[i], cv::Scalar(0, 255, 0), 2);
            }
        }

        cv::imshow("Optical Flow", flowDisplay);
        int keyboard = cv::waitKey(30);
        if (keyboard == 'q' || keyboard == 27)
            break;

        // Update frames and feature points
        std::swap(prevFrame, currFrame);
        std::swap(prevPoints, currPoints);
        cv::goodFeaturesToTrack(prevFrame, prevPoints, 500, 0.01, 10);
    }

    cv::destroyAllWindows();
    video.release();

    return 0;
}
