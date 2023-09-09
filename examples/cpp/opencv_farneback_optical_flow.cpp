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

    video >> prevFrame;
    cv::cvtColor(prevFrame, prevFrame, cv::COLOR_BGR2GRAY);  // Convert to grayscale

    while (video.read(currFrame))
    {
        cv::cvtColor(currFrame, currFrame, cv::COLOR_BGR2GRAY);  // Convert to grayscale
        cv::calcOpticalFlowFarneback(prevFrame, currFrame, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
        cv::Mat flowDisplay = currFrame.clone();  // Create a copy of the current frame

        for (int y = 0; y < flow.rows; y += 10) {
            for (int x = 0; x < flow.cols; x += 10) {
                // Get the flow vector for the current pixel
                cv::Point2f flowVector = flow.at<cv::Point2f>(y, x);

                // Draw an arrow representing the flow direction
                cv::arrowedLine( flowDisplay, 
                        cv::Point(x, y), 
                        cv::Point(x + flowVector.x, y + flowVector.y),
                        cv::Scalar(0, 255, 0), 2
                );
            }
        }

        cv::imshow("Optical Flow", flowDisplay);
        cv::waitKey(1);

        std::swap(prevFrame, currFrame);  // Update the previous frame
    }

    cv::destroyAllWindows();
    video.release();

    return 0;
}
