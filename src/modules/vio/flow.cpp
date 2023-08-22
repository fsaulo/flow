#ifdef USING_OPENCV
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#ifdef

#include "IOStream.h"
#include "flow.h"

int main(int argc, char* argv[])
{

#ifdef USING_OPENCV
    const std::string keys =
        "{ h help |      | print this help message }"
        "{ @image | vtest.avi | path to image file }";

    cv::CommandLineParser parser(argc, argv, keys);
    std::string pipeline = parser.get<std::string>("@image");
    if (!parser.check()) {
        parser.printErrors();
        return 0;
    }

    IOStream::device.open(pipeline, IOStream::CV_CAP);
#else
    IOStream::device.open();
#endif

    if (!IOStream::device.isOpened()) {
        std::cout << "Failed to open the device file!\n";
        return -1;
    }

    while (IOStream::device.read(currFrame)) {
    }

    IOStream::device.release();

    return 0;
}
