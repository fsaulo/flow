#ifndef _IOSTREAM_H
#define _IOSTREAM_H

#include <chrono>
#include <thread>
#include <iostream>

#define PI 3.14159265358979323846

#ifdef USING_OPENCV
#include <opencv2/videoio.hpp>
#include <opencv2/device.hpp>
#endif

namespace IOStream
{

// Undefined interface
std::unique_ptr device = nullptr;

#ifdef USING_OPENCV
cv::VideoCapture device;

const static int CV_CAP = cv::CAP_GSTREAMER;
#endif

}

#endif // _IOSTREAM_H
