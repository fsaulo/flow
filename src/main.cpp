#include <thread>
#include <opencv2/core.hpp>

#include "Estimator.h"

int main(int argc, char* argv[])
{
    const std::string keys =
        "{ h help |      | print this help message }"
        "{ @image | vtest.avi | path to image file }";

    cv::CommandLineParser parser(argc, argv, keys);
    std::string pipeline_stream = parser.get<std::string>("@image");
    if (!parser.check()) {
        parser.printErrors();
        return 0;
    }

    std::thread optical_flow_thread;
    optical_flow_thread = std::thread(EstimatorCallback, std::cref(pipeline_stream));
    optical_flow_thread.join();

    return 0;
}
