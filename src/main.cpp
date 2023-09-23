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

    std::thread optical_flow_thread,
        mavlink_thread,
        file_manager_thread;

    optical_flow_thread = std::thread(EstimatorCallback, std::cref(pipeline_stream));
    mavlink_thread      = std::thread(MavlinkMessageCallback);
    file_manager_thread = std::thread(FileManagerCallback);

    optical_flow_thread.join();
    mavlink_thread.join();
    file_manager_thread.join();

    return 0;
}
