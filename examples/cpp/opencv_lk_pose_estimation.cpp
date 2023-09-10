#include <fstream>
#include <chrono>
#include <thread>
#include <iterator>

#include <Eigen/Dense>

#include "utils/OpencvInterface.h"
#include "utils/DCOffsetFilter.h"
#ifdef MAVLINK_UDP_ENABLED
#include "utils/include/MavlinkInterface.h"
#endif

template<typename T, int N>
void saveCvVecToFile(std::string& filename, const cv::Vec<T, N>& vect)
{
    std::vector<float> vectorData = cvVecToStdVector(vect);
    std::ofstream dataFile(filename, std::ios::app);
    if (!dataFile.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    std::copy(vectorData.begin(), vectorData.end() - 1, std::ostream_iterator<T>(dataFile, ","));
    dataFile << vectorData.back() << std::endl;
    dataFile.close();
}

void clearFile(std::string& filename)
{
    std::ofstream dataFile(filename, std::ios::trunc);
    while (dataFile.is_open()) {
        dataFile << "x,y,z,w" << std::endl;
        dataFile.close();
    }
}

bool isValidRotationMatrix(const cv::Mat& rotationMatrix) {
    // Check matrix size
    if (rotationMatrix.rows != 3 || rotationMatrix.cols != 3) {
        std::cout << "Invalid rotation matrix size." << std::endl;
        return false;
    }

    // Check orthogonality
    cv::Mat transpose = rotationMatrix.t();
    cv::Mat identity = rotationMatrix * transpose;
    cv::Mat diff = identity - cv::Mat::eye(3, 3, rotationMatrix.type());
    double norm = cv::norm(diff, cv::NORM_L2);

    if (norm > 1e-6) {
        std::cout << "Rotation matrix is not orthogonal." << std::endl;
        return false;
    }

    // Check determinant
    double determinant = cv::determinant(rotationMatrix);
    if (std::abs(determinant - 1.0) > 1e-6) {
        std::cout << "Rotation matrix determinant is not equal to 1." << std::endl;
        return false;
    }

    return true;
}

cv::Vec4f rotationMatrixToQuaternion(const cv::Mat& rotationMatrix)
{
    Eigen::Matrix3d eigenRotationMatrix;
    cv::cv2eigen(rotationMatrix, eigenRotationMatrix);

    Eigen::Quaterniond quaternion(eigenRotationMatrix);
    cv::Vec4f result;
    result[0] = static_cast<float>(quaternion.x());
    result[1] = static_cast<float>(quaternion.y());
    result[2] = static_cast<float>(quaternion.z());
    result[3] = static_cast<float>(quaternion.w());
    return result;
}

cv::Vec3f rotationMatrixToEulerAngles(cv::Mat& R)
{
    // assert(isValidRotationMatrix(R));
 
    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
 
    bool singular = sy < 1e-6;
 
    float x, y, z;
    if (!singular) {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    } else {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }

    return cv::Vec3f(x, y, z);
}

void applyBarrelDistortion(cv::Mat& image, float k)
{
    int width = image.cols;
    int height = image.rows;

    cv::Mat distortedImage = cv::Mat::zeros(image.size(), image.type());
    cv::Point2f center(static_cast<float>(width / 2), static_cast<float>(height / 2));

    cv::Mat mapX, mapY;
    mapX.create(image.size(), CV_32FC1);
    mapY.create(image.size(), CV_32FC1);

    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            float nx = (x - center.x) / center.x;
            float ny = (y - center.y) / center.y;
            float r = sqrt(nx * nx + ny * ny);

            float theta = atan(r);
            float distortion = theta * k;

            float srcX = center.x + distortion * center.x * nx;
            float srcY = center.y + distortion * center.y * ny;

            mapX.at<float>(y, x) = srcX;
            mapY.at<float>(y, x) = srcY;
        }
    }

    cv::remap(image, distortedImage, mapX, mapY, cv::INTER_LINEAR);

    image = distortedImage;
}

#ifdef MAVLINK_UDP_ENABLED
void mav_handle_heartbeat(const mavlink_message_t* message)
{
    mavlink_heartbeat_t heartbeat;
    mavlink_msg_heartbeat_decode(message, &heartbeat);

    printf("[DEBUG] [mav_handle_heartbeat] Got heartbeat from ");
    switch (heartbeat.autopilot) {
        case MAV_AUTOPILOT_GENERIC:
            printf("generic");
            break;
        case MAV_AUTOPILOT_ARDUPILOTMEGA:
            printf("ArduPilot");
            break;
        case MAV_AUTOPILOT_PX4:
            printf("PX4");
            break;
        default:
            printf("other");
            break;
    }
    printf(" autopilot\n");
}

void mav_handle_global_position_int(const mavlink_message_t* message)
{
    mavlink_global_position_int_t global_position_data;
    mavlink_msg_global_position_int_decode(message, &global_position_data);

    // Access and process global position data
    int32_t latitude = global_position_data.lat;
    int32_t longitude = global_position_data.lon;
    int32_t altitude = global_position_data.alt;

    // Process the global position data
    std::cout << "[DEBUG] [mav_handle_odometry] Got global_position_int:\n"
              << "\tLatitude: " << latitude << std::endl
              << "\tLongitude: " << longitude << std::endl
              << "\tAltitude: " << altitude << std::endl;
}

void mav_handle_attitude(const mavlink_message_t* message)
{
    std::cout << "[DEBUG] attitude" << std::endl;
}

void mav_handle_local_position_ned(const mavlink_message_t* message)
{
    std::cout << "[DEBUG] local_position_ned" << std::endl;
}

void mav_receive_some(int socket_fd, 
                  struct sockaddr_in* src_addr, 
                  socklen_t* src_addr_len, 
                  bool* src_addr_set)
{
    char buffer[2048];

    const int ret = recvfrom( socket_fd, buffer, 
            sizeof(buffer), 0, (struct sockaddr*)(src_addr), src_addr_len);

    if (ret < 0) {
        std::cerr << "[ERROR] recvfrom error: " << strerror(errno) << std::endl;
    } else if (ret == 0) {
        return;
    } 

    *src_addr_set = true;

    mavlink_message_t message;
    mavlink_status_t status;
    for (int i = 0; i < ret; ++i) {
        if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &message, &status) == 1) {
            switch (message.msgid) {
            case MAVLINK_MSG_ID_HEARTBEAT:
                mav_handle_heartbeat(&message);
                break;
            case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                mav_handle_global_position_int(&message);
                break;
            case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
                mav_handle_local_position_ned(&message);
                break;
            case MAVLINK_MSG_ID_ATTITUDE:
                mav_handle_attitude(&message);
                break;
            }
        }
    }
}

void mavlink_msg_callback(void) 
{
    mavlink_channel_t channel = MAVLINK_COMM_0;
    mavlink_status_t mav_status;
    mavlink_message_t msg;

    int udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_socket == -1) {
        std::cerr << "Socket creation failed." << std::endl;
        return;
    }

    sockaddr_in server_address;
    server_address.sin_family = AF_INET;
    server_address.sin_addr.s_addr = inet_addr("127.0.0.1");
    server_address.sin_port = htons(14445);
    if (bind(udp_socket, (struct sockaddr*)&server_address, sizeof(server_address)) == -1) {
        std::cerr << "Bind failed." << std::endl;
        close(udp_socket);
        return;
    }

    // We set a timeout at 100ms to prevent being stuck in recvfrom for too
    // long and missing our chance to send some stuff.
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;
    if (setsockopt(udp_socket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        std::cerr << "setsockopt error: " << strerror(errno) << std::endl;
        return;
    }

    while (true) {
        struct sockaddr_in src_addr = {};
        socklen_t src_addr_len = sizeof(src_addr);
        bool src_addr_set = false;

        mav_receive_some(udp_socket, &src_addr, &src_addr_len, &src_addr_set);
    }

    close(udp_socket);
}
#endif

void lk_estimator_callback(const std::string& pipeline)
{
    cv::VideoCapture video(pipeline, cv::CAP_GSTREAMER);
    if (!video.isOpened())
    {
        std::cout << "Failed to open the video file!\n";
        return;
    }

    cv::Mat prevFrame, currFrame;

    std::vector<cv::Point2f> prevPoints, currPoints;
    std::vector<uchar> status;
    std::vector<float> error;
    std::vector<cv::Vec3f> values;

    video >> prevFrame;

    int target_width, target_height;
    target_width = target_height = 640;

    float imratio = target_width / prevFrame.size().width;
    cv::Size size = cv::Size(target_width, int(prevFrame.size().height * imratio));
    cv::Mat distorted;

    cv::resize(prevFrame, prevFrame, size, 0, 0, cv::INTER_LINEAR);
    cv::cvtColor(prevFrame, prevFrame, cv::COLOR_BGR2GRAY);
    cv::goodFeaturesToTrack(prevFrame, prevPoints, 500, 0.01, 10);

    std::string flowAngVelFile = "flow_angular_velocity.csv";
    std::string flowLinVelFile = "flow_linear_velocity.csv";
    std::string flowQuaPosFile = "flow_quaternion_orientation.csv";

    clearFile(flowAngVelFile);
    clearFile(flowLinVelFile);
    clearFile(flowQuaPosFile);

    double cx = 320.5;
    double cy = 240.5;
    double focalLength = 277.191356;

    cv::Mat priorPose = (cv::Mat_<double>(4, 4) <<
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1);
    cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << 0, 0, 0, 0, 0);
    cv::Mat K = (cv::Mat_<double>(3, 3) <<
        focalLength, 0, cx,
        0, focalLength, cy,
        0, 0, 1);

    double maxVelocityThreshold = 10;
    double prevX = target_width / 2.0;
    double prevY = target_height / 2.0;
    double x, y;
    double dt = 0.033;
    double hfov = 1.047;

    bool pause = false;
    bool filterByStatus = true;
    bool filterByVelocity = false;

    x = y = 0;
    std::vector<cv::Point2f> filteredPrevPts, filteredCurrPts;
    cv::Mat trajectory = cv::Mat::zeros(target_width, target_height, CV_8UC3);
    cv::Mat accVelocity = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat priorR = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat lastVec = cv::Mat::zeros(3, 1, CV_64F);

    int loop_counter = 0;
    int scaleFactor = 1;
    int scaleHeight = 100;
    int imx, imy = (int) prevX;

    auto start = std::chrono::high_resolution_clock::now();

    DCOffsetFilter xVelFilter(5); 
    DCOffsetFilter yVelFilter(5); 
    DCOffsetFilter zVelFilter(5); 

    DCOffsetFilter xxVelFilter(20); 
    DCOffsetFilter yyVelFilter(20); 
    DCOffsetFilter zzVelFilter(20); 

    DCOffsetFilter xAngFilter(20); 
    DCOffsetFilter yAngFilter(20); 
    DCOffsetFilter zAngFilter(20); 

    while (video.read(currFrame))
    {
        auto runtime = std::chrono::high_resolution_clock::now();

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

            std::vector<cv::Mat> rvec_matrices, tvec_tvectors, nvec_normals;
            cv::Mat H = cv::findHomography(filteredPrevPts, filteredCurrPts, cv::RANSAC);

            // homography normalization
            double norm = sqrt(H.at<double>(0,0)*H.at<double>(0,0) +
            H.at<double>(1,0)*H.at<double>(1,0) +
            H.at<double>(2,0)*H.at<double>(2,0));
            H /= norm;

            cv::Mat c1 = H.col(0);
            cv::Mat c2 = H.col(1);
            cv::Mat c3 = c1.cross(c2);

            cv::Mat tvec = H.col(2);
            cv::Mat R(3, 3, CV_64F);

            for (int i = 0; i < 3; i++) {
                R.at<double>(i,0) = c1.at<double>(i,0);
                R.at<double>(i,1) = c2.at<double>(i,0);
                R.at<double>(i,2) = c3.at<double>(i,0);
            }

            cv::Mat_<double> W, U, Vt;
            cv::SVDecomp(R, W, U, Vt);
            R = U*Vt;
            double det = cv::determinant(R);

            if (det < 0) {
                for (int i = 0; i < 3; i++)
                    Vt.at<double>(2, i) = Vt.at<double>(2, i) - 1;
                R = U*Vt;
            }

            cv::Mat cam_R_img = K.inv() * R * K;
            cv::Mat cam_t_img = K.inv() * (tvec) * dt * 0.5;
            cv::Mat cam_T_img = cv::Mat::eye(4, 4, CV_64F);

            cam_t_img.at<double>(0) = xVelFilter.removeOffset(cam_t_img.at<double>(0)) * -1;
            cam_t_img.at<double>(1) = yVelFilter.removeOffset(cam_t_img.at<double>(1)) * -1;
            cam_t_img.at<double>(2) = zVelFilter.removeOffset(cam_t_img.at<double>(2));

            cam_R_img.copyTo(cam_T_img(cv::Rect(0, 0, 3, 3)));
            cam_t_img.copyTo(cam_T_img(cv::Rect(3, 0, 1, 3)));

            cv::Mat camPose = cam_T_img * priorPose;
            cv::Mat camRot = camPose(cv::Rect(0, 0, 3, 3)).clone();

            accVelocity = camPose.rowRange(0, 3).col(3);

            cv::Vec4f quatPose = rotationMatrixToQuaternion(camRot);
            cv::Vec3f xyzAngles = rotationMatrixToEulerAngles(cam_R_img);
            cv::Vec3f xyzVelocity = accVelocity;

            xyzAngles[0] = xAngFilter.update(xyzAngles[0]);
            xyzAngles[1] = yAngFilter.update(xyzAngles[1]);
            xyzAngles[2] = zAngFilter.update(xyzAngles[2]) * -1;

            xyzVelocity[0] = xxVelFilter.update(xyzVelocity[0]);
            xyzVelocity[1] = yyVelFilter.update(xyzVelocity[1]);
            xyzVelocity[2] = zzVelFilter.update(xyzVelocity[2]);

            saveCvVecToFile(flowQuaPosFile, quatPose);
            saveCvVecToFile(flowAngVelFile, xyzAngles);
            saveCvVecToFile(flowLinVelFile, xyzVelocity);

            std::cout << "[DEBUG] [lk_pose_estimation] position   : " << xyzVelocity << std::endl;
            std::cout << "[DEBUG] [lk_pose_estimation] angular_vel: " << xyzAngles << std::endl;

            x += xyzVelocity[0];
            y += xyzVelocity[1];
            imx = (int) x * scaleFactor + target_width / 2;
            imy = (int) y * scaleFactor + target_width / 2;

            priorPose = camPose;
            priorR = R;
            lastVec = tvec;
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

        cv::line(trajectory, cv::Point(prevX, prevY), cv::Point(imx, imy), cv::Scalar(0, 0, 255), 2);
        prevX = imx;
        prevY = imy;

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
        std::chrono::duration<double> step = end - runtime;

        dt = step.count();
        
        if (duration.count() >= 1.0) {
            loop_counter = 0;
            start = std::chrono::high_resolution_clock::now();
        }
    }

    cv::destroyAllWindows();
    video.release();
}

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
    optical_flow_thread = std::thread(lk_estimator_callback, std::cref(pipeline_stream));
#ifdef MAVLINK_UDP_ENABLED
    std::thread mavlink_thread(mavlink_msg_callback);
#endif

    optical_flow_thread.join();
#ifdef MAVLINK_UDP_ENABLED
    mavlink_thread.join();
#endif

    return 0;
}
