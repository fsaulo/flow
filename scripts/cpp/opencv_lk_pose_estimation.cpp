#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <iterator>

#include <Eigen/Dense>

#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
#include <opencv2/calib3d.hpp>

#include <mavlink/common/mavlink.h>

#define PI 3.14159265358979323846

class DCOffsetRemover {
public:
  DCOffsetRemover(int windowSize) : windowSize_(windowSize), sum_(0), movingSum_(0) {}

  double removeOffset(double sample) {
    if (windowSize_ == 0) 
        return sample;

    // Add the new sample to the sum
    sum_ += sample;

    // Add the sample to the window buffer
    windowBuffer_.push_back(sample);

    // If the window buffer is larger than the specified window size,
    // remove the oldest sample from the sum and the buffer
    if (windowBuffer_.size() > windowSize_) {
      sum_ -= windowBuffer_.front();
      windowBuffer_.pop_front();
    }

    // Calculate the average and subtract it from the sample
    double average = sum_ / windowBuffer_.size();
    return sample - average;
  }

  double update(double sample) {
      double value = removeOffset(sample);
      movingSum_ += value;
      return movingSum_ / windowBuffer_.size();
  }

private:
  int windowSize_;
  double sum_;
  double movingSum_;
  std::deque<double> windowBuffer_;
};  

template<typename T, int N>
std::vector<T> cvVecToStdVector(const cv::Vec<T, N>& vect) { return std::vector<T>(vect.val, vect.val + N); }

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

    DCOffsetRemover xVelFilter(5); 
    DCOffsetRemover yVelFilter(5); 
    DCOffsetRemover zVelFilter(5); 

    DCOffsetRemover xxVelFilter(20); 
    DCOffsetRemover yyVelFilter(20); 
    DCOffsetRemover zzVelFilter(20); 

    DCOffsetRemover xAngFilter(20); 
    DCOffsetRemover yAngFilter(20); 
    DCOffsetRemover zAngFilter(20); 

    mavlink_channel_t channel = MAVLINK_COMM_0;
    mavlink_status_t mav_status;
    mavlink_message_t msg;
    mavlink_udp_port_t udp_port = open_udp("127.0.0.1", 14445);

    while (video.read(currFrame))
    {
        auto runtime = std::chrono::high_resolution_clock::now();

        try {
            cv::resize(currFrame, currFrame, size, 0, 0, cv::INTER_LINEAR);
            cv::cvtColor(currFrame, currFrame, cv::COLOR_BGR2GRAY);
            cv::calcOpticalFlowPyrLK(prevFrame, currFrame, prevPoints, currPoints, status, error);

            mavlink_msg_command_long_pack(MAVLINK_SYSTEM_ID, MAV_COMP_ID_ALL, &msg,
                                          target_system, target_component,
                                          MAV_CMD_SENSOR_OFFSETS, 0, 0, 0, 0, 0, 0, 0);
            
            receive_udp(udp_port, &msg, &mav_status);

            if (msg.msgid == MAVLINK_MSG_ID_HIGHRES_IMU) {
                mavlink_highres_imu_t imu_data;
                mavlink_msg_highres_imu_decode(&msg, &imu_data);

                double angular_velocity_x = imu_data.xgyro;
                double angular_velocity_y = imu_data.ygyro;
                double angular_velocity_z = imu_data.zgyro;

                std::cout << "Angular Velocity X: " << angular_velocity_x << " rad/s" << std::endl;
                std::cout << "Angular Velocity Y: " << angular_velocity_y << " rad/s" << std::endl;
                std::cout << "Angular Velocity Z: " << angular_velocity_z << " rad/s" << std::endl;
                
            }

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

            std::cout << "[DEBUG] [lk_pose_estimation] Pos = " << xyzVelocity << std::endl;

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
    
    close_udp(udp_port);

    return 0;
}
