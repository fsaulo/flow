#include <fstream>

#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>

#include "opencv/CvInterface.h"
#include "gcs/GCSMavlink.h"

#include "Estimator.h"


void ClearFile(std::string& filename)
{
    std::ofstream dataFile(filename, std::ios::trunc);
    while (dataFile.is_open()) {
        dataFile << "x,y,z,w" << std::endl;
        dataFile.close();
    }
}

void MavlinkMessageCallback(void)
{
    GCSMavlink conn;

    while (true) {
        char   buffer[2048];
        size_t n_msgs;

        GCSResult result;

        // TODO: process result
        n_msgs = conn.ReceiveSome(buffer);
        result = conn.ParseMessage(buffer, n_msgs);

    }
}

void EstimatorCallback(const std::string& pipeline)
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


    ClearFile(flowAngVelFile);
    ClearFile(flowLinVelFile);
    ClearFile(flowQuaPosFile);

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

    // DCOffsetFilter xVelFilter(5); 
    // DCOffsetFilter yVelFilter(5); 
    // DCOffsetFilter zVelFilter(5); 
    //
    // DCOffsetFilter xxVelFilter(20); 
    // DCOffsetFilter yyVelFilter(20); 
    // DCOffsetFilter zzVelFilter(20); 
    //
    // DCOffsetFilter xAngFilter(20); 
    // DCOffsetFilter yAngFilter(20); 
    // DCOffsetFilter zAngFilter(20); 

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

            // cam_t_img.at<double>(0) = xVelFilter.removeOffset(cam_t_img.at<double>(0)) * -1;
            // cam_t_img.at<double>(1) = yVelFilter.removeOffset(cam_t_img.at<double>(1)) * -1;
            // cam_t_img.at<double>(2) = zVelFilter.removeOffset(cam_t_img.at<double>(2));

            cam_R_img.copyTo(cam_T_img(cv::Rect(0, 0, 3, 3)));
            cam_t_img.copyTo(cam_T_img(cv::Rect(3, 0, 1, 3)));

            cv::Mat camPose = cam_T_img * priorPose;
            cv::Mat camRot = camPose(cv::Rect(0, 0, 3, 3)).clone();

            accVelocity = camPose.rowRange(0, 3).col(3);

            cv::Vec4f quatPose = flow::opencv::rotationMatrixToQuaternion(camRot);
            cv::Vec3f xyzAngles = flow::opencv::rotationMatrixToEulerAngles(cam_R_img);
            cv::Vec3f xyzVelocity = accVelocity;

            // xyzAngles[0] = xAngFilter.update(xyzAngles[0]);
            // xyzAngles[1] = yAngFilter.update(xyzAngles[1]);
            // xyzAngles[2] = zAngFilter.update(xyzAngles[2]) * -1;
            //
            // xyzVelocity[0] = xxVelFilter.update(xyzVelocity[0]);
            // xyzVelocity[1] = yyVelFilter.update(xyzVelocity[1]);
            // xyzVelocity[2] = zzVelFilter.update(xyzVelocity[2]);

            flow::opencv::saveCvVecToFile<float, 4>(flowQuaPosFile, quatPose);
            flow::opencv::saveCvVecToFile<float, 3>(flowAngVelFile, xyzAngles);
            flow::opencv::saveCvVecToFile<float, 3>(flowLinVelFile, xyzVelocity);

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
