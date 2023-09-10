#include <fstream>
#include <chrono>

#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>

#include "opencv/CvInterface.h"
#include "gcs/GCSMavlink.h"
#include "utils/DCOffsetFilter.h"
#include "params.h"

#include "Estimator.h"

void ClearFile(std::string& filename)
{
    std::ofstream rw_file(filename, std::ios::trunc);
    while (rw_file.is_open()) {
        rw_file << "x,y,z,w" << std::endl;
        rw_file.close();
    }
}

void MavlinkMessageCallback(void)
{
    GCSMavlink conn;

    const int  thread_frequency = flow::kThreadFrequencyHz;
    const int  thread_milliseconds = static_cast<int>((1.0 / thread_frequency) * 1e3);
    const auto time_window = std::chrono::milliseconds(thread_milliseconds);

    while (true) {
        auto tick_start = std::chrono::steady_clock::now();

        GCSResult result;

        // TODO: process result
        result = conn.ReceiveSome();

        auto time_elapsed = std::chrono::steady_clock::now() - tick_start;
        auto time_sleep = time_window - time_elapsed;
        if (time_sleep > std::chrono::milliseconds::zero()) {
            std::this_thread::sleep_for(time_sleep);
        }
        
        auto  time_thread = std::chrono::steady_clock::now() - tick_start;
        float freq_thread = (1.0 / time_thread.count()) * 1e9;
        std::cout << "[DEBUG] [MavlinkMessageCallback] running:"        << std::endl
                  << "\tfreq_est     = " << freq_thread                 << " Hz" << std::endl
                  << "\tthread_freq  = " << thread_frequency          << " Hz" << std::endl
                  << "\ttime_sleep   = " << time_sleep.count() * 1e-6   << " ms" << std::endl
                  << "\ttime_elapsed = " << time_elapsed.count() * 1e-6 << " ms" << std::endl
                  << "\ttime_window  = " << time_window.count()         << " ms" << std::endl;
    }
}

void EstimatorCallback(const std::string& pipeline)
{
    // TODO: also accept different decoders
    // Make it optional to use OpenCV
    cv::VideoCapture video(pipeline, cv::CAP_GSTREAMER);
    if (!video.isOpened()) {
        std::cout << "Failed to open the video file!\n";
        return;
    }

    cv::Mat prevFrame, currFrame;

    std::vector<cv::Point2f> prevPoints, currPoints;
    std::vector<uchar> status;
    std::vector<float> error;
    std::vector<cv::Vec3f> values;

    video >> prevFrame;

    int target_width  = flow::kTargetWidth, 
        target_height = flow::kTargetHeight;

    float imratio = target_width / prevFrame.size().width;
    cv::Size size = cv::Size(target_width, int(prevFrame.size().height * imratio));
    cv::Mat distorted;

    cv::resize(prevFrame, prevFrame, size, 0, 0, cv::INTER_LINEAR);
    cv::cvtColor(prevFrame, prevFrame, cv::COLOR_BGR2GRAY);
    cv::goodFeaturesToTrack(prevFrame, prevPoints, flow::kMaxCorners, flow::kQualityLevel, flow::kMinDistance);

    std::string flowAngVelFile = flow::kFlowAngVelFilename; 
    std::string flowLinVelFile = flow::kFlowLinVelFilename; 
    std::string flowQuaPosFile = flow::kFlowQuaVelFilename; 

    ClearFile(flowAngVelFile);
    ClearFile(flowLinVelFile);
    ClearFile(flowQuaPosFile);

    double cx = flow::kCx;
    double cy = flow::kCy;
    double focalLength = flow::kFocalLength;

    std::vector<double> dist_coeffs = flow::kDistCoeffs;
    cv::Mat distCoeffs(5, 1, CV_64F, dist_coeffs.data());
    cv::Mat K = (cv::Mat_<double>(3, 3) <<
        focalLength, 0, cx,
        0, focalLength, cy,
        0, 0, 1);

    cv::Mat priorPose = (cv::Mat_<double>(4, 4) <<
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1);

    double prevX = target_width / 2.0;
    double prevY = target_height / 2.0;
    double cumx, cumy = 0;
    double dt = flow::kInitialDt;
    double hfov = flow::kHFov;

    bool pause = false;
    bool filterByStatus = flow::kFilterByStatus;
    bool filterByVelocity = flow::kFilterByVelocity;
    double maxVelocityThreshold = flow::kMaxVelThreshold;

    std::vector<cv::Point2f> filteredPrevPts, filteredCurrPts;
    cv::Mat trajectory = cv::Mat::zeros(target_width, target_height, CV_8UC3);
    cv::Mat accVelocity = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat priorR = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat lastVec = cv::Mat::zeros(3, 1, CV_64F);

    const int scaleFactor = flow::kScaleFactor;
    const int scaleHeight = flow::kScaleHeight;
    int imx, imy = (int) prevX;

    // TODO: refactor this filters
    const int filter_size = flow::kFilterSize;
    DCOffsetFilter xVelFilter(filter_size); 
    DCOffsetFilter yVelFilter(filter_size); 
    DCOffsetFilter zVelFilter(filter_size); 

    DCOffsetFilter xxVelFilter(filter_size); 
    DCOffsetFilter yyVelFilter(filter_size); 
    DCOffsetFilter zzVelFilter(filter_size); 

    DCOffsetFilter xAngFilter(filter_size); 
    DCOffsetFilter yAngFilter(filter_size); 
    DCOffsetFilter zAngFilter(filter_size); 

    const int  thread_frequency = flow::kThreadFrequencyHz;
    const int  thread_milliseconds = static_cast<int>((1.0 / thread_frequency) * 1e3);
    const auto time_window = std::chrono::milliseconds(thread_milliseconds);

    while (video.read(currFrame))
    {
        auto tick_start = std::chrono::steady_clock::now();

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

            cv::Vec4f quatPose = flow::opencv::rotationMatrixToQuaternion(camRot);
            cv::Vec3f xyzAngles = flow::opencv::rotationMatrixToEulerAngles(cam_R_img);
            cv::Vec3f xyzVelocity = accVelocity;

            xyzAngles[0] = xAngFilter.update(xyzAngles[0]);
            xyzAngles[1] = yAngFilter.update(xyzAngles[1]);
            xyzAngles[2] = zAngFilter.update(xyzAngles[2]) * -1;

            xyzVelocity[0] = xxVelFilter.update(xyzVelocity[0]);
            xyzVelocity[1] = yyVelFilter.update(xyzVelocity[1]);
            xyzVelocity[2] = zzVelFilter.update(xyzVelocity[2]);

            flow::opencv::saveCvVecToFile<float, 4>(flowQuaPosFile, quatPose);
            flow::opencv::saveCvVecToFile<float, 3>(flowAngVelFile, xyzAngles);
            flow::opencv::saveCvVecToFile<float, 3>(flowLinVelFile, xyzVelocity);

            std::cout << "[DEBUG] [lk_pose_estimation] position   : " << xyzVelocity << std::endl;
            std::cout << "[DEBUG] [lk_pose_estimation] angular_vel: " << xyzAngles << std::endl;

            cumx += xyzVelocity[0];
            cumy += xyzVelocity[1];
            imx = (int) cumx * scaleFactor + target_width / 2;
            imy = (int) cumy * scaleFactor + target_width / 2;

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
        cv::goodFeaturesToTrack(prevFrame, prevPoints, flow::kMaxCorners, flow::kQualityLevel, flow::kMinDistance);

        auto time_elapsed = std::chrono::steady_clock::now() - tick_start;
        auto time_sleep = time_window - time_elapsed;
        if (time_sleep > std::chrono::milliseconds::zero()) {
            std::this_thread::sleep_for(time_sleep);
        }

        std::chrono::duration<float> step = std::chrono::steady_clock::now() - tick_start;
        dt = step.count();

        auto  time_thread = std::chrono::steady_clock::now() - tick_start;
        float freq_thread = (1.0 / time_thread.count()) * 1e9;
        std::cout << "[DEBUG] [EstimatorCallback] running:"        << std::endl
                  << "\tfreq_est     = " << freq_thread                 << " Hz" << std::endl
                  << "\tthread_freq  = " << thread_frequency          << " Hz" << std::endl
                  << "\ttime_sleep   = " << time_sleep.count()   * 1e-6 << " ms" << std::endl
                  << "\ttime_elapsed = " << time_elapsed.count() * 1e-6 << " ms" << std::endl
                  << "\ttime_window  = " << time_window.count()         << " ms" << std::endl;
    }

    cv::destroyAllWindows();
    video.release();
}
