#include <fstream>
#include <chrono>
#include <iterator>

#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>

#include "opencv/CvInterface.h"
#include "utils/DCOffsetFilter.h"
#include "params.h"

#include "Estimator.h"

std::deque<gcs_local_position_t> g_local_position_queue;
std::deque<gcs_global_position_t> g_global_position_queue;
std::deque<gcs_attitude_t> g_attitude_queue;
std::deque<gcs_highres_imu_t> g_highres_imu_queue;

std::mutex g_global_position_mutex;
std::mutex g_local_position_mutex;
std::mutex g_attitude_mutex;
std::mutex g_highres_imu_mutex;

template <typename T>
void AppendVecToFile(const std::vector<T>& data, const std::string& filename, const uint64_t ts) {
    std::ofstream data_file(filename, std::ios::app);
    if (!data_file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    data_file << ts << ",";
    std::copy(data.begin(), data.end() - 1, std::ostream_iterator<T>(data_file, ","));
    data_file << data.back() << std::endl;
    data_file.close();
}

void ClearFile(std::string& filename, const GCSMessageType& type = GCSMessageType::kUnknown)
{
    std::string header = "ts";
    switch(type) {
    case GCSMessageType::kGlobalPositionInt:
        header = "ts,lat,lon,alt,vx,vy,vz";
        break;
    case GCSMessageType::kAttitude:
        header = "ts,roll,pitch,yaw,rollspeed,pitchspeed,yawspeed";
        break;
    case GCSMessageType::kLocalPositionNed:
        header = "ts,x,y,z,vx,vy,vz";
        break;
    case GCSMessageType::kHihghresImu:
        header = "ts,xacc,yacc,zacc,xgyro,ygyro,zgyro,xmag,ymag,zmag,temperature";
        break;
    default:
        header = "ts,x,y,z,w";
        break;
    }

    std::ofstream rw_file(filename, std::ios::trunc);
    while (rw_file.is_open()) {
        rw_file << header << std::endl;
        rw_file.close();
    }
}

void FileManagerCallback(void)
{
    std::string local_position_filename = flow::kGCSLocalPositionFilename;
    std::string global_position_filename = flow::kGCSGlobalPositionFilename;
    std::string attitude_filename = flow::kGCSAttitudeFilename;
    std::string scaled_imu_filename = flow::kGCSHighresImuFilename;

    std::vector<std::pair<std::string, GCSMessageType>> files_with_type = {
        {local_position_filename, GCSMessageType::kLocalPositionNed},
        {global_position_filename, GCSMessageType::kGlobalPositionInt},
        {attitude_filename, GCSMessageType::kAttitude},
        {scaled_imu_filename, GCSMessageType::kHihghresImu}
    };

    for (auto data : files_with_type) {
       ClearFile(data.first, data.second);
    }

    auto time_sleep_seconds = flow::kFileManagerSleepTimeSeconds;
    while (true) {
        LocalPositionFileUpdate(local_position_filename);
        GlobalPositionFileUpdate(global_position_filename);
        AttitudeFileUpdate(attitude_filename);
        HighresImuFileUpdate(scaled_imu_filename);
        std::this_thread::sleep_for(std::chrono::seconds(time_sleep_seconds));

        std::cout << "[DEBUG] [FileManagerCallback] Stream frequency: " << std::endl
                  << "\tlocal_position_ned: " << g_local_position_queue.size()  << " Hz" << std::endl
                  << "\tglobal_position:    " << g_global_position_queue.size() << " Hz" << std::endl
                  << "\tattitude:           " << g_attitude_queue.size()        << " Hz" << std::endl
                  << "\thighres_imu:        " << g_highres_imu_queue.size()     << " Hz" << std::endl;
    }
}

void InsertHighresImu(const GCSHighresImu& highres_imu) {
    std::lock_guard<std::mutex> guard(g_highres_imu_mutex);
    g_highres_imu_queue.push_back(highres_imu);
}

void InsertAttitude(const GCSAttitude& attitude) {
    std::lock_guard<std::mutex> guard(g_attitude_mutex);
    g_attitude_queue.push_back(attitude);
}

void InsertGlobalPosition(const GCSGlobalPosition& global_position) {
    std::lock_guard<std::mutex> guard(g_global_position_mutex);
    g_global_position_queue.push_back(global_position);
}

void InsertLocalPosition(const GCSLocalPositionNed& local_position) {
    std::lock_guard<std::mutex> guard(g_local_position_mutex);
    g_local_position_queue.push_back(local_position);
}

void HighresImuFileUpdate(const std::string& filename)
{
    std::lock_guard<std::mutex> guard(g_highres_imu_mutex);
    for ( auto scaled_imu : g_highres_imu_queue ) {
        auto timestamp = scaled_imu.time_ms;
        std::vector<double> scaled_imu_vect = {
            scaled_imu.xacc,
            scaled_imu.yacc,
            scaled_imu.zacc,
            scaled_imu.xgyro,
            scaled_imu.ygyro,
            scaled_imu.zgyro,
            scaled_imu.xmag,
            scaled_imu.ymag,
            scaled_imu.zmag,
            scaled_imu.temperature
        };

        AppendVecToFile(scaled_imu_vect, filename, timestamp);
        g_highres_imu_queue.pop_front();
    }
}

void AttitudeFileUpdate(const std::string& filename)
{
    std::lock_guard<std::mutex> guard(g_attitude_mutex);
    for ( auto attitude : g_attitude_queue ) {
        auto timestamp = attitude.time_ms;
        std::vector<double> attitude_vect = {
            attitude.roll,
            attitude.pitch,
            attitude.yaw,
            attitude.rollspeed,
            attitude.pitchspeed,
            attitude.yawspeed
        };

        AppendVecToFile(attitude_vect, filename, timestamp);
        g_attitude_queue.pop_front();
    }
}

void GlobalPositionFileUpdate(const std::string& filename)
{
    std::lock_guard<std::mutex> guard(g_global_position_mutex);
    for ( auto global_position : g_global_position_queue ) {
        auto timestamp = global_position.time_ms;
        std::vector<int32_t> global_position_vect = {
            static_cast<int32_t>(global_position.latitude),
            static_cast<int32_t>(global_position.longitude),
            static_cast<int32_t>(global_position.altitude),
            global_position.vx,
            global_position.vy,
            global_position.vz
        };

        AppendVecToFile(global_position_vect, filename, timestamp);
        g_global_position_queue.pop_front();
    }
}

void LocalPositionFileUpdate(const std::string& filename)
{
    std::lock_guard<std::mutex> guard(g_local_position_mutex);
    for ( auto local_position : g_local_position_queue ) {
        auto timestamp = local_position.time_ms;
        std::vector<double> local_position_vect = {
            local_position.x,
            local_position.y,
            local_position.z,
            local_position.vx,
            local_position.vy,
            local_position.vz
        };

        AppendVecToFile(local_position_vect, filename, timestamp);
        g_local_position_queue.pop_front();
    }
}

void MavlinkMessageCallback(void)
{
    GCSMavlink conn;

    const int  thread_frequency = flow::kMavlinkFrequencyHz;
    const int  thread_milliseconds = static_cast<int>((1.0 / thread_frequency) * 1e3);
    const auto time_window = std::chrono::milliseconds(thread_milliseconds);

    while (true) {
        auto tick_start = std::chrono::steady_clock::now();

        GCSResult result;

        result = conn.ReceiveSome();
        if (result.message_status != GCSMessageStatus::kMessageOk) {
            continue;
        }

        switch(result.type) {
        case GCSMessageType::kGlobalPositionInt:
            InsertGlobalPosition(result.global_position);
            break;
        case GCSMessageType::kAttitude:
            InsertAttitude(result.attitude);
            break;
        case GCSMessageType::kLocalPositionNed:
            InsertLocalPosition(result.local_position);
            break;
        case GCSMessageType::kHihghresImu:
            InsertHighresImu(result.highres_imu);
            break;
        case GCSMessageType::kHeartbeat:
        case GCSMessageType::kUnknown:
        default:
            break;
        }

        auto time_elapsed = std::chrono::steady_clock::now() - tick_start;
        auto time_sleep = time_window - time_elapsed;
        if (time_sleep > std::chrono::milliseconds::zero()) {
            std::this_thread::sleep_for(time_sleep);
        }

        // auto  time_thread = std::chrono::steady_clock::now() - tick_start;
        // float freq_thread = (1.0 / time_thread.count()) * 1e9;
        // std::cout << "[DEBUG] [EstimatorCallback] running:"             << std::endl
        //           << "\tfreq_est     = " << freq_thread                 << " Hz" << std::endl
        //           << "\tthread_freq  = " << thread_frequency            << " Hz" << std::endl
        //           << "\ttime_sleep   = " << time_sleep.count()   * 1e-6 << " ms" << std::endl
        //           << "\ttime_elapsed = " << time_elapsed.count() * 1e-6 << " ms" << std::endl
        //           << "\ttime_window  = " << time_window.count()         << " ms" << std::endl;
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
    double cumx = 0.0, cumy = 0.0;
    double dt = flow::kInitialDt;
    double hfov = flow::kHFov;

    bool pause = false;

    // TODO: filter by status
    bool filterByStatus = flow::kFilterByStatus;
    bool filterByVelocity = flow::kFilterByVelocity;
    double maxVelocityThreshold = flow::kMaxVelThreshold;

    std::vector<cv::Point2f> filteredPrevPts, filteredCurrPts;
    cv::Mat trajectory = cv::Mat::zeros(target_width, target_height, CV_8UC3);
    cv::Mat accVelocity = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat priorR = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat lastVec = cv::Mat::zeros(3, 1, CV_64F);

    const int scaleFactor = flow::kScaleFactor;
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

        auto current_system_clock = std::chrono::system_clock::now().time_since_epoch();
        auto unix_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(current_system_clock).count();
        auto  time_thread = std::chrono::steady_clock::now() - tick_start;
        float freq_thread = (1.0 / time_thread.count()) * 1e9;
        // std::cout << "[DEBUG] [EstimatorCallback] running:"             << std::endl
        //           << "\ttime         = " << unix_epoch                  << " ms" << std::endl
        //           << "\tfreq_est     = " << freq_thread                 << " Hz" << std::endl
        //           << "\tthread_freq  = " << thread_frequency            << " Hz" << std::endl
        //           << "\ttime_sleep   = " << time_sleep.count()   * 1e-6 << " ms" << std::endl
        //           << "\ttime_elapsed = " << time_elapsed.count() * 1e-6 << " ms" << std::endl
        //           << "\ttime_window  = " << time_window.count()         << " ms" << std::endl;
    }

    cv::destroyAllWindows();
    video.release();
}
