#include <fstream>
#include <chrono>
#include <iterator>

#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
#include <opencv2/imgproc.hpp>

#include "opencv/CvInterface.h"

#include "utils/DCOffsetFilter.h"
#include "utils/LPFilter.h"

#include "params.h"

#include "Estimator.h"

std::deque<gcs_local_position_t> g_local_position_queue;
std::deque<gcs_global_position_t> g_global_position_queue;
std::deque<gcs_attitude_t> g_attitude_queue;
std::deque<gcs_highres_imu_t> g_highres_imu_queue;
std::deque<gcs_optical_flow_t> g_optical_flow_queue;
std::deque<gcs_optical_flow_t> g_px4flow_queue;

std::mutex g_global_position_mutex;
std::mutex g_local_position_mutex;
std::mutex g_attitude_mutex;
std::mutex g_highres_imu_mutex;
std::mutex g_optical_flow_mutex;
std::mutex g_px4flow_mutex;

uint64_t last_local_position_stamp;
uint64_t last_px4flow_position_stamp;

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
    case GCSMessageType::kOpticalFlow:
        header = "ts,integrated_x,integrated_y,xgyro,ygyro,zgyro";
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
    std::string optical_flow_filename = flow::kGCSFlowFilename;
    std::string px4flow_filename = flow::kGCSPx4flowFilename;

    std::vector<std::pair<std::string, GCSMessageType>> files_with_type = {
        { local_position_filename, GCSMessageType::kLocalPositionNed   },
        { global_position_filename, GCSMessageType::kGlobalPositionInt },
        { attitude_filename, GCSMessageType::kAttitude                 },
        { scaled_imu_filename, GCSMessageType::kHihghresImu            },
        { optical_flow_filename, GCSMessageType::kOpticalFlow          },
        { px4flow_filename, GCSMessageType::kOpticalFlow               }
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
        OpticalFlowFileUpdate(optical_flow_filename);
        Px4flowFileUpdate(px4flow_filename);

        std::this_thread::sleep_for(std::chrono::seconds(time_sleep_seconds));
        std::cout << "[DEBUG] [FileManagerCallback] Stream frequency: " << std::endl
                  << "\tlocal_position_ned: " << g_local_position_queue.size()  << " Hz" << std::endl
                  << "\tglobal_position:    " << g_global_position_queue.size() << " Hz" << std::endl
                  << "\tattitude:           " << g_attitude_queue.size()        << " Hz" << std::endl
                  << "\thighres_imu:        " << g_highres_imu_queue.size()     << " Hz" << std::endl
                  << "\toptical_flow:       " << g_optical_flow_queue.size()    << " Hz" << std::endl
                  << "\tpx4flow:            " << g_px4flow_queue.size()         << " Hz" << std::endl;
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

    // TODO: Fix this little hack to sync up poses from optical_flow sensor.
    const int optical_flow_milliseconds_sync = 20;
    const int optical_flow_time_diff = local_position.time_ms - last_local_position_stamp;
    if (optical_flow_time_diff >= optical_flow_milliseconds_sync) {
        g_local_position_queue.push_back(local_position);
    }

    last_local_position_stamp = local_position.time_ms;
}

void InsertOpticalFlow(const GCSOpticalFlow& optical_flow) {
    std::lock_guard<std::mutex> guard(g_optical_flow_mutex);
    g_optical_flow_queue.push_back(optical_flow);
}

void InsertPx4flow(const GCSOpticalFlow& px4flow)
{
    std::lock_guard<std::mutex> guard(g_px4flow_mutex);
    g_px4flow_queue.push_back(px4flow);
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

void OpticalFlowFileUpdate(const std::string& filename)
{
    std::lock_guard<std::mutex> guard(g_optical_flow_mutex);
    for ( auto optical_flow : g_optical_flow_queue ) {
        auto timestamp = optical_flow.time_ms;
        std::vector<double> optical_flow_vect = {
            optical_flow.flow_x,
            optical_flow.flow_y,
            optical_flow.flow_xgyro,
            optical_flow.flow_ygyro,
            optical_flow.flow_zgyro
        };

        AppendVecToFile(optical_flow_vect, filename, timestamp);
        g_optical_flow_queue.pop_front();
    }
}

void Px4flowFileUpdate(const std::string& filename)
{
    std::lock_guard<std::mutex> guard(g_px4flow_mutex);
    for ( auto px4flow : g_px4flow_queue ) {
        auto timestamp = px4flow.time_ms;
        std::vector<double> px4flow_vect = {
            px4flow.flow_x,
            px4flow.flow_y,
            px4flow.flow_xgyro,
            px4flow.flow_ygyro,
            px4flow.flow_zgyro
        };

        AppendVecToFile(px4flow_vect, filename, timestamp);
        g_px4flow_queue.pop_front();
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
        case GCSMessageType::kOpticalFlow:
            InsertPx4flow((result.optical_flow));
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
    }
}


gcs_optical_flow_t
CreateOpticalFlowMessage( const double integrated_x,
                          const double integrated_y,
                          const double gyro_x,
                          const double gyro_y,
                          const double gyro_z )
{
    auto sys_tick = std::chrono::system_clock::now().time_since_epoch();
    auto unix_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(sys_tick).count();

    gcs_optical_flow_t optical_flow;
    optical_flow.time_ms = unix_epoch;
    optical_flow.flow_x = integrated_x;
    optical_flow.flow_y = integrated_y;
    optical_flow.flow_xgyro = gyro_x;
    optical_flow.flow_ygyro = gyro_y;
    optical_flow.flow_zgyro = gyro_z;

    return optical_flow;
}

void ComputeHistogram(std::vector<double>& vector_flow, double& mean, double &stddev) {
    int bins = 255;
    float range[] = {-10, 10};
    const float* histogram_range = {range};
    bool uniform = true;
    bool accumulate = false;

    cv::Mat vector_flow_mat(vector_flow.size(), 1, CV_32F);
    for (int i = 0; i < vector_flow.size(); i++) {
        if (vector_flow[i] > 0.05) {
            vector_flow_mat.at<float>(i, 0) = vector_flow[i];
        }
    }

    cv::Scalar mean_vect, stddev_vect;
    cv::meanStdDev(vector_flow_mat, mean_vect, stddev_vect);

    mean = mean_vect[0];
    stddev = stddev_vect[0];

    cv::Mat histogram;
    cv::calcHist(&vector_flow_mat, 1, 0, cv::Mat(), histogram, 1, &bins, &histogram_range, uniform, accumulate);
    cv::normalize(histogram, histogram, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

    int histWidth = 512;
    int histHeight = 400;
    int binWidth = cvRound((double)histWidth / bins);
    cv::Mat histogram_image(histHeight, histWidth, CV_8UC3, cv::Scalar(0, 0, 0));

    for (int i = 1; i < bins; i++) {
        cv::line(histogram_image,
                 cv::Point(binWidth * (i - 1), histHeight - cvRound(histogram.at<float>(i - 1) * histHeight)),
                 cv::Point(binWidth * i, histHeight - cvRound(histogram.at<float>(i) * histHeight)),
                 cv::Scalar(255, 255, 255), 2, 8, 0);
    }

    cv::imshow("Histogram", histogram_image);
    cv::waitKey(1);
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

    int image_width = prevFrame.cols;
    int image_height = prevFrame.rows;
    int target_width = flow::kTargetWidth;
    int target_height = flow::kTargetHeight;
    int output_width = flow::kOutputWidth;
    int output_height = flow::kOutputHeight;

    int coord_map_rect_x = static_cast<int>((image_width - target_width) / 2);
    int coord_map_rect_y = static_cast<int>((image_height - target_height) / 2);

    cv::Rect roi(
        coord_map_rect_x,
        coord_map_rect_y,
        target_width,
        target_height
    );

    std::cout << roi << std::endl;

    // float imratio = target_width / prevFrame.size().width;
    // cv::Size size = cv::Size(target_width, int(prevFrame.size().height * imratio));
    // cv::Mat distorted;
    // cv::resize(prevFrame, prevFrame, size, 0, 0, cv::INTER_LINEAR);

    // cv::Mat prev_roi_frame = prevFrame(roi);
    cv::Mat prev_roi_frame;
    prevFrame.copyTo(prev_roi_frame);
    cv::cvtColor(prev_roi_frame, prev_roi_frame, cv::COLOR_BGR2GRAY);
    cv::goodFeaturesToTrack(prev_roi_frame, prevPoints, flow::kMaxCorners, flow::kQualityLevel, flow::kMinDistance);

    double cx = flow::kCx;
    double cy = flow::kCy;
    double focal_length = flow::kFocalLength;

    std::vector<double> dist_coeffs = flow::kDistCoeffs;
    cv::Mat distCoeffs(5, 1, CV_64F, dist_coeffs.data());
    cv::Mat K = (cv::Mat_<double>(3, 3) <<
        focal_length, 0, cx,
        0, focal_length, cy,
        0, 0, 1);

    cv::Mat priorPose = (cv::Mat_<double>(4, 4) <<
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1);

    double prevX = output_width / 2.0;
    double prevY = output_height / 2.0;
    double cumx = 0.0, cumy = 0.0;
    double dt = flow::kInitialDt;
    double hfov = flow::kHFov;

    bool pause = false;

    // TODO: filter by status
    bool filterByStatus = flow::kFilterByStatus;
    bool filterByVelocity = flow::kFilterByVelocity;
    double maxVelocityThreshold = flow::kMaxVelThreshold;

    std::vector<cv::Point2f> filteredPrevPts, filteredCurrPts;
    cv::Mat trajectory = cv::Mat::zeros(output_width, output_height, CV_8UC3);
    cv::Mat accVelocity = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat priorR = cv::Mat::zeros(3, 3, CV_64F);

    const int scaleFactor = flow::kScaleFactor;
    int imx, imy = (int) prevX;
    int inertial_count = 0;
    double lower_bound_threshold = 0;
    double upper_bound_threshold = 10;

    // TODO: refactor this filters
    const int filter_size = flow::kFilterSize;
    const double lp_alpha = flow::kLowPassAlpha;

    DCOffsetFilter optical_flow_dc_removal_filter_x(filter_size);
    DCOffsetFilter optical_flow_dc_removal_filter_y(filter_size);
    LPFilter2d optical_flow_lp_filter(lp_alpha);

    const int  thread_frequency = flow::kThreadFrequencyHz;
    const int  thread_milliseconds = static_cast<int>((1.0 / thread_frequency) * 1e3);
    const auto time_window = std::chrono::milliseconds(thread_milliseconds);

    gcs_optical_flow_t initial_optical_flow = CreateOpticalFlowMessage(0, 0, 0, 0, 0);
    InsertOpticalFlow(initial_optical_flow);

    std::vector<double> vector_flow;

    while ( video.read(currFrame) ) {
        auto tick_start = std::chrono::steady_clock::now();
        cv::Mat curr_roi_frame;

        try {
            // curr_roi_frame = currFrame(roi);
            currFrame.copyTo(curr_roi_frame);
            cv::cvtColor(curr_roi_frame, curr_roi_frame, cv::COLOR_BGR2GRAY);
            cv::calcOpticalFlowPyrLK(prev_roi_frame, curr_roi_frame, prevPoints, currPoints, status, error);

            double sumMagnitude = 0.0;
            int count = 0;
            for (size_t i = 0; i < status.size(); ++i) {
                double magnitude = cv::norm(prevPoints[i] - currPoints[i]);
                sumMagnitude += magnitude;
                count++;
            }

            // Filtering computed optical flow using sigma clipping method.
            // We first compute a histogram of the vector flow, then we set an outlier threashold
            // bounded to the histogram's standard deviation.
            double averageMagnitude = sumMagnitude / count;
            for (size_t i = 0; i < status.size(); ++i) {
                bool optical_flow_healthy = true;
                if (!status[i] && filterByStatus) {
                    optical_flow_healthy = false;
                }

                double velocity = cv::norm(prevPoints[i] - currPoints[i]);
                if (velocity >= (averageMagnitude + maxVelocityThreshold) && filterByVelocity) {
                    optical_flow_healthy = false;
                }

                double flow_dx = currPoints[i].x - prevPoints[i].x;
                double flow_dy = currPoints[i].y - prevPoints[i].y;
                double abs_flow = sqrt(flow_dx * flow_dx + flow_dy * flow_dy);
                vector_flow.push_back(abs_flow);

                // Rejecting outliers
                if (abs_flow < lower_bound_threshold || abs_flow >= upper_bound_threshold) {
                    std::cout << "[DEBUG] Rejected optical flow: \n"
                              << "\tupper_bound_threshold = " << upper_bound_threshold << std::endl
                              << "\tlower_boud_threshold  = " << lower_bound_threshold << std::endl
                              << "\tabs_flow              = " << abs_flow << std::endl;
                   optical_flow_healthy = false;
                }

                if (optical_flow_healthy) {
                    filteredPrevPts.push_back(prevPoints[i]);
                    filteredCurrPts.push_back(currPoints[i]);
                }
            }

            // Inserting zero flow when optical flow cannot be computed or is not healthy.
            if (filteredCurrPts.size() <= 10 || filteredPrevPts.size() != filteredPrevPts.size()) {
                gcs_optical_flow_t optical_flow_zero;
                optical_flow_zero = CreateOpticalFlowMessage( 0, 0, 0, 0, 0 );
                InsertOpticalFlow(optical_flow_zero);

                // Update frames and feature points
                std::swap(prev_roi_frame, curr_roi_frame);
                std::swap(prevPoints, currPoints);
                cv::goodFeaturesToTrack(prev_roi_frame, prevPoints, flow::kMaxCorners, flow::kQualityLevel, flow::kMinDistance);

                filteredPrevPts.clear();
                filteredCurrPts.clear();
                continue;
            }

            std::vector<cv::Point2f> undistorted_points = filteredCurrPts;
            cv::undistortPoints(undistorted_points, filteredCurrPts, K, distCoeffs);
            for ( int i = 0; i < undistorted_points.size(); i++ ) {
                filteredCurrPts[i].x = filteredCurrPts[i].x * focal_length + cx;
                filteredCurrPts[i].y = filteredCurrPts[i].y * focal_length + cy;
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

            // cv::Mat cam_t_img = K.inv() * (tvec) * dt * 0.5;
            cv::Mat cam_t_img = tvec;
            cv::Mat cam_R_img = K.inv() * R * K;
            cv::Mat cam_T_img = cv::Mat::eye(4, 4, CV_64F);

            double flow_x = tvec.at<double>(0);
            double flow_y = tvec.at<double>(1);

            // Calibrate the sensor by removing the offset
            // Calibration must be performed prior to integration
            flow_x = optical_flow_dc_removal_filter_x.update(flow_x);
            flow_y = optical_flow_dc_removal_filter_y.update(flow_y);

            cam_t_img.at<double>(0) = flow_x;
            cam_t_img.at<double>(1) = flow_y;

            cam_R_img.copyTo(cam_T_img(cv::Rect(0, 0, 3, 3)));
            cam_t_img.copyTo(cam_T_img(cv::Rect(3, 0, 1, 3)));

            cv::Mat camPose = cam_T_img * priorPose;
            cv::Mat camRot = camPose(cv::Rect(0, 0, 3, 3)).clone();

            accVelocity = camPose.rowRange(0, 3).col(3);

            cv::Vec3f xyzAngles = flow::opencv::rotationMatrixToEulerAngles(cam_R_img);

            std::vector<double> flow_xy = { flow_x, flow_y };
            flow_xy = optical_flow_lp_filter.update(flow_xy);

            gcs_optical_flow_t optical_flow;
            optical_flow = CreateOpticalFlowMessage(
                atan2(flow_xy[0], focal_length),
                atan2(flow_xy[1], focal_length),
                xyzAngles[0],
                xyzAngles[1],
                xyzAngles[2]
            );

            cumx = flow_xy[0];
            cumx = flow_xy[1];

            InsertOpticalFlow(optical_flow);

            priorPose = camPose;
            priorR = R;
        } catch (const cv::Exception& e) {
            gcs_optical_flow_t optical_flow_zero;
            optical_flow_zero = CreateOpticalFlowMessage( 0, 0, 0, 0, 0 );
            InsertOpticalFlow(optical_flow_zero);

            std::cout << e.what()<< std::endl;
        }

        double mean, stddev;
        ComputeHistogram(vector_flow, mean, stddev);

        double n_stddev = 5;
        lower_bound_threshold = mean - n_stddev * stddev;
        upper_bound_threshold = mean + n_stddev * stddev;

        if (vector_flow.size() >= 1000) {
            vector_flow.erase(vector_flow.begin());
        }

        cv::Mat flowDisplay = curr_roi_frame.clone();
        cv::Mat coloredFlowDisplay;
        cv::Mat coloredFlowDisplayBad;
        cv::cvtColor(flowDisplay, coloredFlowDisplay, cv::COLOR_GRAY2BGR);
        cv::cvtColor(flowDisplay, coloredFlowDisplayBad, cv::COLOR_GRAY2BGR);
        for (size_t i = 0; i < prevPoints.size(); ++i) {
            if (status[i]) {
                cv::arrowedLine(coloredFlowDisplayBad, prevPoints[i], currPoints[i], cv::Scalar(0, 0, 255), 1);
                cv::arrowedLine(coloredFlowDisplay, filteredPrevPts[i], filteredCurrPts[i], cv::Scalar(30, 150, 0), 1);
            }
        }

        filteredPrevPts.clear();
        filteredCurrPts.clear();

        // cv::line(trajectory, cv::Point(prevX, prevY), cv::Point(imx, imy), cv::Scalar(0, 0, 255), 2);
        // prevX = imx;
        // prevY = imy;

        cv::Mat optical_flow_output_image;
        cv::hconcat(coloredFlowDisplayBad, coloredFlowDisplay, optical_flow_output_image);
        cv::resize(optical_flow_output_image, optical_flow_output_image, cv::Size(800, 400), 0, 0, cv::INTER_LINEAR);
        if (!pause) {
            cv::imshow("Optical Flow", optical_flow_output_image);
        //     cv::imshow("Trajectory", trajectory);
        }

        int keyboard = cv::waitKey(10);
        if (keyboard == 'q' || keyboard == 27)
            break;
        if (keyboard == 'p')
            pause = !pause;

        // Update frames and feature points
        std::swap(prev_roi_frame, curr_roi_frame);
        std::swap(prevPoints, currPoints);
        cv::goodFeaturesToTrack(prev_roi_frame, prevPoints, flow::kMaxCorners, flow::kQualityLevel, flow::kMinDistance);

        auto time_elapsed = std::chrono::steady_clock::now() - tick_start;
        auto time_sleep = time_window - time_elapsed;
        if (time_sleep > std::chrono::milliseconds::zero()) {
            std::this_thread::sleep_for(time_sleep);
        }

        std::chrono::duration<float> step = std::chrono::steady_clock::now() - tick_start;
        dt = step.count();
    }

    cv::destroyAllWindows();
    video.release();
}
