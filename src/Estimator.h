#ifndef LK_ESTIMATOR_H
#define LK_ESTIMATOR_H

#include <string>
#include <deque>
#include <mutex>
#include <vector>
#include <fstream>

#include <opencv2/opencv.hpp>

#include "params.h"
#include "gcs/GCSMavlink.h"

template <typename T>
void AppendVecToFile(const std::vector<T>& data, const std::string& filename, const uint64_t ts = 0);

void InsertLocalPosition(const GCSLocalPositionNed& local_position);
void InsertGlobalPosition(const GCSGlobalPosition& global_position);
void InsertAttitude(const GCSAttitude& attitude);
void InsertHighresImu(const GCSHighresImu& scaled_imu);
void InsertOpticalFlow(const GCSOpticalFlow& optical_flow);
void InsertPx4flow(const GCSOpticalFlow& px4flow);

void LocalPositionFileUpdate(const std::string& filename);
void GlobalPositionFileUpdate(const std::string& filename);
void AttitudeFileUpdate(const std::string& filename);
void HighresImuFileUpdate(const std::string& filename);
void OpticalFlowFileUpdate(const std::string& filename);
void Px4flowFileUpdate(const std::string& filename);

void ComputeHistogram(std::vector<double>& vector_flow, double& mean, double& stddev);
void ClearFile(const std::string& filename, const GCSMessageType& type = GCSMessageType::kUnknown);
void EstimatorCallback(const std::string& pipeline);
void MavlinkMessageCallback(void);
void FileManagerCallback(void);

gcs_optical_flow_t CreateOpticalFlowMessage(
    const double int_x,
    const double int_y,
    const double gyro_x,
    const double gyro_y,
    const double gyro_z
);

#endif // LK_ESTIMATOR_H
