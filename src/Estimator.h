#ifndef LK_ESTIMATOR_H
#define LK_ESTIMATOR_H

#include <string>
#include <deque>
#include <mutex>
#include <vector>
#include <fstream>

#include "params.h"
#include "gcs/GCSMavlink.h"

template <typename T>
void AppendVecToFile(const std::vector<T>& data, const std::string& filename, const uint64_t ts = 0);

void InsertLocalPosition(const GCSLocalPositionNed& local_position);
void InsertGlobalPosition(const GCSGlobalPosition& global_position);
void InsertAttitude(const GCSAttitude& attitude);
void InsertHighresImu(const GCSHighresImu& scaled_imu);

void LocalPositionFileUpdate(const std::string& filename);
void GlobalPositionFileUpdate(const std::string& filename);
void AttitudeFileUpdate(const std::string& filename);
void ScaledImuFileUpdate(const std::string& filename);

void ClearFile(const std::string& filename, const GCSMessageType& type = GCSMessageType::kUnknown);
void EstimatorCallback(const std::string& pipeline);
void MavlinkMessageCallback(void);
void FileManagerCallback(void);

#endif // LK_ESTIMATOR_H
