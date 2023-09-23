#ifndef FLOW_PARAMETERS
#define FLOW_PARAMETERS

#include <vector>

namespace flow {

const int kFileManagerSleepTimeSeconds = 10;
const int kThreadFrequencyHz = 30;
const int kTargetWidth  = 640;
const int kTargetHeight = 640;

const int kMaxCorners = 500;
const double kQualityLevel = 0.01;
const double kMinDistance = 10.0;

// Camera parameters
const double kCx = 320.5;
const double kCy = 240.5;
const double kFocalLength = 277.191356;
const double kHFov = 1.047;
const std::vector<double> kDistCoeffs = { 0, 0, 0, 0 ,0 };

const double kMaxVelThreshold = 10;
const double kInitialDt = 0.033;
const bool kFilterByStatus = true;
const bool kFilterByVelocity = false;
const int kScaleFactor = 1;
const int kScaleHeight = 100;
const int kFilterSize = 10;

const std::string kFlowAngVelFilename = "flow_angular_velocity.csv";
const std::string kFlowLinVelFilename = "flow_linear_velocity.csv";
const std::string kFlowQuaVelFilename = "flow_quaternion_orientation.csv";

const std::string kGCSAttitudeFilename = "gcs_attitude.csv";
const std::string kGCSGlobalPositionFilename = "gcs_global_position_int.csv";
const std::string kGCSLocalPositionFilename = "gcs_local_position.csv";
const std::string kGCSHighresImuFilename = "gcs_highres_imu.csv";

}

#endif // FLOW_PARAMETERS
