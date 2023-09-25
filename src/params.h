#ifndef FLOW_PARAMETERS
#define FLOW_PARAMETERS

#include <vector>

namespace flow {

const int kFileManagerSleepTimeSeconds = 1;
const int kThreadFrequencyHz = 30;
const int kMavlinkFrequencyHz = 500;
const int kTargetWidth  = 64;
const int kTargetHeight = 64;
const int kOutputWidth = 640;
const int kOutputHeight = 640;

const int kMaxCorners = 500;
const double kQualityLevel = 0.01;
const double kMinDistance = 10.0;

// Camera parameters
const double kCx = 320.5;
const double kCy = 240.5;
const double kFocalLength = 277.191356;
const double kHFov = 1.047;
const std::vector<double> kDistCoeffs = { 0, 0, 0, 0 ,0 };

const double kMaxVelThreshold = 5.0;
const double kInitialDt = 0.033;
const double kLowPassAlpha = 0.85;
const bool kFilterByStatus = true;
const bool kFilterByVelocity = false;
const int kScaleFactor = 1;
const int kScaleHeight = 100;
const int kFilterSize = 5;

const double kFlowXScaler = 1.0;
const double kFlowYScaler = 1.0;
const int kFlowIntertialCountMax = 100;

const std::string kGCSFlowFilename = "gcs_optical_flow.csv";
const std::string kGCSAttitudeFilename = "gcs_attitude.csv";
const std::string kGCSGlobalPositionFilename = "gcs_global_position_int.csv";
const std::string kGCSLocalPositionFilename = "gcs_local_position.csv";
const std::string kGCSHighresImuFilename = "gcs_highres_imu.csv";
const std::string kGCSPx4flowFilename = "gcs_px4flow.csv";

}

#endif // FLOW_PARAMETERS
