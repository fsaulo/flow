#ifndef FLOW_GCS_MAVLINK_H
#define FLOW_GCS_MAVLINK_H

#include <mavlink/common/mavlink.h>

#include "UDPLink.h"

enum class GCSMessageType {
    kUnknown = 0,
    kHeartbeat,
    kGlobalPositionInt,
    kAttitude,
    kLocalPositionNed,
    kHihghresImu,
    kOpticalFlow
};

enum class GCSMessageStatus {
    kMessageOk = 0,
    kMessageUndefined,
    kMessageError,
    kMessageIncomplete
};

typedef struct GCSOpticalFlow {
    uint64_t time_ms;
    double flow_x;
    double flow_y;
    double flow_xgyro;
    double flow_ygyro;
    double flow_zgyro;
    uint8_t quality;
} gcs_optical_flow_t;

typedef struct GCSLocalPositionNed {
    uint64_t time_ms;
    double x;
    double y;
    double z;
    double vx;
    double vy;
    double vz;
} gcs_local_position_t;

typedef struct GCSGlobalPosition {
    uint64_t time_ms;
    uint32_t latitude;
    uint32_t longitude;
    int32_t altitude;
    int16_t vx;
    int16_t vy;
    int16_t vz;
} gcs_global_position_t;

typedef struct GCSAttitude {
    uint64_t time_ms;
    double   yaw;
    double   pitch;
    double   roll;
    double   rollspeed;
    double   pitchspeed;
    double   yawspeed;
} gcs_attitude_t;

typedef struct GCSAttitudeQuaternion {
    uint64_t time_ms;
    double   qw;
    double   qx;
    double   qy;
    double   qz;
} gcs_attitude_quaternion_t;

typedef struct GCSHighresImu {
    uint64_t time_ms;
    double xacc;
    double yacc;
    double zacc;
    double xgyro;
    double ygyro;
    double zgyro;
    double xmag;
    double ymag;
    double zmag;
    double temperature;
} gcs_highres_imu_t;

struct GCSResult {
    uint8_t  autopilot;

    GCSMessageType   type;
    GCSMessageStatus message_status;

    gcs_local_position_t local_position;
    gcs_global_position_t global_position;
    gcs_attitude_t attitude;
    gcs_attitude_quaternion_t attitude_quaternion;
    gcs_highres_imu_t highres_imu;
    gcs_optical_flow_t optical_flow;
};


class GCSMavlink
{
public:

    GCSMavlink();

    ~GCSMavlink();

    GCSResult ReceiveSome();

private:
    GCSResult handle_heartbeat(const mavlink_message_t& message) const;
    GCSResult handle_global_position_int(const mavlink_message_t& message) const;
    GCSResult handle_attitude(const mavlink_message_t& message) const;
    GCSResult handle_local_position_ned(const mavlink_message_t& message) const;
    GCSResult handle_highres_imu(const mavlink_message_t& message) const;
    GCSResult handle_optical_flow_rad(const mavlink_message_t& message) const;
    void handle_command_long(const mavlink_message_t& message) const;

    UDPLink m_Socket;
};


#endif // FLOW_GCS_MAVLINK_H
