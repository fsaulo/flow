#ifndef FLOW_GCS_MAVLINK_H
#define FLOW_GCS_MAVLINK_H

#include <mavlink/common/mavlink.h>

#include "UDPLink.h"

enum class GCSMessageType {
    kUnknown = 0,
    kHeartbeat,
    kGlobalPositionInt,
    kAttitude,
    kLocalPositonNed
};

enum class GCSMessageStatus {
    kMessageOk = 0,
    kMessageUndefined,
    kMessageError,
    kMessageIncomplete
};

typedef struct GCSPosition {
   double x;
   double y;
   double z;
} gcs_local_position_t;

typedef struct GCSGlobalPosition {
    uint32_t latitude;
    uint32_t longitude;
    uint32_t altitude;
} gcs_global_position_t;

typedef struct GCSAttitude {
    double   linear_velocity;
    double   angluar_velocity;
    double   yaw;
    double   pitch;
    double   roll;
} gcs_attitude_t;

typedef struct GCSAttitudeQuaternion {
    double   qw;
    double   qx;
    double   qy;
    double   qz;
} gcs_attitude_quaternion_t;

struct GCSResult {
    uint8_t  autopilot;

    GCSMessageType   type;
    GCSMessageStatus message_status;

    gcs_local_position_t local_position;
    gcs_global_position_t global_position;
    gcs_attitude_t attitude;
    gcs_attitude_quaternion_t attitude_quaternion;
};


class GCSMavlink
{
public:

    GCSMavlink();

    ~GCSMavlink();

    size_t ReceiveSome(char *buffer);
    GCSResult ParseMessage(const char* buffer, const size_t index) const;


private:
    GCSResult handle_heartbeat(const mavlink_message_t& message) const;
    GCSResult handle_global_position_int(const mavlink_message_t& message) const;
    GCSResult handle_attitude(const mavlink_message_t& message) const;
    GCSResult handle_local_position_ned(const mavlink_message_t& message) const;

    UDPLink m_Socket;
};


#endif // FLOW_GCS_MAVLINK_H
