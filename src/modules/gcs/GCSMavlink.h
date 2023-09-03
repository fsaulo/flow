#ifndef FLOW_GCS_MAVLINK_H
#define FLOW_GCS_MAVLINK_H

#include <mavlink/common/mavlink.h>

#include "UDPLink.h"

class GCSMavlink
{
private:
    void handle_heartbeat(const mavlink_message_t* message);
    void handle_global_position_int(const mavlink_message_t* message);
    void handle_attitude(const mavlink_message_t* message);
    void handle_local_position_ned(const mavlink_message_t* message);

    UDPLink m_Socket;
public:
    enum class GCSMessageType {
        kHeartbeat = 0,
        kGlobalPositionInt,
        kAttitude,
        kLocalPositonNed
    };

    struct GCSResult {
        GCSMessageType type;
        uint8_t autopilot;
        uint32_t latitude;
        uint32_t longitude;
        uint32_t altitude;
        double   linear_velocity;
        double   angluar_velocity;
        double   yaw;
        double   pitch;
        double   roll;
        double   qw;
        double   qx;
        double   qy;
        double   qz;
    }

    GCSMavlink();

    ~GCSMavlink();

    GCSResult ReceiveSome() const;
}

#endif // FLOW_GCS_MAVLINK_H
