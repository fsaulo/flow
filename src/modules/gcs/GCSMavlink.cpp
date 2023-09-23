#include <iostream>
#include <chrono>

#include "GCSMavlink.h"


GCSResult GCSMavlink::ReceiveSome()
{
    int  nret;
    char buffer[2048];

    nret = m_Socket.ReceiveData(buffer, sizeof(buffer));

    if (nret < 0) {
        perror("GCSResult: invalid buffer index");
        
        GCSResult result;
        result.message_status = GCSMessageStatus::kMessageUndefined;

        return result;
    }

    if (nret == 0) {
        GCSResult result;
        result.message_status = GCSMessageStatus::kMessageUndefined;

        printf("failed\n");
        return result;
    }

    GCSResult result;

    mavlink_message_t message;
    mavlink_status_t  message_status;
    
    for (int i = 0; i < nret; ++i) {
        uint8_t mavlink_status = mavlink_parse_char(
                MAVLINK_COMM_0, 
                buffer[i], 
                &message, 
                &message_status
        );

        if (mavlink_status != MAVLINK_FRAMING_OK) {
            continue;
        }

        switch (message.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
            result = handle_heartbeat(message);
            break;
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
            result = handle_global_position_int(message);
            break;
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
            result = handle_local_position_ned(message);
            break;
        case MAVLINK_MSG_ID_ATTITUDE:
            result = handle_attitude(message);
            break;
        case MAVLINK_MSG_ID_HIGHRES_IMU:
            result = handle_highres_imu(message);
        default:
            break;
        }
    }

    return result;
}

GCSMavlink::GCSMavlink()
{
    const char*   kAddress = "0.0.0.0";
    const uint16_t kPort   = 14540;
    
    bool success = m_Socket.Initialize(kAddress, kPort);
    if(!success) {
        perror("GCSMavlink: couldn't create a connection");
    }
}

GCSMavlink::~GCSMavlink() {
    m_Socket.Close();
}

GCSResult GCSMavlink::handle_heartbeat(const mavlink_message_t& message) const
{
    GCSResult result;
    result.message_status = GCSMessageStatus::kMessageUndefined;

    return result;
}

GCSResult GCSMavlink::handle_global_position_int(const mavlink_message_t& message) const
{
    mavlink_global_position_int_t global_position_data;
    mavlink_msg_global_position_int_decode(&message, &global_position_data);

    // Compute timestamp relative to Unix current time since epoch
    // This is not really accurate data timestamp, but it is good enough
    auto sys_tick = std::chrono::system_clock::now().time_since_epoch();
    auto unix_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(sys_tick).count();

    gcs_global_position_t global_position;
    global_position.time_ms = unix_epoch;
    global_position.latitude = global_position_data.lat;
    global_position.longitude = global_position_data.lon;
    global_position.altitude = global_position_data.alt;
    global_position.vx = global_position_data.vx;
    global_position.vy = global_position_data.vy;
    global_position.vz = global_position_data.vz;

    GCSResult result;
    result.type = GCSMessageType::kGlobalPositionInt;
    result.message_status = GCSMessageStatus::kMessageOk;
    result.global_position = global_position;

    return result;
}

GCSResult GCSMavlink::handle_attitude(const mavlink_message_t& message) const
{
    mavlink_attitude_t attitude_data;
    mavlink_msg_attitude_decode(&message, &attitude_data);

    auto sys_tick = std::chrono::system_clock::now().time_since_epoch();
    auto unix_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(sys_tick).count();

    gcs_attitude_t attitude;
    attitude.time_ms = unix_epoch;
    attitude.roll = attitude_data.roll;
    attitude.pitch = attitude_data.pitch;
    attitude.yaw = attitude_data.yaw;
    attitude.rollspeed = attitude_data.rollspeed;
    attitude.pitchspeed = attitude_data.pitchspeed;
    attitude.yawspeed = attitude_data.yawspeed;

    GCSResult result;
    result.type = GCSMessageType::kAttitude;
    result.message_status = GCSMessageStatus::kMessageOk;
    result.attitude = attitude;

    return result;
}

GCSResult GCSMavlink::handle_local_position_ned(const mavlink_message_t& message) const
{
    mavlink_local_position_ned_t local_position_data;
    mavlink_msg_local_position_ned_decode(&message, &local_position_data);

    auto sys_tick = std::chrono::system_clock::now().time_since_epoch();
    auto unix_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(sys_tick).count();

    gcs_local_position_t local_position;
    local_position.time_ms = unix_epoch;
    local_position.x = local_position_data.x;
    local_position.y = local_position_data.y;
    local_position.z = local_position_data.z;
    local_position.vx = local_position_data.vx;
    local_position.vy = local_position_data.vy;
    local_position.vz = local_position_data.vz;

    GCSResult result;
    result.message_status = GCSMessageStatus::kMessageOk;
    result.type = GCSMessageType::kLocalPositionNed;
    result.local_position = local_position;

    return result;
}

GCSResult GCSMavlink::handle_highres_imu(const mavlink_message_t& message) const
{
    mavlink_highres_imu_t highres_imu_data;
    mavlink_msg_highres_imu_decode(&message, &highres_imu_data);

    auto sys_tick = std::chrono::system_clock::now().time_since_epoch();
    auto unix_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(sys_tick).count();

    gcs_highres_imu_t highres_imu;
    highres_imu.time_ms = unix_epoch;
    highres_imu.xacc = highres_imu_data.xacc;
    highres_imu.yacc = highres_imu_data.yacc;
    highres_imu.zacc = highres_imu_data.zacc;
    highres_imu.xgyro = highres_imu_data.xgyro;
    highres_imu.ygyro = highres_imu_data.ygyro;
    highres_imu.zgyro = highres_imu_data.zgyro;
    highres_imu.xmag = highres_imu_data.xmag;
    highres_imu.ymag = highres_imu_data.ymag;
    highres_imu.zmag = highres_imu_data.zmag;
    highres_imu.temperature = highres_imu_data.temperature;

    GCSResult result;
    result.message_status = GCSMessageStatus::kMessageOk;
    result.type = GCSMessageType::kHihghresImu;
    result.highres_imu = highres_imu;

    return result;
}
