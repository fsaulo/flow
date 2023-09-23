#include <iostream>

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
    std::cout << "[DEBUG] [handle_heartbeat] not implemented" << std::endl;

    GCSResult result;
    result.message_status = GCSMessageStatus::kMessageUndefined;

    return result;
}

GCSResult GCSMavlink::handle_global_position_int(const mavlink_message_t& message) const
{
    mavlink_global_position_int_t global_position_data;
    mavlink_msg_global_position_int_decode(&message, &global_position_data);

    gcs_global_position_t global_position;
    global_position.latitude  = global_position_data.lat;
    global_position.longitude = global_position_data.lon;
    global_position.altitude  = global_position_data.alt;

    GCSResult result;
    result.type            = GCSMessageType::kGlobalPositionInt;
    result.global_position = global_position;

    std::cout << "[DEBUG] [handle_global_position_int] Got global_position_int:\n"
              << "\tLatitude:  " << global_position.latitude  << std::endl
              << "\tLongitude: " << global_position.longitude << std::endl
              << "\tAltitude:  " << global_position.altitude  << std::endl;

    return result;
}

GCSResult GCSMavlink::handle_attitude(const mavlink_message_t& message) const
{
    printf("Not implemented\n");

    GCSResult result;
    result.message_status = GCSMessageStatus::kMessageUndefined;

    return result;
}

GCSResult GCSMavlink::handle_local_position_ned(const mavlink_message_t& message) const
{
    printf("Not implemented\n");

    GCSResult result;
    result.message_status = GCSMessageStatus::kMessageUndefined;

    return result;
}
