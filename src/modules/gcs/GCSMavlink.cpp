#include "GCSMavlink.h"

size_t GCSMavlink::ReceiveSome(char* buffer) 
{
    size_t result_size;

    result_size = m_Socket.ReceiveData(buffer);
    if (result_size < 0) {
        perror("GCSMavlink: invalid buffer size");
    }
    
    return result_size;
}

GCSResult GCSMavlink::ParseMessage(const char* buffer, const size_t index) const
{
    if (index < 0) {
        perror("GCSResult: invalid buffer index");
        
        GCSResult result;
        result.message_status = GCSMessageStatus::kMessageUndefined;

        return result;
    }

    mavlink_message_t message;
    mavlink_status_t  message_status;

    bool mavlink_status = mavlink_parse_char(
            MAVLINK_COMM_0, 
            buffer[index], 
            &message, 
            &message_status
    );

    GCSResult result;

    if (mavlink_status != MAVLINK_FRAMING_OK) {
        result.message_status = GCSMessageStatus::kMessageError;
        return result;
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

    return result;
}

GCSMavlink::GCSMavlink()
{
    const char*   kAddress = "127.0.0.1";
    const uint16_t kPort    = 14445;

    m_Socket.Initialize(kAddress, kPort);
}

GCSMavlink::~GCSMavlink() {
    m_Socket.Close();
}

GCSResult GCSMavlink::handle_heartbeat(const mavlink_message_t& message) const
{
    printf("Not implemented\n");

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

GCSResult GCSMavlink::handle_local_position_ned(const mavlink_message_t& message) const
{
    printf("Not implemented\n");

    GCSResult result;
    result.message_status = GCSMessageStatus::kMessageUndefined;

    return result;
}
