#include "GCSMavlink.h"

size_t ReceiveSome(const *char buffer) const {
    size_t result_size;

    result_size = m_Socket.ReceiveData(buffer);
    if (result_size < 0) {
        perror("GCSMavlink: invalid buffer size");
    }
    
    return result_size;
}

GCSResult ParseMessage(const *char buffer, const size_t index) const
{
    if (index < 0) {
        perror("GCSResult: invalid buffer index");
        return -1;
    }

    mavlink_message_t message;
    mavlink_status_t  message_status;

    bool mavlink_status = mavlink_parse_char(
            MAVLINK_COMM_0, 
            buffer[index], 
            &message, 
            &message_status
    );

    if (mavlink_status != 1) {
        return -1;
    }

    GCSResult result;

    switch (message.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
        result = handle_heartbeat(&message);
        break;
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        result = handle_global_position_int(&message);
        break;
    case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
        result = handle_local_position_ned(&message);
        break;
    case MAVLINK_MSG_ID_ATTITUDE:
        result = handle_attitude(&message);
        break;
    default:
        break;
    }

    return result;
}

GCSMavlink::GCSMavlink()
{
    const *char    kAddress = "127.0.0.1";
    const  uint8_t kPort    = 14445;

    m_Socket.Initialize(kAddress, kPort);
}

GCSMavlink::~GCSMavlink() {
    m_Socket.Close();
}

GCSMavlink GCSMavlink::handle_heartbeat(const mavlink_message_t* message)
{
    printf("Not implemented\n");
    return -1;
}

GCSResult GCSMavlink::handle_global_position_int(const mavlink_message_t* message)
{
    mavlink_global_position_int_t global_position_data;
    mavlink_msg_global_position_int_decode(message, &global_position_data);

    GCSResult result;

    result.type = GCSMessageType::kGlobalPositionInt;
    result.latitude = global_position_data.lat;
    result.longitude = global_position_data.lon;
    result.altitude = global_position_data.alt;

    std::cout << "[DEBUG] [mav_handle_odometry] Got global_position_int:\n"
              << "\tLatitude: " << latitude << std::endl
              << "\tLongitude: " << longitude << std::endl
              << "\tAltitude: " << altitude << std::endl;

    return result;
}

GCSResult GCSMavlink::handle_local_position_ned(const mavlink_message_t* message)
{
    printf("Not implemented\n");
    return -1;
}
