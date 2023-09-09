#ifndef MAVLINK_INTERFACE_H
#define MAVLINK_INTERFACE_H

#include <unistd.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <mavlink/common/mavlink.h>

void mav_handle_heartbeat(const mavlink_message_t* message);
void mav_handle_global_position_int(const mavlink_message_t* message);
void mav_handle_attitude(const mavlink_message_t* message);
void mav_handle_local_position_ned(const mavlink_message_t* message);
void mav_receive_some(int socket_fd, struct sockaddr_in* src_addr, socklen_t* src_addr_len, bool* src_addr_set);
void mavlink_msg_callback(void);

#endif // MAVLINK_INTERFACE_H
