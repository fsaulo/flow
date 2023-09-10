#include "UDPLink.h"

bool UDPLink::Initialize(const char* bind_address, int bind_port)
{
    m_socket_fd = socket(PF_INET, SOCK_DGRAM, 0);
    if (m_socket_fd < 0) {
        perror("UDPLink: Socket error");
        return false;
    }

    memset(&m_src_addr, 0, sizeof(m_src_addr));
    m_src_addr.sin_family = AF_INET;
    inet_pton(AF_INET, bind_address, &(m_src_addr.sin_addr));
    m_src_addr.sin_port = htons(bind_port);

    if (bind(m_socket_fd, (struct sockaddr*)(&m_src_addr), sizeof(m_src_addr)) != 0) {
        perror("UDPLink: Bind error");
        close(m_socket_fd);
        return false;
    }

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 1e5;
    if (setsockopt(m_socket_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        perror("UDPLink: setsockopt error");
        close(m_socket_fd);
        return false;
    }

    m_src_addr_len = sizeof(m_src_addr);
    m_src_addr_set = false;
    return true;
}

ssize_t UDPLink::SendData(const void* data, size_t data_size, const sockaddr_in& dest_addr)
{
    if (!isSourceAddressSet()) {
        perror("UDPLink: source address not set");
        return -1;
    }

    return sendto(
        m_socket_fd,
        data,
        data_size,
        0,
        (struct sockaddr*)&dest_addr, 
        sizeof(dest_addr)
    );
}

ssize_t UDPLink::ReceiveData(char* buffer, size_t size)
{
    const int bytes_received = recvfrom(
        m_socket_fd, 
        buffer, 
        size, 
        0, 
        (struct sockaddr*)&m_src_addr, 
        &m_src_addr_len
    );

    if (bytes_received < 0) {
        perror("UDPLink: recfrom error");
    }

    if (bytes_received > 0) {
        m_src_addr_set = true;
    }

    return bytes_received;
}
