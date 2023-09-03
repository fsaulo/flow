#include "UDPLink.h"

bool UDPLink::Initialize(const char* bind_address, int bind_port)
{
    m_socket_fd = socket(PF_INET, SOCK_DGRAM, 0);
    if (m_socket_fd < 0) {
        perror("UDPLink: Socket error");
        return false;
    }

    memset(&m_addr, 0, sizeof(m_addr));
    m_addr.sin_family = AF_INET;
    inet_pton(AF_INET, bind_address, &(m_addr.sin_addr));
    m_addr.sin_port = htons(bind_port);

    if (bind(m_socket_fd, (struct sockaddr*)(&m_addr), sizeof(m_addr)) != 0) {
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
    return sendto(
        m_socket_fd, 
        data, 
        data_size, 
        0, 
        (struct sockaddr*)&dest_addr, 
        sizeof(dest_addr)
    );
}

ssize_t UDPLink::ReceiveData(void* buffer)
{
    if (!isSourceAddressSet()) {
        perror("UDPLink: source address not set");
        return -1;
    }

    ssize_t bytes_received = recvfrom(
            m_socket_fd, 
            buffer, 
            sizeof(buffer), 
            0, 
            (struct sockaddr*)&m_src_addr, 
            &m_src_addr_len
    );

    if (bytes_received > 0) {
        m_src_addr_set = true;
    }

    return bytes_received;
}
