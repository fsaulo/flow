#ifndef FLOW_UDP_LINK_H
#define FLOW_UDP_LINK_H

#include <iostream>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

class UDPLink {
public:

    UDPLink(void) { m_socket_fd = -1; }
    ~UDPLink() { Close(); }

    ssize_t SendData(const void* data, size_t data_size, const sockaddr_in& dest_addr);
    ssize_t ReceiveData(void* buffer);

    bool isSourceAddressSet() const { return m_src_addr_set; }
    bool Initialize(const char* bind_address, int bind_port);
    void Close() const { close(m_socket_fd); }

private:
    int  m_socket_fd;
    bool m_src_addr_set;

    struct sockaddr_in m_addr;
    struct sockaddr_in m_src_addr;

    socklen_t m_src_addr_len;
};

#endif // FLOW_UDP_LINK_H
