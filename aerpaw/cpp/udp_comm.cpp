#include "udp_comm.h"
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>

static int sock = -1;
static sockaddr_in remote_addr;

void initComms(const char* ip, uint16_t port)
{
    if (sock != -1) return;

    sock = socket(AF_INET, SOCK_DGRAM, 0);

    memset(&remote_addr, 0, sizeof(remote_addr));
    remote_addr.sin_family = AF_INET;
    remote_addr.sin_port = htons(port);
    inet_pton(AF_INET, ip, &remote_addr.sin_addr);
}

void sendTarget(const double* pos)
{
    sendto(sock, pos, 3*sizeof(double), 0,
           (sockaddr*)&remote_addr, sizeof(remote_addr));
}

void recvOpMode(uint8_t* mode)
{
    recvfrom(sock, mode, sizeof(uint8_t), MSG_DONTWAIT,
             nullptr, nullptr);
}