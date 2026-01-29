#ifndef UDP_COMM_H
#define UDP_COMM_H

#ifdef __cplusplus
extern "C" {
#endif

void initComms(const char* ip, uint16_t port);
void sendTarget(const double* pos);

#ifdef __cplusplus
}
#endif

#endif