#ifndef CONTROLLER_IMPL_H
#define CONTROLLER_IMPL_H

#ifdef __cplusplus
extern "C" {
#endif

void initServer();
void acceptClient(int clientId);
void sendMessage(int clientId);
int receiveAck(int clientId);
void closeServer();

#ifdef __cplusplus
}
#endif

#endif // CONTROLLER_IMPL_H
