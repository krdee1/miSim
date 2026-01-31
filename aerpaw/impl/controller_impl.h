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

// Target location protocol functions
int loadTargets(const char* filename, double* targets, int maxClients);
void sendTarget(int clientId, const double* target);
int receiveTargetAck(int clientId);
int waitForReady(int clientId);
void sendFinished(int clientId);

// RTL and LAND protocol functions
void sendRTL(int clientId);
void sendLAND(int clientId);
int waitForAllRTLComplete(int numClients);
int waitForAllLANDComplete(int numClients);
void waitForUserInput();

#ifdef __cplusplus
}
#endif

#endif // CONTROLLER_IMPL_H
