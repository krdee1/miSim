#ifndef CONTROLLER_IMPL_H
#define CONTROLLER_IMPL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Server lifecycle
void initServer();
void acceptClient(int clientId);
void closeServer();

// Configuration
int loadTargets(const char* filename, double* targets, int maxClients);

// User interaction
void waitForUserInput();

// Binary protocol operations
int sendMessageType(int clientId, int msgType);
int sendTarget(int clientId, const double* coords);
int waitForAllMessageType(int numClients, int expectedType);

// Guidance loop operations
void sendGuidanceToggle(int numClients);
int  sendRequestPositions(int numClients);
int  recvPositions(int numClients, double* positions, int maxClients); // column-major maxClients x 3
void sleepMs(int ms);

#ifdef __cplusplus
}
#endif

#endif // CONTROLLER_IMPL_H
