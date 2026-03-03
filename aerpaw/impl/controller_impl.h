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

// Scenario loading (scenario.csv)
// Number of elements in the flat params array passed to guidance_step.
// Indices: 0-13 scalars, 14-16 domainMin, 17-19 domainMax, 20-21 objectivePos,
//          22-25 objectiveVar (2x2 col-major: [v11,v12,v21,v22]), 26 sensorPerformanceMinimum.
#define NUM_SCENARIO_PARAMS 27
// Maximum number of obstacles (upper bound for pre-allocated arrays).
#define MAX_OBSTACLES 8

// Load guidance parameters from scenario.csv into flat params[NUM_SCENARIO_PARAMS].
// Returns 1 on success, 0 on failure.
int loadScenario(const char* filename, double* params);

// Load initial UAV positions from scenario.csv (initialPositions column).
// targets is a column-major [maxClients x 3] array (same layout as loadTargets).
// Returns number of positions loaded.
int loadInitialPositions(const char* filename, double* targets, int maxClients);

// Load obstacle bounding-box corners from scenario.csv.
// obstacleMin and obstacleMax are column-major [maxObstacles x 3] arrays
// (same layout convention as loadTargets / recvPositions).
// Returns the number of obstacles loaded.
int loadObstacles(const char* filename,
                  double* obstacleMin,
                  double* obstacleMax,
                  int maxObstacles);

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
