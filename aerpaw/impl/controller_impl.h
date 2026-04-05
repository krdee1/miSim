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
// Indices (0-based):
//   0-7   scalars (timestep..barrierExponent)
//   8-11  collisionRadius[1:4]  (per-UAV, MAX_CLIENTS slots)
//   12-15 comRange[1:4]
//   16-19 alphaDist[1:4]
//   20-23 betaDist[1:4]
//   24-27 alphaTilt[1:4]
//   28-31 betaTilt[1:4]
//   32-34 domainMin
//   35-37 domainMax
//   38-39 objectivePos
//   40-43 objectiveVar (2x2 col-major)
//   44    sensorPerformanceMinimum
//   45    useDoubleIntegrator
//   46    dampingCoeff
//   47    useFixedTopology
#define NUM_SCENARIO_PARAMS 48
#define MAX_CLIENTS_PER_PARAM 4
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
