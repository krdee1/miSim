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
// Indices (0-based): 0-7 scalars (timestep, maxIter, minAlt, discretizationStep,
//   protectedRange, initialStepSize, barrierGain, barrierExponent),
//   8-10 domainMin, 11-13 domainMax, 14-15 objectivePos,
//   16-19 objectiveVar (2x2 col-major: [v11,v12,v21,v22]), 20 sensorPerformanceMinimum.
#define NUM_SCENARIO_PARAMS 21
// Maximum number of UAVs / obstacles (upper bounds for pre-allocated arrays).
#define MAX_CLIENTS 4
#define MAX_OBSTACLES 8

// Load global guidance parameters from scenario.csv into flat params[NUM_SCENARIO_PARAMS].
// Returns 1 on success, 0 on failure.
int loadScenario(const char* filename, double* params);

// Load per-UAV parameters from scenario.csv (CSV columns 8-13).
// perAgentParams is column-major [maxClients x 6]:
//   perAgentParams[agent + param * maxClients], param order:
//   0 collisionRadius, 1 comRange, 2 alphaDist, 3 betaDist, 4 alphaTilt, 5 betaTilt.
// Returns number of UAVs found (values per column); 0 on error.
int loadPerAgentParams(const char* filename, double* perAgentParams, int maxClients);

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
double getTimeMs(); // monotonic wall-clock time in milliseconds

#ifdef __cplusplus
}
#endif

#endif // CONTROLLER_IMPL_H
