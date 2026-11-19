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
//   38    numObjectiveComponents (1-3; inferred from objectivePos field length)
//   39-42 objectivePos flat [x1,y1,x2,y2] (components 1-2; zero-padded)
//   43-50 objectiveVar flat [v11,v12,v21,v22 per component] (components 1-2; zero-padded)
//   51    sensorPerformanceMinimum
//   52    useDoubleIntegrator  (0=single-integrator, 1=double-integrator)
//   53    dampingCoeff
//   54    useFixedTopology     (0=dynamic lesser-neighbor, 1=fixed)
//   55-58 txPower[1:4]         (per-UAV transmit power, Watts; SINR comms model)
//   59    useSinrComms         (0=fixed-radius comms, 1=SINR/path-loss comms)
//   60    sinrThreshold        (SINR connectivity/CBF threshold, dB)
//   61    pathLossExponent     (path-loss exponent n)
//   62    ambientTemp          (ambient temperature, K; thermal noise)
//   63    centerFreq           (carrier frequency, Hz; path-loss offset)
//   64    bandwidth            (channel bandwidth, Hz; thermal noise)
//   65    sensorType           (0=sigmoidSensor sensing, 1=rfSensor service; must match buildSensorType.m)
//   66-69 serviceTxPower[1:4]  (rfSensor transmit power, W)
//   70-73 beamwidthExponent[1:4] (rfSensor cos^n pattern exponent)
//   74-77 serviceTilt[1:4]     (fixed boresight tilt from nadir, deg)
//   78-81 serviceAzimuth[1:4]  (fixed boresight azimuth from +y, deg)
//   82    serviceRxGain        (user antenna gain, dBi)
//   83    serviceLossExponent  (service path-loss exponent)
//   84    serviceTemp          (service noise temperature, K)
//   85    serviceCenterFreq    (service carrier, Hz)
//   86    serviceBandwidth     (service bandwidth, Hz)
//   87-88 objectivePos [x3,y3] (component 3; zero-padded)
//   89-92 objectiveVar [component 3] (zero-padded)
#define NUM_SCENARIO_PARAMS 93
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
// Block until a SINGLE client sends expectedType (used by the staggered
// flyout/flyback to wait on one UAV while the other is mid-flight).
int waitForClientMessageType(int clientId, int expectedType);

// Guidance loop operations
void setGuidanceStep(int step, int totalSteps);  // call at the top of each guidance iteration
void sendGuidanceToggle(int numClients);
int  sendRequestPositions(int numClients);
int  recvPositions(int numClients, double* positions, int maxClients); // column-major maxClients x 3
void sleepMs(int ms);

#ifdef __cplusplus
}
#endif

#endif // CONTROLLER_IMPL_H
