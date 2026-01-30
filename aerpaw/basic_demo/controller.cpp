//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// controller.cpp
//
// Code generation for function 'controller'
//

// Include files
#include "controller.h"
#include "controller_impl.h"

// Function Definitions
void controller(int numClients)
{
  static const char b_filename[12]{"targets.txt"};
  double targets[12];
  char filename[12];
  //  Maximum clients supported
  //  Allocate targets array (MAX_CLIENTS x 3)
  //  Load targets from file
  //  Define filename as null-terminated character array for C compatibility
  for (int i{0}; i < 12; i++) {
    targets[i] = 0.0;
    filename[i] = b_filename[i];
  }
  //  loadTargets fills targets array (row-major: x1,y1,z1,x2,y2,z2,...)
  loadTargets(&filename[0], &targets[0], 4);
  //  Initialize server
  initServer();
  //  Accept clients
  for (int i{0}; i < numClients; i++) {
    acceptClient(i + 1);
  }
  //  Send target coordinates to each client
  for (int i{0}; i < numClients; i++) {
    double target[3];
    //  Get target for this client (1x3 array)
    target[0] = targets[i];
    target[1] = targets[i + 4];
    target[2] = targets[i + 8];
    sendTarget(i + 1, &target[0]);
  }
  //  Receive TARGET acknowledgments
  for (int i{0}; i < numClients; i++) {
    receiveTargetAck(i + 1);
  }
  //  Check all ACKs received
  //  Wait for READY signals (UAVs have reached their targets)
  for (int i{0}; i < numClients; i++) {
    waitForReady(i + 1);
  }
  //  Check all READY signals received
  //  Send COMPLETE to all clients before closing
  for (int i{0}; i < numClients; i++) {
    sendFinished(i + 1);
  }
  //  Close server
  closeServer();
}

// End of code generation (controller.cpp)
