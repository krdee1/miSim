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
  //  Initialize server
  initServer();
  //  Accept clients
  for (int i{0}; i < numClients; i++) {
    acceptClient(i + 1);
  }
  //  Send messages to clients
  for (int i{0}; i < numClients; i++) {
    sendMessage(i + 1);
  }
  //  Receive acknowledgements
  for (int i{0}; i < numClients; i++) {
    receiveAck(i + 1);
  }
  //  Digest ACKs
  //  Close server
  closeServer();
}

// End of code generation (controller.cpp)
