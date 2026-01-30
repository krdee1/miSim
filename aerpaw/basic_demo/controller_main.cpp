#include <iostream>
#include "controller.h" 
#include "controller_impl.h" // TCP implementation header

int main() {
    // Number of clients to handle
    int numClients = 2;  // for now

    std::cout << "Initializing TCP server...\n";

    // Call MATLAB-generated server function
    controller(numClients);

    std::cout << "Server finished.\n";
    return 0;
}