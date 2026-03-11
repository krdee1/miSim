#include <iostream>
#include "controller.h"
#include "controller_impl.h" // TCP implementation header

int main() {
    // Derive numClients from initialPositions in scenario.csv
    double targets[MAX_CLIENTS_PER_PARAM * 3];
    int numClients = loadInitialPositions("config/scenario.csv",
                                          targets, MAX_CLIENTS_PER_PARAM);
    if (numClients < 1) {
        std::cerr << "Failed to parse numClients from scenario.csv\n";
        return 1;
    }
    std::cout << "Parsed " << numClients << " UAV(s) from scenario.csv\n";

    // Call MATLAB-generated server function
    controller(numClients);

    std::cout << "Server finished.\n";
    return 0;
}