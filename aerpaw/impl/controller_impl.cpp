#include "controller_impl.h"
#include <iostream>
#include <vector>
#include <string>
#include <cstring>
#include <cstdio>
#include <limits>
#include <sys/socket.h>
#include <sys/select.h>
#include <arpa/inet.h>
#include <unistd.h>

#define SERVER_PORT 5000
#define SERVER_IP "127.0.0.1"

static int serverSocket = -1;
static std::vector<int> clientSockets;

void initSockets() {}
void cleanupSockets() {}

void initServer() {
    initSockets();
    serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    if(serverSocket < 0) { std::cerr << "Socket creation failed\n"; return; }

    sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    serverAddr.sin_port = htons(SERVER_PORT);

    int opt = 1;
    setsockopt(serverSocket, SOL_SOCKET, SO_REUSEADDR, (char*)&opt, sizeof(opt));

    if(bind(serverSocket, (sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
        std::cerr << "Bind failed\n"; return;
    }
    if(listen(serverSocket, 5) < 0) {
        std::cerr << "Listen failed\n"; return;
    }

    std::cout << "Server initialized\n";
}

void acceptClient(int clientId) {
    sockaddr_in clientAddr;
    socklen_t addrLen = sizeof(clientAddr);
    int clientSock = accept(serverSocket, (sockaddr*)&clientAddr, &addrLen);
    if(clientSock < 0) { std::cerr << "Accept failed for client " << clientId << "\n"; return; }
    clientSockets.push_back(clientSock);
    std::cout << "Client " << clientId << " connected\n";
}

void sendMessage(int clientId) {
    if(clientId <= 0 || clientId > clientSockets.size()) return;
    const char* msg = "Hello from server";
    send(clientSockets[clientId-1], msg, strlen(msg), 0);
    std::cout << "Sent message to client " << clientId << "\n";
}

int receiveAck(int clientId) {
    if(clientId <= 0 || clientId > clientSockets.size()) return 0;
    char buffer[1024];
    int len = recv(clientSockets[clientId-1], buffer, sizeof(buffer)-1, 0);
    if(len <= 0) return 0;
    buffer[len] = '\0';
    std::cout << "Received ACK from client " << clientId << ": " << buffer << "\n";
    return 1;
}

void closeServer() {
    for(auto sock : clientSockets) {
        close(sock);
    }
    close(serverSocket);
    cleanupSockets();
}

// Load target coordinates from file
// File format: one line per UAV with "x,y,z" coordinates
// Returns number of targets loaded
int loadTargets(const char* filename, double* targets, int maxClients) {
    FILE* file = fopen(filename, "r");
    if (!file) {
        std::cerr << "Failed to open targets file: " << filename << "\n";
        return 0;
    }

    int count = 0;
    double x, y, z;
    // MATLAB uses column-major order, so for a maxClients x 3 matrix:
    // Column 1 (x): indices 0, 1, 2, ...
    // Column 2 (y): indices maxClients, maxClients+1, ...
    // Column 3 (z): indices 2*maxClients, 2*maxClients+1, ...
    while (count < maxClients && fscanf(file, "%lf,%lf,%lf", &x, &y, &z) == 3) {
        targets[count + 0 * maxClients] = x;  // Column 1
        targets[count + 1 * maxClients] = y;  // Column 2
        targets[count + 2 * maxClients] = z;  // Column 3
        std::cout << "Loaded target " << (count + 1) << ": " << x << "," << y << "," << z << "\n";
        count++;
    }

    fclose(file);
    return count;
}

// Send target coordinates to a client
// target points to 3 doubles: [x, y, z]
void sendTarget(int clientId, const double* target) {
    if (clientId <= 0 || clientId > (int)clientSockets.size()) return;

    char buffer[256];
    snprintf(buffer, sizeof(buffer), "TARGET:%.6f,%.6f,%.6f",
             target[0], target[1], target[2]);

    send(clientSockets[clientId - 1], buffer, strlen(buffer), 0);
    std::cout << "Sent target to client " << clientId << ": " << buffer << "\n";
}

// Receive and validate ACK:TARGET response
// Returns 1 if ACK:TARGET received, 0 otherwise
int receiveTargetAck(int clientId) {
    if (clientId <= 0 || clientId > (int)clientSockets.size()) return 0;

    char buffer[256];
    int len = recv(clientSockets[clientId - 1], buffer, sizeof(buffer) - 1, 0);
    if (len <= 0) return 0;
    buffer[len] = '\0';

    std::cout << "Received from client " << clientId << ": " << buffer << "\n";

    if (strncmp(buffer, "ACK:TARGET", 10) == 0) {
        return 1;
    }
    return 0;
}

// Wait for READY signal from client
// Returns 1 if READY received, 0 otherwise
int waitForReady(int clientId) {
    if (clientId <= 0 || clientId > (int)clientSockets.size()) return 0;

    char buffer[256];
    int len = recv(clientSockets[clientId - 1], buffer, sizeof(buffer) - 1, 0);
    if (len <= 0) return 0;
    buffer[len] = '\0';

    std::cout << "Received from client " << clientId << ": " << buffer << "\n";

    if (strncmp(buffer, "READY", 5) == 0) {
        return 1;
    }
    return 0;
}

// Send COMPLETE message to signal graceful shutdown
void sendFinished(int clientId) {
    if (clientId <= 0 || clientId > (int)clientSockets.size()) return;

    const char* msg = "FINISHED";
    send(clientSockets[clientId - 1], msg, strlen(msg), 0);
    std::cout << "Sent FINISHED to client " << clientId << "\n";
}

// Send RTL (Return-To-Launch) command to a client
void sendRTL(int clientId) {
    if (clientId <= 0 || clientId > (int)clientSockets.size()) return;

    const char* msg = "RTL";
    send(clientSockets[clientId - 1], msg, strlen(msg), 0);
    std::cout << "Sent RTL to client " << clientId << "\n";
}

// Send LAND command to a client
void sendLAND(int clientId) {
    if (clientId <= 0 || clientId > (int)clientSockets.size()) return;

    const char* msg = "LAND";
    send(clientSockets[clientId - 1], msg, strlen(msg), 0);
    std::cout << "Sent LAND to client " << clientId << "\n";
}

// Wait for a specific message from ALL clients simultaneously using select()
// Returns 1 if all clients sent the expected message, 0 otherwise
static int waitForAllMessage(int numClients, const char* expectedMessage) {
    if (numClients <= 0 || numClients > (int)clientSockets.size()) return 0;

    std::vector<std::string> accumulated(numClients);
    std::vector<bool> completed(numClients, false);
    int completedCount = 0;
    char buffer[512];

    while (completedCount < numClients) {
        // Build fd_set for select()
        fd_set readfds;
        FD_ZERO(&readfds);
        int maxfd = -1;

        for (int i = 0; i < numClients; i++) {
            if (!completed[i]) {
                FD_SET(clientSockets[i], &readfds);
                if (clientSockets[i] > maxfd) maxfd = clientSockets[i];
            }
        }

        // Wait for any socket to have data
        int ready = select(maxfd + 1, &readfds, nullptr, nullptr, nullptr);
        if (ready <= 0) return 0;

        // Check each socket
        for (int i = 0; i < numClients; i++) {
            if (!completed[i] && FD_ISSET(clientSockets[i], &readfds)) {
                int len = recv(clientSockets[i], buffer, sizeof(buffer) - 1, 0);
                if (len <= 0) return 0;
                buffer[len] = '\0';
                std::cout << "Received from client " << (i + 1) << ": " << buffer << "\n";
                accumulated[i] += buffer;

                // Check if expected message received
                if (accumulated[i].find(expectedMessage) != std::string::npos) {
                    completed[i] = true;
                    completedCount++;
                    std::cout << "Client " << (i + 1) << " completed " << expectedMessage << "\n";
                }
            }
        }
    }

    return 1;
}

// Wait for RTL_COMPLETE from ALL clients simultaneously
// Returns 1 if all clients completed RTL, 0 otherwise
int waitForAllRTLComplete(int numClients) {
    return waitForAllMessage(numClients, "RTL_COMPLETE");
}

// Wait for LAND_COMPLETE from ALL clients simultaneously
// Returns 1 if all clients completed LAND, 0 otherwise
int waitForAllLANDComplete(int numClients) {
    return waitForAllMessage(numClients, "LAND_COMPLETE");
}

// Wait for user to press Enter
void waitForUserInput() {
    std::cout << "Press Enter to close experiment (RTL + LAND)...\n";
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}
