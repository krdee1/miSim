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
        std::cerr << "Failed to open config file: " << filename << "\n";
        return 0;
    }

    int count = 0;
    char line[256];
    bool inTargets = false;

    // Simple YAML parser for targets section
    // Expects format:
    //   targets:
    //     - [x, y, z]
    //     - [x, y, z]
    while (fgets(line, sizeof(line), file) && count < maxClients) {
        // Check if we've entered the targets section
        if (strstr(line, "targets:") != nullptr) {
            inTargets = true;
            continue;
        }

        // If we hit another top-level key (no leading whitespace), exit targets section
        if (inTargets && line[0] != ' ' && line[0] != '\t' && line[0] != '\n' && line[0] != '#') {
            break;
        }

        // Parse target entries: "  - [x, y, z]"
        if (inTargets) {
            double x, y, z;
            // Try to match the array format
            if (sscanf(line, " - [%lf, %lf, %lf]", &x, &y, &z) == 3) {
                // MATLAB uses column-major order, so for a maxClients x 3 matrix:
                // Column 1 (x): indices 0, 1, 2, ...
                // Column 2 (y): indices maxClients, maxClients+1, ...
                // Column 3 (z): indices 2*maxClients, 2*maxClients+1, ...
                targets[count + 0 * maxClients] = x;
                targets[count + 1 * maxClients] = y;
                targets[count + 2 * maxClients] = z;
                std::cout << "Loaded target " << (count + 1) << ": " << x << "," << y << "," << z << "\n";
                count++;
            }
        }
    }

    fclose(file);
    return count;
}

// Message type names for logging
static const char* messageTypeName(uint8_t msgType) {
    switch (msgType) {
        case 1: return "TARGET";
        case 2: return "ACK";
        case 3: return "READY";
        case 4: return "RTL";
        case 5: return "LAND";
        default: return "UNKNOWN";
    }
}

// Send a single-byte message type to a client
int sendMessageType(int clientId, int msgType) {
    if (clientId <= 0 || clientId > (int)clientSockets.size()) return 0;

    uint8_t msg = (uint8_t)msgType;
    ssize_t sent = send(clientSockets[clientId - 1], &msg, 1, 0);
    if (sent != 1) {
        std::cerr << "Send failed for client " << clientId << "\n";
        return 0;
    }

    std::cout << "Sent to client " << clientId << ": " << messageTypeName(msg) << " (" << (int)msg << ")\n";
    return 1;
}

// Send TARGET message with coordinates (1 byte type + 24 bytes coords)
int sendTarget(int clientId, const double* coords) {
    if (clientId <= 0 || clientId > (int)clientSockets.size()) return 0;

    // Build message: 1 byte type + 3 doubles (little-endian)
    uint8_t buffer[1 + 3 * sizeof(double)];
    buffer[0] = 1;  // TARGET = 1
    memcpy(buffer + 1, coords, 3 * sizeof(double));

    ssize_t sent = send(clientSockets[clientId - 1], buffer, sizeof(buffer), 0);
    if (sent != sizeof(buffer)) {
        std::cerr << "Send target failed for client " << clientId << "\n";
        return 0;
    }

    std::cout << "Sent TARGET to client " << clientId << ": "
              << coords[0] << "," << coords[1] << "," << coords[2] << "\n";
    return 1;
}

// Wait for a specific message type from ALL clients simultaneously using select()
// Returns 1 if all clients responded with expected message type, 0 on failure
int waitForAllMessageType(int numClients, int expectedType) {
    if (numClients <= 0 || numClients > (int)clientSockets.size()) return 0;

    uint8_t expected = (uint8_t)expectedType;
    std::vector<bool> completed(numClients, false);
    int completedCount = 0;

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
                uint8_t msgType;
                int len = recv(clientSockets[i], &msgType, 1, 0);
                if (len != 1) return 0;

                std::cout << "Received from client " << (i + 1) << ": "
                          << messageTypeName(msgType) << " (" << (int)msgType << ")\n";

                if (msgType == expected) {
                    completed[i] = true;
                    completedCount++;
                    std::cout << "Client " << (i + 1) << " completed: " << messageTypeName(expected) << "\n";
                }
            }
        }
    }

    return 1;
}

// Wait for user to press Enter
void waitForUserInput() {
    std::cout << "Press Enter to close experiment (RTL + LAND)...\n";
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}
