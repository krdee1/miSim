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
#include <time.h>

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

// ---------------------------------------------------------------------------
// Scenario CSV loading
// ---------------------------------------------------------------------------

// Split a CSV data row into fields, respecting quoted commas.
// Inserts NUL terminators into `line` in place.
// Returns the number of fields found.
static int splitCSVRow(char* line, char** fields, int maxFields) {
    int n = 0;
    bool inQuote = false;
    if (n < maxFields) fields[n++] = line;
    for (char* p = line; *p && *p != '\n' && *p != '\r'; ++p) {
        if (*p == '"') {
            inQuote = !inQuote;
        } else if (*p == ',' && !inQuote && n < maxFields) {
            *p = '\0';
            fields[n++] = p + 1;
        }
    }
    // NUL-terminate the last field (strip trailing newline)
    for (char* p = fields[n - 1]; *p; ++p) {
        if (*p == '\n' || *p == '\r') { *p = '\0'; break; }
    }
    return n;
}

// Trim leading/trailing ASCII whitespace and remove enclosing double-quotes.
// Modifies the string in place and returns the start of the trimmed content.
static char* trimField(char* s) {
    // Trim leading whitespace
    while (*s == ' ' || *s == '\t') ++s;
    // Remove enclosing quotes
    size_t len = strlen(s);
    if (len >= 2 && s[0] == '"' && s[len - 1] == '"') {
        s[len - 1] = '\0';
        ++s;
    }
    // Trim trailing whitespace
    char* end = s + strlen(s) - 1;
    while (end > s && (*end == ' ' || *end == '\t')) { *end-- = '\0'; }
    return s;
}

// Open scenario.csv, skip the header row, return the data row in `line`.
// Returns 1 on success, 0 on failure.
static int readScenarioDataRow(const char* filename, char* line, int lineSize) {
    FILE* f = fopen(filename, "r");
    if (!f) {
        std::cerr << "loadScenario: cannot open " << filename << "\n";
        return 0;
    }
    // Skip header row
    if (!fgets(line, lineSize, f)) { fclose(f); return 0; }
    // Read data row
    int ok = (fgets(line, lineSize, f) != NULL);
    fclose(f);
    return ok ? 1 : 0;
}

// Load guidance parameters from scenario.csv into flat params[NUM_SCENARIO_PARAMS].
// Index mapping (0-based):
//   0-13 : timestep, maxIter, minAlt, discretizationStep, protectedRange,
//           initialStepSize, barrierGain, barrierExponent, collisionRadius,
//           comRange, alphaDist, betaDist, alphaTilt, betaTilt
//   14-16: domainMin  (east, north, up)
//   17-19: domainMax  (east, north, up)
//   20-21: objectivePos (east, north)
// Returns 1 on success, 0 on failure.
int loadScenario(const char* filename, double* params) {
    char line[4096];
    if (!readScenarioDataRow(filename, line, sizeof(line))) return 0;

    char copy[4096];
    strncpy(copy, line, sizeof(copy) - 1);
    copy[sizeof(copy) - 1] = '\0';

    char* fields[32];
    int nf = splitCSVRow(copy, fields, 32);
    if (nf < 21) {
        fprintf(stderr, "loadScenario: expected >=21 columns, got %d\n", nf);
        return 0;
    }

    // Scalar fields (columns 0–13)
    for (int i = 0; i < 14; i++) {
        params[i] = atof(trimField(fields[i]));
    }

    // domainMin: column 14, format "east, north, up"
    {
        char tmp[256]; strncpy(tmp, fields[14], sizeof(tmp) - 1); tmp[sizeof(tmp)-1] = '\0';
        char* t = trimField(tmp);
        if (sscanf(t, "%lf , %lf , %lf", &params[14], &params[15], &params[16]) != 3) {
            fprintf(stderr, "loadScenario: failed to parse domainMin: %s\n", t);
            return 0;
        }
    }

    // domainMax: column 15
    {
        char tmp[256]; strncpy(tmp, fields[15], sizeof(tmp) - 1); tmp[sizeof(tmp)-1] = '\0';
        char* t = trimField(tmp);
        if (sscanf(t, "%lf , %lf , %lf", &params[17], &params[18], &params[19]) != 3) {
            fprintf(stderr, "loadScenario: failed to parse domainMax: %s\n", t);
            return 0;
        }
    }

    // objectivePos: column 16
    {
        char tmp[256]; strncpy(tmp, fields[16], sizeof(tmp) - 1); tmp[sizeof(tmp)-1] = '\0';
        char* t = trimField(tmp);
        if (sscanf(t, "%lf , %lf", &params[20], &params[21]) != 2) {
            fprintf(stderr, "loadScenario: failed to parse objectivePos: %s\n", t);
            return 0;
        }
    }

    // sensorPerformanceMinimum: column 17
    {
        char tmp[64]; strncpy(tmp, fields[17], sizeof(tmp) - 1); tmp[sizeof(tmp)-1] = '\0';
        params[22] = atof(trimField(tmp));
    }

    printf("Loaded scenario: domain [%g,%g,%g] to [%g,%g,%g]\n",
           params[14], params[15], params[16], params[17], params[18], params[19]);
    return 1;
}

// Load initial UAV positions from scenario.csv (column 17: initialPositions).
// targets is a column-major [maxClients x 3] array (same layout as loadTargets):
//   targets[i + 0*maxClients] = east
//   targets[i + 1*maxClients] = north
//   targets[i + 2*maxClients] = up
// Returns number of positions loaded.
int loadInitialPositions(const char* filename, double* targets, int maxClients) {
    char line[4096];
    if (!readScenarioDataRow(filename, line, sizeof(line))) return 0;

    char copy[4096];
    strncpy(copy, line, sizeof(copy) - 1);
    copy[sizeof(copy) - 1] = '\0';

    char* fields[32];
    int nf = splitCSVRow(copy, fields, 32);
    if (nf < 19) {
        fprintf(stderr, "loadInitialPositions: expected >=19 columns, got %d\n", nf);
        return 0;
    }

    // Column 18: initialPositions flat "x1,y1,z1, x2,y2,z2, ..."
    char tmp[1024]; strncpy(tmp, fields[18], sizeof(tmp) - 1); tmp[sizeof(tmp)-1] = '\0';
    char* t = trimField(tmp);

    double vals[3 * 4];  // up to MAX_CLIENTS triples
    int parsed = 0;
    char* tok = strtok(t, ",");
    while (tok && parsed < 3 * maxClients) {
        vals[parsed++] = atof(tok);
        tok = strtok(nullptr, ",");
    }

    int count = parsed / 3;
    for (int i = 0; i < count; i++) {
        targets[i + 0 * maxClients] = vals[i * 3 + 0];  // east
        targets[i + 1 * maxClients] = vals[i * 3 + 1];  // north
        targets[i + 2 * maxClients] = vals[i * 3 + 2];  // up
    }

    printf("Loaded %d initial position(s) from scenario\n", count);
    return count;
}

// Load obstacle bounding-box corners from scenario.csv.
// Columns used: 18 (numObstacles), 19 (obstacleMin flat), 20 (obstacleMax flat).
// obstacleMin and obstacleMax are column-major [maxObstacles x 3] arrays:
//   obstacleMin[obs + 0*maxObstacles] = east_min
//   obstacleMin[obs + 1*maxObstacles] = north_min
//   obstacleMin[obs + 2*maxObstacles] = up_min
// Returns number of obstacles loaded (0 if none or on error).
int loadObstacles(const char* filename, double* obstacleMin, double* obstacleMax,
                  int maxObstacles) {
    char line[4096];
    if (!readScenarioDataRow(filename, line, sizeof(line))) return 0;

    char copy[4096];
    strncpy(copy, line, sizeof(copy) - 1);
    copy[sizeof(copy) - 1] = '\0';

    char* fields[32];
    int nf = splitCSVRow(copy, fields, 32);
    if (nf < 22) return 0;

    // Column 19: numObstacles
    int n = (int)atof(trimField(fields[19]));
    if (n <= 0) return 0;
    if (n > maxObstacles) {
        fprintf(stderr, "loadObstacles: %d obstacles exceeds MAX_OBSTACLES=%d\n", n, maxObstacles);
        n = maxObstacles;
    }

    // Column 20: obstacleMin flat "x0,y0,z0, x1,y1,z1, ..."
    {
        char tmp[1024]; strncpy(tmp, fields[20], sizeof(tmp) - 1); tmp[sizeof(tmp)-1] = '\0';
        char* t = trimField(tmp);
        double vals[3 * 8];  // up to MAX_OBSTACLES triples
        int parsed = 0;
        char* tok = strtok(t, ",");
        while (tok && parsed < 3 * n) {
            vals[parsed++] = atof(tok);
            tok = strtok(nullptr, ",");
        }
        if (parsed < 3 * n) {
            fprintf(stderr, "loadObstacles: obstacleMin has fewer values than expected\n");
            return 0;
        }
        for (int obs = 0; obs < n; obs++) {
            obstacleMin[obs + 0 * maxObstacles] = vals[obs * 3 + 0];  // east
            obstacleMin[obs + 1 * maxObstacles] = vals[obs * 3 + 1];  // north
            obstacleMin[obs + 2 * maxObstacles] = vals[obs * 3 + 2];  // up
        }
    }

    // Column 21: obstacleMax flat
    {
        char tmp[1024]; strncpy(tmp, fields[21], sizeof(tmp) - 1); tmp[sizeof(tmp)-1] = '\0';
        char* t = trimField(tmp);
        double vals[3 * 8];
        int parsed = 0;
        char* tok = strtok(t, ",");
        while (tok && parsed < 3 * n) {
            vals[parsed++] = atof(tok);
            tok = strtok(nullptr, ",");
        }
        if (parsed < 3 * n) {
            fprintf(stderr, "loadObstacles: obstacleMax has fewer values than expected\n");
            return 0;
        }
        for (int obs = 0; obs < n; obs++) {
            obstacleMax[obs + 0 * maxObstacles] = vals[obs * 3 + 0];
            obstacleMax[obs + 1 * maxObstacles] = vals[obs * 3 + 1];
            obstacleMax[obs + 2 * maxObstacles] = vals[obs * 3 + 2];
        }
    }

    printf("Loaded %d obstacle(s) from scenario\n", n);
    return n;
}

// Message type names for logging
static const char* messageTypeName(uint8_t msgType) {
    switch (msgType) {
        case 1: return "TARGET";
        case 2: return "ACK";
        case 3: return "READY";
        case 4: return "RTL";
        case 5: return "LAND";
        case 6: return "GUIDANCE_TOGGLE";
        case 7: return "REQUEST_POSITION";
        case 8: return "POSITION";
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

    std::cout << "Sent " << messageTypeName(msg) << " to client " << clientId << "\n";
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
                if (len == 0) {
                    std::cerr << "waitForAllMessageType: client " << (i + 1)
                              << " disconnected while waiting for "
                              << messageTypeName(expected) << "\n";
                    return 0;
                }
                if (len < 0) {
                    std::cerr << "waitForAllMessageType: recv error for client " << (i + 1)
                              << " while waiting for " << messageTypeName(expected) << "\n";
                    return 0;
                }

                std::cout << "Received " << messageTypeName(msgType) << " from client " << (i + 1) << "\n";

                if (msgType == expected) {
                    completed[i] = true;
                    completedCount++;
                }
            }
        }
    }

    return 1;
}

// Wait for user to press Enter
void waitForUserInput() {
    std::cout << "Press Enter to continue...\n";
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

// Broadcast GUIDANCE_TOGGLE to all clients
void sendGuidanceToggle(int numClients) {
    for (int i = 1; i <= numClients; i++) {
        sendMessageType(i, 6);  // GUIDANCE_TOGGLE = 6
    }
}

// Send REQUEST_POSITION to all clients
int sendRequestPositions(int numClients) {
    for (int i = 1; i <= numClients; i++) {
        if (!sendMessageType(i, 7)) return 0;  // REQUEST_POSITION = 7
    }
    return 1;
}

// Receive POSITION response (1 byte type + 24 bytes ENU) from all clients.
// Stores results in column-major order: positions[client + 0*maxClients] = x (east),
//                                       positions[client + 1*maxClients] = y (north),
//                                       positions[client + 2*maxClients] = z (up)
int recvPositions(int numClients, double* positions, int maxClients) {
    if (numClients <= 0 || numClients > (int)clientSockets.size()) return 0;

    for (int i = 0; i < numClients; i++) {
        // Expect: POSITION byte (1) + 3 doubles (24)
        uint8_t msgType;
        if (recv(clientSockets[i], &msgType, 1, MSG_WAITALL) != 1) {
            std::cerr << "recvPositions: failed to read msg type from client " << (i + 1) << "\n";
            return 0;
        }
        if (msgType != 8) {  // POSITION = 8
            std::cerr << "recvPositions: expected POSITION(8), got " << (int)msgType
                      << " from client " << (i + 1) << "\n";
            return 0;
        }

        double coords[3];
        if (recv(clientSockets[i], coords, sizeof(coords), MSG_WAITALL) != (ssize_t)sizeof(coords)) {
            std::cerr << "recvPositions: failed to read coords from client " << (i + 1) << "\n";
            return 0;
        }

        // Store column-major (MATLAB layout): col 0 = east, col 1 = north, col 2 = up
        positions[i + 0 * maxClients] = coords[0];  // east  (x)
        positions[i + 1 * maxClients] = coords[1];  // north (y)
        positions[i + 2 * maxClients] = coords[2];  // up    (z)

        std::cout << "Position from client " << (i + 1) << ": "
                  << coords[0] << "," << coords[1] << "," << coords[2] << "\n";
    }
    return 1;
}

// Sleep for the given number of milliseconds
void sleepMs(int ms) {
    struct timespec ts;
    ts.tv_sec  = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1000000L;
    nanosleep(&ts, nullptr);
}
