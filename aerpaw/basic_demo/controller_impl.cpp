#include "controller_impl.h"
#include <iostream>
#include <vector>
#include <cstring>
#include <sys/socket.h>
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
