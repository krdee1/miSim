#!/bin/python3
import socket

SERVER_IP = "127.0.0.1"
SERVER_PORT = 5000

for client_id in range(2):  # match numClients
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((SERVER_IP, SERVER_PORT))
    data = s.recv(1024)  # receive message from server
    print(f"Client {client_id+1} received: {data.decode()}")
    s.sendall(b"ACK")    # send acknowledgment
    s.close()
