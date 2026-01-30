#!/usr/bin/env python3
"""
UAV Client for AERPAW Target Location Protocol

Protocol:
  Server sends: TARGET:x,y,z
  Client sends: ACK:TARGET
  Client (after moving): READY
"""

import socket
import sys
import time

SERVER_IP = "127.0.0.1"
SERVER_PORT = 5000


def parse_target(message: str) -> tuple:
    """Parse TARGET:x,y,z message and return (x, y, z) coordinates."""
    if not message.startswith("TARGET:"):
        raise ValueError(f"Invalid message format: {message}")

    coords_str = message[7:]  # Remove "TARGET:" prefix
    parts = coords_str.split(",")
    if len(parts) != 3:
        raise ValueError(f"Expected 3 coordinates, got {len(parts)}")

    x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
    return (x, y, z)


def move_to_target(x: float, y: float, z: float):
    """
    Placeholder for AERPAW API integration.
    In the future, this will command the UAV to move to (x, y, z).
    """
    print(f"  [PLACEHOLDER] Moving to target: ({x}, {y}, {z})")
    # Simulate movement time
    time.sleep(1.0)
    print(f"  [PLACEHOLDER] Arrived at target: ({x}, {y}, {z})")


def run_uav_client(client_id: int):
    """Run a single UAV client."""
    print(f"UAV {client_id}: Connecting to {SERVER_IP}:{SERVER_PORT}")

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((SERVER_IP, SERVER_PORT))

    try:
        # Receive TARGET command
        data = s.recv(1024).decode().strip()
        print(f"UAV {client_id}: Received: {data}")

        # Parse target coordinates
        x, y, z = parse_target(data)
        print(f"UAV {client_id}: Parsed target: x={x}, y={y}, z={z}")

        # Send acknowledgment
        s.sendall(b"ACK:TARGET")
        print(f"UAV {client_id}: Sent ACK:TARGET")

        # Move to target (placeholder)
        move_to_target(x, y, z)

        # Signal ready
        s.sendall(b"READY")
        print(f"UAV {client_id}: Sent READY")

        # Wait for FINISHED signal from server
        data = s.recv(1024).decode().strip()
        if data == "FINISHED":
            print(f"UAV {client_id}: Received FINISHED - session ended normally")
        else:
            print(f"UAV {client_id}: Unexpected message: {data}")

    except ValueError as e:
        error_msg = f"ERROR:{str(e)}"
        s.sendall(error_msg.encode())
        print(f"UAV {client_id}: Error - {e}")

    finally:
        s.close()
        print(f"UAV {client_id}: Connection closed")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <client_id>")
        print("  Run one instance per UAV, e.g.:")
        print("    Terminal 1: ./uav.py 1")
        print("    Terminal 2: ./uav.py 2")
        sys.exit(1)

    client_id = int(sys.argv[1])
    run_uav_client(client_id)
