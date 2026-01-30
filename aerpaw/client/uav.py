#!/usr/bin/env python3
"""
UAV Client for AERPAW Target Location Protocol

Protocol:
  Server sends: TARGET:x,y,z (ENU coordinates in meters)
  Client sends: ACK:TARGET
  Client (after moving): READY
  Server sends: FINISHED

Coordinates:
  - Server sends local ENU (East, North, Up) coordinates in meters
  - Client converts to lat/lon using shared origin from config/origin.txt
  - Origin must match between controller and UAV

Auto-detection:
  - If flight controller is available, uses aerpawlib to move drone
  - If no flight controller, runs in test mode with placeholder movement
"""

import asyncio
import socket
import sys
import time
from pathlib import Path
from typing import Optional, Tuple

# Get the aerpaw directory (parent of client/)
AERPAW_DIR = Path(__file__).parent.parent
CONFIG_DIR = AERPAW_DIR / "config"


def load_origin(path: Path) -> Tuple[float, float, float]:
    """
    Load ENU origin from config file.
    Returns (lat, lon, alt) tuple.
    """
    with open(path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split(',')
            if len(parts) == 3:
                return float(parts[0]), float(parts[1]), float(parts[2])
    raise ValueError(f"No valid origin found in {path}")


def load_connection(path: Path) -> str:
    """
    Load drone connection string from config file.
    Returns connection string (e.g., 'udp:127.0.0.1:14550').
    """
    with open(path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            return line
    raise ValueError(f"No valid connection string found in {path}")


def load_server(path: Path) -> Tuple[str, int]:
    """
    Load controller server address from config file.
    Returns (ip, port) tuple.
    """
    with open(path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split(',')
            if len(parts) == 2:
                return parts[0].strip(), int(parts[1].strip())
    raise ValueError(f"No valid server address found in {path}")


def parse_target(message: str) -> Tuple[float, float, float]:
    """Parse TARGET:x,y,z message and return (x, y, z) ENU coordinates."""
    if not message.startswith("TARGET:"):
        raise ValueError(f"Invalid message format: {message}")

    coords_str = message[7:]  # Remove "TARGET:" prefix
    parts = coords_str.split(",")
    if len(parts) != 3:
        raise ValueError(f"Expected 3 coordinates, got {len(parts)}")

    x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
    return (x, y, z)


def enu_to_coordinate(origin_lat: float, origin_lon: float, origin_alt: float,
                      enu_x: float, enu_y: float, enu_z: float):
    """
    Convert ENU (East, North, Up) coordinates to lat/lon/alt.

    Args:
        origin_lat, origin_lon, origin_alt: Origin in degrees/meters
        enu_x: East offset in meters
        enu_y: North offset in meters
        enu_z: Up offset in meters (altitude above origin)

    Returns:
        aerpawlib Coordinate object
    """
    from aerpawlib.util import Coordinate, VectorNED

    origin = Coordinate(origin_lat, origin_lon, origin_alt)
    # ENU to NED: north=enu_y, east=enu_x, down=-enu_z
    offset = VectorNED(north=enu_y, east=enu_x, down=-enu_z)
    return origin + offset


def try_connect_drone(conn_str: str, timeout: float = 10.0):
    """
    Try to connect to drone flight controller.
    Returns Drone object if successful, None if not available.
    """
    try:
        from aerpawlib.vehicle import Drone
        print(f"  Attempting to connect to flight controller: {conn_str}")
        drone = Drone(conn_str)
        print(f"  Connected to flight controller")
        return drone
    except Exception as e:
        print(f"  Could not connect to flight controller: {e}")
        return None


def try_connect_server(ip: str, port: int, timeout: float = 2.0) -> Optional[socket.socket]:
    """
    Try to connect to controller server with timeout.
    Returns socket if successful, None if connection fails.
    """
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(timeout)
        s.connect((ip, port))
        s.settimeout(None)  # Reset to blocking mode after connect
        return s
    except (socket.timeout, ConnectionRefusedError, OSError):
        return None


def move_to_target_test(enu_x: float, enu_y: float, enu_z: float,
                        lat: float, lon: float, alt: float):
    """
    Placeholder movement for test mode.
    """
    print(f"  [TEST MODE] Target ENU: ({enu_x}, {enu_y}, {enu_z}) meters")
    print(f"  [TEST MODE] Target lat/lon: ({lat:.6f}, {lon:.6f}, {alt:.1f})")
    print(f"  [TEST MODE] Simulating movement...")
    time.sleep(1.0)
    print(f"  [TEST MODE] Arrived at target")


async def move_to_target_real(drone, target_coord, tolerance: float = 2.0):
    """
    Move drone to target using aerpawlib.
    """
    print(f"  [REAL MODE] Moving to: ({target_coord.lat:.6f}, {target_coord.lon:.6f}, {target_coord.alt:.1f})")
    await drone.goto_coordinates(target_coord, tolerance=tolerance)
    print(f"  [REAL MODE] Arrived at target")


async def run_uav_client(client_id: int):
    """Run a single UAV client."""
    print(f"UAV {client_id}: Starting...")

    # Load configuration
    origin_path = CONFIG_DIR / "origin.txt"
    connection_path = CONFIG_DIR / "connection.txt"
    server_testbed_path = CONFIG_DIR / "server_testbed.txt"
    server_local_path = CONFIG_DIR / "server.txt"

    print(f"UAV {client_id}: Loading origin from {origin_path}")
    origin_lat, origin_lon, origin_alt = load_origin(origin_path)
    print(f"UAV {client_id}: Origin: lat={origin_lat}, lon={origin_lon}, alt={origin_alt}")

    print(f"UAV {client_id}: Loading connection from {connection_path}")
    conn_str = load_connection(connection_path)

    # Auto-detect mode based on flight controller availability
    drone = try_connect_drone(conn_str)
    real_mode = drone is not None

    if real_mode:
        print(f"UAV {client_id}: Running in REAL mode")
    else:
        print(f"UAV {client_id}: Running in TEST mode (no flight controller)")

    # Try testbed server first, fall back to local
    testbed_ip, testbed_port = load_server(server_testbed_path)
    local_ip, local_port = load_server(server_local_path)

    print(f"UAV {client_id}: Trying testbed server at {testbed_ip}:{testbed_port}...")
    s = try_connect_server(testbed_ip, testbed_port)

    if s:
        print(f"UAV {client_id}: Connected to testbed server")
    else:
        print(f"UAV {client_id}: Testbed server not available, trying local server at {local_ip}:{local_port}...")
        s = try_connect_server(local_ip, local_port)
        if s:
            print(f"UAV {client_id}: Connected to local server")
        else:
            print(f"UAV {client_id}: ERROR - Could not connect to any server")
            if drone:
                drone.close()
            return

    try:
        # Receive TARGET command
        data = s.recv(1024).decode().strip()
        print(f"UAV {client_id}: Received: {data}")

        # Parse target coordinates (ENU)
        enu_x, enu_y, enu_z = parse_target(data)
        print(f"UAV {client_id}: Parsed ENU target: x={enu_x}, y={enu_y}, z={enu_z}")

        # Convert ENU to lat/lon
        target_coord = enu_to_coordinate(origin_lat, origin_lon, origin_alt,
                                         enu_x, enu_y, enu_z)
        print(f"UAV {client_id}: Target coordinate: lat={target_coord.lat:.6f}, lon={target_coord.lon:.6f}, alt={target_coord.alt:.1f}")

        # Send acknowledgment
        s.sendall(b"ACK:TARGET")
        print(f"UAV {client_id}: Sent ACK:TARGET")

        # Move to target
        if real_mode:
            await move_to_target_real(drone, target_coord)
        else:
            move_to_target_test(enu_x, enu_y, enu_z,
                              target_coord.lat, target_coord.lon, target_coord.alt)

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
        if drone:
            drone.close()
        print(f"UAV {client_id}: Connection closed")


def main():
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <client_id>")
        print("  Run one instance per UAV, e.g.:")
        print("    Terminal 1: ./uav.py 1")
        print("    Terminal 2: ./uav.py 2")
        sys.exit(1)

    client_id = int(sys.argv[1])
    asyncio.run(run_uav_client(client_id))


if __name__ == "__main__":
    main()