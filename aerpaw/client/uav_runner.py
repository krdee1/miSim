#!/usr/bin/env python3
"""
UAV Runner for AERPAW - aerpawlib BasicRunner implementation.

Run via:
    python -m aerpawlib --script client.uav_runner --conn <conn> --vehicle drone

Or use the auto-detecting wrapper:
    ./run_uav.sh

Binary Protocol:
    For each waypoint:
        Server sends: TARGET (1 byte) + x,y,z (24 bytes as 3 doubles)
        Client sends: ACK (1 byte)
        Client (after moving): READY (1 byte)
    After all waypoints:
        Server sends: RTL (1 byte) → Client: ACK, return home, READY
        Server sends: LAND (1 byte) → Client: ACK, land, READY
        Server sends: READY (1 byte) - mission complete, disconnect
"""
from enum import IntEnum
from pathlib import Path
import asyncio
import os
import socket
import struct
import yaml

from aerpawlib.runner import BasicRunner, entrypoint
from aerpawlib.util import Coordinate, VectorNED
from aerpawlib.vehicle import Drone


# Message types - must match MESSAGE_TYPE.m enum
class MessageType(IntEnum):
    TARGET = 1
    ACK = 2
    READY = 3
    RTL = 4
    LAND = 5


AERPAW_DIR = Path(__file__).parent.parent
CONFIG_FILE = AERPAW_DIR / "config" / "client.yaml"


def load_config():
    """Load configuration from YAML file."""
    with open(CONFIG_FILE, 'r') as f:
        return yaml.safe_load(f)


def get_environment():
    """Get environment from AERPAW_ENV variable. Fails if not set."""
    env = os.environ.get('AERPAW_ENV')
    if env is None:
        raise RuntimeError(
            "AERPAW_ENV environment variable not set. "
            "Set to 'local' or 'testbed', or use: ./run_uav.sh [local|testbed]"
        )
    if env not in ('local', 'testbed'):
        raise RuntimeError(
            f"Invalid AERPAW_ENV '{env}'. Must be 'local' or 'testbed'."
        )
    return env


def recv_exactly(sock: socket.socket, n: int) -> bytes:
    """Receive exactly n bytes from socket."""
    data = b''
    while len(data) < n:
        chunk = sock.recv(n - len(data))
        if not chunk:
            raise ConnectionError("Connection closed while receiving data")
        data += chunk
    return data


def recv_message_type(sock: socket.socket) -> MessageType:
    """Receive a single-byte message type."""
    data = recv_exactly(sock, 1)
    return MessageType(data[0])


def send_message_type(sock: socket.socket, msg_type: MessageType):
    """Send a single-byte message type."""
    sock.sendall(bytes([msg_type]))


class UAVRunner(BasicRunner):
    def initialize_args(self, extra_args):
        """Load configuration from YAML config file."""
        config = load_config()
        env = get_environment()
        print(f"[UAV] Environment: {env}")

        # Load origin
        origin = config['origin']
        self.origin = Coordinate(origin['lat'], origin['lon'], origin['alt'])
        print(f"[UAV] Origin: {origin['lat']}, {origin['lon']}, {origin['alt']}")

        # Load controller address for this environment
        env_config = config['environments'][env]
        self.server_ip = env_config['controller']['ip']
        self.server_port = env_config['controller']['port']
        print(f"[UAV] Controller: {self.server_ip}:{self.server_port}")

    @entrypoint
    async def run_mission(self, drone: Drone):
        """Main mission entry point."""
        # Enable built-in telemetry logging
        drone._verbose_logging = True
        drone._verbose_logging_file_prefix = "uav_telemetry"
        drone._verbose_logging_delay = 1.0  # 1 Hz

        print(f"[UAV] Connecting to controller at {self.server_ip}:{self.server_port}")

        # Retry connection up to 10 times (~30 seconds total)
        sock = None
        for attempt in range(10):
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(5)
                sock.connect((self.server_ip, self.server_port))
                sock.settimeout(None)
                print(f"[UAV] Connected to controller")
                break
            except (ConnectionRefusedError, socket.timeout, OSError) as e:
                if sock:
                    sock.close()
                print(f"[UAV] Connection attempt {attempt + 1}/10 failed: {e}")
                if attempt < 9:
                    await asyncio.sleep(3)

        if sock is None or sock.fileno() == -1:
            print("[UAV] Failed to connect to controller after 10 attempts")
            return

        try:
            # Takeoff to above AERPAW minimum altitude
            print("[UAV] Taking off...")
            await drone.takeoff(25)
            print("[UAV] Takeoff complete, waiting for commands...")

            # Command loop - handle TARGET, RTL, LAND, READY from controller
            waypoint_num = 0
            while True:
                msg_type = recv_message_type(sock)
                print(f"[UAV] Received: {msg_type.name}")

                if msg_type == MessageType.TARGET:
                    # Read 24 bytes of coordinates (3 little-endian doubles)
                    data = recv_exactly(sock, 24)
                    enu_x, enu_y, enu_z = struct.unpack('<ddd', data)
                    waypoint_num += 1
                    print(f"[UAV] TARGET (waypoint {waypoint_num}): x={enu_x}, y={enu_y}, z={enu_z}")

                    # Convert ENU to lat/lon (ENU: x=East, y=North, z=Up)
                    target = self.origin + VectorNED(north=enu_y, east=enu_x, down=-enu_z)
                    print(f"[UAV] Target coord: {target.lat:.6f}, {target.lon:.6f}, {target.alt:.1f}")

                    send_message_type(sock, MessageType.ACK)
                    print(f"[UAV] Sent ACK")

                    print(f"[UAV] Moving to waypoint {waypoint_num}...")
                    await drone.goto_coordinates(target)
                    print(f"[UAV] Arrived at waypoint {waypoint_num}")

                    send_message_type(sock, MessageType.READY)
                    print(f"[UAV] Sent READY")

                elif msg_type == MessageType.RTL:
                    send_message_type(sock, MessageType.ACK)
                    print(f"[UAV] Sent ACK")
                    print("[UAV] Returning to home...")
                    home = drone.home_coords
                    safe_alt = 25
                    rtl_target = Coordinate(home.lat, home.lon, safe_alt)
                    print(f"[UAV] RTL to {home.lat:.6f}, {home.lon:.6f} at {safe_alt:.1f}m")
                    await drone.goto_coordinates(rtl_target)
                    print("[UAV] Arrived at home position")
                    send_message_type(sock, MessageType.READY)
                    print(f"[UAV] Sent READY")

                elif msg_type == MessageType.LAND:
                    send_message_type(sock, MessageType.ACK)
                    print(f"[UAV] Sent ACK")
                    print("[UAV] Landing...")
                    await drone.land()
                    # Switch out of LAND mode so the drone is re-armable
                    from dronekit import VehicleMode
                    drone._vehicle.mode = VehicleMode("ALT_HOLD")
                    print("[UAV] Landed and disarmed (ALT_HOLD)")
                    send_message_type(sock, MessageType.READY)
                    print(f"[UAV] Sent READY")

                elif msg_type == MessageType.READY:
                    print("[UAV] Mission complete")
                    break

                else:
                    print(f"[UAV] Unknown command: {msg_type}")

        except (ValueError, ConnectionError) as e:
            print(f"[UAV] Error: {e}")

        finally:
            sock.close()
            print("[UAV] Socket closed")
