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


async def recv_exactly(reader: asyncio.StreamReader, n: int) -> bytes:
    """Receive exactly n bytes from async stream."""
    data = await reader.readexactly(n)
    return data


async def recv_message_type(reader: asyncio.StreamReader) -> MessageType:
    """Receive a single-byte message type."""
    data = await recv_exactly(reader, 1)
    return MessageType(data[0])


async def send_message_type(writer: asyncio.StreamWriter, msg_type: MessageType):
    """Send a single-byte message type."""
    writer.write(bytes([msg_type]))
    await writer.drain()


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
        reader, writer = None, None
        for attempt in range(10):
            try:
                reader, writer = await asyncio.wait_for(
                    asyncio.open_connection(self.server_ip, self.server_port),
                    timeout=5,
                )
                print(f"[UAV] Connected to controller")
                break
            except (ConnectionRefusedError, asyncio.TimeoutError, OSError) as e:
                print(f"[UAV] Connection attempt {attempt + 1}/10 failed: {e}")
                if attempt < 9:
                    await asyncio.sleep(3)

        if reader is None:
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
                msg_type = await recv_message_type(reader)
                print(f"[UAV] Received: {msg_type.name}")

                if msg_type == MessageType.TARGET:
                    # Read 24 bytes of coordinates (3 little-endian doubles)
                    data = await recv_exactly(reader, 24)
                    enu_x, enu_y, enu_z = struct.unpack('<ddd', data)
                    waypoint_num += 1
                    print(f"[UAV] TARGET (waypoint {waypoint_num}): x={enu_x}, y={enu_y}, z={enu_z}")

                    # Convert ENU to lat/lon (ENU: x=East, y=North, z=Up)
                    target = self.origin + VectorNED(north=enu_y, east=enu_x, down=-enu_z)
                    print(f"[UAV] Target coord: {target.lat:.6f}, {target.lon:.6f}, {target.alt:.1f}")

                    await send_message_type(writer, MessageType.ACK)
                    print(f"[UAV] Sent ACK")

                    print(f"[UAV] Moving to waypoint {waypoint_num}...")
                    await drone.goto_coordinates(target)
                    print(f"[UAV] Arrived at waypoint {waypoint_num}")

                    await send_message_type(writer, MessageType.READY)
                    print(f"[UAV] Sent READY")

                elif msg_type == MessageType.RTL:
                    await send_message_type(writer, MessageType.ACK)
                    print(f"[UAV] Sent ACK")
                    print("[UAV] Returning to home...")
                    home = drone.home_coords
                    safe_alt = 25
                    rtl_target = Coordinate(home.lat, home.lon, safe_alt)
                    print(f"[UAV] RTL to {home.lat:.6f}, {home.lon:.6f} at {safe_alt:.1f}m")
                    await drone.goto_coordinates(rtl_target)
                    print("[UAV] Arrived at home position")
                    await send_message_type(writer, MessageType.READY)
                    print(f"[UAV] Sent READY")

                elif msg_type == MessageType.LAND:
                    await send_message_type(writer, MessageType.ACK)
                    print(f"[UAV] Sent ACK")
                    print("[UAV] Landing...")
                    await drone.land()
                    # Switch out of LAND mode so the drone is re-armable
                    from dronekit import VehicleMode
                    drone._vehicle.mode = VehicleMode("ALT_HOLD")
                    print("[UAV] Landed and disarmed (ALT_HOLD)")
                    await send_message_type(writer, MessageType.READY)
                    print(f"[UAV] Sent READY")

                elif msg_type == MessageType.READY:
                    print("[UAV] Mission complete")
                    break

                else:
                    print(f"[UAV] Unknown command: {msg_type}")

        except (ValueError, asyncio.IncompleteReadError, ConnectionError) as e:
            print(f"[UAV] Error: {e}")

        finally:
            writer.close()
            await writer.wait_closed()
            print("[UAV] Connection closed")
