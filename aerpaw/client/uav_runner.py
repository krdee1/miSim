#!/usr/bin/env python3
"""
UAV Runner for AERPAW - aerpawlib BasicRunner implementation.

Run via:
    python -m aerpawlib --script client.uav_runner --conn <conn> --vehicle drone

Or use the auto-detecting wrapper:
    ./run_uav.sh

Protocol:
    Server sends: TARGET:x,y,z (ENU coordinates in meters)
    Client sends: ACK:TARGET
    Client (after moving): READY
    Server sends: RTL
    Client sends: ACK:RTL
    Client (after returning home): RTL_COMPLETE
    Server sends: LAND
    Client sends: ACK:LAND
    Client (after landing): LAND_COMPLETE
    Server sends: FINISHED
"""
from pathlib import Path
import asyncio
import socket

from aerpawlib.runner import BasicRunner, entrypoint
from aerpawlib.util import Coordinate, VectorNED
from aerpawlib.vehicle import Drone

AERPAW_DIR = Path(__file__).parent.parent
CONFIG_DIR = AERPAW_DIR / "config"


def load_origin(path: Path):
    """Load ENU origin from config file. Returns (lat, lon, alt)."""
    with open(path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split(',')
            if len(parts) == 3:
                return float(parts[0]), float(parts[1]), float(parts[2])
    raise ValueError(f"No valid origin found in {path}")


def load_server(path: Path):
    """Load server address from config file. Returns (ip, port)."""
    with open(path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split(',')
            if len(parts) == 2:
                return parts[0].strip(), int(parts[1].strip())
    raise ValueError(f"No valid server address found in {path}")


def parse_target(message: str):
    """Parse TARGET:x,y,z message. Returns (x, y, z) ENU coordinates."""
    if not message.startswith("TARGET:"):
        raise ValueError(f"Invalid message format: {message}")
    coords_str = message[7:]
    parts = coords_str.split(",")
    if len(parts) != 3:
        raise ValueError(f"Expected 3 coordinates, got {len(parts)}")
    return float(parts[0]), float(parts[1]), float(parts[2])


class UAVRunner(BasicRunner):
    def initialize_args(self, extra_args):
        """Load configuration from config files."""
        # Load origin
        origin_lat, origin_lon, origin_alt = load_origin(CONFIG_DIR / "origin.txt")
        self.origin = Coordinate(origin_lat, origin_lon, origin_alt)
        print(f"[UAV] Origin: {origin_lat}, {origin_lon}, {origin_alt}")

        # Load server address - try testbed first, fall back to local
        testbed_path = CONFIG_DIR / "server_testbed.txt"
        local_path = CONFIG_DIR / "server.txt"

        if testbed_path.exists():
            try:
                self.server_ip, self.server_port = load_server(testbed_path)
                print(f"[UAV] Loaded testbed server config: {self.server_ip}:{self.server_port}")
            except ValueError:
                self.server_ip, self.server_port = load_server(local_path)
                print(f"[UAV] Loaded local server config: {self.server_ip}:{self.server_port}")
        else:
            self.server_ip, self.server_port = load_server(local_path)
            print(f"[UAV] Loaded local server config: {self.server_ip}:{self.server_port}")

    @entrypoint
    async def run_mission(self, drone: Drone):
        """Main mission entry point."""
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
            print("[UAV] Takeoff complete, waiting for target...")

            # Receive TARGET command
            data = sock.recv(1024).decode().strip()
            print(f"[UAV] Received: {data}")

            enu_x, enu_y, enu_z = parse_target(data)
            print(f"[UAV] Target ENU: x={enu_x}, y={enu_y}, z={enu_z}")

            # Convert ENU to lat/lon
            # ENU: x=East, y=North, z=Up
            # NED: north, east, down
            target = self.origin + VectorNED(north=enu_y, east=enu_x, down=-enu_z)
            print(f"[UAV] Target coord: {target.lat:.6f}, {target.lon:.6f}, {target.alt:.1f}")

            # Acknowledge
            sock.sendall(b"ACK:TARGET")
            print("[UAV] Sent ACK:TARGET")

            # Move to target
            print("[UAV] Moving to target...")
            await drone.goto_coordinates(target)
            print("[UAV] Arrived at target")

            # Signal ready
            sock.sendall(b"READY")
            print("[UAV] Sent READY, waiting for commands...")

            # Command loop - handle RTL, LAND, FINISHED from controller
            while True:
                data = sock.recv(1024).decode().strip()
                print(f"[UAV] Received: {data}")

                if data == "RTL":
                    sock.sendall(b"ACK:RTL")
                    print("[UAV] Returning to home...")
                    # Navigate to home lat/lon but stay at current altitude
                    # (AERPAW blocks goto_coordinates below 20m, but land() mode is allowed)
                    home = drone.home_coords
                    current_alt = drone.position.alt
                    safe_alt = 25  # Set to 25m alt for RTL motion
                    rtl_target = Coordinate(home.lat, home.lon, safe_alt)
                    print(f"[UAV] RTL to {home.lat:.6f}, {home.lon:.6f} at {safe_alt:.1f}m")
                    await drone.goto_coordinates(rtl_target)
                    print("[UAV] Arrived at home position")
                    sock.sendall(b"RTL_COMPLETE")
                    print("[UAV] Sent RTL_COMPLETE")

                elif data == "LAND":
                    sock.sendall(b"ACK:LAND")
                    print("[UAV] Landing...")
                    await drone.land()
                    print("[UAV] Landed and disarmed")
                    sock.sendall(b"LAND_COMPLETE")
                    print("[UAV] Sent LAND_COMPLETE")

                elif data == "FINISHED":
                    print("[UAV] Received FINISHED - mission complete")
                    break

                else:
                    print(f"[UAV] Unknown command: {data}")

        except ValueError as e:
            error_msg = f"ERROR:{str(e)}"
            try:
                sock.sendall(error_msg.encode())
            except Exception:
                pass
            print(f"[UAV] Error: {e}")

        finally:
            sock.close()
            print("[UAV] Socket closed")