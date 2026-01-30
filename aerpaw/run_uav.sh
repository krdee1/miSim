#!/bin/bash
# run_uav.sh - Auto-detecting wrapper for UAV runner
# Detects testbed vs local environment and launches with appropriate connection
#
# Usage:
#   ./run_uav.sh           # Auto-detect and run
#   ./run_uav.sh testbed   # Force testbed mode
#   ./run_uav.sh local     # Force local mode

set -e

# Change to script directory
cd "$(dirname "$0")"

# Activate venv if it exists
if [ -d "venv" ]; then
    source venv/bin/activate
fi

# Connection strings
# AERPAW testbed: E-VM LISTENS on port 14550, MAVLink Filter connects TO us
# See: https://sites.google.com/ncsu.edu/aerpaw-user-manual/6-sample-experiments-repository/6-2-vehicle-control-software/vcs1-preplanned-trajectory
# "the IP address corresponds to the E-VM IP address and the port is where
# the MAVLink filter expects to find the vehicle control script"
TESTBED_CONN="udp:0.0.0.0:14550"
LOCAL_CONN="udp:127.0.0.1:14550"

# Function to check if we're in testbed environment
check_testbed() {
    # Check if we're on the AERPAW testbed network (192.168.122.X)
    ip addr | grep -q "192.168.122"
    return $?
}

# Determine connection based on argument or auto-detection
if [ "$1" = "testbed" ]; then
    CONN="$TESTBED_CONN"
    echo "[run_uav] Forced testbed mode: $CONN"
elif [ "$1" = "local" ]; then
    CONN="$LOCAL_CONN"
    echo "[run_uav] Forced local mode: $CONN"
else
    echo "[run_uav] Auto-detecting environment..."
    if check_testbed; then
        CONN="$TESTBED_CONN"
        echo "[run_uav] Testbed detected, using: $CONN"
    else
        CONN="$LOCAL_CONN"
        echo "[run_uav] Local mode (testbed not reachable), using: $CONN"
    fi
fi

# Run via aerpawlib
echo "[run_uav] Starting UAV runner..."
python3 -m aerpawlib \
    --script client.uav_runner \
    --conn "$CONN" \
    --vehicle drone