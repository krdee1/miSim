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

# Function to check if we're in testbed environment
check_testbed() {
    ip addr | grep -q "192.168.122"
    return $?
}

# Determine environment based on argument or auto-detection
if [ "$1" = "testbed" ]; then
    ENV="testbed"
    echo "[run_uav] Forced testbed mode"
elif [ "$1" = "local" ]; then
    ENV="local"
    echo "[run_uav] Forced local mode"
else
    echo "[run_uav] Auto-detecting environment..."
    if check_testbed; then
        ENV="testbed"
        echo "[run_uav] Testbed detected"
    else
        ENV="local"
        echo "[run_uav] Local mode"
    fi
fi

# Export environment for Python to use
export AERPAW_ENV="$ENV"

# Read MAVLink connection from config.yaml using Python
CONN=$(python3 -c "
import yaml
with open('config/config.yaml') as f:
    cfg = yaml.safe_load(f)
env = cfg['environments']['$ENV']['mavlink']
print(f\"udp:{env['ip']}:{env['port']}\")
")

echo "[run_uav] MAVLink connection: $CONN"

# Run via aerpawlib
echo "[run_uav] Starting UAV runner..."
python3 -m aerpawlib \
    --script client.uav_runner \
    --conn "$CONN" \
    --vehicle drone