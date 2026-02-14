#!/bin/bash
# run_uav.sh - Wrapper for UAV runner
# Launches UAV client with environment-specific configuration
#
# Usage:
#   ./run_uav.sh local     # Use local/simulation configuration
#   ./run_uav.sh testbed   # Use AERPAW testbed configuration

set -e

# Change to script directory
cd "$(dirname "$0")"

# Activate venv if it exists
if [ -d "venv" ]; then
    source venv/bin/activate
fi

# Determine environment from argument (required)
if [ "$1" = "testbed" ]; then
    ENV="testbed"
elif [ "$1" = "local" ]; then
    ENV="local"
else
    echo "Error: Environment not specified."
    echo "Usage: $0 [local|testbed]"
    echo ""
    echo "  local   - Use local/simulation configuration"
    echo "  testbed - Use AERPAW testbed configuration"
    exit 1
fi

echo "[run_uav] Environment: $ENV"

# Export environment for Python to use
export AERPAW_ENV="$ENV"

# Read MAVLink connection from config.yaml using Python
CONN=$(python3 -c "
import yaml
with open('config/client.yaml') as f:
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