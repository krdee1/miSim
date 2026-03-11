#!/bin/bash
# run_uav.sh - Wrapper for UAV runner
# Launches UAV client with environment-specific configuration
#
# Usage:
#   ./run_uav.sh local                          # defaults to config/client.yaml
#   ./run_uav.sh testbed config/client2.yaml     # use a specific config file

set -e

# Change to aerpaw directory
cd /root/miSim/aerpaw

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
    echo "Usage: $0 [local|testbed] [config_file]"
    echo ""
    echo "  local   - Use local/simulation configuration"
    echo "  testbed - Use AERPAW testbed configuration"
    echo ""
    echo "  config_file - Path to client YAML (default: config/client.yaml)"
    exit 1
fi

# Client config file: 2nd argument > AERPAW_CLIENT_CONFIG env var > default
CONFIG_FILE="${2:-${AERPAW_CLIENT_CONFIG:-config/client.yaml}}"
if [ ! -f "$CONFIG_FILE" ]; then
    echo "Error: Config file not found: $CONFIG_FILE"
    exit 1
fi

echo "[run_uav] Environment: $ENV"
echo "[run_uav] Config file: $CONFIG_FILE"

# Export for Python scripts to use
export AERPAW_ENV="$ENV"
export AERPAW_CLIENT_CONFIG="$(realpath "$CONFIG_FILE")"

# Read MAVLink connection from config file using Python
CONN=$(python3 -c "
import yaml
with open('$CONFIG_FILE') as f:
    cfg = yaml.safe_load(f)
env = cfg['environments']['$ENV']['mavlink']
print(f\"udp:{env['ip']}:{env['port']}\")
")

echo "[run_uav] MAVLink connection: $CONN"

# Run via aerpawlib
echo "[run_uav] Starting UAV runner..."
python3 -u -m aerpawlib \
    --script client.uav_runner \
    --conn "$CONN" \
    --vehicle drone