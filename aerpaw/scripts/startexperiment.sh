#!/bin/bash
/root/stopexperiment.sh

source /root/.ap-set-experiment-env.sh
source /root/.bashrc

# set path to client config YAML
export AERPAW_CLIENT_CONFIG=/root/miSim/aerpaw/config/client1.yaml

export AERPAW_REPO=${AERPAW_REPO:-/root/AERPAW-Dev}
export AERPAW_PYTHON=${AERPAW_PYTHON:-python3}
export PYTHONPATH=/usr/local/lib/python3/dist-packages/
export EXP_NUMBER=${EXP_NUMBER:-1}

if [ "$AP_EXPENV_THIS_CONTAINER_NODE_VEHICLE" == "vehicle_uav" ]; then
    export VEHICLE_TYPE=drone
elif [ "$AP_EXPENV_THIS_CONTAINER_NODE_VEHICLE" == "vehicle_ugv" ]; then
    export VEHICLE_TYPE=rover
else
    export VEHICLE_TYPE=none
fi

if [ "$AP_EXPENV_SESSION_ENV" == "Virtual" ]; then
    export LAUNCH_MODE=EMULATION
elif [ "$AP_EXPENV_SESSION_ENV" == "Testbed" ]; then
    export LAUNCH_MODE=TESTBED
else
    export LAUNCH_MODE=none
fi

# prepare results directory
export UAV_ID=$(python3 -c "import yaml; print(yaml.safe_load(open('$AERPAW_CLIENT_CONFIG'))['uav_id'])")
export RESULTS_DIR_TIMESTAMP=$(date +%Y-%m-%d_%H_%M_%S)
export RESULTS_DIR="/root/Results/uav${UAV_ID}_${RESULTS_DIR_TIMESTAMP}"
mkdir -p "$RESULTS_DIR"

export TS_FORMAT="${TS_FORMAT:-'[%Y-%m-%d %H:%M:%.S]'}"
export LOG_PREFIX="$(date +%Y-%m-%d_%H_%M_%S)"

export TX_FREQ=3.32e9
export RX_FREQ=3.32e9


export PROFILE_DIR=$AERPAW_REPO"/AHN/E-VM/Profile_software"
cd $PROFILE_DIR"/ProfileScripts"



./Radio/startRadio.sh
#./Traffic/startTraffic.sh
./Vehicle/startVehicle.sh

schedule_stop.sh 30
