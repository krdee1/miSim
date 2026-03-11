#!/bin/bash

### Sample GPS logger portion
# use vehicle type generic to skip the arming requirement
export VEHICLE_TYPE="${VEHICLE_TYPE:-generic}"

# GPS Logger sample application (this does not move the vehicle)

#cd $PROFILE_DIR"/ProfileScripts/Vehicle/Helpers"
#
#screen -S vehicle -dm \
#       bash -c "stdbuf -oL -eL ./gpsLoggerHelper.sh \
#       2> >(ts $TS_FORMAT >> $RESULTS_DIR/${LOG_PREFIX}_vehicle_log_err.txt) \
#       | ts $TS_FORMAT \
#       | tee $RESULTS_DIR/$LOG_PREFIX\_vehicle_log.txt"
#
#cd -

### Actual control portion (custom)
export VEHICLE_TYPE="${VEHICLE_TYPE:-drone}" # out of rover, drone, generic

cd /root/miSim/aerpaw

# Use screen/ts/tee aerpawism from sample script
screen -S vehicle -dm \
       bash -c "stdbuf -oL -eL ./run_uav.sh testbed \
       | ts $TS_FORMAT \
       | tee $RESULTS_DIR/$LOG_PREFIX\_vehicle_log.txt"     

cd -
