#!/bin/bash

# Drop in replacements for channel sounder scripts
cp startchannelsounderRXGRC.sh /root/Profiles/ProfileScripts/Radio/Helpers/.
cp startchannelsounderTXGRC.sh /root/Profiles/ProfileScripts/Radio/Helpers/.
cp CSwSNRRX.py /root/Profiles/SDR_control/Channel_Sounderv3/.
cp CSwSNRTX.py /root/Profiles/SDR_control/Channel_Sounderv3/.

# Replace start scripts
cp ../scripts/startexperiment.sh /root/.
cp ../scripts/startRadio.sh /root/Profiles/ProfileScripts/Radio/.
cp ../scripts/startVehicle.sh /root/Profiles/ProfileScripts/Vehicle/.

echo "REMEMBER! Manually edit startexperiment.sh to point to the correct client.yaml"
echo "REMEMBER! Manually copy startexperiment_controller.sh to startexperiment.sh on the fixed node"