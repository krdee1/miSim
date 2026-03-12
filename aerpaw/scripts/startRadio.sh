#!/bin/bash

# Derive number of UAVs from scenario.csv
NUM_UAVS=$(python3 -c "
import csv, os
csv_path = '/root/miSim/aerpaw/config/scenario.csv'
with open(csv_path, 'r') as f:
    reader = csv.reader(f, skipinitialspace=True)
    header = [h.strip() for h in next(reader)]
    row = next(reader)
col = header.index('initialPositions')
vals = [v.strip() for v in row[col].strip().split(',') if v.strip()]
print(len(vals) // 3)
" 2>/dev/null || echo 0)

cd $PROFILE_DIR"/ProfileScripts/Radio/Helpers"

if [ "$NUM_UAVS" -eq 2 ]; then
    # Direct 1-to-1 mode: UAV 0 = TX only, UAV 1 = RX only
    echo "[Radio] 2-UAV direct mode: UAV_ID=$UAV_ID"

    if [ "$UAV_ID" -eq 0 ]; then
        # TX only (--num-uavs 1 disables TDM muting)
        screen -S txGRC -dm \
               bash -c "stdbuf -oL -eL ./startchannelsounderTXGRC.sh --num-uavs 1 \
               2>&1 | ts $TS_FORMAT \
               | tee $RESULTS_DIR/$LOG_PREFIX\_radio_channelsoundertxgrc_log.txt"
    else
        # RX only (--num-uavs 1 disables TDM tagging)
        screen -S rxGRC -dm \
               bash -c "stdbuf -oL -eL ./startchannelsounderRXGRC.sh --num-uavs 1 \
               2>&1 | ts $TS_FORMAT \
               | tee $RESULTS_DIR/$LOG_PREFIX\_radio_channelsounderrxgrc_log.txt"

        screen -S power -dm \
               bash -c "stdbuf -oL -eL tail -F /root/Power\
                2>&1 | ts $TS_FORMAT \
               | tee $RESULTS_DIR/$LOG_PREFIX\_power_log.txt"

        screen -S quality -dm \
               bash -c "stdbuf -oL -eL tail -F /root/Quality\
                2>&1 | ts $TS_FORMAT \
               | tee $RESULTS_DIR/$LOG_PREFIX\_quality_log.txt"

        screen -S snr -dm \
               bash -c "stdbuf -oL -eL tail -F /root/SNR\
                2>&1 | ts $TS_FORMAT \
               | tee $RESULTS_DIR/$LOG_PREFIX\_snr_log.txt"

        screen -S noisefloor -dm \
              bash -c "stdbuf -oL -eL tail -F /root/NoiseFloor\
                2>&1 | ts $TS_FORMAT \
               | tee $RESULTS_DIR/$LOG_PREFIX\_noisefloor_log.txt"

        screen -S freqoffset -dm \
               bash -c "stdbuf -oL -eL tail -F /root/FreqOffset\
                2>&1 | ts $TS_FORMAT \
               | tee $RESULTS_DIR/$LOG_PREFIX\_freqoffset_log.txt"
    fi
else
    # 3+ UAVs: full TDM mode — every node runs both TX and RX
    echo "[Radio] TDM mode: $NUM_UAVS UAVs, UAV_ID=$UAV_ID"

    screen -S rxGRC -dm \
           bash -c "stdbuf -oL -eL ./startchannelsounderRXGRC.sh \
           2>&1 | ts $TS_FORMAT \
           | tee $RESULTS_DIR/$LOG_PREFIX\_radio_channelsounderrxgrc_log.txt"

    screen -S power -dm \
           bash -c "stdbuf -oL -eL tail -F /root/Power\
            2>&1 | ts $TS_FORMAT \
           | tee $RESULTS_DIR/$LOG_PREFIX\_power_log.txt"

    screen -S quality -dm \
           bash -c "stdbuf -oL -eL tail -F /root/Quality\
            2>&1 | ts $TS_FORMAT \
           | tee $RESULTS_DIR/$LOG_PREFIX\_quality_log.txt"

    screen -S snr -dm \
           bash -c "stdbuf -oL -eL tail -F /root/SNR\
            2>&1 | ts $TS_FORMAT \
           | tee $RESULTS_DIR/$LOG_PREFIX\_snr_log.txt"

    screen -S noisefloor -dm \
          bash -c "stdbuf -oL -eL tail -F /root/NoiseFloor\
            2>&1 | ts $TS_FORMAT \
           | tee $RESULTS_DIR/$LOG_PREFIX\_noisefloor_log.txt"

    screen -S freqoffset -dm \
           bash -c "stdbuf -oL -eL tail -F /root/FreqOffset\
            2>&1 | ts $TS_FORMAT \
           | tee $RESULTS_DIR/$LOG_PREFIX\_freqoffset_log.txt"

    screen -S txGRC -dm \
           bash -c "stdbuf -oL -eL ./startchannelsounderTXGRC.sh \
           2>&1 | ts $TS_FORMAT \
           | tee $RESULTS_DIR/$LOG_PREFIX\_radio_channelsoundertxgrc_log.txt"
fi

cd -
