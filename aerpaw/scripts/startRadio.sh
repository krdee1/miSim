#!/bin/bash
#RX

cd $PROFILE_DIR"/ProfileScripts/Radio/Helpers"

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


#TX

screen -S txGRC -dm \
       bash -c "stdbuf -oL -eL ./startchannelsounderTXGRC.sh \
       2>&1 | ts $TS_FORMAT \
       | tee $RESULTS_DIR/$LOG_PREFIX\_radio_channelsoundertxgrc_log.txt"
cd -
