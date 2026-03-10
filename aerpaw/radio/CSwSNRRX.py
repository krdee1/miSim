#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: CSwSNRRX
# Author: Ozgur Ozdemir
# GNU Radio version: v3.8.5.0-6-g57bd109d

from gnuradio import analog
from gnuradio import blocks
from gnuradio import digital
from gnuradio import filter
from gnuradio.filter import firdes
from gnuradio import gr
import sys
import signal
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
from gnuradio import uhd
import time
import epy_block_0
import math
import threading


def derive_num_uavs_from_csv(csv_path=None):
    """Derive number of UAVs from scenario.csv by counting initial positions.

    The initialPositions column contains a quoted comma-separated list of
    x,y,z triples.  num_uavs = len(values) / 3.
    """
    import os, csv
    if csv_path is None:
        csv_path = '/root/miSim/aerpaw/config/scenario.csv'
    with open(csv_path, 'r') as f:
        reader = csv.reader(f, skipinitialspace=True)
        header = [h.strip() for h in next(reader)]
        row = next(reader)
    col = header.index('initialPositions')
    init_pos = row[col].strip()
    n_vals = len([v.strip() for v in init_pos.split(',') if v.strip()])
    if n_vals % 3 != 0:
        raise ValueError(f"initialPositions has {n_vals} values; expected a multiple of 3")
    return n_vals // 3


class TdmTaggedFileSink(gr.sync_block):
    """GNU Radio sink that writes CSV rows tagged with the current TDM TX
    slot ID.  Each row has the form:  tx_uav_id,value

    During the guard interval the TX ID is written as -1.
    """

    def __init__(self, filepath, num_uavs, slot_duration, guard_interval):
        import numpy
        gr.sync_block.__init__(
            self,
            name='TDM Tagged File Sink',
            in_sig=[numpy.float32],
            out_sig=None)
        self._num_uavs = num_uavs
        self._slot_duration = slot_duration
        self._guard_interval = guard_interval
        self._frame_duration = slot_duration * num_uavs
        self._f = open(filepath, 'w')
        self._f.write('tx_uav_id,value\n')

    def work(self, input_items, output_items):
        now = time.time()
        slot_time = now % self._frame_duration
        current_slot = int(slot_time / self._slot_duration)
        time_into_slot = slot_time - current_slot * self._slot_duration

        if time_into_slot < self._guard_interval:
            tx_id = -1  # guard interval — ambiguous
        else:
            tx_id = current_slot

        for val in input_items[0]:
            self._f.write(f'{tx_id},{val}\n')

        self._f.flush()
        return len(input_items[0])

    def stop(self):
        self._f.close()
        return True


class CSwSNRRX(gr.top_block):

    def __init__(self, args='', freq=3.32e9, gainrx=30, noise=8, offset=250e3, samp_rate=2e6, sps=16,
                 uav_id=0, num_uavs=1, slot_duration=0.5, guard_interval=0.05):
        gr.top_block.__init__(self, "CSwSNRRX")

        ##################################################
        # Parameters
        ##################################################
        self.args = args
        self.freq = freq
        self.gainrx = gainrx
        self.noise = noise
        self.offset = offset
        self.samp_rate = samp_rate
        self.sps = sps
        self.uav_id = uav_id
        self.num_uavs = num_uavs
        self.slot_duration = slot_duration
        self.guard_interval = guard_interval

        ##################################################
        # Variables
        ##################################################
        self.nfilts = nfilts = 32
        self.alpha = alpha = 0.99
        self.rrc_taps = rrc_taps = firdes.root_raised_cosine(nfilts, nfilts*samp_rate,samp_rate/sps, alpha, 11*sps*nfilts)
        self.lbc = lbc = 0.5
        self.fun_prob = fun_prob = 0

        ##################################################
        # Blocks
        ##################################################
        self.AGCprob = blocks.probe_signal_f()
        def _fun_prob_probe():
            while True:

                val = self.AGCprob.level()
                try:
                    self.set_fun_prob(val)
                except AttributeError:
                    pass
                time.sleep(1.0 / (100))
        _fun_prob_thread = threading.Thread(target=_fun_prob_probe)
        _fun_prob_thread.daemon = True
        _fun_prob_thread.start()

        self.uhd_usrp_source_0 = uhd.usrp_source(
            ",".join(("", args)),
            uhd.stream_args(
                cpu_format="fc32",
                args='',
                channels=list(range(0,1)),
            ),
        )
        self.uhd_usrp_source_0.set_center_freq(freq, 0)
        self.uhd_usrp_source_0.set_gain(gainrx, 0)
        self.uhd_usrp_source_0.set_antenna('RX2', 0)
        self.uhd_usrp_source_0.set_samp_rate(samp_rate)
        self.uhd_usrp_source_0.set_time_unknown_pps(uhd.time_spec())
        self.freq_xlating_fft_filter_ccc_0_0 = filter.freq_xlating_fft_filter_ccc(1, (-1,    -1,    -1,    -1,    -1,    -1,    -1,    -1,    -1,    -1,    -1,     1,    -1,    -1,    -1,    -1,    -1,     1,    -1,     1,    -1,    -1,     1,    -1,    -1,    -1,    -1,     1,    -1,     1,    -1,
   1,    -1,    -1,     1,    -1,    -1,    -1,     1,    -1,    -1,     1,    -1,     1,     1,     1,    -1,     1,    -1,     1,    -1,    -1,    -1,     1,     1,    -1,    -1,    -1,     1,    -1,    -1,    -1,     1,    -1,    -1,    -1,    -1,    -1,     1,     1,    -1,    -1,    -1,     1,
  -1,     1,     1,     1,    -1,    -1,    -1,     1,     1,    -1,    -1,    -1,     1,     1,     1,    -1,     1,    -1,    -1,     1,     1,     1,    -1,     1,     1,     1,    -1,     1,    -1,    -1,    -1,    -1,     1,     1,     1,     1,    -1,     1,    -1,    -1,    -1,    -1,     1,
   1,    -1,     1,    -1,     1,    -1,    -1,     1,    -1,    -1,     1,    -1,    -1,    -1,     1,    -1,    -1,    -1,    -1,     1,     1,     1,    -1,    -1,    -1,    -1,    -1,    -1,     1,     1,     1,    -1,    -1,     1,     1,     1,    -1,     1,     1,    -1,     1,    -1,    -1,
  -1,    -1,     1,    -1,     1,    -1,    -1,     1,    -1,     1,    -1,    -1,     1,    -1,     1,     1,     1,     1,    -1,     1,     1,    -1,    -1,     1,    -1,     1,     1,     1,     1,    -1,     1,    -1,    -1,    -1,     1,    -1,     1,    -1,     1,    -1,    -1,     1,     1,
  -1,    -1,     1,    -1,    -1,    -1,    -1,    -1,     1,     1,     1,     1,    -1,     1,    -1,     1,    -1,    -1,     1,     1,    -1,    -1,    -1,    -1,    -1,    -1,    -1,    -1,    -1,     1,    -1,     1,    -1,    -1,    -1,     1,    -1,    -1,    -1,     1,     1,    -1,     1,
  -1,    -1,     1,    -1,    -1,    -1,    -1,    -1,     1,     1,    -1,     1,    -1,     1,    -1,     1,     1,    -1,    -1,     1,    -1,     1,    -1,    -1,    -1,    -1,     1,    -1,     1,     1,     1,     1,    -1,     1,    -1,     1,    -1,     1,    -1,     1,    -1,    -1,    -1,
   1,     1,     1,     1,    -1,     1,    -1,    -1,     1,    -1,     1,     1,    -1,     1,     1,     1,     1,    -1,     1,     1,    -1,     1,    -1,    -1,     1,     1,     1,    -1,     1,    -1,     1,    -1,     1,    -1,    -1,     1,    -1,    -1,     1,     1,    -1,    -1,     1,
  -1,    -1,     1,    -1,    -1,     1,     1,     1,    -1,    -1,    -1,    -1,     1,     1,    -1,     1,     1,    -1,     1,    -1,     1,    -1,     1,    -1,    -1,     1,     1,    -1,     1,     1,    -1,    -1,    -1,    -1,     1,     1,    -1,     1,     1,     1,     1,    -1,     1,
  -1,     1,     1,    -1,    -1,     1,    -1,    -1,     1,    -1,    -1,    -1,     1,     1,    -1,    -1,    -1,     1,     1,    -1,    -1,     1,    -1,    -1,     1,    -1,     1,     1,     1,     1,    -1,    -1,     1,    -1,    -1,     1,    -1,    -1,     1,    -1,     1,    -1,    -1,
  -1,    -1,    -1,     1,     1,     1,     1,     1,     1,    -1,     1,    -1,    -1,    -1,     1,     1,    -1,    -1,     1,    -1,    -1,    -1,    -1,     1,     1,     1,     1,     1,    -1,    -1,    -1,    -1,    -1,    -1,    -1,     1,    -1,    -1,    -1,     1,    -1,     1,    -1,
   1,    -1,     1,     1,     1,    -1,    -1,     1,     1,    -1,     1,    -1,    -1,    -1,     1,     1,     1,    -1,    -1,    -1,    -1,    -1,     1,     1,     1,     1,    -1,    -1,    -1,     1,    -1,    -1,     1,    -1,    -1,     1,    -1,    -1,     1,    -1,    -1,    -1,    -1,
  -1,    -1,    -1,    -1,     1,    -1,     1,     1,    -1,    -1,     1,    -1,    -1,     1,     1,    -1,    -1,     1,     1,    -1,     1,    -1,    -1,     1,    -1,     1,     1,    -1,    -1,     1,     1,     1,    -1,     1,    -1,    -1,    -1,    -1,    -1,    -1,     1,     1,    -1,
   1,     1,     1,     1,     1,     1,    -1,     1,     1,    -1,     1,     1,     1,    -1,     1,     1,    -1,    -1,     1,     1,     1,    -1,     1,     1,    -1,    -1,    -1,    -1,    -1,    -1,     1,     1,     1,     1,    -1,     1,     1,     1,    -1,    -1,     1,     1,     1,
  -1,     1,    -1,    -1,     1,    -1,    -1,    -1,     1,     1,     1,     1,    -1,     1,     1,    -1,     1,    -1,     1,     1,     1,     1,    -1,     1,     1,     1,     1,     1,    -1,     1,     1,    -1,    -1,     1,    -1,    -1,     1,     1,     1,    -1,     1,     1,    -1,
   1,     1,    -1,    -1,    -1,     1,    -1,    -1,    -1,     1,     1,    -1,    -1,    -1,    -1,     1,    -1,    -1,     1,    -1,     1,     1,     1,     1,     1,    -1,     1,    -1,    -1,     1,     1,    -1,    -1,    -1,     1,     1,    -1,    -1,    -1,    -1,    -1,     1,    -1,
   1,    -1,     1,    -1,    -1,    -1,    -1,    -1,    -1,     1,    -1,     1,     1,     1,     1,     1,    -1,    -1,     1,    -1,     1,     1,    -1,     1,     1,    -1,    -1,    -1,     1,     1,    -1,    -1,     1,     1,    -1,     1,    -1,     1,     1,    -1,     1,     1,    -1,
   1,     1,    -1,     1,    -1,    -1,    -1,    -1,    -1,    -1,     1,    -1,    -1,     1,     1,     1,     1,    -1,     1,     1,     1,     1,     1,     1,     1,     1,    -1,    -1,     1,     1,    -1,    -1,     1,     1,     1,     1,     1,    -1,     1,    -1,    -1,     1,    -1,
  -1,    -1,    -1,     1,     1,     1,    -1,     1,    -1,    -1,    -1,    -1,     1,    -1,     1,     1,    -1,     1,    -1,     1,    -1,     1,     1,     1,     1,     1,    -1,     1,    -1,     1,     1,     1,    -1,    -1,    -1,    -1,     1,     1,    -1,    -1,     1,    -1,     1,
  -1,     1,     1,     1,     1,    -1,     1,    -1,    -1,     1,     1,    -1,     1,    -1,     1,     1,    -1,    -1,     1,    -1,     1,     1,    -1,    -1,    -1,     1,    -1,    -1,     1,    -1,     1,    -1,    -1,    -1,     1,    -1,     1,     1,     1,     1,    -1,     1,     1,
   1,    -1,     1,    -1,     1,     1,    -1,     1,     1,     1,    -1,     1,    -1,     1,    -1,     1,     1,     1,     1,    -1,    -1,     1,    -1,     1,     1,    -1,    -1,     1,    -1,    -1,    -1,     1,    -1,    -1,     1,     1,     1,     1,     1,    -1,     1,     1,     1,
   1,    -1,    -1,    -1,     1,    -1,    -1,    -1,    -1,    -1,     1,    -1,    -1,    -1,    -1,     1,    -1,    -1,     1,    -1,    -1,    -1,     1,     1,     1,    -1,    -1,     1,     1,    -1,     1,     1,     1,    -1,     1,     1,     1,     1,     1,     1,     1,    -1,    -1,
  -1,     1,     1,    -1,     1,     1,    -1,     1,     1,    -1,    -1,     1,    -1,    -1,    -1,    -1,     1,    -1,     1,     1,     1,    -1,    -1,     1,    -1,     1,    -1,    -1,    -1,    -1,    -1,    -1,     1,     1,     1,     1,     1,     1,     1,     1,    -1,    -1,    -1,
   1,    -1,    -1,     1,     1,    -1,     1,    -1,    -1,     1,     1,    -1,     1,    -1,    -1,     1,    -1,    -1,     1,    -1,    -1,     1,     1,    -1,    -1,    -1,    -1,    -1,     1,    -1,    -1,    -1,     1,    -1,    -1,    -1,     1,    -1,     1,     1,    -1,    -1,     1,
   1,     1,     1,     1,    -1,    -1,    -1,    -1,     1,    -1,    -1,     1,    -1,    -1,     1,     1,     1,     1,    -1,    -1,    -1,     1,     1,     1,     1,    -1,    -1,     1,     1,     1,    -1,     1,    -1,     1,     1,    -1,    -1,    -1,     1,    -1,     1,    -1,    -1,
   1,    -1,    -1,     1,    -1,     1,    -1,     1,    -1,    -1,    -1,     1,     1,    -1,     1,    -1,     1,    -1,    -1,    -1,    -1,    -1,     1,    -1,    -1,     1,     1,     1,    -1,    -1,     1,     1,     1,     1,    -1,     1,    -1,     1,    -1,     1,     1,     1,    -1,
  -1,    -1,     1,    -1,     1,    -1,    -1,     1,     1,    -1,     1,    -1,     1,    -1,    -1,    -1,     1,    -1,     1,    -1,    -1,     1,    -1,     1,     1,    -1,     1,    -1,     1,     1,    -1,     1,     1,     1,     1,     1,    -1,     1,    -1,     1,    -1,     1,    -1,
  -1,    -1,    -1,    -1,     1,     1,    -1,     1,     1,     1,    -1,     1,     1,    -1,     1,     1,     1,     1,    -1,     1,    -1,    -1,     1,    -1,    -1,     1,    -1,     1,     1,     1,    -1,    -1,    -1,     1,    -1,    -1,    -1,    -1,     1,     1,    -1,    -1,    -1,
  -1,    -1,    -1,     1,     1,    -1,     1,    -1,     1,     1,     1,     1,    -1,    -1,     1,     1,     1,     1,    -1,    -1,     1,     1,    -1,     1,     1,    -1,     1,     1,     1,     1,     1,    -1,    -1,    -1,     1,    -1,     1,    -1,     1,    -1,     1,    -1,     1,
  -1,    -1,     1,     1,     1,     1,     1,    -1,    -1,    -1,     1,     1,    -1,    -1,     1,    -1,     1,     1,    -1,     1,     1,     1,    -1,    -1,     1,     1,    -1,     1,     1,    -1,    -1,     1,     1,     1,     1,    -1,     1,    -1,    -1,    -1,     1,     1,     1,
  -1,     1,    -1,    -1,    -1,     1,     1,    -1,     1,     1,    -1,    -1,    -1,    -1,    -1,     1,    -1,     1,     1,     1,    -1,    -1,    -1,    -1,     1,    -1,    -1,    -1,     1,    -1,     1,     1,     1,    -1,     1,     1,     1,     1,    -1,    -1,     1,    -1,    -1,
  -1,    -1,    -1,     1,    -1,     1,     1,    -1,     1,    -1,    -1,    -1,     1,     1,     1,     1,    -1,    -1,    -1,    -1,     1,    -1,     1,    -1,    -1,    -1,     1,     1,    -1,    -1,     1,     1,    -1,    -1,    -1,     1,     1,    -1,     1,    -1,    -1,    -1,     1,
  -1,    -1,    -1,    -1,    -1,    -1,     1,    -1,    -1,    -1,     1,     1,     1,    -1,     1,    -1,     1,    -1,     1,     1,    -1,     1,    -1,    -1,     1,    -1,    -1,     1,     1,    -1,     1,     1,    -1,    -1,     1,    -1,     1,     1,    -1,     1,    -1,     1,    -1,
  -1,     1,     1,     1,     1,    -1,    -1,    -1,    -1,     1,     1,     1,    -1,    -1,    -1,     1,    -1,    -1,     1,     1,     1,     1,    -1,    -1,     1,     1,     1,     1,     1,    -1,     1,     1,    -1,     1,    -1,    -1,    -1,     1,     1,    -1,     1,    -1,    -1,
  -1,    -1,    -1,    -1,    -1,    -1,    -1,     1,     1,     1,    -1,    -1,    -1,     1,     1,    -1,     1,     1,     1,     1,     1,    -1,    -1,     1,     1,    -1,     1,    -1,     1,     1,     1,     1,     1,    -1,     1,     1,     1,     1,     1,    -1,    -1,     1,    -1,
  -1,     1,    -1,     1,     1,    -1,     1,    -1,    -1,     1,    -1,     1,     1,     1,    -1,     1,     1,     1,    -1,    -1,    -1,     1,    -1,    -1,     1,    -1,     1,     1,    -1,    -1,     1,    -1,     1,    -1,     1,    -1,    -1,     1,    -1,     1,    -1,     1,    -1,
  -1,     1,     1,     1,    -1,     1,    -1,    -1,    -1,     1,    -1,    -1,     1,     1,    -1,    -1,     1,    -1,     1,     1,    -1,    -1,     1,     1,    -1,    -1,     1,    -1,    -1,    -1,     1,    -1,     1,     1,     1,     1,     1,     1,     1,     1,    -1,     1,     1,
   1,    -1,    -1,     1,    -1,     1,    -1,     1,    -1,    -1,    -1,    -1,     1,    -1,     1,    -1,     1,     1,    -1,     1,    -1,    -1,    -1,    -1,    -1,     1,     1,    -1,    -1,     1,     1,    -1,     1,     1,     1,     1,    -1,     1,     1,     1,     1,    -1,    -1,
   1,     1,    -1,    -1,    -1,    -1,     1,     1,     1,    -1,    -1,     1,     1,    -1,    -1,     1,     1,    -1,     1,     1,    -1,     1,    -1,     1,     1,     1,    -1,    -1,     1,     1,     1,     1,    -1,    -1,    -1,     1,    -1,     1,     1,    -1,    -1,     1,    -1,
   1,     1,     1,    -1,    -1,     1,    -1,    -1,    -1,    -1,    -1,    -1,    -1,     1,     1,    -1,     1,     1,    -1,     1,     1,     1,    -1,     1,    -1,    -1,    -1,     1,     1,     1,     1,     1,    -1,    -1,    -1,     1,    -1,    -1,    -1,     1,    -1,     1,    -1,
  -1,    -1,     1,     1,     1,    -1,     1,     1,    -1,    -1,     1,     1,    -1,    -1,     1,     1,    -1,    -1,     1,    -1,     1,    -1,     1,    -1,     1,     1,    -1,     1,    -1,     1,     1,    -1,    -1,     1,     1,     1,     1,    -1,    -1,    -1,    -1,    -1,     1,
   1,    -1,    -1,    -1,    -1,    -1,     1,     1,     1,    -1,     1,    -1,    -1,     1,    -1,     1,    -1,     1,     1,     1,     1,     1,     1,    -1,    -1,     1,     1,     1,     1,     1,     1,     1,     1,    -1,     1,    -1,     1,    -1,    -1,     1,    -1,    -1,    -1,
  -1,    -1,    -1,     1,    -1,     1,    -1,     1,     1,     1,    -1,    -1,    -1,    -1,    -1,     1,    -1,    -1,     1,    -1,    -1,    -1,    -1,     1,     1,    -1,    -1,     1,    -1,    -1,    -1,     1,     1,     1,     1,     1,     1,     1,    -1,     1,    -1,    -1,     1,
  -1,     1,    -1,    -1,     1,     1,     1,     1,     1,     1,    -1,    -1,     1,     1,    -1,     1,     1,     1,     1,     1,     1,     1,     1,     1,    -1,     1,     1,    -1,    -1,    -1,     1,    -1,     1,     1,     1,     1,    -1,    -1,     1,     1,    -1,     1,    -1,
  -1,     1,     1,     1,     1,    -1,    -1,     1,    -1,     1,     1,     1,    -1,     1,    -1,    -1,    -1,    -1,    -1,     1,     1,     1,    -1,     1,     1,    -1,     1,    -1,     1,    -1,    -1,     1,    -1,     1,     1,    -1,    -1,    -1,     1,     1,    -1,     1,    -1,
   1,    -1,     1,    -1,    -1,    -1,     1,    -1,     1,     1,    -1,     1,    -1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,    -1,    -1,    -1,    -1,    -1,    -1,     1,     1,    -1,    -1,    -1,     1,     1,     1,     1,     1,    -1,    -1,
   1,     1,    -1,    -1,    -1,     1,     1,     1,     1,    -1,    -1,    -1,     1,     1,    -1,     1,    -1,    -1,     1,     1,    -1,    -1,    -1,    -1,     1,    -1,    -1,    -1,    -1,     1,     1,     1,     1,    -1,    -1,    -1,    -1,    -1,    -1,     1,    -1,    -1,    -1,
  -1,     1,     1,    -1,     1,    -1,    -1,    -1,    -1,     1,    -1,    -1,    -1,    -1,     1,    -1,     1,     1,    -1,    -1,    -1,     1,    -1,     1,     1,    -1,     1,    -1,    -1,     1,     1,     1,     1,     1,    -1,     1,    -1,     1,     1,    -1,    -1,    -1,    -1,
  -1,     1,    -1,    -1,     1,     1,    -1,    -1,    -1,     1,     1,     1,    -1,    -1,    -1,    -1,     1,     1,     1,     1,     1,    -1,     1,    -1,    -1,    -1,    -1,    -1,    -1,    -1,     1,    -1,     1,     1,     1,    -1,     1,    -1,    -1,     1,    -1,    -1,     1,
   1,     1,     1,     1,    -1,    -1,     1,     1,     1,    -1,    -1,     1,     1,     1,    -1,    -1,     1,    -1,     1,    -1,    -1,     1,    -1,    -1,    -1,     1,     1,    -1,     1,    -1,     1,     1,    -1,    -1,    -1,    -1,     1,     1,    -1,    -1,     1,     1,     1,
  -1,     1,     1,     1,    -1,    -1,    -1,    -1,    -1,    -1,     1,    -1,     1,    -1,    -1,     1,     1,    -1,    -1,    -1,     1,    -1,    -1,    -1,    -1,    -1,    -1,    -1,    -1,    -1,    -1,     1,     1,    -1,    -1,    -1,    -1,     1,     1,     1,     1,    -1,     1,
   1,    -1,    -1,    -1,     1,     1,     1,     1,     1,     1,    -1,     1,     1,    -1,    -1,     1,     1,    -1,     1,     1,     1,    -1,    -1,     1,     1,     1,     1,     1,    -1,    -1,     1,    -1,     1,    -1,    -1,     1,     1,    -1,    -1,     1,     1,    -1,    -1,
  -1,    -1,     1,    -1,     1,    -1,    -1,     1,     1,     1,    -1,    -1,     1,    -1,    -1,     1,    -1,     1,    -1,    -1,     1,    -1,    -1,     1,     1,     1,    -1,     1,    -1,    -1,     1,     1,    -1,    -1,     1,     1,     1,    -1,    -1,    -1,     1,    -1,    -1,
  -1,     1,     1,     1,    -1,    -1,    -1,     1,    -1,     1,     1,     1,     1,     1,    -1,     1,     1,    -1,     1,     1,    -1,    -1,     1,     1,    -1,    -1,    -1,     1,    -1,    -1,     1,    -1,    -1,    -1,    -1,    -1,     1,    -1,    -1,     1,    -1,     1,    -1,
  -1,     1,     1,    -1,     1,     1,     1,    -1,    -1,    -1,     1,     1,     1,     1,    -1,     1,     1,     1,     1,    -1,     1,     1,     1,    -1,    -1,    -1,     1,     1,    -1,     1,    -1,     1,     1,     1,    -1,    -1,    -1,     1,     1,     1,    -1,    -1,     1,
   1,     1,     1,     1,     1,    -1,     1,    -1,     1,    -1,     1,     1,    -1,    -1,    -1,    -1,     1,    -1,    -1,    -1,     1,     1,     1,     1,     1,    -1,     1,    -1,     1,    -1,    -1,    -1,    -1,    -1,    -1,    -1,    -1,     1,     1,     1,     1,    -1,    -1,
   1,     1,    -1,    -1,     1,    -1,     1,     1,     1,    -1,     1,     1,    -1,    -1,    -1,    -1,     1,    -1,     1,     1,     1,     1,     1,     1,    -1,     1,    -1,     1,     1,     1,     1,    -1,    -1,    -1,     1,     1,     1,    -1,    -1,    -1,     1,     1,     1,
   1,     1,     1,     1,     1,     1,    -1,    -1,     1,    -1,    -1,    -1,     1,     1,     1,    -1,     1,     1,     1,    -1,     1,     1,    -1,    -1,    -1,     1,     1,    -1,     1,     1,     1,    -1,     1,    -1,    -1,     1,     1,     1,     1,     1,     1,     1,    -1,
   1,     1,    -1,     1,    -1,     1,    -1,     1,     1,    -1,     1,     1,    -1,     1,    -1,    -1,     1,    -1,    -1,    -1,     1,    -1,     1,     1,    -1,     1,     1,     1,     1,     1,     1,     1,    -1,     1,    -1,     1,     1,    -1,     1,    -1,    -1,    -1,     1,
  -1,     1,     1,    -1,    -1,    -1,     1,     1,     1,     1,    -1,     1,    -1,     1,     1,    -1,     1,     1,    -1,    -1,     1,    -1,     1,    -1,    -1,     1,    -1,     1,    -1,     1,     1,    -1,     1,     1,     1,    -1,    -1,    -1,     1,    -1,     1,     1,    -1,
   1,     1,    -1,     1,     1,     1,     1,    -1,    -1,    -1,    -1,     1,    -1,    -1,    -1,    -1,    -1,     1,     1,     1,    -1,    -1,     1,    -1,     1,    -1,     1,     1,    -1,    -1,    -1,     1,    -1,    -1,    -1,    -1,     1,    -1,    -1,    -1,    -1,    -1,    -1,
   1,     1,    -1,    -1,     1,     1,     1,     1,     1,     1,     1,    -1,    -1,     1,    -1,     1,    -1,     1,     1,     1,    -1,    -1,     1,    -1,    -1,     1,    -1,    -1,    -1,    -1,     1,    -1,    -1,    -1,     1,    -1,    -1,     1,     1,    -1,     1,     1,    -1,
   1,     1,    -1,     1,     1,    -1,    -1,    -1,    -1,    -1,    -1,    -1,     1,     1,     1,    -1,     1,    -1,     1,     1,    -1,     1,    -1,     1,    -1,     1,    -1,     1,     1,     1,    -1,     1,     1,     1,    -1,     1,    -1,     1,    -1,    -1,     1,     1,     1,
  -1,    -1,    -1,    -1,    -1,     1,    -1,     1,     1,    -1,    -1,    -1,    -1,    -1,     1,     1,    -1,     1,     1,    -1,    -1,     1,     1,    -1,     1,    -1,     1,    -1,    -1,     1,     1,    -1,     1,    -1,    -1,    -1,    -1,    -1,     1,    -1,    -1,    -1,     1,
   1,    -1,    -1,     1,    -1,     1,    -1,    -1,     1,     1,     1,    -1,     1,     1,    -1,    -1,     1,    -1,    -1,    -1,     1,     1,    -1,     1,     1,     1,     1,    -1,    -1,    -1,     1,     1,    -1,    -1,    -1,    -1,     1,     1,    -1,     1,    -1,     1,     1,
  -1,     1,    -1,    -1,     1,     1,    -1,     1,     1,    -1,     1,    -1,    -1,     1,     1,    -1,    -1,     1,    -1,     1,    -1,    -1,    -1,     1,     1,    -1,     1,     1,     1,    -1,    -1,    -1,    -1,     1,     1,     1,    -1,     1,    -1,     1,    -1,    -1,     1,
  -1,     1,    -1,    -1,    -1,    -1,     1,     1,     1,     1,     1,     1,    -1,    -1,    -1,    -1,    -1,     1,     1,     1,    -1,    -1,    -1,    -1,     1,    -1,     1,     1,     1,    -1,     1,     1,    -1,     1,    -1,    -1,     1,    -1,     1,    -1,     1,    -1,     1,
   1,     1,     1,    -1,     1,     1,    -1,     1,     1,    -1,     1,     1,     1,    -1,    -1,    -1,    -1,    -1,     1,     1,    -1,     1,    -1,    -1,    -1,     1,     1,    -1,    -1,    -1,    -1,    -1,    -1,    -1,     1,    -1,     1,    -1,     1,    -1,     1,    -1,    -1,
  -1,    -1,     1,     1,     1,    -1,     1,     1,    -1,    -1,    -1,     1,    -1,    -1,     1,     1,     1,    -1,    -1,    -1,     1,     1,     1,    -1,     1,     1,     1,     1,     1,     1,    -1,    -1,    -1,    -1,     1,     1,     1,     1,    -1,    -1,     1,    -1,    -1,
  -1,     1,    -1,     1,    -1,     1,     1,     1,     1,     1,    -1,    -1,    -1,     1,     1,     1,    -1,     1,    -1,     1,     1,     1,     1,    -1,     1,    -1,     1,     1,     1,    -1,     1,    -1,    -1,     1,     1,    -1,     1,     1,     1,     1,    -1,    -1,     1,
   1,     1,    -1,    -1,    -1,     1,     1,    -1,    -1,     1,     1,     1,     1,    -1,     1,     1,    -1,    -1,     1,     1,     1,     1,     1,     1,    -1,    -1,    -1,     1,    -1,     1,     1,     1,    -1,     1,    -1,     1,     1,    -1,    -1,     1,     1,    -1,     1,
  -1,    -1,    -1,    -1,     1,     1,    -1,    -1,    -1,     1,    -1,    -1,     1,     1,    -1,    -1,    -1,    -1,     1,     1,    -1,    -1,    -1,     1,     1,    -1,     1,     1,    -1,    -1,     1,    -1,    -1,     1,    -1,     1,    -1,     1,     1,    -1,    -1,     1,     1,
  -1,    -1,    -1,    -1,    -1,    -1,     1,    -1,    -1,     1,    -1,     1,     1,    -1,     1,     1,    -1,     1,    -1,     1,     1,    -1,    -1,    -1,     1,     1,     1,    -1,    -1,     1,    -1,     1,     1,     1,     1,    -1,    -1,    -1,    -1,    -1,     1,    -1,    -1,
  -1,    -1,    -1,    -1,    -1,     1,    -1,    -1,     1,     1,    -1,     1,    -1,     1,     1,     1,    -1,     1,    -1,     1,     1,     1,    -1,     1,     1,    -1,     1,     1,    -1,     1,    -1,     1,    -1,    -1,    -1,    -1,     1,     1,    -1,    -1,     1,     1,    -1,
  -1,     1,     1,     1,    -1,     1,    -1,     1,    -1,    -1,    -1,    -1,     1,    -1,    -1,    -1,     1,     1,    -1,     1,     1,    -1,     1,    -1,    -1,    -1,     1,    -1,    -1,     1,    -1,    -1,    -1,     1,    -1,     1,    -1,    -1,     1,     1,     1,     1,    -1,
   1,    -1,    -1,     1,     1,     1,     1,    -1,     1,     1,    -1,     1,     1,     1,     1,     1,     1,    -1,    -1,     1,    -1,     1,     1,     1,     1,     1,    -1,    -1,    -1,    -1,     1,     1,    -1,     1,    -1,    -1,     1,    -1,     1,    -1,    -1,    -1,     1,
   1,     1,     1,     1,     1,    -1,    -1,     1,    -1,    -1,     1,     1,     1,     1,    -1,     1,    -1,     1,     1,     1,     1,     1,    -1,    -1,     1,     1,     1,     1,    -1,     1,     1,     1,    -1,     1,     1,     1,     1,    -1,     1,     1,    -1,    -1,    -1,    -1,     1,     1,     1,     1,     1,     1,     1,    -1,    -1,    -1,    -1,     1,    -1,     1,     1,    -1,    -1,     1,     1,    -1,     1,     1,    -1,    -1,    -1,     1,     1,     1,    -1,     1,     1,    -1,     1,     1,     1,    -1,    -1,     1,    -1,    -1,     1,     1,    -1,    -1,    -1,     1,    -1,     1,    -1,    -1,    -1,    -1,    -1,     1,    -1,     1,     1,     1,     1,    -1,    -1,    -1,     1,    -1,     1,    -1,    -1,    -1,     1,    -1,     1,    -1,     1,     1,    -1,     1,     1,    -1,    -1,    -1,    -1,     1,    -1,    -1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,    -1,     1,    -1,    -1,    -1,    -1,     1,    -1,    -1,     1,    -1,     1,    -1,     1,     1,     1,    -1,     1,     1,    -1,    -1,     1,    -1,     1,    -1,     1,     1,    -1,     1,    -1,     1,    -1,    -1,    -1,     1,     1,     1,    -1,    -1,     1,    -1,    -1,     1,     1,     1,    -1,    -1,     1,    -1,     1,     1,    -1,     1,    -1,    -1,    -1,    -1,     1,     1,     1,    -1,    -1,     1,    -1,    -1,    -1,     1,     1,    -1,    -1,     1,     1,     1,    -1,    -1,     1,     1,    -1,    -1,    -1,     1,    -1,     1,     1,    -1,    -1,    -1,    -1,     1,     1,     1,    -1,     1,     1,     1,    -1,    -1,     1,    -1,    -1,    -1,     1,    -1,    -1,    -1,     1,     1,     1,     1,    -1,    -1,     1,    -1,     1,    -1,     1,    -1,     1,    -1,    -1,     1,    -1,     1,     1,     1,    -1,    -1,     1,     1,    -1,    -1,    -1,    -1,    -1,     1,     1,    -1,    -1,     1,    -1,    -1,     1,     1,     1,     1,     1,     1,    -1,     1,     1,     1,    -1,     1,     1,    -1,     1,    -1,     1,     1,    -1,     1,    -1,     1,     1,     1,    -1,     1,     1,     1,     1,     1,    -1,     1,    -1,    -1,    -1,     1,    -1,    -1,    -1,     1,    -1,    -1,     1,    -1,    -1,     1,     1,    -1,     1,    -1,    -1,    -1,     1,    -1,     1,    -1,    -1,    -1,    -1,     1,     1,    -1,     1,     1,     1,    -1,    -1,     1,    -1,     1,     1,     1,    -1,    -1,    -1,    -1,    -1,    -1,    -1,    -1,     1,    -1,    -1,     1,    -1,    -1,     1,    -1,     1,     1,    -1,    -1,    -1,    -1,     1,    -1,     1,    -1,     1,     1,     1,     1,    -1,    -1,    -1,    -1,     1,     1,    -1,    -1,    -1,    -1,     1,    -1,     1,     1,    -1,     1,     1,     1,    -1,     1,     1,     1,    -1,     1,     1,     1,    -1,    -1,     1,     1,    -1,    -1,     1,    -1,    -1,     1,     1,    -1,     1,     1,     1,    -1,     1,    -1,     1,     1,     1,     1,     1,     1,    -1,     1,     1,     1,     1,     1,     1,    -1,     1,    -1,    -1,     1,     1,     1,    -1,    -1,     1,     1,    -1,     1,    -1,     1,    -1,     1,     1,     1,    -1,     1,    -1,     1,    -1,     1,    -1,     1,     1,    -1,    -1,     1,     1,     1,    -1,    -1,    -1,    -1,    -1,    -1,    -1,     1,     1,    -1,    -1,     1,    -1,     1,     1,     1,     1,     1,     1,    -1,    -1,    -1,     1,     1,     1,     1,     1,    -1,     1,     1,     1,    -1,    -1,    -1,    -1,     1,    -1,     1,    -1,     1,    -1,     1,     1,    -1,    -1,    -1,     1,     1,    -1,    -1,    -1,     1,    -1,     1,    -1,     1,    -1,    -1,    -1,     1,    -1,    -1,     1,    -1,     1,    -1,     1,    -1,     1,    -1,     1,     1,    -1,     1,     1,     1,     1,    -1,    -1,     1,    -1,     1,    -1,    -1,    -1,     1,    -1,    -1,     1,     1,     1,    -1,     1,    -1,     1,     1,     1,    -1,    -1,     1,    -1,     1,     1,    -1,    -1,    -1,    -1,    -1,    -1,     1,    -1,     1,     1,    -1,     1,     1,    -1,    -1,     1,     1,     1,    -1,    -1,     1,    -1,    -1,    -1,    -1,     1,    -1,    -1,     1,     1,    -1,    -1,     1,     1,     1,     1,    -1,    -1,     1,    -1,    -1,     1,     1,    -1,     1,    -1,     1,    -1,     1,    -1,     1,    -1,     1,    -1,     1,     1,     1,     1,     1,     1,     1,    -1,     1,     1,     1,     1,    -1,     1,    -1,     1,    -1,    -1,    -1,     1,    -1,    -1,    -1,    -1,     1,    -1,     1,    -1,    -1,    -1,    -1,     1,    -1,    -1,     1,     1,     1,    -1,     1,     1,     1,     1,     1,    -1,    -1,    -1,    -1,    -1,     1,    -1,     1,    -1,    -1,    -1,    -1,    -1,    -1,    -1,     1,     1,     1,     1,     1,    -1,     1,     1,    -1,    -1,    -1,    -1,    -1,     1,     1,     1,     1,     1,    -1,    -1,     1,    -1,    -1,    -1,    -1,     1,     1,    -1,     1,     1,    -1,    -1,    -1,     1,    -1,     1,    -1,     1,     1,    -1,    -1,     1,    -1,    -1,    -1,    -1,    -1,    -1,     1,     1,     1,    -1,     1,     1,     1,     1,    -1,     1,    -1,    -1,    -1,    -1,    -1,     1,    -1,     1,    -1,     1,     1,    -1,    -1,    -1,    -1,    -1,    -1,    -1,    -1,     1,     1,    -1,     1,    -1,    -1,     1,     1,     1,    -1,    -1,    -1,     1,    -1,     1,    -1,     1,     1,     1,    -1,     1,    -1,    -1,    -1,     1,    -1,     1,     1,     1,    -1,    -1,     1,     1,     1,    -1,    -1,    -1,    -1,     1,    -1,    -1,     1,     1,    -1,     1,     1,     1,     1,     1,    -1,     1,     1,     1,    -1,     1,    -1,    -1,     1,    -1,     1,     1,     1,     1,     1,     1,     1,    -1,    -1,     1,     1,     1,    -1,     1,     1,     1,     1), 0, samp_rate/sps)
        self.freq_xlating_fft_filter_ccc_0_0.set_nthreads(8)
        self.freq_xlating_fft_filter_ccc_0_0.declare_sample_delay(0)
        self.epy_block_0 = epy_block_0.blk(example_param=1)
        self.digital_symbol_sync_xx_0 = digital.symbol_sync_cc(
            digital.TED_SIGNAL_TIMES_SLOPE_ML,
            sps,
            0.045,
            1.0,
            1.0,
            1.5,
            1,
            digital.constellation_bpsk().base(),
            digital.IR_PFB_MF,
            nfilts,
            rrc_taps)
        self.digital_fll_band_edge_cc_0_0 = digital.fll_band_edge_cc(sps, alpha, sps*2+1, 2*math.pi/sps/100)
        self.digital_costas_loop_cc_0 = digital.costas_loop_cc(lbc, 2, False)
        self.blocks_vector_to_stream_0_0 = blocks.vector_to_stream(gr.sizeof_gr_complex*1, 4095)
        self.blocks_sub_xx_0 = blocks.sub_ff(1)
        self.blocks_stream_to_vector_0_0 = blocks.stream_to_vector(gr.sizeof_gr_complex*1, 4095)
        self.blocks_nlog10_ff_0_0_0 = blocks.nlog10_ff(20, 1, 0)
        self.blocks_nlog10_ff_0_0 = blocks.nlog10_ff(20, 1, 0)
        self.blocks_multiply_xx_1 = blocks.multiply_vcc(1)
        self.blocks_multiply_xx_0 = blocks.multiply_vcc(1)
        self.blocks_moving_average_xx_1_0 = blocks.moving_average_cc(1000, 1/1000, 4000, 1)
        self.blocks_moving_average_xx_1 = blocks.moving_average_cc(3, 1, 4000, 1)
        self.blocks_moving_average_xx_0_0 = blocks.moving_average_ff(sps, 1, 4000, 1)
        self.blocks_moving_average_xx_0 = blocks.moving_average_ff(sps, 1, 4000, 1)
        self.blocks_keep_m_in_n_1_0 = blocks.keep_m_in_n(gr.sizeof_gr_complex, 1, 1000, 999)
        self.blocks_keep_m_in_n_1 = blocks.keep_m_in_n(gr.sizeof_gr_complex, 1, 3, 2)
        self.blocks_keep_m_in_n_0_0 = blocks.keep_m_in_n(gr.sizeof_gr_complex, 1000, 4095, 2000)
        self.blocks_keep_m_in_n_0 = blocks.keep_m_in_n(gr.sizeof_gr_complex, 3, 4095, 1)
        self.blocks_float_to_complex_0 = blocks.float_to_complex(1)
        self.blocks_file_sink_0_0_0_0 = TdmTaggedFileSink(
            '/root/SNR', num_uavs, slot_duration, guard_interval)
        self.blocks_file_sink_0_0_0 = TdmTaggedFileSink(
            '/root/Quality', num_uavs, slot_duration, guard_interval)
        self.blocks_file_sink_0 = TdmTaggedFileSink(
            '/root/Power', num_uavs, slot_duration, guard_interval)
        self.blocks_file_sink_noisefloor = TdmTaggedFileSink(
            '/root/NoiseFloor', num_uavs, slot_duration, guard_interval)
        self._freqoffset_file = open('/root/FreqOffset', 'w')
        self._freqoffset_file.write('tx_uav_id,value\n')
        self.blocks_divide_xx_0 = blocks.divide_ff(1)
        self.blocks_complex_to_real_0_0 = blocks.complex_to_real(1)
        self.blocks_complex_to_real_0 = blocks.complex_to_real(1)
        self.blocks_complex_to_mag_0_0_0 = blocks.complex_to_mag(1)
        self.blocks_complex_to_mag_0_0 = blocks.complex_to_mag(1)
        self.blocks_add_const_vxx_0_0 = blocks.add_const_ff(-noise)
        self.blocks_add_const_vxx_0 = blocks.add_const_ff(-gainrx)
        self.analog_sig_source_x_0 = analog.sig_source_c(samp_rate, analog.GR_COS_WAVE, -offset, 1, 0, 0)
        self.analog_const_source_x_0_0 = analog.sig_source_f(0, analog.GR_CONST_WAVE, 0, 0, fun_prob)
        self.analog_const_source_x_0 = analog.sig_source_f(0, analog.GR_CONST_WAVE, 0, 0, 0)
        self.analog_agc_xx_0 = analog.agc_cc(1e-4, 1.0, 1.0)
        self.analog_agc_xx_0.set_max_gain(65536)
        ##################################################
        # Connections
        ##################################################
        self.connect((self.analog_agc_xx_0, 0), (self.blocks_complex_to_real_0, 0))
        self.connect((self.analog_agc_xx_0, 0), (self.digital_fll_band_edge_cc_0_0, 0))
        self.connect((self.analog_const_source_x_0, 0), (self.blocks_float_to_complex_0, 1))
        self.connect((self.analog_const_source_x_0_0, 0), (self.blocks_float_to_complex_0, 0))
        self.connect((self.analog_sig_source_x_0, 0), (self.blocks_multiply_xx_0, 1))
        self.connect((self.blocks_add_const_vxx_0, 0), (self.blocks_add_const_vxx_0_0, 0))
        self.connect((self.blocks_add_const_vxx_0, 0), (self.blocks_file_sink_0, 0))
        self.connect((self.blocks_add_const_vxx_0_0, 0), (self.blocks_file_sink_0_0_0_0, 0))
        self.connect((self.blocks_complex_to_mag_0_0, 0), (self.blocks_nlog10_ff_0_0, 0))
        self.connect((self.blocks_complex_to_mag_0_0_0, 0), (self.blocks_nlog10_ff_0_0_0, 0))
        self.connect((self.blocks_complex_to_real_0, 0), (self.blocks_moving_average_xx_0_0, 0))
        self.connect((self.blocks_complex_to_real_0_0, 0), (self.blocks_moving_average_xx_0, 0))
        self.connect((self.blocks_divide_xx_0, 0), (self.AGCprob, 0))
        self.connect((self.blocks_float_to_complex_0, 0), (self.blocks_multiply_xx_1, 1))
        self.connect((self.blocks_keep_m_in_n_0, 0), (self.blocks_moving_average_xx_1, 0))
        self.connect((self.blocks_keep_m_in_n_0_0, 0), (self.blocks_moving_average_xx_1_0, 0))
        self.connect((self.blocks_keep_m_in_n_1, 0), (self.blocks_complex_to_mag_0_0, 0))
        self.connect((self.blocks_keep_m_in_n_1_0, 0), (self.blocks_complex_to_mag_0_0_0, 0))
        self.connect((self.blocks_moving_average_xx_0, 0), (self.blocks_divide_xx_0, 0))
        self.connect((self.blocks_moving_average_xx_0_0, 0), (self.blocks_divide_xx_0, 1))
        self.connect((self.blocks_moving_average_xx_1, 0), (self.blocks_keep_m_in_n_1, 0))
        self.connect((self.blocks_moving_average_xx_1_0, 0), (self.blocks_keep_m_in_n_1_0, 0))
        self.connect((self.blocks_multiply_xx_0, 0), (self.analog_agc_xx_0, 0))
        self.connect((self.blocks_multiply_xx_0, 0), (self.blocks_complex_to_real_0_0, 0))
        self.connect((self.blocks_multiply_xx_1, 0), (self.freq_xlating_fft_filter_ccc_0_0, 0))
        self.connect((self.blocks_nlog10_ff_0_0, 0), (self.blocks_add_const_vxx_0, 0))
        self.connect((self.blocks_nlog10_ff_0_0, 0), (self.blocks_sub_xx_0, 0))
        self.connect((self.blocks_nlog10_ff_0_0_0, 0), (self.blocks_sub_xx_0, 1))
        self.connect((self.blocks_nlog10_ff_0_0_0, 0), (self.blocks_file_sink_noisefloor, 0))
        self.connect((self.blocks_stream_to_vector_0_0, 0), (self.epy_block_0, 0))
        self.connect((self.blocks_sub_xx_0, 0), (self.blocks_file_sink_0_0_0, 0))
        self.connect((self.blocks_vector_to_stream_0_0, 0), (self.blocks_keep_m_in_n_0, 0))
        self.connect((self.blocks_vector_to_stream_0_0, 0), (self.blocks_keep_m_in_n_0_0, 0))
        self.connect((self.digital_costas_loop_cc_0, 0), (self.blocks_multiply_xx_1, 0))
        self.connect((self.digital_fll_band_edge_cc_0_0, 0), (self.digital_symbol_sync_xx_0, 0))
        self.connect((self.digital_symbol_sync_xx_0, 0), (self.digital_costas_loop_cc_0, 0))
        self.connect((self.epy_block_0, 0), (self.blocks_vector_to_stream_0_0, 0))
        self.connect((self.freq_xlating_fft_filter_ccc_0_0, 0), (self.blocks_stream_to_vector_0_0, 0))
        self.connect((self.uhd_usrp_source_0, 0), (self.blocks_multiply_xx_0, 0))

        ##################################################
        # Frequency offset polling thread
        ##################################################
        def _freq_offset_probe():
            frame_dur = slot_duration * num_uavs
            while True:
                val = self.digital_fll_band_edge_cc_0_0.get_frequency()
                freq_hz = val * samp_rate / (2 * math.pi)
                now = time.time()
                slot_time = now % frame_dur
                current_slot = int(slot_time / slot_duration)
                time_into_slot = slot_time - current_slot * slot_duration
                tx_id = -1 if time_into_slot < guard_interval else current_slot
                self._freqoffset_file.write(f'{tx_id},{freq_hz}\n')
                self._freqoffset_file.flush()
                time.sleep(0.01)
        _freq_offset_thread = threading.Thread(target=_freq_offset_probe)
        _freq_offset_thread.daemon = True
        _freq_offset_thread.start()


    def get_args(self):
        return self.args

    def set_args(self, args):
        self.args = args

    def get_freq(self):
        return self.freq

    def set_freq(self, freq):
        self.freq = freq
        self.uhd_usrp_source_0.set_center_freq(self.freq, 0)

    def get_gainrx(self):
        return self.gainrx

    def set_gainrx(self, gainrx):
        self.gainrx = gainrx
        self.blocks_add_const_vxx_0.set_k(-self.gainrx)
        self.uhd_usrp_source_0.set_gain(self.gainrx, 0)
        self.uhd_usrp_source_0.set_gain(self.gainrx, 1)

    def get_noise(self):
        return self.noise

    def set_noise(self, noise):
        self.noise = noise
        self.blocks_add_const_vxx_0_0.set_k(-self.noise)

    def get_offset(self):
        return self.offset

    def set_offset(self, offset):
        self.offset = offset
        self.analog_sig_source_x_0.set_frequency(-self.offset)

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.set_rrc_taps(firdes.root_raised_cosine(self.nfilts, self.nfilts*self.samp_rate, self.samp_rate/self.sps, self.alpha, 11*self.sps*self.nfilts))
        self.analog_sig_source_x_0.set_sampling_freq(self.samp_rate)
        self.uhd_usrp_source_0.set_samp_rate(self.samp_rate)

    def get_sps(self):
        return self.sps

    def set_sps(self, sps):
        self.sps = sps
        self.set_rrc_taps(firdes.root_raised_cosine(self.nfilts, self.nfilts*self.samp_rate, self.samp_rate/self.sps, self.alpha, 11*self.sps*self.nfilts))
        self.blocks_moving_average_xx_0.set_length_and_scale(self.sps, 1)
        self.blocks_moving_average_xx_0_0.set_length_and_scale(self.sps, 1)
        self.digital_fll_band_edge_cc_0_0.set_loop_bandwidth(2*math.pi/self.sps/100)

    def get_nfilts(self):
        return self.nfilts

    def set_nfilts(self, nfilts):
        self.nfilts = nfilts
        self.set_rrc_taps(firdes.root_raised_cosine(self.nfilts, self.nfilts*self.samp_rate, self.samp_rate/self.sps, self.alpha, 11*self.sps*self.nfilts))

    def get_alpha(self):
        return self.alpha

    def set_alpha(self, alpha):
        self.alpha = alpha
        self.set_rrc_taps(firdes.root_raised_cosine(self.nfilts, self.nfilts*self.samp_rate, self.samp_rate/self.sps, self.alpha, 11*self.sps*self.nfilts))

    def get_rrc_taps(self):
        return self.rrc_taps

    def set_rrc_taps(self, rrc_taps):
        self.rrc_taps = rrc_taps

    def get_lbc(self):
        return self.lbc

    def set_lbc(self, lbc):
        self.lbc = lbc
        self.digital_costas_loop_cc_0.set_loop_bandwidth(self.lbc)

    def get_fun_prob(self):
        return self.fun_prob

    def set_fun_prob(self, fun_prob):
        self.fun_prob = fun_prob
        self.analog_const_source_x_0_0.set_offset(self.fun_prob)

def argument_parser():
    parser = ArgumentParser()
    parser.add_argument(
        "--args", dest="args", type=str, default='',
        help="Set args [default=%(default)r]")
    parser.add_argument(
        "--freq", dest="freq", type=eng_float, default="3.32G",
        help="Set freq [default=%(default)r]")
    parser.add_argument(
        "--gainrx", dest="gainrx", type=eng_float, default="30.0",
        help="Set gainrx [default=%(default)r]")
    parser.add_argument(
        "--noise", dest="noise", type=eng_float, default="8.0",
        help="Set noise [default=%(default)r]")
    parser.add_argument(
        "--offset", dest="offset", type=eng_float, default="250.0k",
        help="Set offset [default=%(default)r]")
    parser.add_argument(
        "--samp-rate", dest="samp_rate", type=eng_float, default="2.0M",
        help="Set samp_rate [default=%(default)r]")
    parser.add_argument(
        "--sps", dest="sps", type=intx, default=16,
        help="Set sps [default=%(default)r]")
    parser.add_argument(
        "--uav-id", dest="uav_id", type=int, default=None,
        help="TDM slot index for this UAV (0-indexed). "
             "If omitted, read from config/client.yaml.")
    parser.add_argument(
        "--num-uavs", dest="num_uavs", type=int, default=None,
        help="Total number of UAVs (TDM slots). "
             "If omitted, derived from config/scenario.csv.")
    parser.add_argument(
        "--slot-duration", dest="slot_duration", type=float, default=None,
        help="TDM slot duration in seconds [default: 0.5 or from client.yaml]")
    parser.add_argument(
        "--guard-interval", dest="guard_interval", type=float, default=None,
        help="TDM guard interval in seconds [default: 0.05 or from client.yaml]")
    return parser


def _resolve_tdm_options(options):
    """Fill in TDM parameters from client.yaml / scenario.csv when not
    provided on the command line."""
    import os, yaml
    cfg_dir = '/root/miSim/aerpaw/config'
    env_cfg = os.environ.get('AERPAW_CLIENT_CONFIG', '')
    if env_cfg:
        yaml_path = env_cfg if os.path.isabs(env_cfg) else os.path.join('/root/miSim/aerpaw', env_cfg)
    else:
        yaml_path = os.path.join(cfg_dir, 'client.yaml')

    cfg = {}
    if os.path.isfile(yaml_path):
        with open(yaml_path, 'r') as f:
            cfg = yaml.safe_load(f) or {}

    tdm_cfg = cfg.get('tdm', {})

    if options.uav_id is None:
        options.uav_id = int(cfg.get('uav_id', 0))
    if options.slot_duration is None:
        options.slot_duration = float(tdm_cfg.get('slot_duration', 0.5))
    if options.guard_interval is None:
        options.guard_interval = float(tdm_cfg.get('guard_interval', 0.05))
    if options.num_uavs is None:
        try:
            options.num_uavs = derive_num_uavs_from_csv(
                '/root/miSim/aerpaw/config/scenario.csv')
        except Exception as e:
            print(f"[TDM] Warning: could not derive num_uavs from scenario.csv: {e}")
            print("[TDM] Defaulting to num_uavs=1 (TDM effectively disabled)")
            options.num_uavs = 1

    return options


def main(top_block_cls=CSwSNRRX, options=None):
    if options is None:
        options = argument_parser().parse_args()

    options = _resolve_tdm_options(options)

    print(f"[TDM-RX] Config: uav_id={options.uav_id}, "
          f"num_uavs={options.num_uavs}, slot={options.slot_duration}s, "
          f"guard={options.guard_interval}s")
    print(f"[TDM-RX] Receiver runs continuously; measurements from all "
          f"TX slots interleave into output files.")

    tb = top_block_cls(
        args=options.args, freq=options.freq, gainrx=options.gainrx,
        noise=options.noise, offset=options.offset,
        samp_rate=options.samp_rate, sps=options.sps,
        uav_id=options.uav_id, num_uavs=options.num_uavs,
        slot_duration=options.slot_duration,
        guard_interval=options.guard_interval)

    def sig_handler(sig=None, frame=None):
        tb.stop()
        tb.wait()
        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    tb.start()

    try:
        input('Press Enter to quit: ')
    except EOFError:
        pass
    tb.stop()
    tb.wait()


if __name__ == '__main__':
    main()
