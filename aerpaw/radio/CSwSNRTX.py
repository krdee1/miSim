#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: CSwSNRTX
# Author: Ozgur Ozdemir
# Description: Channel Sounder Transmitter with offset freq
# GNU Radio version: v3.8.5.0-6-g57bd109d

from gnuradio import analog
from gnuradio import blocks
from gnuradio import digital
from gnuradio import filter
from gnuradio.filter import firdes
from gnuradio import gr
import sys
import signal
import threading
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
from gnuradio import uhd
import time


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


class TdmScheduler(threading.Thread):
    """Daemon thread that mutes/unmutes a GNU Radio mute_cc block on a
    wall-clock TDM schedule.

    Slot assignment:  current_slot = floor(utc_time / slot_duration) % num_uavs
    Guard interval:   the first *guard_interval* seconds of every slot are
                      always muted to avoid TX overlap due to clock skew.
    """

    def __init__(self, mute_block, uav_id, num_uavs,
                 slot_duration=0.5, guard_interval=0.05):
        super().__init__(daemon=True)
        self.mute_block = mute_block
        self.uav_id = uav_id
        self.num_uavs = num_uavs
        self.slot_duration = slot_duration
        self.guard_interval = guard_interval
        self._stop_event = threading.Event()

    def run(self):
        print(f"[TDM] Scheduler started: uav_id={self.uav_id}, "
              f"num_uavs={self.num_uavs}, slot={self.slot_duration}s, "
              f"guard={self.guard_interval}s")
        while not self._stop_event.is_set():
            now = time.time()
            slot_time = now % (self.slot_duration * self.num_uavs)
            current_slot = int(slot_time / self.slot_duration)
            time_into_slot = slot_time - current_slot * self.slot_duration

            my_slot = (current_slot == self.uav_id)
            in_guard = (time_into_slot < self.guard_interval)

            should_mute = (not my_slot) or in_guard
            self.mute_block.set_mute(should_mute)

            # Sleep ~1 ms for responsive timing without busy-waiting
            self._stop_event.wait(0.001)

    def stop(self):
        self._stop_event.set()


class CSwSNRTX(gr.top_block):

    def __init__(self, args='', freq=3.32e9, gaintx=76, offset=250e3, samp_rate=2e6, sps=16,
                 uav_id=0, num_uavs=1, slot_duration=0.5, guard_interval=0.05):
        gr.top_block.__init__(self, "CSwSNRTX")

        ##################################################
        # Parameters
        ##################################################
        self.args = args
        self.freq = freq
        self.gaintx = gaintx
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
        self.alpha = alpha = 0.99

        ##################################################
        # Blocks
        ##################################################
        self.uhd_usrp_sink_0 = uhd.usrp_sink(
            ",".join(("", args)),
            uhd.stream_args(
                cpu_format="fc32",
                args='',
                channels=list(range(0,1)),
            ),
            '',
        )
        self.uhd_usrp_sink_0.set_center_freq(freq, 0)
        self.uhd_usrp_sink_0.set_gain(gaintx, 0)
        self.uhd_usrp_sink_0.set_antenna('TX/RX', 0)
        self.uhd_usrp_sink_0.set_samp_rate(samp_rate)
        self.uhd_usrp_sink_0.set_time_unknown_pps(uhd.time_spec())
        self.root_raised_cosine_filter_0 = filter.fir_filter_ccf(
            1,
            firdes.root_raised_cosine(
                sps,
                samp_rate,
                samp_rate/sps,
                alpha,
                10*sps+1))
        self.interp_fir_filter_xxx_0 = filter.interp_fir_filter_ccc(sps, [1]+[0]*(sps-1))
        self.interp_fir_filter_xxx_0.declare_sample_delay(0)
        self.digital_glfsr_source_x_0 = digital.glfsr_source_b(12, True, 0, 1)
        self.digital_chunks_to_symbols_xx_0 = digital.chunks_to_symbols_bc((-1,1), 1)
        self.blocks_multiply_xx_0 = blocks.multiply_vcc(1)
        self.blocks_multiply_const_vxx_0 = blocks.multiply_const_cc(1/1.58)
        self.blocks_mute_0 = blocks.mute_cc(True)  # TDM: start muted
        self.analog_sig_source_x_0 = analog.sig_source_c(samp_rate, analog.GR_COS_WAVE, offset, 1, 0, 0)


        ##################################################
        # Connections
        ##################################################
        self.connect((self.analog_sig_source_x_0, 0), (self.blocks_multiply_xx_0, 1))
        self.connect((self.blocks_multiply_const_vxx_0, 0), (self.blocks_mute_0, 0))
        self.connect((self.blocks_mute_0, 0), (self.uhd_usrp_sink_0, 0))
        self.connect((self.blocks_multiply_xx_0, 0), (self.blocks_multiply_const_vxx_0, 0))
        self.connect((self.digital_chunks_to_symbols_xx_0, 0), (self.interp_fir_filter_xxx_0, 0))
        self.connect((self.digital_glfsr_source_x_0, 0), (self.digital_chunks_to_symbols_xx_0, 0))
        self.connect((self.interp_fir_filter_xxx_0, 0), (self.root_raised_cosine_filter_0, 0))
        self.connect((self.root_raised_cosine_filter_0, 0), (self.blocks_multiply_xx_0, 0))


    def get_args(self):
        return self.args

    def set_args(self, args):
        self.args = args

    def get_freq(self):
        return self.freq

    def set_freq(self, freq):
        self.freq = freq
        self.uhd_usrp_sink_0.set_center_freq(self.freq, 0)

    def get_gaintx(self):
        return self.gaintx

    def set_gaintx(self, gaintx):
        self.gaintx = gaintx
        self.uhd_usrp_sink_0.set_gain(self.gaintx, 0)

    def get_offset(self):
        return self.offset

    def set_offset(self, offset):
        self.offset = offset
        self.analog_sig_source_x_0.set_frequency(self.offset)

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.analog_sig_source_x_0.set_sampling_freq(self.samp_rate)
        self.root_raised_cosine_filter_0.set_taps(firdes.root_raised_cosine(self.sps, self.samp_rate, self.samp_rate/self.sps, self.alpha, 10*self.sps+1))
        self.uhd_usrp_sink_0.set_samp_rate(self.samp_rate)

    def get_sps(self):
        return self.sps

    def set_sps(self, sps):
        self.sps = sps
        self.interp_fir_filter_xxx_0.set_taps([1]+[0]*(self.sps-1))
        self.root_raised_cosine_filter_0.set_taps(firdes.root_raised_cosine(self.sps, self.samp_rate, self.samp_rate/self.sps, self.alpha, 10*self.sps+1))

    def get_alpha(self):
        return self.alpha

    def set_alpha(self, alpha):
        self.alpha = alpha
        self.root_raised_cosine_filter_0.set_taps(firdes.root_raised_cosine(self.sps, self.samp_rate, self.samp_rate/self.sps, self.alpha, 10*self.sps+1))


def argument_parser():
    description = 'Channel Sounder Transmitter with offset freq'
    parser = ArgumentParser(description=description)
    parser.add_argument(
        "--args", dest="args", type=str, default='',
        help="Set args [default=%(default)r]")
    parser.add_argument(
        "--freq", dest="freq", type=eng_float, default="3.32G",
        help="Set freq [default=%(default)r]")
    parser.add_argument(
        "--gaintx", dest="gaintx", type=eng_float, default="76.0",
        help="Set gaintx [default=%(default)r]")
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


def main(top_block_cls=CSwSNRTX, options=None):
    if options is None:
        options = argument_parser().parse_args()

    options = _resolve_tdm_options(options)

    tb = top_block_cls(
        args=options.args, freq=options.freq, gaintx=options.gaintx,
        offset=options.offset, samp_rate=options.samp_rate, sps=options.sps,
        uav_id=options.uav_id, num_uavs=options.num_uavs,
        slot_duration=options.slot_duration,
        guard_interval=options.guard_interval)

    tdm_sched = TdmScheduler(
        tb.blocks_mute_0,
        uav_id=options.uav_id,
        num_uavs=options.num_uavs,
        slot_duration=options.slot_duration,
        guard_interval=options.guard_interval)

    def sig_handler(sig=None, frame=None):
        tdm_sched.stop()
        tb.stop()
        tb.wait()
        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    tb.start()
    tdm_sched.start()

    try:
        input('Press Enter to quit: ')
    except EOFError:
        pass
    tdm_sched.stop()
    tb.stop()
    tb.wait()


if __name__ == '__main__':
    main()
(END)


