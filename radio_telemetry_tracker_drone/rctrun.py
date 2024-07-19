'''RCT Run
'''
import logging
import os
import shutil
import subprocess
import sys
import threading
import time
from enum import Enum, auto
from pathlib import Path
from threading import Condition, Event
from typing import Callable, Dict, List, Optional

from RCTComms.comms import EVENTS, rctBinaryPacket
from RCTComms.transport import RCTTransportFactory

from radio_telemetry_tracker_drone.command_listener import CommandListener
from radio_telemetry_tracker_drone.networking import (NetworkMonitor,
                                                      NetworkProfileNotFound)
from radio_telemetry_tracker_drone.options import Options, RCTOpts
from radio_telemetry_tracker_drone.states import RctStates, SdrInitStates, OutputDirStates
from radio_telemetry_tracker_drone.ui_board import UIBoard
from radio_telemetry_tracker_drone.utils import (InstrumentedThread,
                                                 platform_directories)


class AutostartNotSupportedException(Exception):
    """Autostart Not Supported
    """


class RCTRun:
    """RCT Run Application
    """
    class Event(Enum):
        """Callback events

        """
        START_RUN = auto()
        STOP_RUN = auto()

    class Flags(Enum):
        """Events
        """
        SDR_READY = auto()
        GPS_READY = auto()
        STORAGE_READY = auto()
        INIT_COMPLETE = auto()

    def __init__(self,
                 test: bool = False,
                 *,
                 config_path: Path = None,
                 allow_nonmount: bool = False,
                 service: bool = False) -> None:
        self.do_run = True

        self.__log = logging.getLogger('RCTRun')
        if not config_path:
            config_path = platform_directories.site_config_path.joinpath(
                'rct_config')
        self.__log.info('Set log path to {}', config_path.as_posix())

        self.__options = RCTOpts.get_instance(config_path)

        if service and not self.__options[Options.SYS_AUTOSTART]:
            raise AutostartNotSupportedException

        self.__config_path = config_path
        self.__allow_nonmount = allow_nonmount

        self.__cb: Dict[RCTRun.Event, List[Callable[[RCTRun.Event], None]]] = \
            {evt: [] for evt in RCTRun.Event}

        self.events: Dict[RCTRun.Event, Condition] = {
            evt: Condition() for evt in RCTRun.Event}

        self.flags = {evt: Event() for evt in RCTRun.Flags}

        self.__output_path: Optional[Path] = None

        self.__log.debug('Started Payload')
        baud = int(self.__options[Options.GPS_BAUD])
        serial_port = self.__options[Options.GPS_DEVICE]
        test_gps = self.__options[Options.GPS_MODE]
        self.gcs_spec = self.__options[Options.GCS_SPEC]
        self.uib_singleton = UIBoard(serial_port, baud, test_gps)
        self.__log.debug('RCTRun init: created UIB')
        self.cmd_listener = None
        self.test = test

        self.serial_port = serial_port
        self.ping_finder = None
        self.delete_comms_thread = None

        self.heatbeat_thread_stop = threading.Event()

        self.heartbeat_thread = InstrumentedThread(target=self.uib_heartbeat,
                                                 name='UIB Heartbeat',
                                                 daemon=True)
        self.init_sdr_thread: Optional[InstrumentedThread] = None
        self.init_output_thread: Optional[InstrumentedThread] = None
        self.init_gps_thread: Optional[InstrumentedThread] = None
        try:
            self.network_monitor = NetworkMonitor(
                network_profile=self.__options[Options.SYS_NETWORK],
                monitor_interval=int(self.__options[Options.SYS_WIFI_MONITOR_INTERVAL])
            )
        except KeyError:
            self.network_monitor = None
        except NetworkProfileNotFound:
            self.network_monitor = None

    def uib_heartbeat(self):
        """UI Board Heartbeat generator
        """
        heartbeat_period = self.__options[Options.SYS_HEARTBEAT_PERIOD]
        self.__log.info('Sending heartbeats every {} seconds', heartbeat_period)
        while not self.heatbeat_thread_stop.is_set():
            if self.heatbeat_thread_stop.wait(timeout=heartbeat_period):
                break
            self.uib_singleton.send_heartbeat()

    def start(self):
        """Starts all RCTRun Threads
        """
        self.check_for_external_update()
        self.init_comms()
        self.init_threads()
        self.heartbeat_thread.start()
        if self.network_monitor:
            self.network_monitor.start()

    def check_for_external_update(self):
        """Checks for external updates.

        This must only be called if we are searching into an external drive.

        We use output_dir/install - if there exist wheels, we install all of
        the wheels.  If there is an rct_config file, we install it.
        output_dir/install will then be deleted, and we restart.
        """
        output_dir = Path(self.__options[Options.SYS_OUTPUT_DIR])
        if not output_dir.is_dir():
            self.__log.warning('Invalid output dir')
            return

        if not output_dir.is_mount():
            self.__log.warning('Output dir is not a mount, not checking for '
                               'updates')
            return
        install_path = output_dir.joinpath('install')
        if not install_path.exists():
            self.__log.info('install not found, no updates')
            return
        if not install_path.is_dir():
            self.__log.warning('install is not a directory, failing update')
            return

        wheels = list(install_path.glob('*.whl'))
        config = list(install_path.glob('rct_config'))

        install_cmd = [sys.executable, '-m', 'pip', 'install', '--force-reinstall']
        install_cmd.extend(wheels)

        subprocess.run(install_cmd, check=True)
        shutil.copy(config, self.__config_path)

        shutil.rmtree(install_path)
        os.execv(sys.argv[0], sys.argv)

    def init_comms(self):
        """Sets up the connection configuration
        """
        if self.cmd_listener is None:
            self.__log.debug('CommandListener initialized')
            transport = RCTTransportFactory.create_transport(self.gcs_spec)
            self.cmd_listener = CommandListener(
                ui_board=self.uib_singleton,
                transport=transport,
                config_path=self.__config_path)
            self.__log.info('CommandListener connected')
            self.cmd_listener.port.register_callback(EVENTS.COMMAND_START, self.start_cmd_received)
            self.cmd_listener.port.register_callback(EVENTS.COMMAND_STOP, self.stop_run_cb)

    def start_cmd_received(self, packet: rctBinaryPacket, addr: str):
        """Start Command Received Callback

        Args:
            packet (rctBinaryPacket): Start Command packet
            addr (str): Source Address
        """
        self.run()

    def run(self):
        """Main application logic
        """
        self.execute_cb(self.Event.START_RUN)
        try:
            if self.cmd_listener.start_flag:
                self.uib_singleton.system_state = RctStates.START.value

                if not self.test:
                    run_dirs = list(self.__output_path.glob('RUN_*'))
                    if len(run_dirs) > 0:
                        run_num = int(sorted([run_dir.name for run_dir in run_dirs])[-1][4:]) + 1
                        self.__log.debug('Run Num: {}', run_num)
                    else:
                        run_num = 1
                run_dir = self.__output_path.joinpath(f'RUN_{run_num:06d}')
                if not self.test:
                    run_dir.mkdir(parents=True, exist_ok=True)
                else:
                    try:
                        run_dir.mkdir(parents=True, exist_ok=True)
                    except Exception: # pylint: disable=broad-except
                        pass

                localize_file = run_dir.joinpath(f'LOCALIZE_{run_num:06d}')
                localize_file.touch()

                self.cmd_listener.set_run(run_dir, run_num)
                time.sleep(1)

                #TODO add dynamic sdr_record

                opts = self.cmd_listener.options
                self.uib_singleton.system_state = RctStates.WAIT_END.value
                self.__log.debug('Creating pingFinder')
                self.ping_finder = PingFinder()
                self.__log.debug('Configuring pingFinder')
                self.ping_finder.gain = opts.get_option(Options.SDR_GAIN)
                self.ping_finder.sampling_rate = opts.get_option(Options.SDR_SAMPLING_FREQ)
                self.ping_finder.center_frequency = opts.get_option(Options.SDR_CENTER_FREQ)
                self.ping_finder.run_num = run_num
                self.ping_finder.enable_test_data = False
                self.ping_finder.output_dir = run_dir.as_posix()
                self.ping_finder.ping_width_ms = int(opts.get_option(Options.DSP_PING_WIDTH))
                self.ping_finder.ping_min_snr = opts.get_option(Options.DSP_PING_SNR)
                self.ping_finder.ping_max_len_mult = opts.get_option(Options.DSP_PING_MAX)
                self.ping_finder.ping_min_len_mult = opts.get_option(Options.DSP_PING_MIN)
                self.ping_finder.target_frequencies = opts.get_option(Options.TGT_FREQUENCIES)

                self.ping_finder.register_callback(self.UIB_Singleton.send_ping)

                self.ping_finder.start()
                self.__log.debug('Started pingFinder')
        except Exception as exc:
            self.__log.exception(exc)
            raise exc

    def execute_cb(self, event: Event) -> None:
        """Executes an event's callbacks

        Args:
            event (Event): Event to execute
        """
        with self.events[event]:
            self.events[event].notify_all()
        for cb_ in self.__cb[event]:
            cb_(event)

    def init_threads(self):
        """System Initialization thread execution
        """
        self.flags[self.Flags.INIT_COMPLETE].clear()
        self.init_sdr_thread = InstrumentedThread(target=self._init_sdr,
                                                kwargs={'test':self.test},
                                                name='SDR Init')
        self.init_sdr_thread.start()
        self.__log.debug('RCTRun init: started SDR thread')
        self.init_output_thread = InstrumentedThread(target=self._init_output,
                                                   kwargs={'test':self.test},
                                                   name='Output Init')
        self.init_output_thread.start()
        self.__log.debug('RCTRun init: started output thread')
        self.init_gps_thread = InstrumentedThread(target=self._init_gps,
                                                kwargs={'test':self.test},
                                                name='GPS Init')
        self.init_gps_thread.start()
        self.__log.debug('RCTRun init: started GPS thread')

        self.do_run = True

        self.init_sdr_thread.join()
        self.__log.debug('SDR init thread joined')
        self.init_output_thread.join()
        self.__log.debug('Output init thread joined')
        self.init_gps_thread.join()
        self.__log.debug('GPS init thread joined')
        self.uib_singleton.system_state = RctStates.WAIT_START.value

        self.flags[self.Flags.INIT_COMPLETE].set()

    def _init_sdr(self, test = False):
        log = logging.getLogger('SDR Init')
        self.flags[self.Flags.SDR_READY].clear()
        initialized = False
        devices_found = False
        usrp_device_initialized = False
        try:
            while not initialized:

                if not devices_found:
                    if not test:
                        self.uib_singleton.sdr_state = SdrInitStates.FIND_DEVICES
                        uhd_find_dev_retval = subprocess.run([
                            '/usr/bin/uhd_find_devices',
                            '--args=\"type=b200\"'],
                            capture_output=True,
                            encoding='utf-8',
                            check=False)
                        if uhd_find_dev_retval.returncode == 0:
                            devices_found = True
                            self.uib_singleton.sdr_state = SdrInitStates.USRP_PROBE
                            log.info('Devices Found')
                        else:
                            log.error('Devices not found: {}', uhd_find_dev_retval.stdout)
                            time.sleep(2)
                            self.uib_singleton.sdr_state = SdrInitStates.WAIT_RECYCLE
                elif not usrp_device_initialized:
                    if not test:
                        uhd_usrp_probe_retval = subprocess.run([
                            '/usr/bin/uhd_usrp_probe',
                            '--args=\"type=b200\"',
                            '--init-only'],
                            capture_output=True,
                            encoding='utf-8',
                            env={
                                'HOME': '/tmp'
                            },
                            check=False)
                        if uhd_usrp_probe_retval.returncode == 0:
                            log.info('USRP Initialized')
                            usrp_device_initialized = True
                            initialized = True
                            self.uib_singleton.sdr_state = SdrInitStates.RDY
                        else:
                            log.error('USRP Initialized: {}', uhd_usrp_probe_retval.stderr)
                            self.uib_singleton.sdr_state = SdrInitStates.FAIL
                            devices_found = False
                            usrp_device_initialized = False
        except Exception as exc: # pylint: disable=broad-except
            log.exception(exc)
            raise exc
        self.flags[self.Flags.SDR_READY].set()

    def stop_run_cb(self, packet, addr): # pylint: disable=unused-argument
        """Callback for the stop recording command
        """
        self.__log.info('Stop Run Callback')
        self.uib_singleton.system_state = RctStates.WAIT_END.value
        if self.ping_finder is not None:
            self.ping_finder.stop()
        self.execute_cb(self.Event.STOP_RUN)
        self.init_threads()

    def _init_output(self, test: bool):
        """Initialize Output Directories

        Args:
            test (bool): Test Mode
        """
        log = logging.getLogger('InitOutput')
        self.flags[self.Flags.STORAGE_READY].clear()
        try:
            output_dir_initialized = False
            dir_name_found = False
            output_dir_found = False
            enough_space = False

            self.uib_singleton.storage_state = OutputDirStates.GET_OUTPUT_DIR

            while not output_dir_initialized:
                if not dir_name_found:
                    output_dir = self.__options[Options.SYS_OUTPUT_DIR]
                    if output_dir is not None:
                        self.__output_path = Path(output_dir)
                        dir_name_found = True
                        log.info('rctConfig Directory: {}', output_dir)
                        self.uib_singleton.storage_state = OutputDirStates.CHECK_OUTPUT_DIR
                elif not output_dir_found:
                    if test:
                        self.__output_path = Path('..', 'test_output')
                    valid_dir = True
                    if not self.__output_path.is_dir():
                        valid_dir = False
                        log.error('Output path is not a directory')
                    if not self.__allow_nonmount and not self.__output_path.is_mount():
                        valid_dir = False
                        log.error('Output path is not a mount, nonmount not permitted')
                    if valid_dir:
                        output_dir_found = True
                        self.uib_singleton.storage_state = OutputDirStates.CHECK_SPACE
                    else:
                        time.sleep(10)
                        self.uib_singleton.storage_state = OutputDirStates.WAIT_RECYCLE
                elif not enough_space:
                    cmd = ['df', '-B1', self.__output_path.as_posix()]
                    with subprocess.Popen(cmd, stdout=subprocess.PIPE) as df:
                        output = df.communicate()[0].decode('utf-8')
                        _, _, _, available, _, _ = output.split('\n')[1].split()
                        if int(available) > 20 * 60 * 1500000 * 4:
                            enough_space = True
                            output_dir_initialized = True
                            self.uib_singleton.storage_state = OutputDirStates.RDY
                        else:
                            dir_name_found = False
                            output_dir_found = False
                            enough_space = False
                            self.uib_singleton.storage_state = OutputDirStates.FAIL
                            log.error('Not Enough Storage Space!')
        except Exception as exc:
            log.exception(exc)
            raise exc
        self.flags[self.Flags.STORAGE_READY].set()

    def _init_gps(self, test = False):
        # pylint: disable=unused-argument
        self.flags[self.Flags.GPS_READY].clear()
        self.uib_singleton.gps_ready.wait()
        self.flags[self.Flags.GPS_READY].set()
