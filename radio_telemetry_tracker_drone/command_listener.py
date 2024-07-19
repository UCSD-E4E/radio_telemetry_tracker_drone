'''Command Listener
'''
import datetime
import logging
import time
from enum import IntEnum
from io import TextIOWrapper
from pathlib import Path
from typing import Optional

from RCTComms.comms import (EVENTS, mavComms, rctACKCommand,
                            rctBinaryPacketFactory, rctFrequenciesPacket,
                            rctGETFCommand, rctGETOPTCommand,
                            rctHeartBeatPacket, rctOptionsPacket,
                            rctSETFCommand, rctSETOPTCommand, rctSTARTCommand,
                            rctSTOPCommand)
from RCTComms.options import Options
from RCTComms.transport import RCTAbstractTransport

from radio_telemetry_tracker_drone.options import RCTOpts
from radio_telemetry_tracker_drone.ui_board import UIBoard
from radio_telemetry_tracker_drone.utils import InstrumentedThread


class CommsStates(IntEnum):
    """Comms Listener States
    """
    DISCONNECTED = 0
    CONNECTED = 1

class CommandListener:
    """Command Listener
    """
    def __init__(self,
            ui_board: UIBoard,
            transport: RCTAbstractTransport,
            *,
            config_path: Path = Path('/usr/local/etc/rct_config')):
        self.__log = logging.getLogger('Command Listener')
        self.transport = transport
        self.port = mavComms(self.transport)

        self.ping_file: Optional[TextIOWrapper] = None
        self.num = None
        self.new_run = False
        self._run = True

        self.state = CommsStates.DISCONNECTED
        self.sender = InstrumentedThread(target=self._sender,
                                         name='CommandListener_sender',
                                         daemon=True)
        self.reconnect = InstrumentedThread(target=self._reconnect_comms,
                                            name='CommandListener_reconnect')

        self.start_flag = False
        self.uib = ui_board
        self.uib.switch = 0
        self.factory = rctBinaryPacketFactory()

        self.options = RCTOpts.get_instance(path=config_path)

        self.setup()

        self.port.start()
        self.sender.start()

    def _sender(self):
        prev_time = datetime.datetime.now()

        self.port.port_open_event.wait()

        heartbeat_period = self.options[Options.SYS_HEARTBEAT_PERIOD]
        while self.port.isOpen():
            try:
                now = datetime.datetime.now()
                if (now - prev_time).total_seconds() > heartbeat_period:
                    hearbeat_packet = rctHeartBeatPacket(self.uib.system_state,
                            self.uib.sdr_state, self.uib.sensor_state,
                            self.uib.storage_state, self.uib.switch, now)

                    msg = hearbeat_packet
                    self.port.sendToGCS(msg)
                    prev_time = now
            except BrokenPipeError:
                self.__log.exception('Broken Pipe!')
                self._run = False
                self.state = CommsStates.DISCONNECTED
                self.start_flag = False
                self.uib.switch = 0
                self.port.stop()
                while self.transport.isOpen():
                    time.sleep(1)
                    self.__log.error('Pipe is still open!')
                self.__log.error('Socket closed')
                self.reconnect.start()
            except Exception: # pylint: disable=broad-except
                self.__log.exception('Early Fail!')

                time.sleep(1)
                continue

    def _reconnect_comms(self):
        self.sender.join()
        self.sender = InstrumentedThread(target=self._sender,
                                         name='CommandListener_sender',
                                         daemon=True)
        self.reconnect = InstrumentedThread(target=self._reconnect_comms,
                                            name='CommandListener_sender')

        self.port.start()
        while not self.transport.isOpen():
            time.sleep(1)
        self.state = CommsStates.CONNECTED
        self.__log.info('Starting sender')
        self._run = True
        self.sender.start()

    def setup(self):
        """Sets up callbacks
        """
        self.port.registerCallback(
            EVENTS.COMMAND_GETF, self._got_get_f_cmd)
        self.port.registerCallback(
            EVENTS.COMMAND_SETF, self._got_set_f_cmd)
        self.port.registerCallback(
            EVENTS.COMMAND_GETOPT, self._got_get_opts_cmd)
        self.port.registerCallback(
            EVENTS.COMMAND_SETOPT, self._got_set_opts_cmd)
        self.port.registerCallback(
            EVENTS.COMMAND_START, self._got_start_cmd)
        self.port.registerCallback(
            EVENTS.COMMAND_STOP, self._got_stop_cmd)
        # self.port.registerCallback(
        #     EVENTS.COMMAND_UPGRADE, self._upgradeCmd)
        self.uib.register_callback(
            EVENTS.DATA_PING, self.port.sendPing)
        self.uib.register_callback(
            EVENTS.DATA_VEHICLE, self.port.sendVehicle)

    def _send_ack(self, packet_id: int, result: bool):
        now = datetime.datetime.now()
        packet = rctACKCommand(packet_id, result, now)
        self.port.sendToGCS(packet)

    def _got_get_f_cmd(self, packet: rctGETFCommand, addr):
        freqs = self.options[Options.TGT_FREQUENCIES]
        packet = rctFrequenciesPacket(freqs)
        msg = packet
        self._send_ack(0x02, True)
        self.port.sendToGCS(msg)

    def _got_set_f_cmd(self, packet: rctSETFCommand, addr):
        if packet.frequencies is None:
            return
        freqs = packet.frequencies
        self.options[Options.TGT_FREQUENCIES] = freqs
        self.options.write_options()
        packet = rctFrequenciesPacket(freqs)

        msg = packet
        self._send_ack(0x03, True)
        self.port.sendToGCS(msg)

    def _got_get_opts_cmd(self, packet: rctGETOPTCommand, addr):
        opts = self.options.get_all_options()

        self.__log.info('Get Comms Opts: {}', opts)

        msg = rctOptionsPacket(packet.scope, opts)
        self._send_ack(0x04, True)
        self.port.sendToGCS(msg)

    def _got_set_opts_cmd(self, packet: rctSETOPTCommand, addr):
        opts = packet.options
        self.options.set_options(opts)
        self.options.write_options()
        options = self.options.get_all_options()
        msg = rctOptionsPacket(packet.scope, options)
        self._send_ack(0x05, True)
        self.port.sendToGCS(msg)
        self._send_ack(0x05, True)

    def _got_start_cmd(self, packet: rctSTARTCommand, addr):
        if self.uib.ready():
            self.start_flag = True
            self.uib.switch = 1
            self._send_ack(0x07, True)
            self.__log.info('Set start flag')
            self._send_ack(packet._pid, True)
        else:
            if not self.uib.storage_state == 4:
                self.__log.error('Storage not ready! {}', self.uib.storage_state)
            if not self.uib.sensor_state == 3:
                self.__log.error('GPS not ready! {}', self.uib.sensor_state)
            if not self.uib.sdr_state == 3:
                self.__log.error('SDR not ready! {}', self.uib.sdr_state)
            self._send_ack(packet._pid, True)

    def _got_stop_cmd(self, packet: rctSTOPCommand, addr):
        self.start_flag = False
        self.uib.switch = 0
        self._send_ack(0x09, True)
        try:
            self.ping_file.close()
        except Exception: # pylint: disable=broad-except
            self.__log.exception('Failed to close ping file!')
        self.ping_file = None
        self._send_ack(packet._pid, True)

    def set_run(self, runDir: Path, runNum):
        self.new_run = True
        if self.ping_file is not None:
            self.ping_file.close()
            self.__log.info('Closing file')
        path = runDir.joinpath(f'LOCALIZE_{runNum:06d}')

        if path.is_file():
            self.ping_file = open(path)
            self.__log.info('Set and open file to {}', path.as_posix())
        else:
            raise Exception('File non existent!')
