'''Test Configuration/Fixtures
'''
import datetime as dt
import json
from pathlib import Path
from tempfile import NamedTemporaryFile, TemporaryDirectory
from threading import Event, Thread
from typing import Optional, Tuple

import numpy as np
import pytest
import utm
import yaml
from serial import Serial
from virtualserialports import VirtualSerialPorts


class FakeUIBoard:
    """Creates a fake UI Board for testing
    """

    # pylint: disable=too-many-instance-attributes
    def __init__(self, *,
            lat: float = 32.884041,
            lon: float = -117.235153,
            hdg: float = 45,
            step: float = 5,
            vcc: int = 5000,
            fix: int = 1,
            sat: int = 7):
        self.__ports = VirtualSerialPorts(num_ports=2, loopback=False)

        self.serial_pty = self.__ports.slave_names[0].as_posix()
        self.__port = self.__ports.slave_names[1]

        self.lat = lat
        self.lon = lon
        self.hdg = hdg
        hdg = np.deg2rad(hdg)
        rot_mat = np.array([[np.cos(hdg), -np.sin(hdg)],
                            [np.sin(hdg), np.cos(hdg)]])
        step_vec = np.array([0, step])
        self.step = np.matmul(rot_mat, step_vec)
        self.vcc = vcc
        self.fix = fix
        self.sat = sat
        self.__sendthread: Optional[Thread] = None
        self.__stop_event = Event()

    def start(self):
        """Starts the fake UI Board
        """
        self.__ports.run()
        self.__stop_event.clear()
        self.__sendthread = Thread(target=self.__send, name='MockUIBoardSend')
        self.__sendthread.start()

    def stop(self):
        """Stops the fake UI Board
        """
        self.__stop_event.set()
        self.__sendthread.join()
        self.__sendthread = None
        self.__ports.stop()

    def __send(self):
        seq = 0
        with Serial(port=self.__port.as_posix(), baudrate=9600) as port:
            while True:
                # do stuff
                now = dt.datetime.now()
                tme = now.strftime('%H%M%S.00')
                dte = now.strftime('%d%m%y')

                data = {
                    'seq': seq,
                    'lat': int(self.lat * 1e7),
                    'lon': int(self.lon * 1e7),
                    'hdg': self.hdg,
                    'tme': tme,
                    'run': False,
                    'fix': int(self.fix),
                    'sat': self.sat,
                    'dat': dte
                }
                output_str = json.dumps(data)

                port.write(output_str.encode('ascii'))

                east, north, zonenum, zone = utm.from_latlon(self.lat, self.lon)
                new_coord = np.array([east, north]) + self.step

                self.lat, self.lon = utm.to_latlon(
                    easting=new_coord[0],
                    northing=new_coord[1],
                    zone_number=zonenum,
                    zone_letter=zone)
                seq += 1
                try:
                    if self.__stop_event.wait(timeout=1):
                        break
                except TimeoutError:
                    continue

@pytest.fixture(name='test_env')
def create_test_env() -> Tuple[Path]:
    """Creates a test environment

    Returns:
        Tuple[Path]: Path object of test config

    Yields:
        Iterator[Tuple[Path]]: _description_
    """
    ui_board = FakeUIBoard()
    ui_board.start()
    with TemporaryDirectory() as tmp_output_dir:
        config = {
            'DSP_ping_max': 1.5,
            'DSP_ping_min': 0.5,
            'DSP_ping_snr': 0.1,
            'DSP_ping_width': 27,
            'GPS_baud': 9600,
            'GPS_device': ui_board.serial_pty,
            'GPS_mode': True,
            'SDR_center_freq': 173500000,
            'SDR_gain': 20.0,
            'SDR_sampling_freq': 1500000,
            'SYS_autostart': False,
            'SYS_output_dir': tmp_output_dir,
            'TGT_frequencies': [173964000],
            'GCS_spec': 'serial:/dev/ttyUSB0?baud=57600',
            'SYS_heartbeat_period': 5,
        }
        with NamedTemporaryFile(mode='w+') as handle:
            yaml.safe_dump(config, handle)
            yield (Path(handle.name),)
    ui_board.stop()