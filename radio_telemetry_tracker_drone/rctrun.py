'''RCT Run
'''
import logging
from enum import Enum, auto
from pathlib import Path
from threading import Condition, Event
from typing import Callable, Dict, List, Optional

from radio_telemetry_tracker_drone.options import Options, RCTOpts
from radio_telemetry_tracker_drone.utils import platform_directories


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
