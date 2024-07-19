'''Radio Telemetry Tracker Drone Utils
'''
import logging
import os
import platform
from pathlib import Path
from threading import Thread

from platformdirs import PlatformDirs

platform_directories = PlatformDirs('rct_drone')

def get_log_dir() -> Path:
    """Retrieves the log directory
    """
    if platform.system() == 'Linux' and os.getuid() == 0:
        return Path('/var/log/')
    return platform_directories.user_log_path

class InstrumentedThread(Thread):
    """Extention of threading.Thread that captures any exception from the thread

    """
    def run(self) -> None:
        try:
            return super().run()
        except Exception as exc:
            logging.exception('Unhandled fatal exception in {}', self.name)
            raise exc
