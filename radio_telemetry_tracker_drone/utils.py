'''Radio Telemetry Tracker Drone Utils
'''
import platform
from pathlib import Path

from platformdirs import PlatformDirs
import os

platform_directories = PlatformDirs('rct_drone')

def get_log_dir() -> Path:
    """Retrieves the log directory
    """
    if platform.system() == 'Linux' and os.getuid() == 0:
        return Path('/var/log/')
    return platform_directories.user_log_path
