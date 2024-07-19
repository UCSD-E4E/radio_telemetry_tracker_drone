'''RCT System States
'''
from enum import IntEnum, Enum


class GpsStates(IntEnum):
    """GPS Operational States

    """
    GET_TTY = 0
    GET_MSG = 1
    WAIT_RECYCLE = 2
    RDY = 3
    FAIL = 4

class SdrInitStates(IntEnum):
    """SDR Initialization States
    """
    FIND_DEVICES = 0
    WAIT_RECYCLE = 1
    USRP_PROBE = 2
    RDY = 3
    FAIL = 4

class OutputDirStates(IntEnum):
    """Output Directory Initialization States
    """
    GET_OUTPUT_DIR = 0
    CHECK_OUTPUT_DIR = 1
    CHECK_SPACE = 2
    WAIT_RECYCLE = 3
    RDY = 4
    FAIL = 5

class RctStates(Enum):
    """RCT Application States
    """
    INIT		=	0
    WAIT_INIT	=	1
    WAIT_START	=	2
    START		=	3
    WAIT_END	=	4
    FINISH		=	5
    FAIL		=	6
