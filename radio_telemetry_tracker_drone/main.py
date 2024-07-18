'''Main Entry Point
'''
import argparse
import logging
import logging.handlers
import time
from pathlib import Path

from radio_telemetry_tracker_drone.rctrun import RCTRun
from radio_telemetry_tracker_drone.utils import get_log_dir


def configure_loggers():
    """Configures logging
    """
    root_logger = logging.getLogger()
    root_logger.setLevel(logging.DEBUG)

    log_dest = get_log_dir()
    log_file_handler = logging.handlers.RotatingFileHandler(log_dest,
                                                            maxBytes=5*1024*1024,
                                                            backupCount=5)
    log_file_handler.setLevel(logging.DEBUG)

    msg_fmt = '%(asctime)s.%(msecs)03d - %(name)s - %(levelname)s - %(message)s'
    root_formatter = logging.Formatter(msg_fmt, datefmt='%Y-%m-%d %H:%M:%S')
    log_file_handler.setFormatter(root_formatter)
    root_logger.addHandler(log_file_handler)

    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.WARN)

    error_formatter = logging.Formatter(msg_fmt, datefmt='%Y-%m-%d %H:%M:%S')
    console_handler.setFormatter(error_formatter)
    root_logger.addHandler(console_handler)
    logging.Formatter.converter = time.gmtime

def main():
    """Main Entry Point
    """
    configure_loggers()

    parser = argparse.ArgumentParser()
    parser.add_argument('--config', '-c', type=Path)
    parser.add_argument('--no_mount', action='store_true')
    parser.add_argument('--service', action='store_true')
    args = parser.parse_args()

    kwargs = {
    }
    if args.config:
        kwargs['config_path'] = args.config
    kwargs['allow_nonmount'] = args.no_mount
    kwargs['service'] = args.service
    try:
        app = RCTRun(**kwargs)
        app.start()
    except Exception as exc:
        logging.exception('Unhandled fatal exception!')
        raise exc
    while True:
        # Wait for the system to end
        pass

if __name__ == '__main__':
    main()
