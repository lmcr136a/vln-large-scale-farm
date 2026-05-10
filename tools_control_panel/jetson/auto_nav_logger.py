import logging
import os
from datetime import datetime

# auto_nav_log/ sits one level above tools_control_panel/
_PROJ_ROOT      = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
AUTO_NAV_LOG_DIR = os.path.join(_PROJ_ROOT, 'auto_nav_log')


def create_session_logger() -> tuple:
    """Create a timestamped log file. Returns (logger, log_path)."""
    os.makedirs(AUTO_NAV_LOG_DIR, exist_ok=True)
    ts   = datetime.now().strftime('%y%m%d-%H%M%S')
    path = os.path.join(AUTO_NAV_LOG_DIR, f'{ts}.log')

    logger = logging.getLogger(f'auto_nav_{ts}')
    logger.setLevel(logging.INFO)
    if not logger.handlers:
        fh = logging.FileHandler(path)
        fh.setFormatter(logging.Formatter('%(asctime)s  %(message)s', datefmt='%H:%M:%S'))
        logger.addHandler(fh)
    return logger, path