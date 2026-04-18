# Python Libraries
import sys
import types
import logging
from datetime import datetime
from logging.handlers import RotatingFileHandler

# Project Modules
import State

LEFT  = 0  # Driver left output is channel 0
RIGHT = 1  # Driver right output is channel 1

class MicrosecondFormatter(logging.Formatter):
    """
    Custom formatter to include microsecond-precision timestamps.
    """
    def formatTime(self, record: logging.LogRecord, datefmt: str | None =None) -> str:
        dt = datetime.fromtimestamp(record.created)
        if datefmt:
            return dt.strftime(datefmt)
        return dt.strftime("%Y-%m-%d %H:%M:%S.%f")
    
def setup_logging(console_logging: bool = False) -> None:
    """
    Configures the logger.
    """
    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG)
    
    if logger.handlers:
        logger.handlers.clear()
    
    # Formatter
    formatter = MicrosecondFormatter(
        fmt="%(asctime)s [ %(levelname)s ] %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S.%f"
    )
    
    # Rotating file handler (5 files, 100 MB each)
    file_handler = RotatingFileHandler("logs/system.log", maxBytes=100_000_000, backupCount=5)
    file_handler.setLevel(logging.DEBUG)
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)
    
    # Create console handler if specified
    if console_logging:
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)
        console_handler.setFormatter(formatter)
        logger.addHandler(console_handler)
        
def signal_handler(signum: int, frame: types.FrameType | None) -> None:
    """
    Handles interrupt signals.
    """
    State.signal_received.set()
    logging.info(f"Signal {signum} received")
    
def redraw(lines: list) -> None:
    """
    Re-draw a fixed block of lines in-place in a terminal.
    
    - Moves cursor back up to the start of the previous block.
    - Clears each line before writing new content.
    """
    if not hasattr(redraw, "_prev_n"):
        redraw._prev_n = 0
        
    # Move to top of previous block
    if redraw._prev_n:
        sys.stdout.write("\x1b[F" * redraw._prev_n)
        
    # Write the new block, clearing each line
    for line in lines:
        sys.stdout.write("\x1b[2K\r" + line + "\n")
        
    sys.stdout.flush()
    redraw._prev_n = len(lines)