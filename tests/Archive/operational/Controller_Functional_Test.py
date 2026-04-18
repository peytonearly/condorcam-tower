# === Import Libraries === #
# Python libraries
import time
import pigpio
import sys
import os
import logging
import signal
from datetime import datetime
from logging.handlers import RotatingFileHandler
from statistics import median

# Custom classes
from Tower_Class import Tower_with_sled
# === #

# === Global Variables === #
signal_received = False  # Tracks if a signal has been received
console_logging = False  # Indicates if the logger should output to the console
# === #

# === Helpers === #
class MicrosecondFormatter(logging.Formatter):
    """
    Custom formatter to include microsecond-precision timestamps.
    """
    def formatTime(self, record, datefmt=None):
        dt = datetime.fromtimestamp(record.created)
        if datefmt:
            return dt.strftime(datefmt)
        return dt.strftime("%Y-%m-%d %H:%M:%S.%f")
    
def setup_logging():
    """
    Configures the logger.
    """
    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG)

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
    global console_logging
    if console_logging:
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)
        console_handler.setFormatter(formatter)
        logger.addHandler(console_handler)
        
def signal_handler(signum, frame):
    """
    Handles interrupt signals.
    """
    global signal_received
    signal_received = True
    logging.info(f"Signal {signum} received")
    
def redraw(lines):
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
# === #

def main():
    # Start the logger
    os.makedirs("logs", exist_ok=True)
    setup_logging()
    
    # Signal interrupt
    global signal_received
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Create pigpio connection
    pi = pigpio.pi()
    if not pi.connected:
        logging.critical("Could not connect to pigpio daemon. Closing...")
        print("Could not connect to pigpio daemon. Closing...")
        exit()
        
    # Class instantiation
    tower = Tower_with_sled(pi)
    
    # Test loop
    try:
        logging.info("Beginning loop")
        while not signal_received:
            throttle_input, steering_input = tower.get_input_averages()  # Collect inputs
            
            lines = [
                f"Channel 1 | High time: {int(median(tower.steering_dq))} | Value: {tower.steering_input:.3f} | Input: {steering_input:.3f}",
                f"Channel 2 | High time: {int(median(tower.throttle_dq))} | Value: {tower.throttle_input:.3f} | Input: {throttle_input:.3f}",
                f"Channel 3 | High time: {int(median(tower.channel3_dq))} | Value: {tower._pedals_connected}",
                f"Channel 4 | High time: {int(median(tower.channel4_dq))} | Value: {tower._steering_direction}",
                f"Channel 5 | High time: {int(median(tower.channel5_dq))} | Value: {tower._max_allowed_sled_speed}",
                f"Channel 6 | High time: {int(median(tower.channel6_dq))} | Value: {tower._max_allowed_tower_speed}",
                f"Max Allowed Tower Speed = {tower._max_allowed_tower_speed:.3f}",
            ]
            
            redraw(lines)
            
            # Log debug values
            tower.log_debug_values()
            time.sleep(0.1)
    finally:
        if signal_received:
            logging.warning("Interrupt signal received. Closing program.")
            print("Interrupt signal received. Closing...")
        tower.disconnect_devices()
        pi.stop()
        logging.info("Clean shutdown complete.")
        
if __name__ == "__main__":
    main()