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
from Driver_Class import AF160
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
        print("Could not connected to pigpio daemon. Closing...")
        exit()
        
    # Class instantiation
    tower = Tower_with_sled(pi)
    driver = AF160()
    
    # Test loop
    try:
        logging.info("Beginning loop")
        while not signal_received:
            throttle_input, steering_input = tower.get_input_averages()
            throttle_command = driver._scale_input(throttle_input)
            steering_command = driver._scale_input(steering_input)
            
            lines = [
                f"Steering | Value: {tower.steering_input:.3f} | Input: {steering_input:.3f} | Command: {steering_command}",
                f"Throttle | Value: {tower.throttle_input:.3f} | Input: {throttle_input:.3f} | Command: {throttle_command}",
            ]
            
            redraw(lines)
            
            # Log debug values
            tower.log_debug_values()
            driver.log_debug_values()
            
            time.sleep(0.1)
    finally:
        if signal_received:
            logging.warning("Interrupt signal received. Closing program.")
            print("Interrupt signal received. Closing...")
        tower.disconnect_devices()
        driver.disconnect_driver()
        pi.stop()
        logging.info("Clean shutdown complete.")
        
if __name__ == "__main__":
    main()