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
            _, _ = tower.get_input_averages()  # Collect inputs
            
            # Channel 1 values (steering)
            print(f"Channel 1 | High time: {int(median(tower.steering_dq))} | Value: {tower.steering_input}")
            
            # Channel 2 values (throttle)
            print(f"Channel 2 | High time: {int(median(tower.throttle_dq))} | Value: {tower.throttle_input}")
            
            # Channel 3 values (button)
            print(f"Channel 3 | High time: {int(median(tower.channel3_dq))} | Value: {tower._pedals_connected}")
            
            # Channel 4 values (switch)
            print(f"Channel 4 | High time: {int(median(tower.channel4_dq))} | Value: {tower._steering_direction}")
            
            # Channel 5 values (top-left knob)
            print(f"Channel 5 | High time: {int(median(tower.channel5_dq))} | Value: {tower._max_allowed_sled_speed}")   
            
            # Channel 6 values (top-right knob)
            print(f"Channel 6 | High time: {int(median(tower.channel6_dq))} | Value: {tower._max_allowed_tower_speed}")
            
            # Log debug values
            tower.log_debug_values()
            
            # Move cursor to overwrite (print in-place)
            time.sleep(0.1)
            sys.stdout.write("\033[F" * 6)
    finally:
        if signal_received:
            logging.warning("Interrupt signal received. Closing program.")
            print("Interrupt signal received. Closing...")
        tower.disconnect_devices()
        pi.stop()
        logging.info("Clean shutdown complete.")
        
if __name__ == "__main__":
    main()