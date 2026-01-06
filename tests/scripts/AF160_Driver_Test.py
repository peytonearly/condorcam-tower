# === Import Libraries === #
# Python libraries
import os
import logging
import signal
from datetime import datetime
from logging.handlers import RotatingFileHandler

# Custom classes
from Driver_Class import AF160, AF160_with_Encoder
# === #

# === Global Variables === #
signal_received = False  # Tracks if a signal has been received
console_logging = True  # Indicates if the logger should output to the console
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
    
    # Create class instance
    driver = AF160()
    
    # Initialize runtime variables
    driver_throttle = 0  # Initialize throttle speed value
    count = 0  # Track number of loops at a given speed
    count_dir = 1  # Track direction of speed change
    
    # Main loop
    try:
        logging.info("Beginning loop")
        while not signal_received:
            if count < 249:
                # Keep current speed for 250 counts
                count += 1
            else:
                count = 0
                # Increment throttle input. Keep between -0.25 and 0.25
                if abs(driver_throttle) < 0.50:
                    driver_throttle += 0.01 * count_dir  # 1% increments
                else:
                    count_dir *= -1  # Reverse direction
                    driver_throttle += 0.01 * count_dir

                # Send motor command
                print(f"Input to motor: {driver_throttle * 100}%")
                driver.send_payloads(0, driver_throttle)
                
            # Log debug values
            driver.log_debug_values()
    finally:
        if signal_received:
            logging.warning("Interrupt signal received. Closing program")
        driver.disconnect_driver()
        logging.info("Clean shutdown complete")
        
if __name__ == "__main__":
    main()