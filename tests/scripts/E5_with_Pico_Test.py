# === Import Libraries === #
# Python Libraries
import os
import logging
import signal
import time
from datetime import datetime
from logging.handlers import RotatingFileHandler

# Custom classes
from Encoder_Class import E5_with_Pico_USB
from Driver_Class import AF160
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
    
    # Rotating file handler (5 file, 100 MB each)
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
    
    # Create class instances
    driver = AF160()
    encoder = E5_with_Pico_USB()
    
    # Initialize runtime variables
    encoder_position = encoder.get_position()  # Get initial encoder position
    encoder_trusted = encoder.get_encoder_trust()  # Determines if the encoder can be trusted on startup
    driver_steering = 0  # Initialize steering value
    count_dir = 1
    loop_interval = 0.05  # Seconds per loop (20 Hz)
    command_interval = 1.0  # Send new motor command every 1 second
    last_command_time = time.time()
    
    # Main loop
    try:
        logging.info("Beginning loop")
        while not signal_received:
            now = time.time()
            
            # Periodically change the steering input
            if now - last_command_time >= command_interval:
                if abs(driver_steering) < 0.95:
                    driver_steering += 0.10 * count_dir
                else:
                    count_dir *= -1
                    driver_steering += 0.10 * count_dir
                    
                print(f"Input to motor: {driver_steering * 100:.1f}%")
                driver.send_payloads(0, driver_steering)
                last_command_time = now
            
            # Read encoder
            encoder_position = encoder.get_position()
            print(f"Encoder position: {encoder_position}")
            
            # Log debug values
            driver.log_debug_values()
            encoder.log_debug_values()
            
            # Control loop timing
            time.sleep(loop_interval)
    finally:
        if signal_received:
            logging.warning("Interrupt signal received. Closing program")
        driver.disconnect_driver()
        encoder.disconnect_encoder()
        logging.info("Clean shutdown complete.")
        
def main_bare():
    # Start the logger
    os.makedirs("logs", exist_ok=True)
    setup_logging()
    
    # Signal interrupt
    global signal_received
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Create class instance
    encoder = E5_with_Pico()
    
    # Main loop
    try:
        logging.info("Beginning loop")
        while not signal_received:
            if not encoder.get_encoder_trust():
                encoder.reconnect_encoder()
                time.sleep(0.5)
                continue
            
            pos = encoder.get_position()
            rate = encoder.get_travel_rate()
            print(f"Position: {pos}, Rate: {rate:.6f}")
            
            encoder.log_debug_values()
            
            time.sleep(0.1)  # Polling interval
    finally:
        if signal_received:
            logging.warning("Interrupt signal received. Closing program")
        encoder.disconnect_encoder()
        logging.info("Clean shutdown complete.")
        
if __name__ == "__main__":
    main()