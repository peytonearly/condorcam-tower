# === Import Libraries === #
# Python libraries
import pigpio
import os
import logging
import signal
from datetime import datetime
from logging.handlers import RotatingFileHandler

# Custom classes
from main import MicrosecondFormatter
from Driver_Class import AF160
from Encoder_Class import HEDS_9140
# === #

# === Global Variables === #
signal_received = False  # Tracks if a signal has been received
console_logging = True  # Indicates if the logger should be output to the console
# === #

# === Helpers === #
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
        exit()
        
    # Create class instances
    driver = AF160()
    encoder = HEDS_9140(pi)
    
    # Initialize runtime variables
    dummy_input = 0
    count = 0  # Track number of loops at a given speed
    encoder_position = encoder.get_position()  # Get initial encoder position
    
    # Main loop
    try:
        logging.info("Beginning loop")
        while not signal_received:
            # Read encoder
            encoder_position = encoder.get_position()
            print(f"Encoder position: {encoder_position}")
            
            # === Send Known Motor Signals === #
            if count < 249:
                # Keep current speed for 250 counts
                count += 1
            else:
                count = 0
                # Increment dummy input
                if dummy_input < 1.0:
                    dummy_input += 0.1  # 10% increments
                else:
                    dummy_input = -1.0
                    
                # Send motor commands
                # print(f"Input to motor: {dummy_input*100}%")
                driver.send_payloads(dummy_input, dummy_input)
            # === #
            
            # Log debug values
            driver.log_debug_values()
            encoder.log_debug_values()
    finally:
        if signal_received:
            logging.warning("Interrupt signal received. Closing program")
        driver.disconnect_driver()
        encoder.disconnect_encoder()
        pi.stop
        logging.info("Clean shutdown complete.")
        
if __name__ == "__main__":
    main()