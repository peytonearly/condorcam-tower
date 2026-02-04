# === Import Libraries === #
# Python libraries
import pigpio
import os
import sys
import logging
import time
import signal
from datetime import datetime
from logging.handlers import RotatingFileHandler
from collections import deque

# Custom classes
from Tower_Class import Tower_with_sled
from Driver_Class import AF160
from Encoder_Class import E5_with_Pico_USB
# === #

# === Global Variables === #
signal_received = False  # Tracks if a signal has been received
console_logging = False  # Indicates if the logger should output to the console
tower_connected = True  # Indicates if the tower is connected to the motor
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
        return dt.strftime("%Y-%m-%d %H:$M:$S.$f")
    
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
    
    # Instantiate classes
    tower = Tower_with_sled(pi)
    driver = AF160()
    encoder = E5_with_Pico_USB()

    # Configure zero button flag
    tower.on_zero_button_change.subscribe(encoder.handle_zero_button_tripped)

    # Initialize runtime variables
    global tower_connected
    tower_input, _   = tower.get_input_averages()  # Get initial controller inputs
    encoder_position = encoder.get_position()      # Get initial encoder position
    encoder_velocity = encoder.get_travel_rate()   # Get initial encoder velocity
    encoder_max      = encoder.get_encoder_max()   # Get max encoder value
    driver_throttle  = 0                           # Initialize throttle speed value
    slow_range       = 0.1                         # Sets the slow-down region to outer 10% of travel range

    # Main loop
    try:
        logging.info("Beginning loop")
        while not signal_received:
            tower_input, _ = tower.get_input_averages()
            encoder_position = encoder.get_position()
            encoder_velocity = encoder.get_travel_rate()

            if tower_connected:
                if tower_input != 0:
                    # Handle zones
                    match encoder_position:
                        case _ if encoder_position < 0:
                            # Below zero point
                            driver_throttle = tower.under_lower_zone()
                        case _ if 0 <= encoder_position <= (slow_range * encoder_max):
                            # In lower slow-down range
                            driver_throttle = tower.lower_zone(encoder_position)
                        case _ if (slow_range * encoder_max) < encoder_position < ((1 - slow_range) * encoder_max):
                            # In middle of travel range
                            driver_throttle = tower.middle_zone()
                        case _ if ((1 - slow_range) * encoder_max) <= encoder_position <= encoder_max:
                            # In upper slow-down range
                            driver_throttle = tower.upper_zone(encoder_position, encoder_max)
                        case _ if encoder_position > encoder_max:
                            # Above max point
                            driver_throttle = tower.above_upper_zone()
                else:
                    # Hold tower position
                    driver_throttle = tower.position_hold(encoder.get_travel_rate())
            else:
                driver_throttle = tower.middle_zone()

            # Send motor commands
            driver.send_payloads(driver_throttle, driver_throttle)  # Send to both motors to avoid worrying about wiring for now.

            # Log debug values
            tower.log_debug_values()
            driver.log_debug_values()
            encoder.log_debug_values()

            # Command-line output (print-in-place)
            sys.stdout.write("\r\033[K")  # Clear to end of line
            sys.stdout.write(f"Position: {encoder_position:06} | Velocity: {encoder_velocity:05.3f}")
            sys.stdout.flush()
    finally:
        if signal_received:
            logging.warning("Interrupt signal received. Closing program...")
        
        # Disconnect tower devices
        tower.disconnect_devices()
        driver.disconnect_driver()
        encoder.disconnect_encoder()
        pi.stop()

        print("\n")
        logging.info("Clean shutdown complete.")

if __name__ == "__main__":
    main()