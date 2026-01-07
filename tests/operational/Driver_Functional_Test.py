# === Import Libraries === #
# Python libraries
import time
import sys
import os
import logging
import signal
from datetime import datetime
from logging.handlers import RotatingFileHandler

# Custom classes
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
# === #

def main():
    # Start the logger
    os.makedirs("logs", exist_ok=True)
    setup_logging()
    
    # Signal interrupt
    global signal_received
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Class instatiation
    driver = AF160()
    
    # Runtime variables
    driver_speed = 0  # Initialize driver speed value
    count_dir = 1  # Track direction of speed change
    last_command_time = time.time()  # Track when last driver command was sent
    SPEED_DELTA = 0.01  # [%] Amount of speed change per cycle.
    INPUT_LIMIT = 0.5  # Driver speed limiter
    LOOP_INTERVAL = 0.05  # Seconds per loop (20 Hz)
    COMMAND_INTERVAL = 1.0  # Send new motor command every 1 second
    
    # Test loop
    try:
        logging.info("Beginning loop")
        while not signal_received:
            now = time.time
            
            # Periodically change the driver speed
            if now - last_command_time >= COMMAND_INTERVAL:
                if abs(driver_speed) < INPUT_LIMIT:
                    driver_speed += SPEED_DELTA * count_dir
                else:
                    count_dir *= -1
                    driver_speed += SPEED_DELTA * count_dir
                    
                driver.send_payloads(driver_speed, driver_speed)
                last_command_time = now
            
            # --- Collect current speed values --- #
            # Left motor speed
            driver._send_command(f"@0gt\r".encode("ascii"))
            left_actual = driver.response
            
            # Right motor speed
            driver._send_command(f"@1gt\r".encode("ascii"))
            right_actual = driver.response
            # --- #
            
            # Display values
            print(f"Desired Speed (raw) | Left: {driver_speed * 100}% | Right: {driver_speed * 100}%")
            print(f"Desired Speed (scaled) | Left: {driver.throttle_input_scaled} | Right: {driver.steering_input_scaled}")
            print(f"Actual Speed | Left: {left_actual} | Right: {right_actual}")
            
            # Log debug values
            driver.log_debug_values()
            
            # Control loop timing
            time.sleep(LOOP_INTERVAL)
            
            # Move cursor to overwrite (print in-place)
            sys.stdout.write("\033[F" * 3)
    finally:
        if signal_received:
            logging.warning("Interrupt signal received. Closing program.")
            print("Interrupt signal received. Closing...")
        driver.disconnect_driver()
        logging.info("Clean shutdown complete.")
        
if __name__ == "__main__":
    main()