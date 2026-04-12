# Python Libraries
import os
import time
import types
import pigpio
import signal
import logging
from datetime import datetime
from logging.handlers import RotatingFileHandler

# Project Libraries
from Tower_Class import RigController
from Driver_Class import AF160
from Encoder_Class import E5_with_Pico_USB

# === Global Variables === #
signal_received = False  # Tracks if an interrupt signal has been received
console_logging = False  # Indicates if the logger should output to the console
# === #

# === Helpers === #
class MicrosecondFormatter(logging.Formatter):
    """
    Custom formatter to include microsecond-precision timestamps.
    """
    def formatTime(self, record: logging.LogRecord, datefmt: str | None =None) -> datetime.strftime:
        dt = datetime.fromtimestamp(record.created)
        if datefmt:
            return dt.strftime(datefmt)
        return dt.strftime("%Y-%m-%d %H:%M:%S.%f")
    
def setup_logging() -> None:
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
        
def signal_handler(signum: int, frame: types.FrameType | None) -> None:
    """
    Handles interrupt signals.
    """
    global signal_received
    signal_received = True
    logging.info(f"Signal {signum} received")
# === #

def main() -> None:
    # Start the logger
    os.makedirs("logs", exist_ok=True)
    setup_logging()
    
    # Signal interrupt
    global signal_received
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Pigpio connection
    pi = pigpio.pi()
    if not pi.connected:
        logging.critical("Could not connect to pigpio daemon. Closing program...")
        exit()
        
    # Class constants
    enable_steering = False  # Indicates if the sled is connected
    tower_channel   = 1      # Indicates which channel the tower is connected to (0 for left, 1 for right)
    sled_channel    = 1      # Indicates which channel the sled is connected to (0 for left, 1 for right)
    
    # Class instances
    rig     = RigController(pi=pi, enable_steering=enable_steering)
    driver  = AF160(throttle_channel=tower_channel, steering_channel=sled_channel, enable_steering=enable_steering)
    encoder = E5_with_Pico_USB()
    
    # Configure zero button flagger
    rig.subscribe_zero_button(encoder.handle_zero_button_tripped)
    
    # Runtime constants
    enc_max            = encoder.get_encoder_max()    # Encoder max position
    slow_region        = 0.1                          # Slow-down region
    lower_region       = slow_region * enc_max        # Encoder position under which is the lower region
    upper_region       = (1 - slow_region) * enc_max  # Encoder position above which is the upper region
    no_enc_slow_factor = 0.3                          # Slow-down factor used when encoder is not connected
    
    # Runtime variables
    tower_input, sled_input = rig.update()                      # Initial control inputs
    enc_pos                 = encoder.get_position()            # Initial encoder position
    enc_vel                 = encoder.get_velocity()            # Initial encoder velocity
    enc_connected           = encoder.get_encoder_connection()  # Indicates encoder connection
    tower_cmd = sled_cmd    = 0                                 # Initial control commands
    
    # Main loop
    try:
        logging.info("Beginning loop")
        
        while not signal_received:
            # Check for control input and current position
            tower_input, sled_input = rig.update()
            enc_pos                 = encoder.get_position()
            enc_vel                 = encoder.get_velocity()
            
            # Determine tower command
            if enc_connected:  # When encoder connected, use zone handlers
                if tower_input != 0.0:
                    match enc_pos:
                        case _ if enc_pos <= lower_region:                # Tower in lower region
                            tower_cmd = rig.throttle.lower_region(enc_pos)
                        case _ if lower_region < enc_pos < upper_region:  # Tower in middle region
                            tower_cmd = rig.throttle.middle_region()
                        case _ if enc_pos >= upper_region:                # Tower in upper region
                            tower_cmd = rig.throttle.upper_region(enc_pos, enc_max)
                else:
                    # Hold tower position
                    tower_cmd = rig.throttle.position_hold(enc_vel)
            else:              # When encoder not connected, use middle region at slower speeds
                tower_cmd = rig.throttle.middle_region() * no_enc_slow_factor
                
            # Determine sled command
            if enable_steering:
                sled_cmd = sled_input * rig.steering.get_steering_direction()
            else:
                sled_cmd = None
                
            # Send motor commands
            driver.send_payloads(throttle_input = tower_cmd, steering_input = sled_cmd)
            
            # Log debug values
            rig.log_debug_values()
            driver.log_debug_values()
            encoder.log_debug_values()
    finally:
        if signal_received:
            logging.warning("Interrupt signal received. Closing program...")
        rig.disconnect()
        driver.disconnect()
        encoder.disconnect()
        pi.stop()
        logging.info("Clean shutdown complete.")
        
if __name__ == "__main__":
    main(0)