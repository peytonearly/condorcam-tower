'''
Functional test script that mimics main.py, but outputs additional information in real-time to assist with testing.
'''

# === Import Libaries === #
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
import matplotlib.pyplot as plt

# Custom classes
from Tower_Class import Tower_with_sled
from Driver_Class import AF160
from Encoder_Class import E5_with_Pico_USB
# === #

# === Global Variables === #
signal_received = False  # Tracks if a signal has been received
console_logging = False  # Indicates if the logger should output to the console
tower_connected = False  # Indicates if the tower is connected to the motor
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

# === Matplotlib Integration === #
WINDOW_S = 15.0            # Rolling window length
REFRESH_HZ = 15.0          # Plot update rate (Hz)
MIN_LOOP_SLEEP_S = 0.0005  # Small sleep to avoid pegging CPU
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

    # Configure zero button flagger
    tower.on_zero_button_change.subscribe(encoder.handle_zero_button_tripped)

    # Initialize runtime variables
    global tower_connected
    tower_input, _   = tower.get_input_averages()  # Get initial controller inputs
    encoder_position = encoder.get_position()      # Get initial encoder position
    encoder_max      = encoder.get_encoder_max()   # Get max encoder value
    driver_throttle  = 0                           # Initialize throttle speed value
    slow_range       = 0.1                         # Sets the slow-down region to outer 10% of travel range

    # --- Plotting --- #
    t0 = time.monotonic()
    ts = deque()
    ys = deque()

    plt.ion()
    fig, ax = plt.subplots()
    (line,) = ax.plot([], [], lw=1)
    ax.set_title("Encoder (last 15s)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Position")
    fig.tight_layout()

    next_refresh = time.monotonic()
    refresh_period = 1.0 / REFRESH_HZ

    DISPLAY_MAX_HZ = 300.0
    display_min_dt = 1.0 / DISPLAY_MAX_HZ
    last_display_t = 0.0
    # ---------------- #

    # Main loop
    try:
        logging.info("Beginning loop")
        while not signal_received and plt.fignum_exists(fig.number):
            tower_input, _ = tower.get_input_averages()
            encoder_position = encoder.get_position()

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
            driver.send_payloads(driver_throttle, driver_throttle)  # Send to both motors. Avoids having to worry about wiring for now

            # Log debug values
            tower.log_debug_values()
            driver.log_debug_values()
            encoder.log_debug_values()
            
            print(f"Encoder position {encoder_position}")
            sys.stdout.write("\033[F" * 1)

            # # --- Plotting --- #
            # now = time.monotonic()
            # t = now - t0

            # if (t - last_display_t) >= display_min_dt:
            #     ts.append(t)
            #     ys.append(encoder_position)
            #     last_display_t = t

            # # Trim to last WINDOW_S seconds
            # while ts and (t - ts[0]) > WINDOW_S:
            #     ts.popleft()
            #     ys.popleft()

            # # Plot refresh on its own schedule
            # if now >= next_refresh and ts:
            #     x = list(ts)
            #     y = list(ys)

            #     line.set_data(x, y)

            #     # Rolling x-window
            #     ax.set_xlim(max(0.0, x[-1] - WINDOW_S), x[-1])

            #     # Autoscale y to visible data (with padding)
            #     ymin = min(y)
            #     ymax = max(y)
            #     pad = 1.0 if ymax == ymin else 0.05 * (ymax - ymin)
            #     ax.set_ylim(ymin - pad, ymax + pad)

            #     fig.canvas.draw_idle()
            #     fig.canvas.flush_events()

            #     next_refresh += refresh_period

            # time.sleep(MIN_LOOP_SLEEP_S)
            # --- #
    finally:
        if signal_received:
            logging.warning("Interrupt signal received. Closing program.")

        # Disconnect tower devices
        tower.disconnect_devices()
        driver.disconnect_driver()
        encoder.disconnect_encoder()
        pi.stop()

        # Close plotter
        plt.ioff()
        plt.close(fig)
        logging.info("Clean shutdown complete.")

if __name__ == "__main__":
    main()