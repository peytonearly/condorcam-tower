# === Import Libraries === #
# Python Libraries
import signal

# Custom classes
from Driver_Class import AF160_with_Encoder
# === #

# === Global Variables === #
signal_received = False  # Tracks if a signal has been received
# === #

# === Helpers === #
def signal_handler(signum, frame):
    """
    Handles interrupt signals.
    """
    global signal_received
    signal_received = True
# === #

def main():
    # Signal interrupt
    global signal_received
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Create class instances
    driver = AF160_with_Encoder()
    
    # Initialize runtime variables
    encoder_value = driver.get_encoder_position()
    
    # Main loop
    try:
        encoder_value = driver.get_encoder_position()
        print(f"Encoder value = {encoder_value}")
    finally:
        if signal_received:
            print(f"Interrupt signal received. Closing program.")
        driver.disconnect_driver()
    
if __name__ == "__main__":
    main()