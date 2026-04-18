# Python Libraries
import os
import logging

# Project Modules
from ..pi_runtime import Utils
from ..pi_runtime.Driver_Class import AF160

def main() -> None:
    # Start the logger
    os.makedirs("logs", exist_ok=True)
    Utils.setup_logging(console_logging = True)
    
    # Class instance
    driver = AF160(throttle_channel=Utils.RIGHT, steering_channel=None, enable_steering=False)

if __name__ == "__main__":
    main()