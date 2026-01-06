import time
import os
import logging
from Tower_Class import Tower
from main import setup_logging

if __name__ == "__main__":
    # Start the logger
    os.makedirs("logs", exist_ok=True)
    setup_logging()
    logging.info("Encoder test started")
    
    tower = Tower()
    tower_pos = tower.get_position()
    
    while True:
        tower_pos = tower.get_position()
        logging.info(f"Encoder position: {tower_pos}")