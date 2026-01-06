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
    
    cnt = 10_000
    
    while True:
        tower.bits_speed = 4
        tower.bit_dir = 0
        tower.transmit_driver_throttle_payload()
        for _ in range(cnt):
            logging.info(f"Encoder last state: {tower._last_state}")
            
        tower.bits_speed = 0
        tower.transmit_driver_throttle_payload()
        
        tower.bits_speed = 2
        tower.bit_dir = 1
        tower.transmit_driver_throttle_payload()
        for _ in range(cnt):
            logging.info(f"Encoder last state: {tower._last_state}")