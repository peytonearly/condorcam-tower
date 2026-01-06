from Tower_Class import Tower_with_sled
import time
import pigpio
import sys
from statistics import median

pi = pigpio.pi()
if not pi.connected:
    print("Pigpiod not running. Closing...")
    exit()
    
tower = Tower_with_sled(pi)

while True:
    tower.get_controller_channel5_command()
    tower.get_controller_channel6_command()
    print(f"Channel 5 | High time: {int(min(tower.channel5_dq))} | Value: {tower._max_allowed_sled_speed:.3f}")
    print(f"Channel 6 | High time: {int(min(tower.channel6_dq))} | Value: {tower._max_allowed_tower_speed:.3f}")
    time.sleep(0.1)
    
    # Move cursor up 2 lines (to overwrite)
    sys.stdout.write("\033[F" * 2)