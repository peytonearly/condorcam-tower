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
    print(f"Channel 1 high time: {int(min(tower.steering_dq))}")
    print(f"Channel 2 high time: {int(min(tower.throttle_dq))}")
    print(f"Channel 3 high time: {int(min(tower.channel3_dq))}")
    print(f"Channel 4 high time: {int(min(tower.channel4_dq))}")
    print(f"Channel 5 high time: {int(min(tower.channel5_dq))}")
    print(f"Channel 6 high time: {int(min(tower.channel6_dq))}")
    time.sleep(0.1)
    
    # Move cursor up 6 lines (to overwrite)
    sys.stdout.write("\033[F" * 6)