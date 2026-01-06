from Tower_Class import Tower
import time
import pigpio

pi = pigpio.pi()
if not pi.connected:
    print("pigpio not open")
    exit()
tower = Tower(pi)

while True:
    print(f"Button pressed: {tower.zero_button_tripped}")
    time.sleep(0.5)