from Tower_Class import Tower
from Encoder_Class import E5_with_Pico_USB
import time
import pigpio

pi = pigpio.pi()
if not pi.connected:
    print("Pigpiod not open. Closing...")
    exit()
    
tower = Tower(pi)
encoder = E5_with_Pico_USB()

tower.on_zero_button_change.subscribe(encoder.handle_zero_button_tripped)

while True:
    print(f"Encoder position: {encoder.get_position()}")
    time.sleep(0.5)