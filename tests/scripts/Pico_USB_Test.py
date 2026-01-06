from Encoder_Class import E5_with_Pico_USB
import time

encoder = E5_with_Pico_USB()

while True:
    pos = encoder.get_position()
    rate = encoder.get_travel_rate()
    print(f"Position: {pos}, Velocity: {rate:.2f}")
    time.sleep(0.1)