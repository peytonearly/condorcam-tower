# Python Libraries
import os
import sys
import time
import pigpio
import signal
import logging

# Project Modules
from pi_runtime import State
from pi_runtime import Utils
from pi_runtime.Tower_Class import RigController
from pi_runtime.Driver_Class import AF160
from pi_runtime.Encoder_Class import E5_with_Pico_USB

# === Tower initialization === #
def initialize_tower(rig: RigController, driver: AF160, encoder: E5_with_Pico_USB) -> tuple[bool, list]:
    """
    Tower initialization protocol:
        *Operator should ensure that tower is fully lowered before power-on.*

        1. 3 second pause for safety.
        2. Verify that zero button is functioning.
        3. Slowly raise the tower.
        4. Stop when position stops changing / velocity is zero. Record position.
        5. Slowly lower the tower.
        6. Stop when zero button is hit. Record position offset/drift if present.
        
    Inputs:
        rig: Rig controller class instance
        driver: Driver class instance
        encoder: Encoder class instance
        
    Return:
        zero_button_operable: (bool) Indicator of properly functioning zero button
        enc_offsets: (list) List of encoder offsets measured during initialization
    """
    tower_move_gentle = 0.1
    
    # --- Step 1: Safety delay --- #
    time.sleep(3)  # 3 second pause
    
    # --- Step 2: Zero button verification --- #
    zero_button_operable = False
    enc_offsets = []
    
    # Tower is expected to start at zero. Ensure this
    enc_pos = encoder.get_encoder_readings()[0]
    if enc_pos != 0:
        encoder.set_zero_position()
        enc_pos = encoder.get_encoder_readings()[0]
        
    # Gently raise tower
    driver.send_payloads(-1 * tower_move_gentle, None)       # Gentle raise
    time.sleep(3)                                       # 1 second raise
    driver.send_payloads(0, None)                       # End raise
    enc_pos = encoder.get_encoder_readings()[0]
    
    # Ensure zero button flag is not tripped
    rig.update()
    
    # Gently lower tower until velocity = 0
    driver.send_payloads(tower_move_gentle, None)  # Gentle lower
    while encoder.get_encoder_readings()[2] != 0:
        time.sleep(0.1)                                 # Brief pause
    driver.send_payloads(0, None)                       # End lower
    
    # Check for button trip and encoder position
    rig.update()
    enc_pos = encoder.get_encoder_readings()[0]
    trip1 = rig.zero_button_tripped
    if enc_pos != 0:
        enc_offsets.append(enc_pos)
        encoder.set_zero_position()
        
    # Gently raise tower
    driver.send_payloads(-1 * tower_move_gentle, None)       # Gentle raise
    time.sleep(3)                                       # 1 second raise
    driver.send_payloads(0, None)                       # End raise
    enc_pos = encoder.get_encoder_readings()[0]
    
    # Ensure zero button flag is not tripped
    rig.update()
    
    # Gently lower tower until velocity = 0
    driver.send_payloads(tower_move_gentle, None)  # Gentle lower
    while encoder.get_encoder_readings()[2] != 0:
        time.sleep(0.1)                                 # Brief pause
    driver.send_payloads(0, None)                       # End lower
    
    # Check for button trip and encoder position
    rig.update()
    enc_pos = encoder.get_encoder_readings()[0]
    trip2 = rig.zero_button_tripped
    if enc_pos != 0:
        enc_offsets.append(enc_pos)
        encoder.set_zero_position()
        
    # Zero button operable if tripped both times
    if trip1 and trip2:
        zero_button_operable = True
        
    # --- Step 3: Slowly raise the tower to top --- 
    enc_pos = encoder.get_encoder_readings()[0]
    driver.send_payloads(-1.5 * tower_move_gentle, None)
    while encoder.get_encoder_readings()[2] != 0:
        time.sleep(0.1)
    driver.send_payloads(0, None)
    
    # --- Step 4: Record max position ---
    enc_pos = encoder.get_encoder_readings()[0]
    enc_max = enc_pos
    encoder.set_encoder_max(enc_max)
    
    # --- Step 5: Slowly lower the tower to bottom ---
    driver.send_payloads(tower_move_gentle, None)
    while encoder.get_encoder_readings()[2] != 0:
        time.sleep(0.1)
    driver.send_payloads(0, None)
        
    # --- Step 6: Record lowest position ---
    enc_pos = encoder.get_encoder_readings()[0]
    if enc_pos != 0:
        enc_offsets.append(enc_pos)
        encoder.set_zero_position()
        
    return zero_button_operable, enc_offsets
# === #

def main() -> None:
    # Start the logger
    os.makedirs("logs", exist_ok=True)
    Utils.setup_logging(console_logging = True)
    logging.info("Beginning new run")
    
    # Signal interrupt
    signal.signal(signal.SIGINT, Utils.signal_handler)
    signal.signal(signal.SIGTERM, Utils.signal_handler)
    
    # Pigpio connection
    pi = pigpio.pi()
    if not pi.connected:
        logging.critical("Could not connect to pigpio daemon. Closing program...")
        exit()
        
    # Class constants
    enable_steering = False        # Indicates if the sled is connected
    tower_channel   = Utils.RIGHT  # Indicates which channel the tower is connected to (0 for left, 1 for right)
    sled_channel    = Utils.LEFT   # Indicates which channel the sled is connected to (0 for left, 1 for right)
    
    # Class instances
    rig     = RigController(pi=pi, enable_steering=enable_steering)
    driver  = AF160(throttle_channel=tower_channel, steering_channel=sled_channel, enable_steering=enable_steering)
    encoder = E5_with_Pico_USB()
    
    # Run tower initialization
    zero_button_operable, enc_init_offsets = initialize_tower(rig, driver, encoder)
    # zero_button_operable = True
    
    # Runtime constants
    enc_max            = encoder.get_encoder_max()         # Encoder max position
    slow_region        = 0.20                              # Slow-down region
    lower_region       = int(slow_region * enc_max)        # Encoder position under which is the lower region
    upper_region       = int((1 - slow_region) * enc_max)  # Encoder position above which is the upper region
    no_enc_slow_factor = 0.3                               # Slow-down factor used when encoder is not connected
    
    # Update rig controller internal encoder values
    rig.update_enc_vals(enc_max, upper_region, lower_region)
    
    # Runtime variables
    tower_input, sled_input = rig.update()                               # Initial control inputs
    enc_pos, enc_vel_inst, enc_vel_avg = encoder.get_encoder_readings()  # Initial encoder readings
    enc_connected           = encoder.get_encoder_connection()           # Indicates encoder connection
    tower_cmd = sled_cmd    = 0                                          # Initial control commands
    
    # Configure zero button flagger if button is operable
    if zero_button_operable:
        rig.subscribe_zero_button(encoder.handle_zero_button_tripped)
        
    # Initialize time-keeping variables (in nanoseconds)
    timer_start = None         # Runtime calc
    timer_end   = None         # Runtime calc
    timer_loop  = None         # Runtime calc - loop length (end - start)
    timer_sum   = 0            # Returns running sum of times (to be used in average)
    timer_cnt   = 0            # Returns number of loops
    timer_high  = 0            # Initialize to 0
    timer_low   = sys.maxsize  # Initialize to largest int possible
    
    # Main loop
    try:
        logging.info("Beginning loop")
        
        while not State.signal_received.is_set():
            # Start timer
            timer_start = time.perf_counter_ns()
            
            # Check for control input and current position
            tower_input, _ = rig.update()
            enc_pos, enc_vel_inst, enc_vel_avg = encoder.get_encoder_readings()
            
            # Determine tower command
            if enc_connected:  # When encoder connected, use zone handlers
                if tower_input:
                    if (enc_pos <= lower_region):               tower_cmd = rig.throttle.lower_region(enc_pos)           # Tower in lower region
                    if (lower_region < enc_pos < upper_region): tower_cmd = rig.throttle.middle_region()                 # Tower in middle region
                    if (enc_pos >= upper_region):               tower_cmd = rig.throttle.upper_region(enc_pos)  # Tower in upper region
                else:
                    tower_cmd = rig.throttle.position_hold(enc_vel_avg)  # Hold tower position
                    # tower_cmd = 0.05
                    
            else:  # When encoder not connected, use middle region at slower speeds
                tower_cmd = rig.throttle.middle_region() * no_enc_slow_factor
                
            # Determine sled command
            if enable_steering:
                sled_cmd = rig.steering.get_steering_command()
            else:
                sled_cmd = None
                
            # Send motor commands
            driver.send_payloads(throttle_input = -1 * tower_cmd, steering_input = sled_cmd)
            
            # Log debug values
            rig.log_debug_values()
            driver.log_debug_values()
            encoder.log_debug_values()
            
            # Timer calcs
            timer_end = time.perf_counter_ns()
            timer_loop = timer_end - timer_start
            timer_sum += timer_loop
            timer_cnt += 1
            if timer_loop < timer_low: timer_low = timer_loop    # Update fastest time
            if timer_loop > timer_high: timer_high = timer_loop  # Update slowest time
    finally:
        if State.signal_received.is_set():
            logging.warning("Interrupt signal received. Closing program...")
        rig.disconnect()
        driver.disconnect()
        encoder.disconnect()
        pi.stop()
        logging.info("Clean shutdown complete.")
        try:
            logging.info(f"Average loop time: {(timer_sum / timer_cnt) / 1_000_000_000} s")
            logging.info(f"Fastest loop time: {timer_low / 1_000_000_000} s")
            logging.info(f"Slowest loop time: {timer_high / 1_000_000_000} s")
        except ZeroDivisionError:
            pass
        
if __name__ == "__main__":
    main()