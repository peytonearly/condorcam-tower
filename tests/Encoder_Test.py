# Python Libraries
import os
import time
import signal
import logging

# Project Modules
from pi_runtime import Utils
from pi_runtime import State
from pi_runtime.Driver_Class import AF160
from pi_runtime.Encoder_Class import E5_with_Pico_USB

def main() -> None:
    # Start the logger
    os.makedirs("logs", exist_ok=True)
    Utils.setup_logging(console_logging = False)
    
    # Signal interrupt
    signal.signal(signal.SIGINT, Utils.signal_handler)
    signal.signal(signal.SIGTERM, Utils.signal_handler)
    
    # Class instances
    driver = AF160(throttle_channel=Utils.RIGHT, steering_channel=None, enable_steering=False)
    encoder = E5_with_Pico_USB()
    
    # Runtime constants
    command_time = 5.0        # Number of seconds to hold velocity
    tower_move_gentle = 0.1  # Gentle tower movement command
    
    # Runtime variables
    enc_pos = encoder.get_position()
    enc_vel = encoder.get_velocity()
    enc_vel_avg = encoder.get_average_velocity()
    last_command = tower_move_gentle
    cur_command = 0
    
    # Time-keeping variables (seconds)
    timer_loop_start = None
    timer_loop_end = None
    
    # Main loop
    try:
        logging.info("Beginning Encoder test loop")
        
        while not State.signal_received.is_set():
            if timer_loop_start is None:
                # Determine next command
                match last_command:
                    case _ if last_command == (-1 * tower_move_gentle) and cur_command == 0.0:
                        # Tower was zeroed after moving up -> move down
                        last_command = cur_command
                        cur_command  = tower_move_gentle
                    case _ if last_command == 0.0 and cur_command == (tower_move_gentle):
                        # Tower was moving down after being zeroed -> zero
                        last_command = cur_command
                        cur_command  = 0.0
                    case _ if last_command == (tower_move_gentle) and cur_command == 0.0:
                        # Tower was zeroed after moving down -> move up
                        last_command = cur_command
                        cur_command  = -1 * tower_move_gentle
                    case _ if last_command == 0.0 and cur_command == (-1 * tower_move_gentle):
                        # Tower was moving up after being zeroed -> zero
                        last_command = cur_command
                        cur_command  = 0.0
                        
                driver.send_payloads(cur_command, None)
                    
                # Restart timer
                timer_loop_start = time.time()
                
            # Get encoder readings
            enc_pos = encoder.get_position()
            enc_vel = encoder.get_velocity()
            enc_vel_avg = encoder.get_average_velocity()
            
            lines = [
                f"Position:       {enc_pos}",
                f"Velocity:       {enc_vel:.3f}",
                f"Velocity (avg): {enc_vel_avg:.3f}"
            ]
            
            Utils.redraw(lines)
            
            # Log debug values
            driver.log_debug_values()
            encoder.log_debug_values()
            
            timer_loop_end = time.time()
            
            # Check if command should be changed
            if timer_loop_end - timer_loop_start >= command_time:
                timer_loop_start = None
            
    finally:
        if State.signal_received.is_set():
            print("Interrupt signal received. Closing program...")
        driver.disconnect()
        encoder.disconnect()
        logging.info("Clean shutdown complete.")

if __name__ == "__main__":
    main()