# Python Libraries
import os
import time
import pigpio
import signal
import logging
from statistics import median

# Project Modules
from pi_runtime.Utils import *
from pi_runtime.State import *
from pi_runtime.Tower_Class import RigController
from pi_runtime.Driver_Class import AF160

def main() -> None:
    # Start the logger
    os.makedirs("logs", exist_ok=True)
    Utils.setup_logging(console_logging = False)
    
    # Signal interrupt
    signal.signal(signal.SIGINT, Utils.signal_handler)
    signal.signal(signal.SIGTERM, Utils.signal_handler)
    
    # Pigpio connection
    pi = pigpio.pi()
    if not pi.connected:
        print("Could not connect to pigpio daemon. Closing program...")
        exit()
        
    # Class instances
    rig = RigController(pi=pi, enable_steering=True)
    driver = AF160(throttle_channel=1, steering_channel=0, enable_steering=True)
    
    # Runtime variables
    steering_smooth = steering_unsmooth = steering_command = None
    throttle_smooth = throttle_unsmooth = throttle_command = None
    steering_high_time = throttle_high_time = channel3_high_time = channel4_high_time = channel5_high_time = channel6_high_time = None
    
    # Main loop
    try:
        logging.info("Beginning RC controller test loop")
        
        while not State.signal_received.is_set():
            throttle_smooth, steering_smooth = rig.update()
            throttle_unsmooth = rig.throttle._throttle_input_unsmooth
            throttle_command = driver._scale_input(throttle_smooth)
            steering_unsmooth = rig.steering._steering_input_unsmooth
            steering_command = driver._scale_input(steering_smooth)
            throttle_high_time = int(median(rig.rc_input.throttle_dq))
            steering_high_time = int(median(rig.rc_input.steering_dq))
            channel3_high_time = int(median(rig.rc_input.channel3_dq))
            channel4_high_time = int(median(rig.rc_input.channel4_dq))
            channel5_high_time = int(median(rig.rc_input.channel5_dq))
            channel6_high_time = int(median(rig.rc_input.channel6_dq))
            
            lines = [
                f"Steering | High Time: {steering_high_time} | Unsmooth: {steering_unsmooth:.3f} | Smooth: {steering_smooth:.3f} | Command: {steering_command:.3f}",
                f"Throttle | High Time: {throttle_high_time} | Unsmooth: {throttle_unsmooth:.3f} | Smooth: {throttle_smooth:.3f} | Command: {throttle_command:.3f}",
                f"Channel3 | High Time: {channel3_high_time}",
                f"Channel4 | High Time: {channel4_high_time}",
                f"Channel5 | High Time: {channel5_high_time}",
                f"Channel6 | High Time: {channel6_high_time}"
            ]
            
            Utils.redraw(lines)
            
            # Log debug values
            rig.log_debug_values()
            driver.log_debug_values()
            
            time.sleep(0.1)
    finally:
        if State.signal_received.is_set():
            print("Interrupt signal received. Closing program...")
        rig.disconnect()
        driver.disconnect()
        pi.stop()
        logging.info("Clean shutdown complete.")

if __name__ == "__main__":
    main()