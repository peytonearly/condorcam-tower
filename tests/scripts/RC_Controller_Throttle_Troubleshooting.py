import pigpio
import serial
import logging
import threading
from collections import deque
from statistics import median

import os
import signal
import sys
from datetime import datetime
from logging.handlers import RotatingFileHandler

class Tower:
    def __init__(self):
        # Controller
        self._controller_timeout = 1_000_000  # Timeout threshold (µs)
        self._input_timeout = 3_000_000  # Input timeout threshold (µs)
        self._deadzone = 0.3  # Deadzone (% of controller input)
        self._controller_throttle_neutral = 1690  # Neutral throttle point
        self._controller_throttle_forward = 1230  # Forward throttle limit
        self._controller_throttle_backward = 2125  # Backward throttle limit
        self._throttle_lock = threading.Lock()  # Prevent race conditions on the throttle calculations
        self._ema_alpha_throttle = 0.5  # Smoothing factor. Lower numbers are smoother, higher numbers are more responsive
        
        self._speed_limiter = 0.3
        self._max_allowed_bits_speed = round(63 * 20_000 / 57_000)
        
        self._pin_controller_throttle = 5  # PWM input from RC controller throttle lever
        self._pin_motor_driver = 14  # Output for sending motor driver commands
        
        self.throttle_bit_channel = 0
        self.throttle_bit_dir = 0
        self.throttle_bits_speed = 0
        
        self.throttle_high_time_dq = deque([self._controller_throttle_neutral for _ in range(7)], maxlen = 7)  # Deque for throttle high time
        
        self._throttle_last_edge_tick = None
        self._throttle_last_rising_tick = None
        self._throttle_last_period_tick = None
        self._throttle_high_time = 0
        self._throttle_period = 0
        self._throttle_input_unsmooth = 0  # Holds unsmoothed throttle input
        self.throttle_input = 0  # -1 <= x < 0 for down | x = 0 for neutral | 0 < x <= 1 for up
        
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.info("Tower class initialized")
        
        self.connect_devices()
        
        self._throttle_last_input_tick = self.pi.get_current_tick()
        
    def connect_devices(self):
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.logger.critical(f"Could not connect to pigpiod. Closing...")
            
        self._connect_motor_driver()
        self._connect_hand_controller()
        
        self.logger.info(f"All devices connected via UART serial")
        
    def disconnect_devices(self):
        self.driver.close()
        self._cb_controller_throttle.cancel()
        self.pi.stop()
        
        self.logger.info(f"All devices disconnected")
        
    def _connect_motor_driver(self):
        self.driver = serial.Serial(
            port='/dev/serial0',
            baudrate=115200,
            timeout=1
        )
        
        self.logger.info(f"Motor driver connected")
        
    def _connect_hand_controller(self):
        self.pi.set_mode(self._pin_controller_throttle, pigpio.INPUT)
        self.pi.set_pull_up_down(self._pin_controller_throttle, pigpio.PUD_DOWN)
        self.pi.set_glitch_filter(self._pin_controller_throttle, 100)
        self._cb_controller_throttle = self.pi.callback(self._pin_controller_throttle, pigpio.EITHER_EDGE, self._handle_controller_throttle_pwm)
        
        self.logger.info(f"Throttle pin and callback configured")
        
    def _handle_controller_throttle_pwm(self, gpio, level, tick):
        with self._throttle_lock:
            self._throttle_last_edge_tick = tick
            
            if level == 1:
                if self._throttle_last_period_tick is not None:
                    self._throttle_period = pigpio.tickDiff(self._throttle_last_period_tick, tick)
                self._throttle_last_period_tick = tick
                self._throttle_last_rising_tick = tick
            elif level == 0:
                if self._throttle_last_rising_tick is not None:
                    pulse_width = pigpio.tickDiff(self._throttle_last_rising_tick, tick)
                    if 1000 <= pulse_width <= 2200:
                        self.throttle_high_time_dq.append(pulse_width)
                    else:
                        self.logger.debug(f"Ignored invalid throttle pulse width: {pulse_width}")
                        
    def get_controller_throttle_command(self):
        with self._throttle_lock:
            now = self.pi.get_current_tick()
            if self._throttle_last_edge_tick is None or pigpio.tickDiff(self._throttle_last_edge_tick, now) > self._controller_timeout:
                # No active signal
                self._throttle_last_edge_tick = self._throttle_last_period_tick = self._throttle_last_rising_tick = None
                self._throttle_high_time = self._throttle_period = self.throttle_input = 0
                
            elif self._throttle_period > 0:
                # Active signal
                high_time = median(self.throttle_high_time_dq)
                if self._controller_throttle_neutral - 25 <= high_time <= self._controller_throttle_neutral + 25:
                    # Noisy
                    self._throttle_input_unsmooth = 0
                else:
                    # Interpret high time as direction
                    if high_time > self._controller_throttle_neutral:
                        # Backward (down) normalization
                        self._throttle_input_unsmooth = (high_time - self._controller_throttle_neutral) / (self._controller_throttle_neutral - self._controller_throttle_backward)
                        self._throttle_input_unsmooth = max(self._throttle_input_unsmooth, -1.0)  # Clamp to -1.0
                    
                    elif high_time < self._controller_throttle_neutral:
                        # Forward (up) normalization
                        self._throttle_input_unsmooth = (high_time - self._controller_throttle_neutral) / (self._controller_throttle_forward - self._controller_throttle_neutral)  # Normalized to forward limit
                        # self._throttle_input_unsmooth = (high_time - self._controller_throttle_neutral) / (self._controller_throttle_neutral - self._controller_throttle_backward)  # Normalized to backward limit
                        self._throttle_input_unsmooth = min(self._throttle_input_unsmooth, 1.0)  # Clamp to +1.0
                    
                    else:
                        self._throttle_input_unsmooth = 0  # Neutral
                
                # Deadzone
                if abs(self._throttle_input_unsmooth) < self._deadzone:
                    self._throttle_input_unsmooth = 0
                    
                # EMA
                self.throttle_input = (self._ema_alpha_throttle * self._throttle_input_unsmooth) + ((1 - self._ema_alpha_throttle) * self.throttle_input)
                
                self._throttle_last_input_tick = now
                
        return self.throttle_input
    
    def transmit_throttle_payload(self):
        self.throttle_bit_channel &= 0b1
        self.throttle_bit_dir &= 0b1
        self.throttle_bits_speed &- 0b111111
        
        self.throttle_bits_payload = (self.throttle_bit_channel << 7) | (self.throttle_bit_dir << 6) | self.throttle_bits_speed
        self.throttle_bits_payload_str = self.throttle_bits_payload.to_bytes(1, byteorder='big')
        
        self.driver.write(self.throttle_bits_payload_str)
        
    def middle_zone(self):
        self.throttle_bit_dir = 0 if self.throttle_input > 0 else 1
        self.throttle_bits_speed = round(abs(self.throttle_input * self._max_allowed_bits_speed))
        
    def kill_program(self):
        self.throttle_bits_speed = 0
        self.transmit_throttle_payload()
        
        self.disconnect_devices()
        
    def log_events(self):
        self.logger.debug(f"Controller throttle input (unsmoothed): {self._throttle_input_unsmooth}")
        self.logger.debug(f"Controller throttle input (smoothed): {self.throttle_input}")
        self.logger.debug(f"Throttle high time: {median(self.throttle_high_time_dq)}")
        self.logger.debug(f"Throttle motor speed value: {self.throttle_bits_speed}")
        self.logger.debug(f"Throttle motor direction: {self.throttle_bit_dir}")
        
class MicrosecondFormatter(logging.Formatter):
    """
    Customer formatter to include microsecond-precision timestamps.
    """
    def formatTime(self, record, datefmt=None):
        dt = datetime.fromtimestamp(record.created)
        if datefmt:
            return dt.strftime(datefmt)
        return dt.strftime("%Y-%m-%d %H:%M:%S.%f")

def setup_logging():
    """
    Configures the logger.
    """
    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG)
    
    # Console handler
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)
    
    # Rotating file handler (5 files, 100 MB each)
    file_handler = RotatingFileHandler("logs/system.log", maxBytes=100_000_000, backupCount=5)
    file_handler.setLevel(logging.DEBUG)
    
    # Formatter
    formatter = MicrosecondFormatter(
        fmt="%(asctime)s [ %(levelname)s ] %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S.%f"
    )
    
    # Apply to handlers
    console_handler.setFormatter(formatter)
    file_handler.setFormatter(formatter)
    
    logger.addHandler(console_handler)
    logger.addHandler(file_handler)
    
def create_signal_handler(tower):
    def handler(sig, frame):
        logging.warning("Ctrl+C caught. Closing program.")
        tower.kill_program()
        sys.exit(0)
    return handler

def main():
    os.makedirs("logs", exist_ok=True)
    setup_logging()
    logging.info("System startup")
    
    tower = Tower()
    
    signal.signal(signal.SIGINT, create_signal_handler(tower))
    
    while True:
        tower_input = tower.get_controller_throttle_command()
        
        if tower_input:
            tower.middle_zone()
        else:
            tower.throttle_bits_speed = 0
        
        tower.transmit_throttle_payload()
        
        tower.log_events()
        
if __name__ == "__main__":
    main()