"""
Need to re-test foot pedals, and modify controller to be able to connect to it.
"""

import pigpio
import logging
import threading
from collections import deque
from statistics import median
from Event_Class import Zero_Button_Event

class Tower:
    """
    Class controlling tower via RC controller.
    """
    def __init__(self, pi):
        """
        Class initialization.
        """
        # === Constants === #
        # Controller
        self._controller_timeout = 1_000_000  # Timeout threshold (µs)
        self._input_timeout =      3_000_000  # Input timeout threshold (µs)
        self._input_min =          0.01       # Minimum input value that will be acted on
        self._deadzone =           0.3        # Deadzone (% of controller input)
        self._pedals_connected =   False      # Indicates if the pedals are connected based on the channel 3 state. False/0 for off | True/1 for on
        self._ema_alpha_throttle = 0.5        # Throttle smoothing factor. Lower numbers are smooth, higher numbers are more responsive
        
        # PWM limits
        self._controller_throttle_neutral =  [1500, 1975]  # Neutral throttle point [Controller, Foot Pedals]
        self._controller_throttle_forward =  [1980, 2125]  # Forward throttle limit [Controller, Foot Pedals]
        self._controller_throttle_backward = [1000, 1195]  # Backward throttle limit [Controller, Foot Pedals]
        
        self._controller_channel_left =  875   # Left channel limit (channels 5 and 6 are the same value)
        self._controller_channel_right = 2125  # Right channel limit (channels 5 and 6 are the same value)
        
        self._controller_channel3_off = 1270  # Channel 3 off value
        self._controller_channel3_on =  1760  # Channel 3 on value
        
        # Threading locks
        self._throttle_lock = threading.Lock()  # Prevent race conditions on the throttle calculations
        self._channel3_lock = threading.Lock()  # Prevent race conditions on the channel 3 calculations
        self._channel6_lock = threading.Lock()  # Prevent race conditions on the channel 6 calculations
        
        # Zero button
        self.active_zone = -1  # Tracks what zone is in use
        self._zero_button_tripped = False
        self.on_zero_button_change = Zero_Button_Event()
        
        # Travel rate
        self._speed_limiter = 0.3  # Limits the speeds at ends of tower travel
        self._max_allowed_tower_speed = 0
        
        # Minimum speeds
        self._last_min_hold_speed = 0  # Track the last minimum hold speed value
        self.min_move_speed = 0  # Minimum speed that will move the tower
        self.min_hold_speed = 0  # Minimum speed value that will hold the tower stationary
        # === #
        
        # === Pi GPIO Pins (Broadcom numbers) === #
        self._pin_controller_throttle = 5  # PWM input from RC controller throttle level
        self._pin_controller_channel6 = 8  # PWM input from RC controller channel 6 knob
        self._pin_zero_button = 16  # Zero position button
        self._pin_controller_channel3 = 23  # PWM input from RC controller channel 3 button
        self._pin_led = 26  # LED output pin
        # === #
        
        # === Deques === #
        # --- All sizes limited to 7 --- #
        self.throttle_dq = deque([self._controller_throttle_neutral[self._pedals_connected] for _ in range(7)], maxlen=7)  # Deque for throttle high time
        self.channel3_dq = deque([self._controller_channel3_off for _ in range(7)], maxlen=7)  # Deque for channel 3 high time
        self.channel6_dq = deque([self._controller_channel_left for _ in range(7)], maxlen=7)  # Deque for channel 6 high time
        # === #
        
        # === Runtime Variables === #
        # Throttle PWM signal variables
        # --- Throttle controls the tower
        self._throttle_last_edge_tick = None
        self._throttle_last_rising_tick = None
        self._throttle_last_period_tick = None
        self._throttle_high_time = 0
        self._throttle_period = 0
        self._throttle_input_unsmooth = 0  # Holds unsmoothed throttle input
        self.throttle_input = 0  # -1 <= x < 0 for down | x = 0 for neutral | 0 < x <= 1 for up
        
        # Channel 3 PWM signal variables
        # --- Channel 3 determines if the foot pedals are connected
        self._channel3_last_edge_tick = None
        self._channel3_last_rising_tick = None
        self._channel3_last_period_tick = None
        self._channel3_period = 0
        
        # Channel 6 PWM signal variables
        # --- Channel 6 determines the max travel speed of the tower
        self._channel6_last_edge_tick = None
        self._channel6_last_rising_tick = None
        self._channel6_last_period_tick = None
        self._channel6_period = 0
        # === #
        
        # === Logger Config === #
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.info("Tower class initialized")
        # === #
        
        # === Connect Devices === #
        self.connect_devices(pi)
        # === #
        
        # === Pigpio current ticks === #
        self._min_hold_last_tick = self.pi.get_current_tick()  # Tracks the last time the minimum hold speed value was changed
        self._throttle_last_input_tick = self.pi.get_current_tick()  # Tracks the last time there was an active throttle input
        
    # === Zero Button Event Handler === #
    @property
    def zero_button_tripped(self):
        return self._zero_button_tripped
    
    @zero_button_tripped.setter
    def zero_button_tripped(self, value):
        if self._zero_button_tripped != value:
            self._zero_button_tripped = value
            self.on_zero_button_change.notify(value)
            self.logger.info("Zero button tripped. Notifying encoder class.")
    # === #
    
    # === Connect to Devices === #
    def connect_devices(self, pi):
        """
        Master function for connecting to GPIO devices.
        """
        # Connect to pigpio daemon
        self.pi = pi
        
        # Run connector functions
        self._connect_hand_controller()
        self._connect_zero_button()
        self._connect_LED()
        
        self.logger.info("All peripherals connected")
        
    def disconnect_devices(self):
        """
        Close class connections.
        """
        # Cancel callbacks
        self._cb_controller_throttle.cancel()  # Cancel controller throttle callback
        self._cb_controller_channel3.cancel()  # Cancel controller channel 3 callback
        self._cb_controller_channel6.cancel()  # Cancel controller channel 6 callback
        self._cb_zero_button.cancel()  # Cancel zero button callback
        
        self.logger.info("All tower callbacks cancelled")
        
    def _connect_hand_controller(self):
        """
        Connects to the hand controller.
        """
        # Configure pin input and pull-down resistor for pins
        for pin in [
            self._pin_controller_throttle,
            self._pin_controller_channel3,
            self._pin_controller_channel6
        ]:
            self.pi.set_mode(pin, pigpio.INPUT)
            self.pi.set_pull_up_down(pin, pigpio.PUD_DOWN)
            self.pi.set_glitch_filter(pin, 100)  # Ignores edges shorter than 100 μs
            
        # Register callbacks
        self._cb_controller_throttle = self.pi.callback(self._pin_controller_throttle, pigpio.EITHER_EDGE, self._handle_controller_throttle_pwm)
        self._cb_controller_channel3 = self.pi.callback(self._pin_controller_channel3, pigpio.EITHER_EDGE, self._handle_controller_channel3_pwm)
        self._cb_controller_channel6 = self.pi.callback(self._pin_controller_channel6, pigpio.EITHER_EDGE, self._handle_controller_channel6_pwm)
        
        self.logger.info("Hand controller pins and callbacks configured")
        
    def _connect_zero_button(self):
        """
        Connects to the zero-position button.
        """
        # Configure button input
        self.pi.set_mode(self._pin_zero_button, pigpio.INPUT)
        self.pi.set_pull_up_down(self._pin_zero_button, pigpio.PUD_DOWN)
        self.pi.set_glitch_filter(self._pin_zero_button, 50_000)  # Ignores edges shorter than 50,000 μs (50 ms)
        
        # Register callback
        self._cb_zero_button = self.pi.callback(self._pin_zero_button, pigpio.EITHER_EDGE, self._handle_zero_button)
        
        self.logger.info("Zero button pin and callback configured")
        
    def _connect_LED(self):
        """
        Connects to the LED.
        """
        self.pi.set_mode(self._pin_led, pigpio.OUTPUT)
        
        self.logger.info("LED pin configured")
    # === #
    
    # === Callback Handlers === #
    def _handle_controller_throttle_pwm(self, gpio, level, tick):
        """
        Gets the controller throttle input from PWM signal changes.
        """
        with self._throttle_lock:
            self._throttle_last_edge_tick = tick
            
            if level == 1:  # Rising edge
                if self._throtle_last_period_tick is not None:
                    self._throttle_period = pigpio.tickDiff(self._throttle_last_period_tick, tick)
                self._throttle_last_period_tick = tick
                self._throttle_last_rising_tick = tick
            elif level == 0:  # Falling edge
                if self._throttle_last_rising_tick is not None:
                    pulse_width = pigpio.tickDiff(self._throttle_last_rising_tick, tick)
                    if 1000 <= pulse_width <= 2200:
                        self.throttle_dq.append(pulse_width)
                    else:
                        self.logger.debug(f"Ignored invalid throttle pulse width: {pulse_width}")
                        
    def _handle_controller_channel3_pwm(self, gpio, level, tick):
        """
        Gets the controller channel 3 input from PWM signal changes.
        """
        with self._channel3_lock:
            self._channel3_last_edge_tick = tick
            
            if level == 1:  # Rising edge
                if self._channel3_last_period_tick is not None:
                    self._channel3_period = pigpio.tickDiff(self._channel3_last_period_tick, tick)
                self._channel3_last_period_tick = tick
                self._channel3_last_rising_tick = tick
            elif level == 0:  # Falling edge
                if self._channel3_last_rising_tick is not None:
                    pulse_width = pigpio.tickDiff(self._channel3_last_rising_tick, tick)
                    if 850 <= pulse_width <= 2000:
                        self.channel3_dq.append(pulse_width)
                    else:
                        self.logger.debug(f"Ignored invalid channel3 pulse width: {pulse_width}")
                        
    def _handle_controller_channel6_pwm(self, gpio, level, tick):
        """
        Gets the controller channel 6 input from PWM signal changes.
        """
        with self._channel6_lock:
            self._channel6_last_edge_tick = tick
            
            if level == 1:  # Rising edge
                if self._channel6_last_period_tick is not None:
                    self._channel6_period = pigpio.tickDiff(self._channel6_last_period_tick, tick)
                self._channel6_last_period_tick = tick
                self._channel6_last_rising_tick = tick
            elif level == 0:  # Falling edge
                if self._channel6_last_rising_tick is not None:
                    pulse_width = pigpio.tickDiff(self._channel6_last_rising_tick, tick)
                    if 850 <= pulse_width <= 2200:
                        self.channel6_dq.append(pulse_width)
                    else:
                        self.logger.debug(f"Ignored invalid channel 6 pulse width: {pulse_width}")
                        
    def _handle_zero_button(self, gpio, level, tick):
        """
        Determines if zeor-limit button is tripped.
        """
        if level == 1:  # Button pressed
            self.zero_button_tripped = True
    # === #
    
    # === Logic Handlers === #
    def get_controller_throttle_command(self):
        """
        Determines the throttle PWM command based on values gathered by handler.
        """
        with self._throttle_lock:
            now = self.pi.get_current_tick()
            if self._throttle_last_edge_tick is None or pigpio.tickDiff(self._throttle_last_edge_tick, now) > self._controller_timeout:
                # No active signal
                self._throttle_high_time = self._throttle_period = self.throttle_input = 0
                
                self.logger.warning("No throttle signal from controller")
            
            elif self._throttle_period > 0:  # Active input signal
                # Handle noisy input
                high_time = median(self.throttle_dq)
                if self._controller_throttle_neutral[self._pedals_connected] - 25 <= high_time <= self._controller_throttle_neutral[self._pedals_connected] + 25:
                    # Inside "noisy" bounds
                    self._throttle_input_unsmooth = 0
                else:
                    # Interpret high time as direction
                    if high_time > self._controller_throttle_neutral[self._pedals_connected]:
                        # Upward normalization
                        self._throttle_input_unsmooth = (high_time - self._controller_throttle_neutral[self._pedals_connected]) / (self._controller_throttle_forward[self._pedals_connected] - self._controller_throttle_neutral[self._pedals_connected])
                        # if not self._pedals_connected:
                        #     # Controller input
                        #     self._throttle_input_unsmooth = (high_time - self._controller_throttle_neutral[self._pedals_connected]) / (self._controller_throttle_backward[self._pedals_connected] - self._controller_throttle_neutral[self._pedals_connected])
                        # else:
                        #     # Foot pedals input
                        #     self._throttle_input_unsmooth = (high_time - self._controller_throttle_neutral[self._pedals_connected]) / (self._controller_throttle_forward[self._pedals_connected] - self._controller_throttle_neutral[self._pedals_connected])
                            
                        # Clamp to 1.0
                        self._throttle_input_unsmooth = min(self._throttle_input_unsmooth, 1.0)
                    
                    elif high_time < self._controller_throttle_neutral[self._pedals_connected]:
                        # Downward normalization
                        self._throttle_input_unsmooth = (high_time - self._controller_throttle_neutral[self._pedals_connected]) / (self._controller_throttle_neutral[self._pedals_connected] - self._controller_throttle_backward[self._pedals_connected])
                        # if not self._pedals_connected:
                        #     # Controller input
                        #     self._throttle_input_unsmooth = (high_time - self._controller_throttle_neutral[self._pedals_connected]) / (self._controller_throttle_neutral[self._pedals_connected] - self._controller_throttle_forward[self._pedals_connected])
                        # else:
                        #     # Foot pedals input
                        #     self._throttle_input_unsmooth = (high_time - self._controller_throttle_neutral[self._pedals_connected]) / (self._controller_throttle_neutral[self._pedals_connected] - self._controller_throttle_backward[self._pedals_connected])
                        
                        # Clamp -1.0
                        self._throttle_input_unsmooth = max(self._throttle_input_unsmooth, -1.0)
                    
                    else:
                        # Neutral command
                        self._throttle_input_unsmooth = 0  # Neutral
                        
                # Handle deadzone
                if abs(self._throttle_input_unsmooth) < self._deadzone:
                    self._throttle_input_unsmooth = 0
                    
                # Apply exponential moving average
                self.throttle_input = (
                    self._ema_alpha_throttle * self._throttle_input_unsmooth +
                    (1 - self._ema_alpha_throttle) * self.throttle_input
                )
                
                # Handle very small inputs
                if abs(self.throttle_input) < self._input_min:
                    self.throttle_input = 0
                    
                # Save input time (if input != 0)
                self._throttle_last_input_tick = now
        
        return self.throttle_input
    
    def get_controller_channel3_command(self):
        """
        Determines if the foot pedals are connected.
        """
        pulse = median(self.channel3_dq)
        match pulse:
            case _ if pulse > 1700:
                # Button on
                self._pedals_connected = True
            case _ if pulse > 1200:
                # Button off
                self._pedals_connected = False
            case _:
                # Controller off
                self._pedals_connected = False
                
    def get_controller_channel6_command(self):
        """
        Determines the max travel speed of the tower.
        """
        pulse = median(self.channel6_dq)
        
        # Handle > 8000 µs
        if pulse > 8000:
            pulse = pulse // 10
            
        self._max_allowed_tower_speed = (pulse - self._controller_channel_left) / (self._controller_channel_right - self._controller_channel_left)
        
    def get_input_averages(self):
        """
        Updates input values based on moving averages.
        """
        # Collect controller input commands
        self.get_controller_throttle_command()
        self.get_controller_channel3_command()
        self.get_controller_channel6_command()
        
        return self.throttle_input, None
    # === #
    
    # === Tower Zone Handlers === #
    def position_hold(self, travel_rate):
        """
        Holds position when no throttle input is received.
        """
        # Check if minimum hold speed needs to change
        now = self.pi.get_current_tick()
        if pigpio.tickDiff(self._throttle_last_input_tick, now) > self._input_timeout:  # Checks if no input for longer than timeout period
            if now - self._min_hold_last_tick > 2_000_000:  # Only updates once every 2 seconds (2_000_000 μs)
                if travel_rate > 0:  # Tower is moving up
                    if self.min_hold_speed > 0:  # No negative speeds
                        self.min_hold_speed -= 0.01  # Decrease speed by 1%
                        self._min_hold_last_tick = now
                elif travel_rate < 0:  # Tower is moving down
                    self.min_hold_speed += 0.01  # Increase speed by 1%
                    self._min_hold_last_tick = now
        
        self.active_zone = 0
        self.logger.info("Holding tower position")
        return self.min_hold_speed
    
    def under_lower_zone(self):
        """
        Handles the tower being under the travel region.
        """
        self.active_zone = 1
        self.logger.warning("Tower under the lower zone")
        return self._max_allowed_tower_speed
    
    def lower_zone(self, position):
        """
        Handles the tower being in the lower 10% of its travel region.
        """
        self.active_zone = 2
        self.logger.info(f"Tower in lower zone, {'moving up' if self.throttle_input > 0 else 'moving down' if (self.throttle_input < 0 and position > 0) else 'holding'}")
        
        if self.throttle_input > 0:  # Moving up in lower zone
            return self.throttle_input * self._max_allowed_tower_speed
        elif self.throttle_input < 0 and position > 0:  # Moving down in lower zone, above zero
            return self._speed_limiter * self.throttle_input * self._max_allowed_tower_speed
        else:
            return 0
        
    def middle_zone(self):
        """
        Handles the tower being in the middle of its travel region (10% - 90%).
        """
        self.active_zone = 3
        self.logger.info(f"Tower in middle zone, moving {'up' if self.throttle_input > 0 else 'down'}")
        
        return self.throttle_input * self._max_allowed_tower_speed
    
    def upper_zone(self, position, max_position):
        """
        Handles the tower being in the upper 10% of its travel region.
        """
        self.active_zone = 4
        self.logger.info(f"Tower in upper zone, {'moving up' if self.throttle_input < 0 else 'moving up' if (self.throttle_input > 0 and position < max_position) else 'holding'}")
        
        if self.throttle_input < 0:  # Moving down in upper zone
            return self.throttle_input * self._max_allowed_tower_speed
        elif self.throttle_input > 0 and position < max_position:  # Moving up in upper zone, below max point
            return self._speed_limiter * self.throttle_input * self._max_allowed_tower_speed
        else:
            return 0
        
    def above_upper_zone(self):
        """
        Handles the tower being above its travel region.
        """
        self.active_zone = 5
        self.logger.warning("Tower above the upper zone")
        return -1.0 * self._max_allowed_tower_speed
    # === #
    
    # === Helper Functions === #
    def get_min_hold_speed(self):
        """
        Returns the current minimum hold speed.
        """
        return self.min_hold_speed
    # === #
    
    # === Logging === #
    def log_debug_values(self):
        """
        Logs all debug values.
        """
        # Log each loop
        self.logger.debug(f"Throttle input (unsmoothed): {self._throttle_input_unsmooth}")
        self.logger.debug(f"Throttle input (smoothed): {self.throttle_input}")
        self.logger.debug(f"Throttle high time: {median(self.throttle_dq)}")
        
        self.logger.debug(f"Pedals connected: {self._pedals_connected}")
        self.logger.debug(f"Max tower speed: {self._max_allowed_tower_speed}")
        
        # Log when changed
        if self.min_hold_speed != self._last_min_hold_speed:  # Minimum hold speed changed
            self.logger.debug(f"Minimum hold speed set to: {self.min_hold_speed}")
            self._last_min_hold_speed = self.min_hold_speed
        if self._zero_button_tripped:  # Zero button tripped
            self.logger.debug("Zero button tripped")
            self.zero_button_tripped = False
    # === #

class Tower_with_sled:
    """
    Class controlling tower and sled via RC controller.
    """
    def __init__(self, pi):
        """
        Class initialization.
        """
        # === Constants === #
        # Controller
        self._controller_timeout = 1_000_000  # Timeout threshold (µs)
        self._input_timeout = 3_000_000  # Input timeout threshold (µs)
        self._input_min = 0.01  # Minimum input value that will be acted on
        self._deadzone = 0.3  # Deadzone (% of controller input)
        self._steering_direction = 1  # Tracks which direction the sled moves per given steering input direction (1 or -1 multiplier)
        self._pedals_connected = False  # Indicates if the pedals are connected based on the channel 3 state. False/0 for off | True/1 for on
        self._ema_alpha_throttle = 0.5  # Throttle smoothing factor. Lower numbers are smoother, higher numbers are more responsive
        self._ema_alpha_steering = 0.5  # Steering smoothing factor. Lower numbers are smoother, higher numbers are more responsive

        # PWM limits
        self._controller_throttle_neutral = [1500, 1975]  # Neutral throttle point [Controller, Foot Pedals]
        self._controller_throttle_forward = [1980, 2125]  # Forward throttle limit [Controller, Foot Pedals]
        self._controller_throttle_backward = [1000, 1195]  # Backward throttle limit [Controller, Foot Pedals]
        
        self._controller_steering_neutral = 1512  # Neutral steering point
        self._controller_steering_forward = 2012  # Forward steering limit
        self._controller_steering_backward = 1012  # Backward steering limit
        
        self._controller_channel_left = 875  # Left channel limit (channels 5 and 6 are the same value)
        self._controller_channel_right = 2125  # Right channel limit (channels 5 and 6 are the same value)
        
        self._controller_channel3_off = 1270  # Channel 3 off value
        self._controller_channel3_on = 1760  # Channel 3 on value
        
        self._controller_channel4_1 = 1270  # Channel 4 position 1 value
        self._controller_channel4_2 = 1515  # Channel 4 position 2 value
        self._controller_channel4_3 = 1760  # Channel 4 position 3 value

        # Threading locks
        self._throttle_lock = threading.Lock()  # Prevent race conditions on the throttle calculations
        self._steering_lock = threading.Lock()  # Prevent race conditions on the steering calculations
        self._channel3_lock = threading.Lock()  # Prevent race conditions on the channel 3 calculations
        self._channel4_lock = threading.Lock()  # Prevent race conditions on the channel 4 calculations
        self._channel5_lock = threading.Lock()  # Prevent race conditions on the channel 5 calculations
        self._channel6_lock = threading.Lock()  # Prevent race conditions on the channel 6 calculations

        # Zero button
        self.active_zone = -1  # Tracks what zone is in use
        self._zero_button_tripped = False
        self.on_zero_button_change = Zero_Button_Event()

        # Travel rate
        self._speed_limiter = 0.3  # Limits the speeds at ends of tower travel
        self._max_allowed_tower_speed = 0
        self._max_allowed_sled_speed = 0
        
        # Minimum speeds
        self._last_min_hold_speed = 0  # Track the last minimum hold speed value
        self.min_move_speed = 0  # Minimum speed value that will move the tower
        self.min_hold_speed = 0  # Minimum speed value that will hold the tower stationary
        # === #

        # === Pi GPIO Pins (Broadcom numbers) === #
        self._pin_controller_throttle = 5  # PWM input from RC controller throttle lever
        self._pin_controller_steering = 6  # PWM input from RC controller steering wheel
        self._pin_controller_channel6 = 8  # PWM input from RC controller channel 6 knob
        self._pin_zero_button = 16  # Zero position button
        self._pin_controller_channel3 = 23  # PWM input from RC controller channel 3 button
        self._pin_controller_channel4 = 24  # PWM input from RC controller channel 4 switch
        self._pin_controller_channel5 = 25  # PWM input from RC cnotroller channel 5 knob
        self._pin_led = 26  # LED output pin
        # === #
        
        # === Deques === #
        # --- All sizes limited to 7 --- #
        self.throttle_dq = deque([self._controller_throttle_neutral[self._pedals_connected] for _ in range(7)], maxlen = 7)  # Deque for throttle high time
        self.steering_dq = deque([self._controller_steering_neutral for _ in range(7)], maxlen = 7)  # Deque for steering high time
        self.channel3_dq = deque([self._controller_channel3_off for _ in range(7)], maxlen = 7)  # Deque for channel 3 high time
        self.channel4_dq = deque([self._controller_channel4_1 for _ in range(7)], maxlen = 7)  # Deque for channel 4 high time
        self.channel5_dq = deque([self._controller_channel_left for _ in range(7)], maxlen = 7)  # Deque for channel 5 high time
        self.channel6_dq = deque([self._controller_channel_left for _ in range(7)], maxlen = 7)  # Deque for channel 6 high time
        # === #
        
        # === Runtime Variables === #
        # Throttle PWM signal variables
        # --- Throttle controls the tower
        self._throttle_last_edge_tick = None
        self._throttle_last_rising_tick = None
        self._throttle_last_period_tick = None
        self._throttle_high_time = 0
        self._throttle_period = 0
        self._throttle_input_unsmooth = 0  # Holds unsmoothed throttle input
        self.throttle_input = 0  # -1 <= x < 0 for down | x = 0 for neutral | 0 < x <= 1 for up
        
        # Steering PWM signal variables
        # --- Steering controls the sled
        self._steering_last_edge_tick = None
        self._steering_last_rising_tick = None
        self._steering_last_period_tick = None
        self._steering_high_time = 0
        self._steering_period = 0
        self._steering_input_unsmooth = 0
        self.steering_input = 0  # -1 <= x < 0 for left | x = 0 for neutral | 0 < x <= 1 for right
        
        # Channel 3 PWM signal variables
        # --- Channel 3 determines if the foot pedals are connected
        self._channel3_last_edge_tick = None
        self._channel3_last_rising_tick = None
        self._channel3_last_period_tick = None
        self._channel3_period = 0
        
        # Channel 4 PWM signal variables
        # --- Channel 4 determines the direction the sled will move with a given steering input
        self._channel4_last_edge_tick = None
        self._channel4_last_rising_tick = None
        self._channel4_last_period_tick = None
        self._channel4_period = 0
        
        # Channel 5 PWM signal variables
        # --- Channel 5 determines the max travel speed of the sled
        self._channel5_last_edge_tick = None
        self._channel5_last_rising_tick = None
        self._channel5_last_period_tick = None
        self._channel5_period = 0
        
        # Channel 6 PWM signal variables
        # --- Channel 6 determines the max travel speed of the tower
        self._channel6_last_edge_tick = None
        self._channel6_last_rising_tick = None
        self._channel6_last_period_tick = None
        self._channel6_period = 0
        # === #
        
        # === Logger Config === #
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.info("Tower with sled class intialized")
        # === #
        
        # === Connect Devices === #
        self.connect_devices(pi)
        # === #
        
        # === Pigpio current ticks === #
        self._min_hold_last_tick = self.pi.get_current_tick()  # Tracks the last time the minimum hold speed value was changed
        self._throttle_last_input_tick = self.pi.get_current_tick()  # Tracks the last time there was an active throttle input
        
    # === Zero Button Event Handler === #
    @property
    def zero_button_tripped(self):
        return self._zero_button_tripped
    
    @zero_button_tripped.setter
    def zero_button_tripped(self, value):
        if self._zero_button_tripped != value:
            self._zero_button_tripped = value
            self.on_zero_button_change.notify(value)
            self.logger.info("Zero button tripped. Notifying encoder class.")
    # === #
        
    # === Connected to Devices === #
    def connect_devices(self, pi):
        """
        Master function for connecting to GPIO devices.
        """
        # Connect to pigpio daemon
        self.pi = pi
        
        # Run connector functions
        self._connect_hand_controller()
        self._connect_zero_button()
        self._connect_LED()
        
        self.logger.info("All peripherals connected")
        
    def disconnect_devices(self):
        """
        Close class connections.
        """
        # Cancel callbacks
        self._cb_controller_throttle.cancel()  # Cancel controller throttle callback
        self._cb_controller_steering.cancel()  # Cancel controller steering callback
        self._cb_controller_channel3.cancel()  # Cancel controller channel 3 callback
        self._cb_controller_channel4.cancel()  # Cancel controller channel 4 callback
        self._cb_controller_channel5.cancel()  # Cancel controller channel 5 callback
        self._cb_controller_channel6.cancel()  # Cancel controller channel 6 callback
        self._cb_zero_button.cancel()  # Cancel zero button callback
        
        self.logger.info("All tower callbacks cancelled")
        
    def _connect_hand_controller(self):
        """
        Connects to the hand controller.
        """
        # Configure pin input and pull-down resistor for pins
        for pin in [
            self._pin_controller_throttle,
            self._pin_controller_steering,
            self._pin_controller_channel3,
            self._pin_controller_channel4,
            self._pin_controller_channel5,
            self._pin_controller_channel6
        ]:
            self.pi.set_mode(pin, pigpio.INPUT)
            self.pi.set_pull_up_down(pin, pigpio.PUD_DOWN)
            self.pi.set_glitch_filter(pin, 100)  # Ignores edges shorter than 100 μs
            
        # Register callbacks
        self._cb_controller_throttle = self.pi.callback(self._pin_controller_throttle, pigpio.EITHER_EDGE, self._handle_controller_throttle_pwm)
        self._cb_controller_steering = self.pi.callback(self._pin_controller_steering, pigpio.EITHER_EDGE, self._handle_controller_steering_pwm)
        self._cb_controller_channel3 = self.pi.callback(self._pin_controller_channel3, pigpio.EITHER_EDGE, self._handle_controller_channel3_pwm)
        self._cb_controller_channel4 = self.pi.callback(self._pin_controller_channel4, pigpio.EITHER_EDGE, self._handle_controller_channel4_pwm)
        self._cb_controller_channel5 = self.pi.callback(self._pin_controller_channel5, pigpio.EITHER_EDGE, self._handle_controller_channel5_pwm)
        self._cb_controller_channel6 = self.pi.callback(self._pin_controller_channel6, pigpio.EITHER_EDGE, self._handle_controller_channel6_pwm)
        
        self.logger.info("Hand controller pins and callbacks configured")
        
    def _connect_zero_button(self):
        """
        Connects to the zero-position button.
        """
        # Configure button input
        self.pi.set_mode(self._pin_zero_button, pigpio.INPUT)
        self.pi.set_pull_up_down(self._pin_zero_button, pigpio.PUD_DOWN)
        self.pi.set_glitch_filter(self._pin_zero_button, 50_000)  # Ignores edges shorter than 50,000 μs (50 ms)
        
        # Register callback
        self._cb_zero_button = self.pi.callback(self._pin_zero_button, pigpio.EITHER_EDGE, self._handle_zero_button)
        
        self.logger.info("Zero button pin and callback configured")
        
    def _connect_LED(self):
        """
        Connects to the LED.
        """
        self.pi.set_mode(self._pin_led, pigpio.OUTPUT)
        
        self.logger.info("LED pin configured")
    # === #
    
    # === Callback Handlers === #
    def _handle_controller_throttle_pwm(self, gpio, level, tick):
        """
        Gets the controller throttle input from PWM signal changes.
        """
        with self._throttle_lock:
            self._throttle_last_edge_tick = tick
            
            if level == 1:  # Rising edge
                if self._throttle_last_period_tick is not None:
                    self._throttle_period = pigpio.tickDiff(self._throttle_last_period_tick, tick)
                self._throttle_last_period_tick = tick
                self._throttle_last_rising_tick = tick
            elif level == 0:  # Falling edge
                if self._throttle_last_rising_tick is not None:
                    pulse_width = pigpio.tickDiff(self._throttle_last_rising_tick, tick)
                    if 1000 <= pulse_width <= 2200:
                        self.throttle_dq.append(pulse_width)
                    else:
                        self.logger.debug(f"Ignored invalid throttle pulse width: {pulse_width}")
                        
    def _handle_controller_steering_pwm(self, gpio, level, tick):
        """
        Gets the controller steering input from PWM signal changes.
        """
        with self._steering_lock:
            self._steering_last_edge_tick = tick
            
            if level == 1:  # Rising edge
                if self._steering_last_period_tick is not None:
                    self._steering_period = pigpio.tickDiff(self._steering_last_period_tick, tick)
                self._steering_last_period_tick = tick
                self._steering_last_rising_tick = tick
            elif level == 0:  # Falling edge
                if self._steering_last_rising_tick is not None:
                    pulse_width = pigpio.tickDiff(self._steering_last_rising_tick, tick)
                    if 1000 <= pulse_width <= 2200:
                        self.steering_dq.append(pulse_width)
                    else:
                        self.logger.debug(f"Ignored invalid steering pulse width: {pulse_width}")

    def _handle_controller_channel3_pwm(self, gpio, level, tick):
        """
        Gets the controller channel 3 input from PWM signal changes.
        """
        with self._channel3_lock:
            self._channel3_last_edge_tick = tick
            
            if level == 1:  # Rising edge
                if self._channel3_last_period_tick is not None:
                    self._channel3_period = pigpio.tickDiff(self._channel3_last_period_tick, tick)
                self._channel3_last_period_tick = tick
                self._channel3_last_rising_tick = tick
            elif level == 0:  # Falling edge
                if self._channel3_last_rising_tick is not None:
                    pulse_width = pigpio.tickDiff(self._channel3_last_rising_tick, tick)
                    if 850 <= pulse_width <= 2000:
                        self.channel3_dq.append(pulse_width)
                    else:
                        self.logger.debug(f"Ignored invalid channel 3 pulse width: {pulse_width}")
                        
    def _handle_controller_channel4_pwm(self, gpio, level, tick):
        """
        Gets the controller channel 4 input from PWM signal changes.
        """
        with self._channel4_lock:
            self._channel4_last_edge_tick = tick
            
            if level == 1:  # Rising edge
                if self._channel4_last_period_tick is not None:
                    self._channel4_period = pigpio.tickDiff(self._channel4_last_period_tick, tick)
                self._channel4_last_period_tick = tick
                self._channel4_last_rising_tick = tick
            elif level == 0:  # Falling edge
                if self._channel4_last_rising_tick is not None:
                    pulse_width = pigpio.tickDiff(self._channel4_last_rising_tick, tick)
                    if 600 <= pulse_width <= 2000:
                        self.channel4_dq.append(pulse_width)
                    else:
                        self.logger.debug(f"Ignored invalid channel 4 pulse width: {pulse_width}")

    def _handle_controller_channel5_pwm(self, gpio, level, tick):
        """
        Gets the controller channel 5 input from PWM signal changes.
        """
        with self._channel5_lock:
            self._channel5_last_edge_tick = tick
            
            if level == 1:  # Rising edge
                if self._channel5_last_period_tick is not None:
                    self._channel5_period = pigpio.tickDiff(self._channel5_last_period_tick, tick)
                self._channel5_last_period_tick = tick
                self._channel5_last_rising_tick = tick
            elif level == 0:  # Falling edge
                if self._channel5_last_rising_tick is not None:
                    pulse_width = pigpio.tickDiff(self._channel5_last_rising_tick, tick)
                    if 850 <= pulse_width <= 2200:
                        self.channel5_dq.append(pulse_width)
                    else:
                        self.logger.debug(f"Ignored invalid channel 5 pulse width: {pulse_width}")
                        
    def _handle_controller_channel6_pwm(self, gpio, level, tick):
        """
        Gets the controller channel 6 input from PWM signal changes.
        """
        with self._channel6_lock:
            self._channel6_last_edge_tick = tick
            
            if level == 1:  # Rising edge
                if self._channel6_last_period_tick is not None:
                    self._channel6_period = pigpio.tickDiff(self._channel6_last_period_tick, tick)
                self._channel6_last_period_tick = tick
                self._channel6_last_rising_tick = tick
            elif level == 0:  # Falling edge
                if self._channel6_last_rising_tick is not None:
                    pulse_width = pigpio.tickDiff(self._channel6_last_rising_tick, tick)
                    if 850 <= pulse_width <= 2200:
                        self.channel6_dq.append(pulse_width)
                    else:
                        self.logger.debug(f"Ignored invalid channel 6 pulse width: {pulse_width}")
                        
    def _handle_zero_button(self, gpio, level, tick):
        """
        Determines if zero-limit button is tripped.
        """
        if level == 1:  # Button pressed
            self.zero_button_tripped = True
    # === #
    
    # === Logic Handlers === #
    def get_controller_throttle_command(self):
        """
        Determines the throttle PWM command based on values gathered by handler.
        """
        with self._throttle_lock:
            now = self.pi.get_current_tick()
            if self._throttle_last_edge_tick is None or pigpio.tickDiff(self._throttle_last_edge_tick, now) > self._controller_timeout:
                # No active signal
                self._throttle_high_time = self._throttle_period = self.throttle_input = 0
                
                self.logger.warning("No throttle signal from controller")
                
            elif self._throttle_period > 0:  # Active input signal
                # Handle noisy input
                high_time = median(self.throttle_dq)
                if self._controller_throttle_neutral[self._pedals_connected] - 25 <= high_time <= self._controller_throttle_neutral[self._pedals_connected] + 25:
                    # Inside "noisy" bounds
                    self._throttle_input_unsmooth = 0
                else:
                    # Interpret high time as direction
                    if high_time > self._controller_throttle_neutral[self._pedals_connected]:
                        # Upward normalization
                        self._throttle_input_unsmooth = (high_time - self._controller_throttle_neutral[self._pedals_connected]) / (self._controller_throttle_forward[self._pedals_connected] - self._controller_throttle_neutral[self._pedals_connected])
                        # if not self._pedals_connected:
                        #     # Controller input
                        #     self._throttle_input_unsmooth = (high_time - self._controller_throttle_neutral[self._pedals_connected]) / (self._controller_throttle_forward[self._pedals_connected] - self._controller_throttle_neutral[self._pedals_connected])
                        # else:
                        #     # Foot pedals input
                        #     self._throttle_input_unsmooth = (high_time - self._controller_throttle_neutral[self._pedals_connected]) / (self._controller_throttle_forward[self._pedals_connected] - self._controller_throttle_neutral[self._pedals_connected])
                            
                        # Clamp to 1.0
                        self._throttle_input_unsmooth = min(self._throttle_input_unsmooth, 1.0)
                        
                    elif high_time < self._controller_throttle_neutral[self._pedals_connected]:
                        # Downward normalization
                        self._throttle_input_unsmooth = (high_time - self._controller_throttle_neutral[self._pedals_connected]) / (self._controller_throttle_neutral[self._pedals_connected] - self._controller_throttle_backward[self._pedals_connected])
                        # if not self._pedals_connected:
                        #     # Controller input
                        #     self._throttle_input_unsmooth = (high_time - self._controller_throttle_neutral[self._pedals_connected]) / (self._controller_throttle_neutral[self._pedals_connected] - self._controller_throttle_backward[self._pedals_connected])
                        # else:
                        #     # Foot pedals input
                        #     self._throttle_input_unsmooth = (high_time - self._controller_throttle_neutral[self._pedals_connected]) / (self._controller_throttle_neutral[self._pedals_connected] - self._controller_throttle_backward[self._pedals_connected])

                        # Clamp to -1.0
                        self._throttle_input_unsmooth = max(self._throttle_input_unsmooth, -1.0)
                        
                    else:
                        # Neutral command
                        self._throttle_input_unsmooth = 0  # Neutral
                        
                # Handle deadzone
                if abs(self._throttle_input_unsmooth) < self._deadzone:
                    self._throttle_input_unsmooth = 0
                    
                # Apply exponential moving average
                self.throttle_input = (
                    self._ema_alpha_throttle * self._throttle_input_unsmooth +
                    (1 - self._ema_alpha_throttle) * self.throttle_input
                )
                
                # Handle very small inputs
                if abs(self.throttle_input) < self._input_min:
                    self.throttle_input = 0
                    
                # Save input time (if input != 0)
                self._throttle_last_input_tick = now
        
        return self.throttle_input
    
    def get_controller_steering_command(self):
        """
        Determines the steering PWM command based on values gathered by handler.
        """
        with self._steering_lock:
            now = self.pi.get_current_tick()
            if self._steering_last_edge_tick is None or pigpio.tickDiff(self._steering_last_edge_tick, now) > self._controller_timeout:
                # No active signal
                self._steering_high_time = self._steering_period = self._steering_input_unsmooth = 0
                
                self.logger.warning("No steering signal from controller")
                
            elif self._steering_period > 0:  # Active input signal
                # Handle noisy input
                high_time = median(self.steering_dq)
                if self._controller_steering_neutral - 25 <= high_time <= self._controller_steering_neutral + 25:
                    # Inside "noisy" bounds
                    self._steering_input_unsmooth = 0
                else:
                    # Interpret high time as direction
                    if high_time > self._controller_steering_neutral:
                        # Forward normalization
                        self._steering_input_unsmooth = (high_time - self._controller_steering_neutral) / (self._controller_steering_forward - self._controller_steering_neutral)
                        self._steering_input_unsmooth = min(self._steering_input_unsmooth, 1.0)  # Clamp 1.0
                    
                    elif high_time < self._controller_steering_neutral:
                        # Backward normalization
                        self._steering_input_unsmooth = (high_time - self._controller_steering_neutral) / (self._controller_steering_neutral - self._controller_steering_backward)
                        self._steering_input_unsmooth = max(self._steering_input_unsmooth, -1.0)  # Clamp to -1.0
                        
                    else:
                        self._steering_input_unsmooth = 0  # Neutral
                        
                # Handle deadzone
                if abs(self._steering_input_unsmooth) < self._deadzone:
                    self._steering_input_unsmooth = 0
                    
                # Apply exponential moving average
                self.steering_input = (
                    self._ema_alpha_steering * self._steering_input_unsmooth +
                    (1 - self._ema_alpha_steering) * self.steering_input
                )
        
        return self.steering_input
    
    def get_controller_channel3_command(self):
        """
        Determines if the foot pedals are connected.
        """
        pulse = median(self.channel3_dq)
        match pulse:
            case _ if pulse > 1700:
                # Button on
                self._pedals_connected = True
            case _ if pulse > 1200:
                # Button off
                self._pedals_connected = False
            case _:
                # Controller off
                self._pedals_connected = False
                
    def get_controller_channel4_command(self):
        """
        Determines the direction the sled will move with a given steering input.
        """
        pulse = median(self.channel4_dq)
        match pulse:
            case _ if pulse > 1700:
                # Position 3 - change direction
                self._steering_direction = -1
            case _ if pulse > 1500:
                # Position 2 - keep last value
                pass
            case _ if pulse > 1200:
                # Position 1 - change direction
                self._steering_direction = 1
            case _:
                # Invalid input - keep last value
                pass
                
    def get_controller_channel5_command(self):
        """
        Determines the max travel speed of the sled.
        """
        pulse = median(self.channel5_dq)

        # Handle > 8000 µs
        if pulse > 8000:
            pulse = pulse // 10

        self._max_allowed_sled_speed = (pulse - self._controller_channel_left) / (self._controller_channel_right - self._controller_channel_left)
        
    def get_controller_channel6_command(self):
        """
        Determines the max travel speed of the tower.
        """
        pulse = median(self.channel6_dq)

        # Handle > 8000 µs
        if pulse > 8000:
            pulse = pulse // 10

        self._max_allowed_tower_speed = (pulse - self._controller_channel_left) / (self._controller_channel_right - self._controller_channel_left)
        
    def get_input_averages(self):
        """
        Updates input values based on moving averages.
        """
        # Collect controller input commands
        self.get_controller_throttle_command()
        self.get_controller_steering_command()
        self.get_controller_channel3_command()
        self.get_controller_channel4_command()
        self.get_controller_channel5_command()
        self.get_controller_channel6_command()
        
        return self.throttle_input, (self.steering_input * self._steering_direction * self._max_allowed_sled_speed)
    # === #
    
    # === Tower Zone Handlers === #
    def position_hold(self, travel_rate):
        """
        Holds position when no throttle input is received.
        """
        # Check if minimum hold speed needs to change
        now = self.pi.get_current_tick()
        if pigpio.tickDiff(self._throttle_last_input_tick, now) > self._input_timeout:  # Checks if no input for longer than timeout period
            if now - self._min_hold_last_tick > 2_000_000:  # Only updates once every 2 seconds (2_000_000 µs)
                if travel_rate > 0:  # Tower is moving up
                    if self.min_hold_speed > 0:  # No negative speeds
                        self.min_hold_speed -= 0.01  # Decrease speed by 1%
                        self._min_hold_last_tick = now
                elif travel_rate < 0:  # Tower is moving down
                    self.min_hold_speed += 0.01  # Increase speed by 1%
                    self._min_hold_last_tick = now
                    
        self.active_zone = 0
        self.logger.info("Holding tower position")
        return self.min_hold_speed
    
    def under_lower_zone(self):
        """
        Handles the tower being under the travel region.
        """
        self.active_zone = 1
        self.logger.warning("Tower under the lower zone")
        return self._max_allowed_tower_speed
    
    def lower_zone(self, position):
        """
        Handles the tower being in the lower 10% of its travel region.
        """
        self.active_zone = 2
        self.logger.info(f"Tower in lower zone, {'moving up' if self.throttle_input > 0 else 'moving down' if (self.throttle_input < 0 and position > 0) else 'holding'}")
        
        if self.throttle_input > 0:  # Moving up in lower zone
            return self.throttle_input * self._max_allowed_tower_speed
        elif self.throttle_input < 0 and position > 0:  # Moving down in lower zone, above zero
            return self._speed_limiter * self.throttle_input * self._max_allowed_tower_speed
        else:
            return 0
        
    def middle_zone(self):
        """
        Handles the tower being in the middle of its travel region (10% - 90%).
        """
        self.active_zone = 3
        self.logger.info(f"Tower in middle zone, moving {'up' if self.throttle_input > 0 else 'down'}")
        
        return self.throttle_input * self._max_allowed_tower_speed
        
    def upper_zone(self, position, max_position):
        """
        Handles the tower being in the upper 10% of its travel region.
        """
        self.active_zone = 4
        self.logger.info(f"Tower in upper zone, {'moving down' if self.throttle_input < 0 else 'moving up' if (self.throttle_input > 0 and position < max_position) else 'holding'}")
        
        if self.throttle_input < 0:  # Moving down in upper zone
            return self.throttle_input * self._max_allowed_tower_speed
        elif self.throttle_input > 0 and position < max_position:  # Moving up in upper zone, below max point
            return self._speed_limiter * self.throttle_input * self._max_allowed_tower_speed
        else:
            return 0
        
    def above_upper_zone(self):
        """
        Handles the tower being above its travel region.
        """
        self.active_zone = 5
        self.logger.warning("Tower above the upper zone")
        return -1.0 * self._max_allowed_tower_speed
    # === #
    
    # === Helper Functions === #
    def get_min_hold_speed(self):
        """
        Returns the current minimum hold speed.
        """
        return self.min_hold_speed
    
    def get_steering_direction(self):
        """
        Returns the current steering direction value.
        """
        return self._steering_direction
    # === #
    
    # === Logging === #
    def log_debug_values(self):
        """
        Logs all debug values.
        """
        # Log each loop
        self.logger.debug(f"Throttle input (unsmoothed): {self._throttle_input_unsmooth}")
        self.logger.debug(f"Throttle input (smoothed): {self.throttle_input}")
        self.logger.debug(f"Throttle high time: {median(self.throttle_dq)}")
        
        self.logger.debug(f"Steering input (unsmoothed): {self._steering_input_unsmooth}")
        self.logger.debug(f"Steering input (smoothed): {self.steering_input}")
        self.logger.debug(f"Steering high time: {median(self.steering_dq)}")
        
        self.logger.debug(f"Pedals connected: {self._pedals_connected}")
        self.logger.debug(f"Steering direction: {self._steering_direction}")
        self.logger.debug(f"Max tower speed: {self._max_allowed_tower_speed}")
        self.logger.debug(f"Max sled speed: {self._max_allowed_sled_speed}")
        
        # Log when changed
        if self.min_hold_speed != self._last_min_hold_speed:  # Minimum hold speed changed 
            self.logger.debug(f"Minimum hold speed set to: {self.min_hold_speed}")
            self._last_min_hold_speed = self.min_hold_speed
        if self._zero_button_tripped:  # Zero button tripped
            self.logger.debug("Zero button tripped")
            self.zero_button_tripped = False
    # === #