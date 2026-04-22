# Python Libraries
import pigpio
import logging
import threading
from collections import deque
from statistics import median
from dataclasses import dataclass

# Project Modules
from pi_runtime.Event_Class import Zero_Button_Event

@dataclass(frozen=True)
class Constants:
    controller_timeout: int   = 1_000_000  # Timeout threshold in microseconds
    input_timeout:      int   = 3_000_000  # Input timeout threshold in microseconds
    deadzone:           float = 0.3        # Deadzone (% of controller input)
    input_min:          float = 0.01       # Minimum input value that will be acted on
    speed_limiter:      float = 0.3        # Limits the speed at ends of tower travel
    ema_alpha_throttle: float = 0.5        # Throttle smoothing factor. Lower numbers are smoother, higher numbers are more responsive
    ema_alpha_steering: float = 0.5        # Steering smoothing factor. Lower numbers are smoother, higher numbers are more resposnive

@dataclass
class RcInputState:
    throttle_input_unsmooth: float  # Normalized [-1.0, 1.0]
    steering_input_unsmooth: float  # Normalized [-1.0, 1.0]
    pedals_connected: bool
    steering_direction: int
    max_allowed_sled_speed: int
    max_allowed_tower_speed: int
    zero_button_tripped: bool
    now_tick: int                   # time.monotonic current tick value

class RcInputReader:
    def __init__(self, pi: pigpio.pi, constants: Constants):
        # === Constants === #
        # Pigpio
        self.pi = pi
        
        # Controller
        self._controller_timeout = constants.controller_timeout  # Timeout threshold (µs)
        self._input_timeout      = constants.input_timeout       # Input timeout threshold (µs)
        self._input_min          = constants.input_min           # Minimum input value that will be acted on
        self._deadzone           = constants.deadzone            # Deadzone (% of controller input)
        
        # PWM Limits
        self._throttle_neutral_us  = [1500, 1975]  # Neutral throttle point [Controller, Food Pedals]
        self._throttle_forward_us  = [1980, 2125]  # Forward throttle limit [Controller, Foot Pedals]
        self._throttle_backward_us = [1000, 1195]  # Backward throttle limit [Controller, Foot Pedals]
        
        self._steering_neutral_us  = 1512  # Neutral steering point
        self._steering_forward_us  = 2012  # Forward steering limit
        self._steering_backward_us = 1012  # Backward steering limit
        
        self._channel3_off_us = 1270  # Channel 3 off value 
        self._channel3_on_us  = 1760  # Channel 3 on value
        
        self._channel4_1_us = 1270  # Channel 4 position 1 value
        self._channel4_2_us = 1515  # Channel 4 position 2 value
        self._channel4_3_us = 1760  # Channel 4 position 3 value
        
        self._channel_left_us  = 875   # Left limit for channels 5 and 6
        self._channel_right_us = 2125  # Right limit for channels 5 and 6
        # === #
        
        # === Threading Locks === #
        self._lock_throttle = threading.Lock()  # Prevent race conditions on the throttle calculations
        self._lock_steering = threading.Lock()  # Prevent race conditions on the steering calculations
        self._lock_channel3 = threading.Lock()  # Prevent race conditions on the channel 3 calculations
        self._lock_channel4 = threading.Lock()  # Prevent race conditions on the channel 4 calculations
        self._lock_channel5 = threading.Lock()  # Prevent race conditions on the channel 5 calculations
        self._lock_channel6 = threading.Lock()  # Prevent race conditions on the channel 6 calculations
        # === #
        
        # === Pi GPIO Pins (Broadcom Numbers) === #
        self._pin_throttle    = 5   # PWM input from RC controller throttle lever
        self._pin_steering    = 6   # PWM input from RC controller steering wheel
        self._pin_channel6    = 8   # PWM input from RC controller channel 6 knob
        self._pin_zero_button = 16  # Zero position button
        self._pin_channel3    = 23  # PWM input from RC controller channel 3 button
        self._pin_channel4    = 24  # PWM input from RC controller channel 4 switch
        self._pin_channel5    = 25  # PWM input from RC controller channel 5 knob
        # === #
        
        # === Runtime Variables === #
        self._pedals_connected        = False  # Indicates if the pedals are connected based on channel 3 state. False/0 for off | True/1 for on
        self._steering_direction      = 1      # Tracks which direction the sled moves per given steering input direction (1 or -1 multiplier)
        self._max_allowed_tower_speed = 0
        self._max_allowed_sled_speed  = 0
        
        # Zero Button
        self._zero_button_tripped = False  # Indicates if the zero button was tripped this cycle
        self._zero_button_pressed = False  # Indicates if the zero button is currently depressed
        self.on_zero_button_change = Zero_Button_Event()
        
        # Throttle PWM signal variables
        self._throttle_last_edge_tick   = None
        self._throttle_last_rising_tick = None
        self._throttle_last_period_tick = None
        self._throttle_last_input_tick  = None
        self._throttle_input_timed_out  = True
        self._throttle_high_time        = 0
        self._throttle_period           = 0
        self._throttle_input_unsmooth   = 0
        
        # Steering PWM signal variables
        self._steering_last_edge_tick   = None
        self._steering_last_rising_tick = None
        self._steering_last_period_tick = None
        self._steering_high_time        = 0
        self._steering_period           = 0
        self._steering_input_unsmooth   = 0
        
        # Channel 3 PWM signal variables
        self._channel3_last_edge_tick   = None
        self._channel3_last_rising_tick = None
        self._channel3_last_period_tick = None
        self._channel3_period           = 0
        
        # Channel 4 PWM signal variables
        self._channel4_last_edge_tick   = None
        self._channel4_last_rising_tick = None
        self._channel4_last_period_tick = None
        self._channel4_period           = 0
        
        # Channel 5 PWM signal variables
        self._channel5_last_edge_tick   = None
        self._channel5_last_rising_tick = None
        self._channel5_last_period_tick = None
        self._channel5_period           = 0
        
        # Channel 6 PWM signal variables
        self._channel6_last_edge_tick   = None
        self._channel6_last_rising_tick = None
        self._channel6_last_period_tick = None
        self._channel6_period           = 0
        # === #
        
        # === Deques === #
        size_limit = 7
        self.throttle_dq = deque([self._throttle_neutral_us[self._pedals_connected] for _ in range(size_limit)], maxlen=size_limit)  # Deque for throttle high time
        self.steering_dq = deque([self._steering_neutral_us                         for _ in range(size_limit)], maxlen=size_limit)  # Deque for steering high time
        self.channel3_dq = deque([self._channel3_off_us                             for _ in range(size_limit)], maxlen=size_limit)  # Deque for channel 3 high time
        self.channel4_dq = deque([self._channel4_1_us                               for _ in range(size_limit)], maxlen=size_limit)  # Deque for channel 4 high time
        self.channel5_dq = deque([self._channel_left_us                             for _ in range(size_limit)], maxlen=size_limit)  # Deque for channel 5 high time
        self.channel6_dq = deque([self._channel_left_us                             for _ in range(size_limit)], maxlen=size_limit)  # Deque for channel 6 high time
        # === #
        
        # === Logger Config === #
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.info("RcInputReader class initialized")
        # === #
        
        # === Connect to Devices === #
        self.connect()
        # === #
        
    # === Connection Management === #
    def connect(self) -> None:
        """
        Master function for connecting GPIO devices.
        """        
        # Run connector functions
        self._connect_hand_controller()
        self._connect_zero_button()
        
        self.logger.info("All peripherals connected")
    
    def disconnect(self) -> None:
        """
        Close class connections.
        """
        # Cancel callbacks
        self._cb_throttle.cancel()     # Cancel controller throttle callback
        self._cb_steering.cancel()     # Cancel controller steering callback
        self._cb_channel3.cancel()     # Cancel controller channel 3 callback
        self._cb_channel4.cancel()     # Cancel controller channel 4 callback
        self._cb_channel5.cancel()     # Cancel controller channel 5 callback
        self._cb_channel6.cancel()     # Cancel controller channel 6 callback
        self._cb_zero_button.cancel()  # Cancel zero button callback
        
        self.logger.info("All callbacks cancelled")
    
    def _connect_hand_controller(self) -> None:
        """
        Connect to the hand controller.
        """
        # Configure pin input and pull-down resistor for pins
        for pin in [
            self._pin_throttle,
            self._pin_steering,
            self._pin_channel3,
            self._pin_channel4,
            self._pin_channel5,
            self._pin_channel6
        ]:
            self.pi.set_mode(pin, pigpio.INPUT)
            self.pi.set_pull_up_down(pin, pigpio.PUD_DOWN)
            self.pi.set_glitch_filter(pin, 100)  # Ignores edges shorter than 100 microseconds
            
        # Register callbacks
        self._cb_throttle = self.pi.callback(self._pin_throttle, pigpio.EITHER_EDGE, self._handle_throttle_pwm)
        self._cb_steering = self.pi.callback(self._pin_steering, pigpio.EITHER_EDGE, self._handle_steering_pwm)
        self._cb_channel3 = self.pi.callback(self._pin_channel3, pigpio.EITHER_EDGE, self._handle_channel3_pwm)
        self._cb_channel4 = self.pi.callback(self._pin_channel4, pigpio.EITHER_EDGE, self._handle_channel4_pwm)
        self._cb_channel5 = self.pi.callback(self._pin_channel5, pigpio.EITHER_EDGE, self._handle_channel5_pwm)
        self._cb_channel6 = self.pi.callback(self._pin_channel6, pigpio.EITHER_EDGE, self._handle_channel6_pwm)
        
        self.logger.info("Hand controller pins and callbacks configured")
    
    def _connect_zero_button(self) -> None:
        """
        Connects to the zero-position button.
        """
        # Configure pin settings
        self.pi.set_mode(self._pin_zero_button, pigpio.INPUT)
        self.pi.set_pull_up_down(self._pin_zero_button, pigpio.PUD_DOWN)
        self.pi.set_glitch_filter(self._pin_zero_button, 50_000)  # Ignores edges shorter than 50,000 microseconds
        
        # Register callback
        self._cb_zero_button = self.pi.callback(self._pin_zero_button, pigpio.EITHER_EDGE, self._handle_zero_button)
        
        self.logger.info("Zero button pin and callback configured")
    # === #
    
    # === Zero Button Event Handler === #
    @property
    def zero_button_tripped(self) -> bool:
        return self._zero_button_tripped
            
    def consume_zero_button_tripped(self) -> bool:
        was_tripped = self._zero_button_tripped
        self._zero_button_tripped = False
        return was_tripped
    # === #
    
    # === Callback Handlers === #
    def _handle_throttle_pwm(self, gpio: int, level: int, tick: int) -> None:
        with self._lock_throttle:
            self._throttle_last_edge_tick = tick
            
            if level == 1:  # Rising edge
                if self._throttle_last_period_tick is not None:
                    self._throttle_period = pigpio.tickDiff(self._throttle_last_period_tick, tick)
                self._throttle_last_period_tick = self._throttle_last_rising_tick = tick
            elif level == 0:  # Falling edge
                if self._throttle_last_rising_tick is not None:
                    pulse = pigpio.tickDiff(self._throttle_last_rising_tick, tick)
                    if 1000 <= pulse <= 2200:
                        self.throttle_dq.append(pulse)
                    else:
                        self.logger.debug(f"Ignored invalid throttle pulse width: {pulse}")
    
    def _handle_steering_pwm(self, gpio: int, level: int, tick: int) -> None:
        with self._lock_steering:
            self._steering_last_edge_tick = tick
            
            if level == 1:  # Rising edge
                if self._steering_last_period_tick is not None:
                    self._steering_period = pigpio.tickDiff(self._steering_last_period_tick, tick)
                self._steering_last_period_tick = self._steering_last_rising_tick = tick
            elif level == 0:  # Falling edge
                if self._steering_last_rising_tick is not None:
                    pulse = pigpio.tickDiff(self._steering_last_rising_tick, tick)
                    if 1000 <= pulse <= 2200:
                        self.steering_dq.append(pulse)
                    else:
                        self.logger.debug(f"Ignored invalid steering pulse width: {pulse}")
    
    def _handle_channel3_pwm(self, gpio: int, level: int, tick: int) -> None:
        with self._lock_channel3:
            self._channel3_last_edge_tick = tick
            
            if level == 1:  # Rising edge
                if self._channel3_last_period_tick is not None:
                    self._channel3_period = pigpio.tickDiff(self._channel3_last_period_tick, tick)
                self._channel3_last_period_tick = self._channel3_last_rising_tick = tick
            elif level == 0:  # Falling edge
                if self._channel3_last_rising_tick is not None:
                    pulse = pigpio.tickDiff(self._channel3_last_rising_tick, tick)
                    if 850 <= pulse <= 2000:
                        self.channel3_dq.append(pulse)
                    else:
                        self.logger.debug(f"Ignored invalid channel 3 pulse width: {pulse}")
    
    def _handle_channel4_pwm(self, gpio: int, level: int, tick: int) -> None:
        with self._lock_channel4:
            self._channel4_last_edge_tick = tick
            
            if level == 1:  # Rising edge
                if self._channel4_last_period_tick is not None:
                    self._channel4_period = pigpio.tickDiff(self._channel4_last_period_tick, tick)
                self._channel4_last_period_tick = self._channel4_last_rising_tick = tick
            elif level == 0:  # Falling edge
                if self._channel4_last_rising_tick is not None:
                    pulse = pigpio.tickDiff(self._channel4_last_rising_tick, tick)
                    if 600 <= pulse <= 2000:
                        self.channel4_dq.append(pulse)
                    else:
                        self.logger.debug(f"Ignored invalid channel 4 pulse pulse: {pulse}")
    
    def _handle_channel5_pwm(self, gpio: int, level: int, tick: int) -> None:
        with self._lock_channel5:
            self._channel5_last_edge_tick = tick

            if level == 1:  # Rising edge
                if self._channel5_last_period_tick is not None:
                    self._channel5_period = pigpio.tickDiff(self._channel5_last_period_tick, tick)
                self._channel5_last_period_tick = self._channel5_last_rising_tick = tick
            elif level == 0:  # Falling edge
                if self._channel5_last_rising_tick is not None:
                    pulse = pigpio.tickDiff(self._channel5_last_rising_tick, tick)
                    if 850 <= pulse <= 2200:
                        self.channel5_dq.append(pulse)
                    else:
                        self.logger.debug(f"Ignored invalid channel 5 pulse width: {pulse}")
    
    def _handle_channel6_pwm(self, gpio: int, level: int, tick: int) -> None:
        with self._lock_channel6:
            self._channel6_last_edge_tick = tick

            if level == 1:  # Rising edge
                if self._channel6_last_period_tick is not None:
                    self._channel6_period = pigpio.tickDiff(self._channel6_last_period_tick, tick)
                self._channel6_last_period_tick = self._channel6_last_rising_tick = tick
            elif level == 0:  # Falling edge
                if self._channel6_last_rising_tick is not None:
                    pulse = pigpio.tickDiff(self._channel6_last_rising_tick, tick)
                    if 850 <= pulse <= 2200:
                        self.channel6_dq.append(pulse)
                    else:
                        self.logger.debug(f"Ignored invalid channel 6 pulse width: {pulse}")
    
    def _handle_zero_button(self, gpio: int, level: int, tick: int) -> None:
        """
        Determines if zero-limit button is tripped.
        """
        if level == 1:    # Button pressed
            self._zero_button_pressed = True
            self._zero_button_tripped = True
            self.on_zero_button_change.notify(True)
            self.logger.info("Zero button tripped. Notifying encoder class.")
        elif level == 0:  # Button released
            self._zero_button_pressed = False
    # === #
    
    # === Logic Handlers === #
    def _decode_throttle_input(self) -> None:
        """
        Determines the unsmoothed throttle value based on PWM value.
        """
        with self._lock_throttle:
            now = self.pi.get_current_tick()
            if self._throttle_last_edge_tick is None or pigpio.tickDiff(self._throttle_last_edge_tick, now) > self._controller_timeout:
                # No active signal
                self._throttle_high_time = self._throttle_period = self.throttle_input_unsmooth = 0.0
                
                self.logger.warning("No throttle signal from controller")
            
            elif self._throttle_period > 0:  # Active input signal
                high_time = median(self.throttle_dq)
                neutral   = self._throttle_neutral_us[self._pedals_connected]
                forward   = self._throttle_forward_us[self._pedals_connected]
                backward  = self._throttle_backward_us[self._pedals_connected]
                
                # Handle noisy input
                if (neutral - 25) <= high_time <= (neutral + 25):
                    # Inside "noisy" bounds
                    self._throttle_input_unsmooth = 0.0
                    
                else:
                    # Interpret high time as direction
                    if high_time > neutral:
                        # Upward normalization
                        self._throttle_input_unsmooth = (high_time - neutral) / (forward - neutral)
                        
                        # Clamp to 1.0
                        self._throttle_input_unsmooth = min(self._throttle_input_unsmooth, 1.0)
                        
                    elif high_time < neutral:
                        # Downward normalization
                        self._throttle_input_unsmooth = (high_time - neutral) / (neutral - backward)
                        
                        # Clamp to -1.0
                        self._throttle_input_unsmooth = max(self._throttle_input_unsmooth, -1.0)
                        
                    else:
                        # Neutral command
                        self._throttle_input_unsmooth = 0.0  # Neutral
                        
                # Handle deadzone
                if abs(self._throttle_input_unsmooth) < self._deadzone:
                    self._throttle_input_unsmooth = 0.0
    
    def _decode_steering_input(self) -> None:
        """
        Determines the unsmoothed steering value based on PWM value.
        """
        with self._lock_steering:
            now = self.pi.get_current_tick()
            if self._steering_last_edge_tick is None or pigpio.tickDiff(self._steering_last_edge_tick, now) > self._controller_timeout:
                # No active signal
                self._steering_high_time = self._steering_period = self._steering_input_unsmooth = 0.0
                
                self.logger.warning("No steering signal from controller")
                
            elif self._steering_period > 0:  # Active input signal
                high_time = median(self.steering_dq)
                
                # Handle noisy input
                if (self._steering_neutral_us - 25) <= high_time <= (self._steering_neutral_us + 25):
                    # Inside "noisy" bounds
                    self._steering_input_unsmooth = 0.0
                    
                else:
                    # Interpret high time as direction
                    if high_time > self._steering_neutral_us:
                        # Forward normalization
                        self._steering_input_unsmooth = (high_time - self._steering_neutral_us) / (self._steering_forward_us - self._steering_neutral_us)
                        
                        # Clamp to 1.0
                        self._steering_input_unsmooth = min(self._steering_input_unsmooth, 1.0)
                        
                    elif high_time < self._steering_neutral_us:
                        # Backward normalization
                        self._steering_input_unsmooth = (high_time - self._steering_neutral_us) / (self._steering_neutral_us - self._steering_backward_us)
                        
                        # Clamp to -1.0
                        self._steering_input_unsmooth = max(self._steering_input_unsmooth, -1.0)
                        
                    else:
                        self._steering_input_unsmooth = 0.0  # Neutral
                        
                # Handle deadzone
                if abs(self._steering_input_unsmooth) < self._deadzone:
                    self._steering_input_unsmooth = 0.0
    
    def _decode_channel3_input(self) -> None:
        """
        Determines if the foot pedals are connected.
        """
        pulse = median(self.channel3_dq)
        match pulse:
            case _ if pulse > 1700:  # Button on
                self._pedals_connected = True
            case _ if pulse > 1200:  # Button off
                self._pedals_connected = False
            case _:                  # Controller off
                self._pedals_connected = False
    
    def _decode_channel4_input(self) -> None:
        """
        Determines the direction the sled will move with a given steering input.
        """
        pulse = median(self.channel4_dq)
        match pulse:
            case _ if pulse > 1700:  # Position 3 - change direction
                self._steering_direction = -1
            case _ if pulse > 1500:  # Position 2 - keep last value
                pass
            case _ if pulse > 1200:  # Position 1 - change direction
                self._steering_direction = 1
            case _:                  # Invalid input - keep last value
                pass
    
    def _decode_channel5_input(self) -> None:
        """
        Determines the max travel speed of the sled.
        """
        pulse = median(self.channel5_dq)
        
        # Handle > 8000 microsecond pulse
        if pulse > 8000:
            pulse = pulse // 10
            
        self._max_allowed_sled_speed = (pulse - self._channel_left_us) / (self._channel_right_us - self._channel_left_us)
    
    def _decode_channel6_input(self) -> None:
        """
        Determines the max travel speed of the tower.
        """
        pulse = median(self.channel6_dq)
        
        # Handle > 8000 microsecond pulses
        if pulse > 8000:
            pulse = pulse // 10
            
        self._max_allowed_tower_speed = (pulse - self._channel_left_us) / (self._channel_right_us - self._channel_left_us)
    
    def _update_inputs(self) -> None:
        """
        Updates input values.
        """
        # Collect controller input commands
        self._decode_throttle_input()
        self._decode_steering_input()
        self._decode_channel3_input()
        self._decode_channel4_input()
        self._decode_channel5_input()
        self._decode_channel6_input()
    # === #
    
    # === Public Interface === #
    def poll(self) -> RcInputState:
        self._update_inputs()
        return RcInputState(
            throttle_input_unsmooth  = self._throttle_input_unsmooth,
            steering_input_unsmooth  = self._steering_input_unsmooth,
            pedals_connected         = self._pedals_connected,
            steering_direction       = self._steering_direction,
            max_allowed_sled_speed   = self._max_allowed_sled_speed,
            max_allowed_tower_speed  = self._max_allowed_tower_speed,
            zero_button_tripped      = self.consume_zero_button_tripped(),
            now_tick                 = self.pi.get_current_tick()
        )
    # === #
    
    # === Logging === #
    def log_debug_values(self) -> None:
        self.logger.debug(f"Pedals connected: {self._pedals_connected}")
        self.logger.debug(f"Steering direction: {self._steering_direction}")
        self.logger.debug(f"Max tower speed: {self._max_allowed_tower_speed}")
        self.logger.debug(f"Max sled speed: {self._max_allowed_sled_speed}")
    # === #
    
class ThrottleController:
    def __init__(self, constants: Constants):
        # === Constants === #
        self._ema_alpha     = constants.ema_alpha_throttle
        self._speed_limiter = constants.speed_limiter
        self._input_min     = constants.input_min
        
        # Timing
        self._input_timeout = constants.input_timeout
        
        # Received variables
        self._enc_max = 8000
        # === #
        
        # === Runtime Variables === #
        self.active_zone               = 0  # Tracks what zone is in use
        self._max_allowed_tower_speed  = 0
        self.throttle_input            = 0
        
        # Timing
        self._last_effective_throttle_tick: int | None = None  # Tracks last valid throttle time
        self._last_min_hold_tick:           int | None = None  # Tracks last time min
        self._rc_now_tick:                  int | None = None
        
        # Minimum speed
        self._last_min_hold_speed = 0.0  # Track the last minimum hold speed value
        self.min_move_speed       = 0.0  # Minimum speed value that will move the tower
        self.min_hold_speed       = 0.0  # Minimum speed value that will hold the tower stationary
        # === #
        
        # === Logger Config === #
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.info("ThrottleController class initialized")
        # === #
    
    def update(self, rc: RcInputState) -> float:
        """
        Updates variables with current RC state.
        
        Returns smoothed throttle input value.
        """
        # Update internal variables
        self._max_allowed_tower_speed = rc.max_allowed_tower_speed
        self._rc_now_tick             = rc.now_tick
        
        # Determine smoothed input
        self.throttle_input = (
            (self._ema_alpha * rc.throttle_input_unsmooth) +
            ((1 - self._ema_alpha) * self.throttle_input)
        )
        
        # Discard small inputs
        if abs(self.throttle_input) < self._input_min:
            self.throttle_input = 0.0
            
        if self.throttle_input:
            self._last_effective_throttle_tick = self._rc_now_tick
            
        return self.throttle_input
    
    # === Tower Zone Handlers === #    
    def position_hold(self, travel_rate: float) -> float:
        """
        Holds position when no throttle input is received.
        """
        # Check if minimum hold speed needs to change
        if self._last_effective_throttle_tick is None:
            timed_out = True
        else:
            # Check if no effective input for longer than the timeout period
            timed_out = (pigpio.tickDiff(self._last_effective_throttle_tick, self._rc_now_tick) > self._input_timeout)
            
        if timed_out:
            if self._last_min_hold_tick is None:
                self._last_min_hold_tick = self._rc_now_tick
                
            # Only updates once every 2 seconds
            if pigpio.tickDiff(self._last_min_hold_tick, self._rc_now_tick) > 2_000_000:
                if travel_rate > 0 and self.min_hold_speed > 0:  # Tower is moving up
                    self.min_hold_speed -= 0.01  # Decrease speed by 1%
                    self._last_min_hold_tick = self._rc_now_tick
                elif travel_rate < 0:  # Tower is moving down
                    self.min_hold_speed += 0.01  # Increase speed by 1%
                    self._last_min_hold_tick = self._rc_now_tick
        
        self.active_zone = 1
        return self.min_hold_speed
    
    def lower_region(self, position: int) -> float:
        """
        Handles the tower being in the lower 10% of its travel region.
        
        Includes safety condition for tower being in negative positions.
        """
        self.active_zone = 2
        
        if self.throttle_input > 0:  # Moving up
            return self.throttle_input * self._max_allowed_tower_speed
        
        elif self.throttle_input < 0 and position > 0:  # Moving down, above zero
            return self.throttle_input * self._max_allowed_tower_speed * self._speed_limiter
        
        elif self.throttle_input < 0 and position < 0:  # Moving down, below zero - proceed with caution
            self.active_zone = -2
            # self.logger.warning("Tower below zero point. Killing input for this cycle.")
            return 0
        
        else:
            return 0
    
    def middle_region(self) -> float:
        """
        Handles the tower being in the middle of it's travel region.
        """
        self.active_zone = 3
        
        return self.throttle_input * self._max_allowed_tower_speed
    
    def upper_region(self, position: int) -> float:
        """
        Handles the tower being in the upper 10% of its travel region.
        
        Includes safety condition for tower being above expected max position.
        """
        self.active_zone = 4
        
        if self.throttle_input < 0:  # Moving down
            return self.throttle_input * self._max_allowed_tower_speed
        
        elif self.throttle_input > 0 and position < self._enc_max:  # Moving up, below max
            return self.throttle_input * self._max_allowed_tower_speed * self._speed_limiter
        
        elif self.throttle_input > 0 and position > self._enc_max:  # Moving up, above max - proceed with caution
            self.active_zone = -4
            # return self.throttle_input * self._max_allowed_tower_speed * self._speed_limiter * 0.1
            return 0
        
        else:
            return 0
    # === #
    
    # === Public Interface === #
    def get_min_hold_speed(self) -> float:
        return self.min_hold_speed
    
    def update_enc_max(self, enc_max: int) -> None:
        """
        Updates encoder max value used in tower position calculations.
        """
        self._enc_max = enc_max
    # === #
    
    # === Logging === #
    def log_debug_values(self) -> None:
        self.logger.debug(f"Throttle input: {self.throttle_input}")
        self.logger.debug(f"Active zone: {self.active_zone}")
        
        # Log when changed
        if self.min_hold_speed != self._last_min_hold_speed:
            self.logger.debug(f"Minimum hold speed set to: {self.min_hold_speed}")
            self._last_min_hold_speed = self.min_hold_speed
    # === #
    
class SteeringController:
    def __init__(self, constants: Constants):
        # === Constants === #
        self._ema_alpha = constants.ema_alpha_steering
        self._input_min = constants.input_min
        # === #
        
        # === Runtime Variables === #
        self._steering_direction      = 1  # Tracks which direction the sled moves
        self._max_allowed_sled_speed  = 0
        self.steering_input           = 0
        # === #
        
        # === Logger Config === #
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.info("SteeringController class initialized")
        # === #
    
    def update(self, rc: RcInputState) -> float:
        """
        Updates variables with current RC state.
        
        Returns smoothed steering input value.
        """
        # Update internal variables
        self._steering_direction      = rc.steering_direction
        self._max_allowed_sled_speed  = rc.max_allowed_sled_speed
        
        # Determine smoothed input
        self.steering_input = (
            (self._ema_alpha * rc.steering_input_unsmooth) +
            ((1 - self._ema_alpha) * self.steering_input)
        )
        
        # Discard small inputs
        if abs(self.steering_input) < self._input_min:
            self.steering_input = 0.0
            
        return self.steering_input
    
    # === Public Interface === #
    
    def get_steering_command(self) -> float:
        """
        Returns steering command.
        """
        return self.steering_input * self._max_allowed_sled_speed * self._steering_direction
    # === #
    
    # === Logging === #
    def log_debug_values(self) -> None:
        self.logger.debug(f"Steering input: {self.steering_input}")
    # === #
    
class RigController:
    def __init__(self, pi: pigpio.pi, enable_steering: bool = False):
        """
        Class initialization.
        
        Inputs:
            pi              - pigpio.pi connection
            enable_steering - Indicates if sled (steering) is controllable
        """
        # === Constants === #
        self.constants = Constants()
        # === #
        
        # === Initialize Classes === #
        self.rc_input = RcInputReader(pi, self.constants)
        self.throttle = ThrottleController(self.constants)
        self.steering = SteeringController(self.constants) if enable_steering else None
        # === #
        
        # === Runtime Variables === #
        self._tower_input         = 0.0
        self._sled_input          = None
        self._zero_button_tripped = False
        # === #
        
        # === Logger Config === #
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.info("RigController class initialized")
        # === #
    
    def update(self) -> tuple[float, float | None]:
        """
        Polls RcInputReader and updates controller classes with those input values.
        
        Returns:
            tuple: Tower and Sled commands
        """
        # Poll RcInputReader
        rc = self.rc_input.poll()
        self._zero_button_tripped = rc.zero_button_tripped
        
        # Update ThrottleController
        self._tower_input = self.throttle.update(rc)
        
        # Update SteeringController if present
        if self.steering:
            self._sled_input = self.steering.update(rc)
        else:
            self._sled_input = None
            
        return self._tower_input, self._sled_input
    
    # === Public Interface === #        
    def connect(self) -> None:
        self.rc_input.connect()
        
    def disconnect(self) -> None:
        self.rc_input.disconnect()
    
    @property
    def zero_button_tripped(self) -> bool:
        return self._zero_button_tripped
    
    def subscribe_zero_button(self, callback) -> None:
        """
        Subscription method for zero button flagging.
        """
        self.rc_input.on_zero_button_change.subscribe(callback)
        
    def get_tower_input(self) -> float:
        """
        Returns tower command.
        """
        return self._tower_input
    
    def get_sled_input(self) -> float | None:
        """
        Returns sled command.
        """
        return self._sled_input
    
    def update_enc_max(self, enc_max: int) -> None:
        """
        Allows the main function to update encoder max value used in ThrottleController class functions.
        """
        self.throttle.update_enc_max(enc_max)
    # === #
    
    # === Logging === #
    def log_debug_values(self) -> None:
        """
        Log all debug values.
        """
        self.rc_input.log_debug_values()
        self.throttle.log_debug_values()
        if self.steering is not None:
            self.steering.log_debug_values()
        
        if self._zero_button_tripped:
            self.logger.debug("Zero button tripped")
    # === #