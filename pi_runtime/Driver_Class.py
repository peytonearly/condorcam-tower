# Python Libraries
import serial
import logging
import time

class AF160:
    # === Driver Resgister Values === #
    CTRL_SELECT  = 0    # Range [0, 3]            Control input selection
    SERVO_MODE   = 1    # Range [0, 5]            Servo operation mode selection
    SERVO_RATE   = 20   # Range [0, 255]          Basic loop rate for servo calculations (ms)

    P_GAIN       = 3    # Range [-32000, 32000]   Corrects error between actual and commanded positions 
    I_GAIN       = 5    # Range [-3200, 3200]     Corrects error between actual and command velocities
    D_GAIN       = 30   # Range [0, 1023]         How strong servo tries to maintain a velocity set point 

    BRAKE        = 31   # Range [0, 31]           Brake % value
    BACK_EMF     = 29   # Range [0, 255]          Used to estimate torque and compensate for friction
    MIN_DRIVE    = 0    # Range [0, 255]          Drive to add to the output to overcome friction
    SLEW_RATE    = 1    # Range [0, 100]          How fast the drive value can changes (100ms)
    TORQUE_LIMIT = 255  # Range [0, 255]          With back-EMF, limits maximum current draw
    # === #
    
    def __init__(self, throttle_channel: int = 1, steering_channel: int | None = None, enable_steering: bool = False) -> None:
        """
        Class initialization.
        
        Inputs:
            throttle_channel - Driver output channel that the tower (throttle) is connected to
            steering_channel - Driver output channel that the sled (steering) is connected to. None indicates sled is not connected.
            enable_steering  - Indicates if sled (steering) is controllable
        """
        # === Serial Connection === #
        self.ser      = None            # Serial connection
        self.ser_port = "/dev/ttyUSB0"  # Serial port
        self.timeout  = 0.1             # Serial read timeout (s)
        # === #
        
        # === Motor Configuration === #
        self.throttle_channel = throttle_channel
        self.steering_channel = steering_channel
        # === #
        
        # === Payload Variables === #
        self.throttle_input_scaled = 0      # Throttle input normalized to AF160 PWM torque limits (-255 to 255)
        self._last_throttle_input_sent = 0  # Tracks last value sent to the motor to avoid repeat calls
        self.steering_input_scaled = 0      # Steering input normalized to AF160 PWM torque limits (-255 to 255)
        self._last_steering_input_sent = 0  # Tracks last value sent to the motor to avoid repeat calls
        # === #
        
        # === Logger Config === #
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.info("AF160 class initialized")
        # === #
        
        # === Connect to Driver === #
        self.connect()
        self.set_driver_configuration(self.throttle_channel)
        if self.steering_channel: self.set_driver_configuration(self.steering_channel, rc_settings=(not enable_steering))
        # === #
        
    # === Connection Management === #
    def connect(self) -> None:
        """
        Connect to the driver via USB serial.
        """
        self.ser = serial.Serial(
            port=self.ser_port,
            baudrate=115200,
            timeout=self.timeout,
        )
        
        time.sleep(0.5)  # Allow link to settle
        
        self.logger.info(f"AF160 driver connected on {self.ser_port} at {time.time():.3f}")
        
    def disconnect(self) -> None:
        """
        Safely close driver connection.
        """
        # Send a 0 command to stop motors
        self.send_payloads(0, 0)
        
        # Close driver connection
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except serial.SerialException:
                pass
            
        self.ser = None
        self.logger.info(f"Driver disconnected at {time.time():.3f}")
        
    def set_driver_configuration(self, channel: int, rc_settings: bool = False) -> None:
        """
        Set driver parameters to pre-configured values on selected channel.
        
        Inputs:
            channel     - Which channel to program
            rc_settings - Indicates if the channel should be programmed for RC control (default is Serial control)
        """
        def log_string(reg: str, chn: str, val: int, res: float) -> str: return f"{reg} | Channel {chn} set to {val} | {'Okay' if (int(res) == int(val)) else 'Fail'}"
        
        # Control Input Select (j)
        self._send_command(channel=channel, register="j", operation="s", value=(self.CTRL_SELECT if not rc_settings else 1), response_expected=False)
        self._send_command(channel=channel, register="j")
        self.logger.info(log_string("Control Input Select (j)", channel, (self.CTRL_SELECT if not rc_settings else 1), self.response))
        
        # Servo Mode (m)
        self._send_command(channel=channel, register="m", operation="s", value=self.SERVO_MODE, response_expected=False)
        self._send_command(channel=channel, register="m")
        self.logger.info(log_string("Servo Mode (m)", channel, self.SERVO_MODE, self.response))
        
        # Servo Update Rate (r)
        self._send_command(channel=channel, register="r", operation="s", value=self.SERVO_RATE, response_expected=False)
        self._send_command(channel=channel, register="r")
        self.logger.info(log_string("Servo Update Rate (r)", channel, self.SERVO_RATE, self.response))
        
        # Proportional Gain (P)
        self._send_command(channel=channel, register="P", operation="s", value=self.P_GAIN, response_expected=False)
        self._send_command(channel=channel, register="P")
        self.logger.info(log_string("Proportional Gain (P)", channel, self.P_GAIN, self.response))
        
        # Integral Gain (I)
        self._send_command(channel=channel, register="I", operation="s", value=self.I_GAIN, response_expected=False)
        self._send_command(channel=channel, register="I")
        self.logger.info(log_string("Integral Gain (I)", channel, self.I_GAIN, self.response))
        
        # Derivative Gain (D)
        self._send_command(channel=channel, register="D", operation="s", value=self.D_GAIN, response_expected=False)
        self._send_command(channel=channel, register="D")
        self.logger.info(log_string("Derivative Gain (D)", channel, self.D_GAIN, self.response))
        
        # Brake (B)
        self._send_command(channel=channel, register="B", operation="s", value=self.BRAKE, response_expected=False)
        self._send_command(channel=channel, register="B")
        self.logger.info(log_string("Brake (B)", channel, self.BRAKE, self.response))
        
        # Back-EMF Factor (E)
        self._send_command(channel=channel, register="E", operation="s", value=self.BACK_EMF, response_expected=False)
        self._send_command(channel=channel, register="E")
        self.logger.info(log_string("Back-EMF Factor (E)", channel, self.BACK_EMF, self.response))
        
        # Minimum Drive (M)
        self._send_command(channel=channel, register="M", operation="s", value=self.MIN_DRIVE, response_expected=False)
        self._send_command(channel=channel, register="M")
        self.logger.info(log_string("Minimum Drive (M)", channel, self.MIN_DRIVE, self.response))
        
        # Power Slew Rate (s)
        self._send_command(channel=channel, register="s", operation="s", value=self.SLEW_RATE, response_expected=False)
        self._send_command(channel=channel, register="s")
        self.logger.info(log_string("Power Slew Rate (s)", channel, self.SLEW_RATE, self.response))
        
        # Torque Limit (T)
        self._send_command(channel=channel, register="T", operation="s", value=self.TORQUE_LIMIT, response_expected=False)
        self._send_command(channel=channel, register="T")
        self.logger.info(log_string("Torque Limit (T)", channel, self.TORQUE_LIMIT, self.response))
    # === #    
    
    # === Logic Handler === #
    def _scale_input(self, input: float) -> int:
        """
        Scales motor input to fall within range of -255 to 255.
        """
        input = max(-1.0, min(1.0, input))  # Clamp between -1.0 and +1.0
        return int(input * 255)
    # === #
    
    # === Serial Commands === #    
    def _send_command(self, channel: int, register: str, operation: str = "a", value: int | None = None, response_expected: bool = True) -> None:
        """
        Sends a command to the motor driver.
        
        Inputs:
            channel           - Channel to send command to
            register          - Register to manipulate
            operation         - Operation to perform ("s", "g", "a"). Defaults to "a" (actual)
            value             - Value to set register to (if applicable). Defualts to None
            response_expected - Inidicates if a response is expected. If True, collect response
        """
        # Create message
        msg = f"@{channel}{operation}{register}{value if value else ''}\r".encode("ascii")
        
        # Send command
        self.ser.reset_input_buffer()  # Clear old responses
        self.ser.write(msg)
        self.ser.flush()
        time.sleep(0.02)
        
        if response_expected:
            # Collect response
            recv = self.ser.read_until(b'>', 256).decode("ascii", errors="ignore")
            
            # Normalize line ending and split into lines
            lines = [line.strip() for line in recv.replace('\r', '').split('\n') if line.strip()]
            
            # Interpret resposne
            if lines:
                try:
                    self.response = float(lines[1])
                except ValueError:
                    self.response = None
    # === #
    
    # === Public Interface === #
    def send_payloads(self, throttle_input: float, steering_input: float | None = None) -> None:
        """
        Creates and transmits throttle and steering payloads based on inputs from Tower class.
        Steering payload may be None if sled is not connected.
        """
        # Send throttle command
        self.throttle_input_scaled = self._scale_input(throttle_input)
        if self.throttle_input_scaled != self._last_throttle_input_sent:
            self._send_command(channel = self.throttle_channel, register = "t", operation = "s", value = self.throttle_input_scaled, response_expected = False)
            self._last_throttle_input_sent = self.throttle_input_scaled
        
        # Send steering command
        if self.steering_channel:
            self.steering_input_scaled = self._scale_input(steering_input)
            if self.steering_input_scaled != self._last_steering_input_sent:
                self._send_command(channel = self.steering_channel, register = "t", operation = "s", value = self.steering_input_scaled, response_expected = False)
                self._last_steering_input_sent = self.steering_input_scaled
    # === #
    
    # === Logging === #
    def log_debug_values(self) -> None:
        """
        Log all debug values.
        """
        self.logger.debug(f"Throttle motor speed set point: {self.throttle_input_scaled}")
        
        if self.steering_channel:
            self.logger.debug(f"Steering motor speed set point: {self.steering_input_scaled}")