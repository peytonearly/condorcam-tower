'''
- Add functionality to AF160.get_driver_configuration() that sets registers to expected values before 
    collecting current state.
'''

import pigpio
import serial
import logging
import time

class MDDS30:
    def __init__(self):
        """
        Class initialization.
        """
        # === Pi GPIO Pins (Broadcom numbers) === #
        self._pin_motor_driver = 14  # Output for sending motor driver commands
        # === #

        # === Motor Driver Payload Variables === #
        # Throttle
        self.throttle_bit_channel = 0  # 0 for LEFT motor | 1 for RIGHT motor
        self.throttle_bit_dir = 0  # 0 for upward direction | 1 for downward direction
        self.throttle_bits_speed = 0  # 0b000000 (0) for stop | 0b111111 (63) for full speed
        self.throttle_bits_payload = 0  # Initialize payload byte

        # Steering
        self.steering_bit_channel = 1  # Steering on the right motor
        self.steering_bit_dir = 0  # 0 for left direction | 1 for right direction
        self.steering_bits_speed = 0  # 0b000000 (0) for stop | 0b111111 (63) for full speed
        self.steering_bits_payload = 0  # Initialize payload byte
        # === #

        # === Logger Config === #
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.info("MDDS30 class initialized")
        # === #

        # === Connect to Driver === #
        self.connect_driver()
        # === #

    # === Connecting to Driver === #
    def connect_driver(self):
        """
        Connect to the driver via UART connection.
        """
        # Open driver connection
        self.driver = serial.Serial(
            port='/dev/serial0',
            baudrate=115200,
            timeout=1
        )

        self.logger.info("MDDS30 driver connected via UART serial")

    def disconnect_driver(self):
        """
        Close driver connection.
        """
        # Send a 0 command to stop motors
        self.send_payloads(0, 0)

        # Close driver connection
        if self.driver.is_open:
            self.driver.close()

        self.logger.info("MDDS30 driver disconnected")

    # === Logic Handlers === #
    def _create_throttle_payload(self, direction, speed):
        """
        Creates an 8-bit payload string for the steering command.

        Structure:
            | 7 (MSB) |     6     | 5 | 4 | 3 | 2 | 1 | 0 (LSB) |
            | Channel | Direction |            Speed            |
        """
        # Assign direction and speed values
        self.throttle_bit_dir = direction
        self.throttle_bits_speed = speed

        # Ensure valid ranges
        self.throttle_bit_channel &= 0b1
        self.throttle_bit_dir &= 0b1
        self.throttle_bits_speed &= 0b111111

        # Construct the payload
        self.throttle_bits_payload = (self.throttle_bit_channel << 7) | (self.throttle_bit_dir << 6) | self.throttle_bits_speed
        self.throttle_bits_payload_str = self.throttle_bits_payload.to_bytes(1, byteorder='big')
    
    def _create_steering_payload(self, direction, speed):
        """
        Creates an 8-bit payload string for the steering command.

        Structure:
            | 7 (MSB) |     6     | 5 | 4 | 3 | 2 | 1 | 0 (LSB) |
            | Channel | Direction |            Speed            |
        """
        # Assign the direction and speed values
        self.steering_bit_dir = direction
        self.steering_bits_speed = speed

        # Ensure valid ranges
        self.steering_bit_channel &= 0b1
        self.steering_bit_dir &= 0b1
        self.steering_bits_speed &= 0b111111

        # Construct the payload
        self.steering_bits_payload = (self.steering_bit_channel << 7) | (self.steering_bit_dir << 6) | self.steering_bits_speed
        self.steering_bits_payload_str = self.steering_bits_payload.to_bytes(1, byteorder='big')

    def _create_payloads(self, throttle_input, steering_input):
        """
        Runs functions to create the payload bytes.
        """
        # Determine direction
        throttle_direction = 0 if throttle_input > 0 else 1
        steering_direction = 0 if steering_input < 0 else 1

        # Determine absolute speed values
        throttle_speed = int(abs(63 * throttle_input))
        steering_speed = int(abs(63 * steering_input))

        # Create the payloads
        self._create_throttle_payload(throttle_direction, throttle_speed)
        self._create_steering_payload(steering_direction, steering_speed)

    def send_payloads(self, throttle_input, steering_input):
        """
        Creates and transmits throttle and steering payloads based on inputs from Tower class.
        """
        # Create payloads
        self._create_payloads(throttle_input, steering_input)

        # Transmit payloads
        self.driver.write(self.throttle_bits_payload_str)
        self.driver.write(self.steering_bits_payload_str)
    # === #

    # === Logging === #
    def log_debug_values(self):
        """
        Logs all debug values.
        """
        # Log each loop
        self.logger.debug(f"Throttle motor speed: {self.throttle_bits_speed}")
        self.logger.debug(f"Throttle motor direction: {self.throttle_bit_dir}")
        self.logger.debug(f"Throttle motor command: {self.throttle_bits_payload}")
        self.logger.debug(f"Steering motor speed: {self.steering_bits_speed}")
        self.logger.debug(f"Steering motor direction: {self.steering_bit_dir}")
        self.logger.debug(f"Steering motor command: {self.steering_bits_payload}")
    # === #

class AF160:
    # === Driver Register Values === #
    ''' Change these to values derived from motor tuning '''
    CHANNEL      = 0    # Range [0, 2]            Channel (left, right, both)

    CTRL_SELECT  = 0    # Range [0, 3]            Control input selection
    SERVO_MODE   = 1    # Range [0, 5]            Servo operation mode selection
    SERVO_RATE   = 20   # Range [0, 255]          Basic loop rate for servo calculations (ms)

    P_GAIN       = 0    # Range [-32000, 32000]   Corrects error between actual and commanded positions 
    I_GAIN       = 0    # Range [-3200, 3200]     Corrects error between actual and command velocities
    D_GAIN       = 0    # Range [0, 1023]         How strong servo tries to maintain a velocity set point 

    BRAKE        = 31   # Range [0, 31]           Brake % value
    BACK_EMF     = 8    # Range [0, 255]          Used to estimate torque and compensate for friction
    MIN_DRIVE    = 0    # Range [0, 255]          Drive to add to the output to overcome friction
    SLEW_RATE    = 1    # Range [0, 100]          How fast the drive value can changes (100ms)
    TORQUE_LIMIT = 0    # Range [0, 255]          With back-EMF, limits maximum current draw
    # === #

    def __init__(self):
        """
        Class initialization.
        """
        # === Pi GPIO Pins (Broadcom numbers) === #
        self._pin_motor_driver = 14  # Output for sending motor driver commands
        # === #

        # === Motor Driver Payload Variables === #
        self.throttle_input_scaled = 0  # Throttle input normalized to AF160 PWM torque limits (-255 to 255)
        self.steering_input_scaled = 0  # Steering input normalized to AF160 PWM torque limits (-255 to 255)
        self.throttle_command = None  # Throttle command (left motor)
        self.steering_command = None  # Steering command (right motor)
        self.response = None  # Response from driver
        # === #

        # === Logger Config === #
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.info("AF160 class initialized")
        # === #

        # === Connect to Driver === #
        self.connect_driver()
        # === #

    # === Connecting to Driver === #
    def connect_driver(self):
        """
        Connect to the driver via UART connection.
        """
        # Open driver connection
        self.driver = serial.Serial(
            port= "/dev/ttyUSB0",  # Adjust as needed
            baudrate=115200,
            timeout=0.1
        )

        if self.CHANNEL == 2:  # Both motors
            self.set_get_driver_configuration(0)  # Set configs for left motor
            self.set_get_driver_configuration(1)  # Set configs for right motor
        else:
            self.set_get_driver_configuration(self.CHANNEL)

        self.logger.info("AF160 driver connected via UART serial")

    def set_get_driver_configuration(self, channel):
        """
        Set driver parameters to pre-configured values.
        """
        def log_string(register, channel, val, response): return f"{register} | Channel {channel} set to {val} | {"Success" if (int(response) == int(val)) else "Failed"}"
        # Control Input Select (j)
        self._send_command(f"{channel}sj{self.CTRL_SELECT}\r".encode("ascii"))
        self._send_command(f"{channel}gj\r".encode("ascii"))
        self.logger.info(log_string("Control Input Select (j)", channel, self.CTRL_SELECT, self.response))

        # Servo Mode (m)
        self._send_command(f"{channel}sm{self.SERVO_MODE}\r".encode("ascii"))
        self._send_command(f"{channel}gm\r".encode("ascii"))
        self.logger.info(log_string("Servo Mode (m)", channel, self.SERVO_MODE, self.response))

        # Servo Update Rate (r)
        self._send_command(f"{channel}sr{self.SERVO_RATE}\r".encode("ascii"))
        self._send_command(f"{channel}gr\r".encode("ascii"))
        self.logger.info(log_string("Servo Update Rate (r)", channel, self.SERVO_RATE, self.response))

        # Proportional Gain (P)
        self._send_command(f"{channel}sP{self.P_GAIN}\r".encode("ascii"))
        self._send_command(f"{channel}gP\r".encode("ascii"))
        self.logger.info(log_string("Proportional Gain (P)", channel, self.P_GAIN, self.response))

        # Integral Gain (I)
        self._send_command(f"{channel}sI{self.I_GAIN}\r".encode("ascii"))
        self._send_command(f"{channel}gI\r".encode("ascii"))
        self.logger.info(log_string("Integral Gain (I)", channel, self.I_GAIN, self.response))

        # Derivative Gain (D)
        self._send_command(f"{channel}sD{self.D_GAIN}\r".encode("ascii"))
        self._send_command(f"{channel}gD\r".encode("ascii"))
        self.logger.info(log_string("Derivative Gain (D)", channel, self.D_GAIN, self.response))
        
        # Brake (B)
        self._send_command(f"{channel}sB{self.BRAKE}\r".encode("ascii"))
        self._send_command(f"{channel}gB\r")
        self.logger.info(log_string("Brake (B)", channel, self.BRAKE, self.response))

        # Back-EMF Factor (E)
        self._send_command(f"{channel}sE{self.BACK_EMF}\r".encode("ascii"))
        self._send_command(f"{channel}gE\r".encode("ascii"))
        self.logger.info(log_string("Back-EMF Factor (E)", channel, self.BACK_EMF, self.response))

        # Minimum Drive (M)
        self._send_command(f"{channel}sM{self.MIN_DRIVE}\r".encode("ascii"))
        self._send_command(f"{channel}gM\r".encode("ascii"))
        self.logger.info(log_string("Minimum Drive (M)", channel, self.MIN_DRIVE, self.response))

        # Power Slew Rate (s)
        self._send_command(f"{channel}ss{self.SLEW_RATE}\r".encode("ascii"))
        self._send_command(f"{channel}gs\r".encode("ascii"))
        self.logger.info(log_string("Power Slew Rate (s)", channel, self.SLEW_RATE, self.response))

        # Torque Limit (T)
        self._send_command(f"{channel}sT{self.TORQUE_LIMIT}\r".encode("ascii"))
        self._send_command(f"{channel}gT\r".encode("ascii"))
        self.logger.info(log_string("Torque Limit (T)", channel, self.TORQUE_LIMIT,))

    def disconnect_driver(self):
        """
        Close driver connection.
        """
        # Send a 0 command to stop motors
        self.send_payloads(0, 0)

        # Close driver connection
        if self.driver.is_open:
            self.driver.close()

        self.logger.info("AF160 driver disconnected")
    # === #

    # === Logic Handlers === #
    def _scale_input(self, input):
        """
        Scales tower input to [-255, 255] range
        """
        input = max(-1.0, min(1.0, input))  # Clamp between [-1.0, 1.0]
        return int(input * 255)
        
    def send_payloads(self, throttle_input, steering_input=None):
        """
        Creates and transmits throttle and steering payloads based on inputs from Tower class.
        Steering payload may be None if sled is not connected.
        """
        # Throttle input
        self.throttle_input_scaled = self._scale_input(throttle_input)
        self.throttle_command = f"@0st{self.throttle_input_scaled}\r".encode("ascii")
        self.driver.write(self.throttle_command)
        self.driver.flush()
        time.sleep(0.01)
        
        # Steering input (if applicable)
        if steering_input is not None:
            self.steering_input_scaled = self._scale_input(steering_input)
            self.steering_command = f"@1st{self.steering_input_scaled}\r".encode("ascii")
            self.driver.write(self.steering_command)
            self.driver.flush()
            time.sleep(0.01)
        
    def _send_command(self, command):
        """
        Sends commands to the motor driver, with a response expected.
        """
        # Send command
        self.driver.reset_input_buffer()  # Clear old responses
        self.driver.write(command)
        
        # Collect response
        response = self.driver.read_until(b'>', 256).decode("ascii", errors="ignore")
        
        # Normalize line ending and split into lines
        lines = [line.strip() for line in response.replace('\r', '').split('\n') if line.strip()]
        
        # Interpret response
        if lines:
            try:
                self.response = float(lines[1])
            except ValueError:
                self.response = None
    # === #

    # === Logging === #
    def log_debug_values(self):
        """
        Logs all debug values.
        """
        # Log each loop
        self.logger.debug(f"Throttle motor speed: {self.throttle_input_scaled}")
        self.logger.debug(f"Throttle motor direction: {0 if self.throttle_input_scaled > 0 else 1}")
        self.logger.debug(f"Throttle motor command: {self.throttle_command}")
        
        if self.steering_command is not None:
            self.logger.debug(f"Steering motor speed: {self.steering_input_scaled}")
            self.logger.debug(f"Steering motor direction: {0 if self.steering_input_scaled < 0 else 1}")
            self.logger.debug(f"Steering motor command: {self.steering_command}")
        
class AF160_with_Encoder:
    def __init__(self):
        """
        Class initialization.
        """
        # === Pi GPIO Pins (Broadcom numbers) === #
        self._pin_motor_driver = 14  # Output for sending motor driver commands
        # === #
        
        # === Motor Driver Payload Variables === #
        self.throttle_input_scaled = 0  # Throttle input normalized to AF160 PWM torque limits (-255 to 255)
        self.steering_input_scaled = 0  # Steering input normalized to AF160 PWM torque limits (-255 to 255)
        self.throttle_command = None  # Throttle command (left motor)
        self.steering_command = None  # Steering command (right motor)
        self.encoder_value = 0
        self.encoder_command = f"@0ge\r".encode("ascii")
        # === #
        
        # === Logger Config === #
        # self.logger = logging.getLogger(self.__class__.__name__)
        # self.logger.info("AF160 class initialized")
        # === #
        
        # === Connect to Driver === #
        self.connect_driver()
        # === #
        
    # === Connecting to Driver === #
    def connect_driver(self):
        """
        Connect to the driver via UART connection.
        """
        # Open driver connection
        self.driver = serial.Serial(
            port='dev/ttyUSB0',
            baudrate=115200,
            timeout=1
        )
        
        # self.logger.info("AF160 driver connected via UART serial")
        
    def disconnect_driver(self):
        """
        Close driver connection.
        """
        # Send a 0 command to stop motors
        self.send_payloads(0, 0)
        
        # Close driver connection
        if self.driver.is_open:
            self.driver.close()
            
        # self.logger.info("AF160 driver disconnected")
    # === #
    
    # === Logic Handlers === #
    def _scale_input(self, input):
        """
        Scales input to [-255, 255] range
        """
        input = max(-1.0, min(1.0, input))  # Clamp between [-1.0, 1.0]
        return int(input * 255)
    
    def send_payloads(self, throttle_input, steering_input):
        """
        Creates and transmits throttle and steering payloads based on inputs from Tower class.
        """
        # Scale input
        self.throttle_input_scaled = self._scale_input(throttle_input)
        self.steering_input_scaled = self._scale_input(steering_input)
        
        # Create commands
        self.throttle_command = f"@0st{self.throttle_input_scaled}\r".encode("ascii")
        self.steering_command = f"@1st{self.steering_input_scaled}\r".encode("ascii")
        
        # Send commands
        self.driver.write(self.throttle_command)
        self.driver.write(self.steering_command)
        
    def _get_encoder_value(self):
        """
        Collect the encoder value from the motor driver.
        """
        # Send command
        self.driver.reset_input_buffer()  # Clear old responses
        self.driver.write(self.encoder_command)
        
        # Collect response
        response = []
        while True:
            line = self.driver.readline().decode("ascii", errors="ignore").strip()
            if not line:  # Timeout
                break
            response.append(line)
            
        # Interpret response
        if response:
            self.encoder_value = int(response[1])
    # === #
    
    # === Helper Functions === #
    def get_encoder_position(self):
        """
        Returns the encoder reading to the user.
        """
        self._get_encoder_value()
        return self.encoder_value

    # === Logging === #
    # def log_debug_values(self):
    #     """
    #     Logs all debug values.
    #     """
    #     # Log each loop
    #     self.logger.debug(f"Throttle motor speed: {self.throttle_input_scaled}")
    #     self.logger.debug(f"Throttle motor direction: {0 if self.throttle_input_scaled > 0 else 1}")
    #     self.logger.debug(f"Throttle motor command: {self.throttle_command}")
    #     self.logger.debug(f"Steering motor speed: {self.steering_input_scaled}")
    #     self.logger.debug(f"Steering motor direction: {0 if self.steering_input_scaled < 0 else 1}")
    #     self.logger.debug(f"Steering motor command: {self.steering_command}")