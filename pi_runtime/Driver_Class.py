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
        
        self.get_driver_configuration()

        self.logger.info("AF160 driver connected via UART serial")
        
    def get_driver_configuration(self):
        """
        Collect and log current motor driver configuration parameters.
        """
        # Servo modes (m)
        self._send_command(f"@0gm\r".encode("ascii"))
        self.logger.info(f"Left motor servo mode: {self.response}")
        self._send_command(f"@1gm\r".encode("ascii"))
        self.logger.info(f"Right motor servo mode: {self.response}")
        
        # Control input selection (j)
        self._send_command(f"@0gj\r".encode("ascii"))
        self.logger.info(f"Left motor control input selection: {self.response}")
        self._send_command(f"@1gj\r".encode("ascii"))
        self.logger.info(f"Right motor control input selection: {self.response}")
        
        # Servo update rate (r)
        self._send_command(f"@0gr\r".encode("ascii"))
        self.logger.info(f"Left motor servo update rate: {self.response}")
        self._send_command(f"@1gr\r".encode("ascii"))
        self.logger.info(f"Right motor servo update rate: {self.response}")
        
        # Power slew rate (s)
        self._send_command(f"@0gs\r".encode("ascii"))
        self.logger.info(f"Left motor power slew rate: {self.response}")
        self._send_command(f"@1gs\r".encode("ascii"))
        self.logger.info(f"Right motor power slew rate: {self.response}")
        
        # Minimum drive value (M)
        self._send_command(f"@0gM\r".encode("ascii"))
        self.logger.info(f"Left motor minimum drive value: {self.response}")
        self._send_command(f"@1gM\r".encode("ascii"))
        self.logger.info(f"Right motor minimum drive value: {self.response}")
        
        # Proportional gain value (P)
        self._send_command(f"@0gP\r".encode("ascii"))
        self.logger.info(f"Left motor proportional gain value: {self.response}")
        self._send_command(f"@1gP\r".encode("ascii"))
        self.logger.info(f"Right motor proportional gain value: {self.response}")
        
        # Integral gain value (I)
        self._send_command(f"@0gI\r".encode("ascii"))
        self.logger.info(f"Left motor integral gain value: {self.response}")
        self._send_command(f"@1gI\r".encode("ascii"))
        self.logger.info(f"Right motor integral gain value: {self.response}")
        
        # Derivative gain value (D)
        self._send_command(f"@0gD\r".encode("ascii"))
        self.logger.info(f"Left motor derivative gain value: {self.response}")
        self._send_command(f"@1gD\r".encode("ascii"))
        self.logger.info(f"Right motor derivative gain value: {self.response}")
        
        # Input factor (F)
        self._send_command(f"@0gF\r".encode("ascii"))
        self.logger.info(f"Left motor input factor: {self.response}")
        self._send_command(f"@1gF\r".encode("ascii"))
        self.logger.info(f"Right motor input factor: {self.response}")
        
        # Brake value (B)
        self._send_command(f"@0gB\r".encode("ascii"))
        self.logger.info(f"Left motor brake value: {self.response}")
        self._send_command(f"@1gB\r".encode("ascii"))
        self.logger.info(f"Right motor brake value: {self.response}")
        
        # Back-EMF factor (E)
        self._send_command(f"@0gE\r".encode("ascii"))
        self.logger.info(f"Left motor back-EMF value: {self.response}")
        self._send_command(f"@1gE\r".encode("ascii"))
        self.logger.info(f"Right motor back-EMF value: {self.response}")
        
        # Torque limit (T)
        self._send_command(f"@0gT\r".encode("ascii"))
        self.logger.info(f"Left motor torque limit: {self.response}")
        self._send_command(f"@1gT\r".encode("ascii"))
        self.logger.info(f"Right motor torque limit: {self.response}")

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