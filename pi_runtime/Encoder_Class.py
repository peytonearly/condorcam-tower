import pigpio
import logging
import serial
import time
import glob
from collections import deque
from statistics import median
from Event_Class import Zero_Button_Event

class HEDS_9140:
    def __init__(self, pi):
        """
        Class initialization.
        """
        # === Constants === #
        self._trust_encoder = False  # Indicates if the encoder values can be trusted
        self._encoder_transition_table = {  # Transition table for determining encoder changes
            (1, 0): +1, (3, 1): +1, (2, 3): +1, (0, 2): +1,
            (0, 1): -1, (1, 3): -1, (3, 2): -1, (2, 0): -1
        }
        self._encoder_cpr = 500  # Encoder counts per revolution
        self._encoder_rev_to_max = 66.5  # Number of revolutions made from lowest to highest tower point
        self._encoder_max = self._encoder_cpr * self._encoder_rev_to_max  # Encoder max position
        # === #

        # === Pi GPIO Pins (Broadcom numbers) === #
        self._pin_encoder_a = 17  # Encoder channel A signal (edge detection)
        self._pin_encoder_i = 22  # Encoder index channel
        self._pin_encoder_b = 27  # Encoder channel B signal (phase shift)
        # === #

        # === Deque === #
        # --- Size limited to 7 --- #
        self.travel_rate_dq = deque([0 for _ in range(7)], maxlen = 7)  # Deque for encoder travel rate
        # === #

        # === Runtime Variables === #
        self._last_state = None  # Hold A and B states in a 2-bit value
        self._last_tick = None  # Tracks the last encoder tick
        self._last_index_tick = None  # Tracks the last index pulse tick
        self.position = 0  # Encoder position
        self.travel_dir = None  # Travel direction based on encoder values. 0 for up | 1 for down
        self.travel_rate = 0.0  # Encoder counts per second. <0 for moving down | >0 for moving up
        # === #

        # === Logger Config === #
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.info("HEDS-9140 encoder class initialized")
        # === #

        # === Connect to Encoder === #
        self.connect_encoder(pi)
        # === #

    def connect_encoder(self, pi):
        """
        Master function for connecting to the encoder.
        """
        # Connect to pigpio daemon
        self.pi = pi

        # Run connector function
        self._connect_encoder()

        self.logger.info("HEDS-9140 encoder connected")

    def disconnect_encoder(self):
        """
        Close class connections.
        """
        # Cancel callbacks
        self._cb_encoder_a.cancel()  # Cancel encoder A callback
        self._cb_encoder_b.cancel()  # Cancel encoder B callback
        self._cb_encoder_i.cancel()  # Cancel encoder I callback

        self.logger.info("All encoder callbacks cancelled")

    def _connect_encoder(self):
        """
        Connects to the HEDS-9140 optical encoder.
        """
        # Set up encoder pin modes and pull-ups
        for pin in [self._pin_encoder_a, self._pin_encoder_b, self._pin_encoder_i]:
            self.pi.set_mode(pin, pigpio.INPUT)
            self.pi.set_pull_up_down(pin, pigpio.PUD_UP)
            self.pi.set_glitch_filter(pin, 1)  # Ignores edges shorter than 1 µs

        # Register callbacks
        self._cb_encoder_a = self.pi.callback(self._pin_encoder_a, pigpio.EITHER_EDGE, self._handle_encoder_change)
        self._cb_encoder_b = self.pi.callback(self._pin_encoder_b, pigpio.EITHER_EDGE, self._handle_encoder_change)
        self._cb_encoder_i = self.pi.callback(self._pin_encoder_i, pigpio.EITHER_EDGE, self._handle_encoder_index)

        # Initialize channel A and B states
        a = self.pi.read(self._pin_encoder_a)
        b = self.pi.read(self._pin_encoder_b)
        self._last_state = (a << 1) | b

        self.logger.info("Encoder pins and callbacks configured")

    # === Callback Handlers === #
    def _handle_encoder_change(self, gpio, level, tick):
        """
        Reads the current encoder value. Triggered when A or B edge is detected.
        """
        # Read pin states
        a = self.pi.read(self._pin_encoder_a)
        b = self.pi.read(self._pin_encoder_b)
        current_state = (a << 1) | b

        # Determine if state has changed
        delta = self._encoder_transition_table.get((self._last_state, current_state), 0)
        self._last_state = current_state

        if delta:
            # Update position and direction
            self.position += delta
            self.travel_dir = 1 if delta > 0 else 0

            # Compute travel rate
            if self._last_tick is not None:
                dt = pigpio.tickDiff(self._last_tick, tick)
                if dt > 0:
                    self.travel_rate_dq.append(1_000_000 * delta / dt)  # Counts per second
            self._last_tick = tick

    def _handle_encoder_index(self, gpio, level, tick):
        """
        Verifies the position when index pulse is detected.
        """
        self._last_index_tick = tick

    def handle_zero_button_tripped(self, value):
        """
        Handles the notification from Tower class that the zero button was tripped.
        """
        if value:  # Check that value is True
            self.position = 0
            self.logger.info("Zero button notification received. Position set to 0.")
    # === #

    # === Helper Functions === #
    def get_position(self):
        """
        Returns tower position.
        """
        return self.position
    
    def get_encoder_max(self):
        """
        Returns the max encoder value.
        """
        return self._encoder_max
    
    def get_travel_rate(self):
        """
        Returns the median of the travel rate deque.
        """
        return median(self.travel_rate_dq)
    
    def get_encoder_trust(self):
        """
        Returns a boolean indicating if the encoder position can be trusted.
        """
        return self._trust_encoder
    # === #

    # === Logging === #
    def log_debug_values(self):
        """
        Logs all debug values.
        """
        # Log each loop
        self.logger.debug(f"Encoder position: {self.position}")
        self.logger.debug(f"Encoder rate: {self.get_travel_rate()}")
        self.logger.debug(f"Encoder last state: {self._last_state}")
    # === #
    
class E5:
    def __init__(self, pi):
        """
        Class initialization.
        """
        # === Constants === #
        self._trust_encoder = False  # Indicates if the encoder values can be trusted
        self._encoder_transition_table = {  # Transition table for determining encoder changes
            (1, 0): +1, (3, 1): +1, (2, 3): +1, (0, 2): +1,
            (0, 1): -1, (1, 3): -1, (3, 2): -1, (2, 0): -1
        }
        self._encoder_cpr = 32  # Encoder counts per revolution
        self._encoder_rev_to_max = 66.5  # Number of revolutions made from lowest to highest tower point
        self._encoder_max = self._encoder_cpr * self._encoder_rev_to_max  # Encoder max position
        # === #
        
        # === Pi GPIO Pins (Broadcom numbers) === #
        self._pin_encoder_a = 17  # Encoder channel A signal (edge detection)
        self._pin_encoder_i = 22  # Encoder index channel
        self._pin_encoder_b = 27  # Encoder channel B signal (phase shift)
        # === #
        
        # === Deque === #
        # --- Size limited to 7 --- #
        self.travel_rate_dq = deque([0 for _ in range(7)], maxlen = 7) # Deque for encoder travel rate
        # === #
        
        # === Runtime Variables === #
        self._last_state = None  # Hold A and B states in a 2-bit value
        self._last_tick = None  # Tracks the last encoder tick
        self._last_index_tick = None  # Tracks the last index pulse tick
        self.position = 0  # Encoder position
        self.travel_dir = None  # Travel direction based on encoder values. 1 for up | 0 for down
        self.travel_rate = 0.0  # ENcoder counts per second. <0 for moving down | >0 for moving up
        # === #
        
        # === Logger Config === #
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.info("E5 encoder class initialized")
        # === #
        
        # === Connect to Encoder === #
        self.connect_encoder(pi)
        # === #
        
    def connect_encoder(self, pi):
        """
        Master function for connecting to the encoder.
        """
        # Connect to pigpio daemon
        self.pi = pi
        
        # Run connector function
        self._connect_encoder()
        
        self.logger.info("E5 encoder connected")
        
    def disconnect_encoder(self):
        """
        Close class connections.
        """
        # Cancel callbacks
        self._cb_encoder_a.cancel()  # Cancels encoder A callback
        self._cb_encoder_b.cancel()  # Cancels encoder B callback
        self._cb_encoder_i.cancel()  # Cancels encoder I callback
        
        self.logger.info("All encoder callbacks cancelled")
        
    def _connect_encoder(self):
        """
        Connects to the E5 optical encoder.
        """
        # Set up encoder pin modes and pull-ups
        for pin in [self._pin_encoder_a, self._pin_encoder_b, self._pin_encoder_i]:
            self.pi.set_mode(pin, pigpio.INPUT)
            self.pi.set_pull_up_down(pin, pigpio.PUD_UP)
            self.pi.set_glitch_filter(pin, 1)  # Ignores edges shorter than 1 µs
            
        # Register callbacks
        self._cb_encoder_a = self.pi.callback(self._pin_encoder_a, pigpio.EITHER_EDGE, self._handle_encoder_change)
        self._cb_encoder_b = self.pi.callback(self._pin_encoder_b, pigpio.EITHER_EDGE, self._handle_encoder_change)
        self._cb_encoder_i = self.pi.callback(self._pin_encoder_i, pigpio.EITHER_EDGE, self._handle_encoder_index)
        
        # Initialize channel A and B states
        a = self.pi.read(self._pin_encoder_a)
        b = self.pi.read(self._pin_encoder_b)
        self._last_state = (a << 1) | b
        
        self.logger.info("Encoder pins and callbacks configured")
        
    # === Callback Handlers === #
    def _handle_encoder_change(self, gpio, level, tick):
        """
        Reads the current encoder value. Triggered when A or B edge is detected.
        """
        # Read pin states
        a = self.pi.read(self._pin_encoder_a)
        b = self.pi.read(self._pin_encoder_b)
        current_state = (a << 1) | b
        
        # Determine if state has changed
        delta = self._encoder_transition_table.get((self._last_state, current_state), 0)
        self._last_state = current_state
        
        if delta:
            # Update position and direction
            self.position += delta
            self.travel_dir = 1 if delta > 0 else 0
            
            # Compute travel rate
            if self._last_tick is not None:
                dt = pigpio.tickDiff(self._last_tick, tick)
                if dt > 0:
                    self.travel_rate_dq.append(1_000_000 * delta / dt)  # Counts per second
            self._last_tick = tick
            
    def _handle_encoder_index(self, gpio, level, tick):
        """
        Verifies the position when index pulse is detected.
        """
        self._last_index_tick = tick
        
    def handle_zero_button_tripped(self, value):
        """
        Handles the notification from Tower class that the zero button was tripped.
        """
        if value:  # Check that value is True
            self.position = 0
            self.logger.info("Zero button notification received. Position set to 0.")
    # === #
    
    # === Helper Functions === #
    def get_position(self):
        """
        Returns tower position.
        """
        return self.position
    
    def get_encoder_max(self):
        """
        Returns the max encoder value.
        """
        return self._encoder_max
    
    def get_travel_rate(self):
        """
        Returns the median of the travel rate deque.
        """
        return median(self.travel_rate_dq)
    
    def get_encoder_trust(self):
        """
        Returns a boolean indicating if the encoder position can be trusted.
        """
        return self._trust_encoder
    # === #
    
    # === Logging === #
    def log_debug_values(self):
        """
        Logs all debug values.
        """
        # Log each loop
        self.logger.debug(f"Encoder position: {self.position}")
        self.logger.debug(f"Encoder rate: {self.get_travel_rate()}")
        self.logger.debug(f"Encoder last state: {self._last_state}")
    # === #

class E5_with_Pico_UART:
    def __init__(self):
        """
        Class initialization.
        """
        # === Constants === #
        # Encoder variables
        self._encoder_cpr = 32  # Encoder counts per revolution
        self._encoder_rev_to_max = 66.5  # Number of revolutions made from lowest to highest tower point
        self._encoder_max = self._encoder_cpr * self._encoder_rev_to_max  # Encoder max position

        # Serial connection variables
        self.ser_port = "/dev/serial0"
        self.timeout = 0.25  # Serial read timeout in seconds
        self.connect_timeout = 3.0  # Max seconds to try connecting before giving up
        self.ser = None  # Serial port connection to the Pico
        self._trust_encoder = False  # Indicates if the Pico is connected (trusted)
        # === #

        # === Deque === #
        # --- Size limited to 7 --- #
        self.encoder_position_dq = deque([0 for _ in range(7)], maxlen = 7)  # Deque for encoder position
        self.position_timestamps_dq = deque([time.perf_counter_ns() for _ in range(7)], maxlen = 7)  # Deque for position timestamps
        self.travel_rate_dq = deque([0 for _ in range(7)], maxlen = 7)  # Deque for encoder travel rate
        # === #

        # === Logger Config === #
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.info("E5 encoder via Pico UART class intialized")
        # === #

        # === Connect to Encoder === #
        self.connect_encoder()
        # === #
        
    # === Connection Management === #
    def connect_encoder(self):
        """
        Attempt to (re)connect to the Pico over serial.
        Non-blocking: Will timeout after connect_timeout seconds.
        """
        start_time = time.time()
        while time.time() - start_time < self.connect_timeout:
            try:
                self.ser = serial.Serial(
                    self.ser_port,
                    baudrate=115200,
                    timeout=self.timeout,
                    write_timeout=self.timeout,
                )
                
                time.sleep(0.5)  # Allow link to settle

                self._trust_encoder = True
                self.logger.info(f"UART encoder connected on {self.ser_port} at {time.time():.3f}")
                return
            
            except serial.SerialException:
                time.sleep(0.5)

        self._trust_encoder = False
        self.logger.warning("Failed to connect to UART encoder (timeout)")

    def disconnect_encoder(self):
        """
        Safely close the serial UART connection to the Pico.
        """
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except serial.SerialException:
                pass
            
        self.ser = None
        self._trust_encoder = False
        self.logger.info(f"UART encoder disconnected at {time.time():.3f}")

    def reconnect_encoder(self):
        """
        Attempt to reconnect to the Pico only if the port is truly closed.
        """
        if not self._trust_encoder:
            self.logger.info(f"Attempting UART reconnection at {time.time():.3f}")
            self.connect_encoder()

        return self._trust_encoder
    # === #

    # === Command / Response Handling === #
    def _send_command(self, cmd):
        """
        Send a numeric command to the Pico and capture the response.
        """
        # Don't send if not connected
        if not self._trust_encoder:
            self.response = None
            return
        
        try:
            # Clear stale data
            garbage = self.ser.read_all()
            if garbage:
                # self.logger.debug(f"Leftover bytes before send: {garbage!r}")
                pass
                
            # Write the command
            time.sleep(0.05)  # Allow previous cycle to finish
            self.ser.write(f"{cmd}\r\n".encode("utf-8"))
            self.ser.flush()  # Force TX buffer to send immediately
            time.sleep(0.02)  # Small pacing delay

            # Capture response
            self.response = self.ser.readline().decode("utf-8").strip()
            
        except serial.SerialException:
            self.logger.warning("Serial exception during write - disconnecting")
            self._trust_encoder = False
            self.disconnect_encoder()
            self.response = None
    # === #
        
    # === Encoder Commands === #
    def _reset_pico_position(self):
        """
        Send command 2: reset encoder position to zero.
        """
        if not self._trust_encoder:
            return None
        
        self._send_command(2)
        return self.response().strip() if self.response else None

    def _get_position(self):
        """
        Send command 1: read current position.
        """
        # Don't send if not connected
        if not self._trust_encoder:
            return
        
        self._send_command(1)
        if not self.response:
            # If no response, just skip this cycle - don't drop connection
            self.logger.debug("No response from Pico this cycle, skipping")
            return
        
        try:
            pos = int(self.response)
            ts = time.perf_counter_ns()
            self.encoder_position_dq.append(pos)
            self.position_timestamps_dq.append(ts)
            
        except ValueError:
            # Don't disconnect, just ignore bad data
            self.logger.warning(f"Invalid response: {self.response!r}")
    # === #
            
    # === Derived Calculations === #
    def _calculate_velocity(self):
        """
        Compute average velocity (counts/s)
        """
        if len(self.encoder_position_dq) < 2:
            return
        
        # Calculate average of recent velocities
        vels = [
            (self.encoder_position_dq[i] - self.encoder_position_dq[i-1]) / 
            ((self.position_timestamps_dq[i] - self.position_timestamps_dq[i-1])/1_000_000_000)
            for i in range(1, len(self.encoder_position_dq))
        ]

        self.travel_rate_dq.append(sum(vels) / len(vels))
    # === #

    # === Public Interface === #
    def handle_zero_button_tripped(self, value):
        """
        Zero position when triggered by Tower class.
        """
        if value:  # Check that value is True
            self.position = 0
            resp = self._reset_pico_position()
            if resp:
                self.logger.info("Encoder position reset (via UART)")
            else:
                self.logger.warning("Attempted position reset but Pico did not respond")

    def get_position(self):
        """
        Returns current tower position.
        """
        self._get_position()
        return self.encoder_position_dq[-1]
    
    def get_travel_rate(self):
        """
        Returns the current travel rate.
        """
        self._get_position()
        self._calculate_velocity()
        return self.travel_rate_dq[-1]
    
    def get_encoder_max(self):
        """
        Returns the max encoder value.
        """
        return self._encoder_max
    
    def get_encoder_trust(self):
        """
        Returns a boolean indicating if the encoder position can be trusted.
        """
        return self._trust_encoder
    # === #

    # === Logging === #
    def log_debug_values(self):
        """
        Logs all debug values.
        """
        # Log each loop
        self.logger.debug(f"Encoder position: {self.encoder_position_dq[-1]}")
        self.logger.debug(f"Encoder rate: {self.travel_rate_dq[-1]}")
        self.logger.debug(f"Encoder trust state: {self._trust_encoder}")
    # === #
    
class E5_with_Pico_USB:
    def __init__(self):
        """
        Class initialization.
        """
        # === Constants === #
        # Encoder variables
        self._encoder_cpr = 32  # Encoder counts per revolution
        self._encoder_rev_to_max = 66.5  # Number of revolutions made from lowest to highest tower point
        self._encoder_max = self._encoder_cpr * self._encoder_rev_to_max  # Encoder max position
        
        # Serial connection variables
        self.ser_port = "/dev/ttyACM0"
        self.timeout = 0.25  # Serial read timeout (s)
        self.connect_timeout = 3.0  # Max seconds to attempt connecting
        self.ser = None  # Serial port connection to the Pico
        self._trust_encoder = False  # Indicates if the Pico is connected (trusted)
        # === #
        
        # === Deques === #
        # --- Size limited to 7 --- #
        self.encoder_position_dq = deque([0 for _ in range(7)], maxlen=7)  # Deque for encoder position
        self.position_timestamps_dq = deque([time.perf_counter_ns() for _ in range(7)], maxlen=7)  # Deque for position timestamps
        self.travel_rate_dq = deque([0 for _ in range(7)], maxlen=7)  # Deque for encoder travel rate
        # === #
        
        # === Logger Config === #
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.info("E5 encoder via Pico USB class initialized")
        # === #
        
        # === Connect to Encoder === #
        self.connect_encoder()
        # === #
    
    # === Connection Management === #
    def connect_encoder(self):
        """
        Attempt to (re)connect to the Pico over USB.
        Non-blocking: Will timeout after connect_timeout seconds.
        """
        start_time = time.time()
        while time.time() - start_time < self.connect_timeout:
            try:
                self.ser = serial.Serial(
                    self.ser_port,
                    baudrate=115200,
                    timeout=self.timeout,
                    write_timeout=self.timeout,
                )
                
                time.sleep(0.5)  # Allow link to settle
                
                self._trust_encoder = True
                self.logger.info(f"USB encoder connected on {self.ser_port} at {time.time():.3f}")
                return
            
            except serial.SerialException:
                time.sleep(0.5)
                
        self._trust_encoder = False
        self.logger.warning("Failedd to connect to USB encoder (timeout)")
    
    def disconnect_encoder(self):
        """
        Safely close the serial USB connection to the Pico
        """
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except serial.SerialException:
                pass
        
        self.ser = None
        self._trust_encoder = False
        self.logger.info(f"USB encoder disconnected at {time.time():.3f}")
    
    def reconnect_encoder(self):
        """
        Attempt to reconnect to the Pico only if the port is truly closed.
        """
        if not self._trust_encoder:
            self.logger.info(f"Attempting USB reconnection at {time.time():.3f}")
            self.connect_encoder()
            
        return self._trust_encoder
    # === #
    
    # === Command / Response Handling === #
    def _send_command(self, cmd):
        """
        Send a numeric command to the Pico and capture the response.
        """
        if not self._trust_encoder:
            self.response = None
            return
        
        try:
            garbage = self.ser.read_all()
            if garbage:
                # self.logger.debug(f"Leftover bytes before send: {garbage!r}")
                pass
            
            # Write the command
            time.sleep(0.05)  # Allow previous cycle to finish
            self.ser.write(f"{cmd}\r\n".encode("utf-8"))
            self.ser.flush()  # Force TX buffer to send immediately
            time.sleep(0.02)  # Small pacing delay
            
            # Capture response
            self.response = self.ser.readline().decode("utf-8", errors="ignore").strip()
            
        except serial.SerialException:
            self.logger.warning("USB serial exception during write - disconnecting")
            self._trust_encoder = False
            self.disconnect_encoder()
            self.resposne = None
    # === #
    
    # === Encoder Commands === #
    def _reset_pico_position(self):
        """
        Send command 2: reset encoder position to zero.
        """
        if not self._trust_encoder:
            return None
        
        self._send_command(2)
        return self.response.strip() if self.response else None
    
    def _get_position(self):
        """
        Send command 1: read current position.
        """
        if not self._trust_encoder:
            return
        
        self._send_command(1)
        if not self.response:
            # If no response, just skip this cycle - don't drop connection
            self.logger.debug("No response from Pico this cycle, skipping")
            return
        
        try:
            pos = int(self.response)
            ts = time.perf_counter_ns()
            self.encoder_position_dq.append(pos)
            self.position_timestamps_dq.append(ts)
            
        except ValueError:
            self.logger.warning(f"Invalid response: {self.response!r}")
    # === #
    
    # === Derived Calculations === #
    def _calculate_velocity(self):
        """
        Compute average velocity (counts/s)
        """
        if len(self.encoder_position_dq) < 2:
            return
        
        vels = [
            (self.encoder_position_dq[i] - self.encoder_position_dq[i-1]) /
            ((self.position_timestamps_dq[i] - self.position_timestamps_dq[i-1])/1_000_000_000)
            for i in range(1, len(self.encoder_position_dq))
        ]
        
        self.travel_rate_dq.append(sum(vels) / len(vels))
    # === #
    
    # === Public Interface === #
    def handle_zero_button_tripped(self, value):
        """
        Zero position when triggered by Tower class.
        """
        if value:  # Check that value is True
            self.position = 0
            resp = self._reset_pico_position()
            if resp:
                self.logger.info("Encoder position reset (via USB)")
            else:
                self.logger.warning("Attempted position reset but Pico did not respond")
    
    def get_position(self):
        """
        Returns current tower position.
        """
        self._get_position()
        return self.encoder_position_dq[-1]
    
    def get_travel_rate(self):
        """
        Returns the current travel rate.
        """
        self._get_position()
        self._calculate_velocity()
        return self.travel_rate_dq[-1]
    
    def get_encoder_max(self):
        """
        Returns the max encoder value.
        """
        return self._encoder_max
    
    def get_encoder_trust(self):
        """
        Returns a boolean indicating if the encoder position can be trusted.
        """
        return self._trust_encoder
    # === #
    
    # === Logging == #
    def log_debug_values(self):
        """
        Logs all debug values.
        """
        # Log each loop
        self.logger.debug(f"Encoder position: {self.encoder_position_dq[-1]}")
        self.logger.debug(f"Encoder rate: {self.travel_rate_dq[-1]}")
        self.logger.debug(f"Encoder trust state: {self._trust_encoder}")
    # === #

"""
ChatGPT scripts.
"""
class Encoder:
    """
    Class to communicate with a Pico-based quadrature encoder over USB serial.
    """
    def __init__(self, port, baudrate=115200, timeout=0.1):
        """
        Initialize the serial connection.

        :param port: Serial port (e.g., 'COM3' on Windows or '/dev/ttyACM0' on Linux)
        :param baudrate: Communication baudrate
        :param timeout: Serial read timeout in seconds
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(2)  # Wait for Pico to reset

    def get_position(self):
        """
        Request the current encoder position from the Pico.

        :return: Integer position
        """
        self._send_command(1)  # 1 = get position
        response = self._read_response()
        try:
            return int(response)
        except ValueError:
            raise RuntimeError(f"Invalid response from Pico: {response}")
        
    def reset_position(self):
        """
        Reset the encoder position to zero on the Pico.
        """
        self._send_command(2)  # 2 = reset position
        # Optionally read acknowledgement
        ack = self._read_response()
        return ack.strip()
    
    def _send_command(self, cmd_int):
        """
        Send a command integer to the Pico.

        :param cmd_int: Command as integer
        """
        self.ser.write(f"{cmd_int}\n".encode("utf-8"))

    def _read_response(self):
        """
        Read a line of response from the Pico.

        :return: Response string
        """
        line = self.ser.readline().decode("utf-8").strip()
        return line
    
    def close(self):
        """
        Close the serial connection.
        """
        if self.ser.is_open:
            self.ser.close()

class Encoder_v2:
    """
    Class to communicate with a Pico-based quadrature encoder over USB serial.
    Includes robust connection handling with velocity computation.
    """

    def __init__(self, port, baudrate=115200, timeout=0.1, connect_timeout=3.0):
        """
        Initialize the encoder interface (does not hang if Pico is disconnected).

        :param port: Serial port (e.g., 'COM3' on Windows or '/dev/ttyACM0' on Linux)
        :param baudrate: Communication baudrate
        :param timeout: Serial read timeout in seconds
        :param connect_timeout: Max seconds to try connecting before giving up
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.connect_timeout = connect_timeout
        self.ser = None
        self.connected = False

        self.last_position = None
        self.last_timestamp = None

        # Try to connect at startup
        self.try_connect()

    def try_connect(self):
        """
        Attempt to (re)connect to the Pico over serial.
        Non-blocking: will timeout after connect_timeout seconds.
        """
        start_time = time.time()
        while time.time() - start_time < self.connect_timeout:
            try:
                self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
                time.sleep(2)  # Wait for pico to reset if just connected
                self.connected = True
                print(f"[Encoder] Connected to {self.port}")
                return True
            except serial.SerialException:
                time.sleep(0.5)
        self.connected = False
        print(f"[Encoder] Could not connect to {self.port} (timed out). Running in offline mode.")
        return False
    
    def get_position(self):
        """
        Request the current encoder position from the Pico.

        :return: (position, timestamp) or (None, None) if not connected
        """
        if not self.connected:
            return None, None
        
        try:
            self._send_command(1)  # 1 = get position
            response = self._read_response()
            position = int(response)
            timestamp = time.perf_counter()

            self.last_position = position
            self.last_timestamp = timestamp
            return position, timestamp
        
        except (ValueError, serial.SerialException, OSError):
            print("[Encoder] Lost connection to Pico.")
            self.connected = False
            self.close()
            return None, None
        
    def get_velocity(self):
        """
        Compute velocity (counts per second) based on change since last reading.
        
        :return: velocity (float) or None if not enough data
        """
        pos, t = self.get_position()
        if pos is None or t is None or self.last_position is None or self.last_timestamp is None:
            return None
        
        delta_pos = pos - self.last_position
        delta_t = t - self.last_timestamp
        if delta_t <= 0:
            return None
        velocity = delta_pos / delta_t
        return velocity
    
    def reset_position(self):
        """
        Reset encoder count to zero (if connected).
        """
        if not self.connected:
            return "Not connected"
        try:
            self._send_command(2)
            ack = self._read_response()
            return ack.strip()
        except serial.SerialException:
            self.connected = False
            self.close()
            return "Connection lost"
        
    def _send_command(self, cmd_int):
        if not self.connected:
            return
        self.ser.write(f"{cmd_int}\n".encode("utf-8"))

    def _read_response(self):
        if not self.connected:
            return ""
        return self.ser.readline().decode("utf-8").strip()
    
    def close(self):
        """
        Close the serial port cleanly.
        """
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except serial.SerialException:
                pass
        self.ser = None

    def reconnect_if_needed(self):
        """
        Attempt reconnection if not currently connected.
        Returns True if reconnected, False otherwise.
        """
        if not self.connected:
            print("[Encoder] Attempting reconnection...")
            return self.try_connect()
        return True
    
class Encoder_v3:
    def __init__(self):
        """
        Class initialization.
        """
        # === Constants === #
        self._encoder_cpr = 32
        self._encoder_rev_to_max = 66.5
        self._encoder_max = self._encoder_cpr * self._encoder_rev_to_max
        
        # === UART serial connection === #
        # Replace with the UART device on the Pi (usually /dev/serial0 or /dev/ttyAMA0)
        self.ser_port = "/dev/serial0"
        self.baudrate = 115200
        self.ser = None
        self._trust_encoder = None
        self.timeout = 0.1
        self.connect_timeout = 3.0
        
        # === Deques === #
        self.encoder_position_dq = deque([0 for _ in range(7)], maxlen=7)
        self.position_timestamps_dq = deque(
            [time.perf_counter_ns() for _ in range(7)], maxlen=7
        )
        self.travel_rate_dq = deque([0 for _ in range(7)], maxlen=7)
        
        # === Logger Config === #
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.info("E5_with_Pico (UART) initialized")
        
        # === Connect to Encoder === #
        self.connect_encoder()
        
    # === Connection Management === #
    def connect_encoder(self):
        """
        Attempt to (re)connect to the Pico over UART.
        """
        start_time = time.time()
        while time.time() - start_time < self.connect_timeout:
            try:
                self.ser = serial.Serial(
                    self.ser_port,
                    baudrate=self.baudrate,
                    timeout=self.timeout,
                    write_timeout=self.timeout,
                )
                time.sleep(0.5)  # Brief pause for link stabilization
                self._trust_encoder = True
                self.logger.info(f"UART encoder connected at {time.time():.3f}")
            except serial.SerialException:
                time.sleep(0.5)
                
        self._trust_encoder = False
        self.logger.warning("UART connection timeout - encoder not trustworthy")
        
    def disconnect_encoder(self):
        """
        Close UART connection.
        """
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except serial.SerialException:
                pass
        self.ser = None
        self._trust_encoder = False
        self.logger.info(f"UART encoder disconnected at {time.time():.3f}")
        
    def reconnect_encoder(self):
        """
        Attempt to reconnect to the encoder if not currently connected.
        """
        if not self._trust_encoder:
            self.logger.info(f"Attempting UART reconnection at {time.time():.3f}")
            self.connect_encoder()
        return self._trust_encoder
    
    # === Command I/O === #
    def _send_command(self, cmd):
        """
        Sends a command to the Pico and reads the response via UART.
        """
        if not self._trust_encoder:
            self.response = None
            return
        
        try:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.ser.write(f"{cmd}\n".encode("utf-8"))
            self.response = self.ser.readline().decode("utf-8").strip()
        except serial.SerialException:
            self._trust_encoder = False
            self.disconnect_encoder()
            self.response = None
            
    def _reset_pico_position(self):
        if not self._trust_encoder:
            return None
        try:
            self._send_command(2)
            return self.response.strip() if self.response else None
        except serial.SerialException:
            self._trust_encoder = False
            self.disconnect_encoder()
            return None
        
    def _get_position(self):
        if not self._trust_encoder:
            return
        try:
            self._send_command(1)
            if not self.response:
                self._trust_encoder = False
                self.disconnect_encoder()
                return
            pos = int(self.response)
            ts = time.perf_counter_ns()
            self.encoder_position_dq.append(pos)
            self.position_timestamps_dq.append(ts)
        except (ValueError, serial.SerialException, OSError):
            self.logger.warning("Lost UART communication with Pico")
            self._trust_encoder = False
            self.disconnect_encoder()
            
    # === Derived Calculations === #
    def _calculate_velocity(self):
        if len(self.encoder_position_dq) < 2:
            return
        vels = [
            (self.encoder_position_dq[i] - self.encoder_position_dq[i-1])
            / (self.position_timestamps_dq[i] - self.position_timestamps_dq[i-1])
            for i in range(1, len(self.encoder_position_dq))
        ]
        self.travel_rate_dq.append(sum(vels) / len(vels))
        
    # === Public Interface === #
    def handle_zero_button_tripped(self, value):
        if value:
            self.position = 0
            resp = self._reset_pico_position()
            if resp:
                self.logger.info("Encoder zeroed via UART")
            else:
                self.logger.warning("Failed to reset encoder position over UART")
                
    def get_position(self):
        self._get_position()
        return self.encoder_position_dq[-1]
    
    def get_encoder_max(self):
        return self._encoder_max
    
    def get_travel_rate(self):
        self._get_position()
        self._calculate_velocity()
        return self.travel_rate_dq[-1]
    
    def get_encoder_trust(self):
        return self._trust_encoder
    
    def log_debug_values(self):
        self.logger.debug(f"Encoder position: {self.encoder_position_dq[-1]}")
        self.logger.debug(f"Encoder rate: {self.travel_rate_dq[-1]}")
        self.logger.debug(f"Encoder trust state: {self.get_encoder_trust()}")
        
class Encoder_v4:
    """
    Communicates with a Raspberry Pi Pico encoder board via UART.
    Sends simple integer commands (1 = read position, 2 = reset position)
        and logs position/velocity data with build-in reconnection safety.
    """
    def __init__(self):
        # === Encoder constants === #
        self._encoder_cpr = 32  # Counts per revolution
        self._encoder_rev_to_max = 66.5
        self._encoder_max = self._encoder_cpr * self._encoder_rev_to_max
        
        # === UART connection === #
        self.ser_port = "/dev/serial0"  # Pi UART device
        self.baudrate = 115200
        self.timeout = 0.25  # Slightly longer timeout for UART
        self.connect_timeout = 3.0
        self.ser = None
        self._trust_encoder = False
        
        # === Position tracking === #
        self.encoder_position_dq = deque([0 for _ in range(7)], maxlen=7)
        self.position_timestamps_dq = deque(
            [time.perf_counter_ns() for _ in range(7)], maxlen=7
        )
        self.travel_rate_dq = deque([0 for _ in range(7)], maxlen=7)
        
        # === Logger === #
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.info("E5_with_Pico (UART) initialized")
        
        # === Attempt connection === #
        self.connect_encoder()
        
    # === Connection Management === #
    def connect_encoder(self):
        """
        Try to connect to the Pico over UART within connect_timeout seconds.
        """
        start_time = time.time()
        while time.time() - start_time < self.connect_timeout:
            try:
                self.ser = serial.Serial(
                    self.ser_port,
                    baudrate=self.baudrate,
                    timeout=self.timeout,
                    write_timeout=self.timeout,
                )
                time.sleep(0.5)  # Let the link settle
                self._trust_encoder = True
                self.logger.info(f"UART encoder connected on {self.ser_port}")
                return
            except serial.SerialException:
                time.sleep(0.5)
                
        self._trust_encoder = False
        self.logger.warning("Failed to connect to UART encoder (timeout)")
        
    def disconnect_encoder(self):
        """
        Close UART port safely.
        """
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except serial.SerialException:
                pass
        self.ser = None
        self._trust_encoder = False
        self.logger.info("UART encoder disconnected")
        
    def reconnect_encoder(self):
        """
        Attempt reconnection only if the port is truly closed.
        """
        if not self._trust_encoder:
            self.logger.info("Attempting UART reconnection...")
            self.connect_encoder()
        return self._trust_encoder
    
    # === Command / Response Handling === #
    def _send_command(self, cmd):
        """
        Send a numeric command and capture the response.
        """
        if not self._trust_encoder:
            self.response = None
            return
        
        try:
            # Write the command (no buffer reset - UART safe)
            self.ser.write(f"{cmd}\n".encode("utf-8"))
            self.response = self.ser.readline().decode("utf-8").strip()
            
        except serial.SerialException:
            self.logger.warning("Serial exception during write - disconnecting")
            self._trust_encoder = False
            self.disconnect_encoder()
            self.response = None
            
        # Small pacing delay to avoid flooding the Pico
        time.sleep(0.05)
        
    # === Encoder Commands === #
    def _reset_pico_position(self):
        """
        Send command 2: reset encoder position to zero.
        """
        if not self._trust_encoder:
            return None
        self._send_command(2)
        return self.response.strip() if self.response else None
    
    def _get_position(self):
        """
        Send command 1: read current position.
        """
        if not self._trust_encoder:
            return
        
        self._send_command(1)
        if not self.response:
            # If no response, just skip this cycle - don't drop connection
            self.logger.debug("No response from Pico this cycle; skipping")
            return
        
        try:
            pos = int(self.response)
            ts = time.perf_counter_ns()
            self.encoder_position_dq.append(pos)
            self.position_timestamps_dq.append(ts)
        except ValueError:
            self.logger.warning(f"Invalid response: {self.response!r}")
            # Don't disconnect; just ignore bad data
            
    # === Derived Calculations === #
    def _calculate_velocity(self):
        """
        Compute average velocity (counts/ns).
        """
        if len(self.encoder_position_dq) < 2:
            return
        
        vels = [
            (self.encoder_position_dq[i] - self.encoder_position_dq[i-1])
            / (self.position_timestamps_dq[i] - self.position_timestamps_dq[i-1])
            for i in range(1, len(self.encoder_position_dq))
        ]
        avg_vel = sum(vels) / len(vels)
        self.travel_rate_dq.append(avg_vel)
        
    # === Public Interface === #
    def handle_zero_button_tripped(self, value):
        """
        Zero position when triggered by external event.
        """
        if value:
            resp = self._reset_pico_position()
            if resp == "OK":
                self.logger.info("Encoder position reset (via UART)")
            else:
                self.logger.warning("Attempted reset but Pico did not respond.")
    
    def get_position(self):
        """
        Return the latest encoder position.
        """
        self._get_position()
        return self.encoder_position_dq[-1]
    
    def get_travel_rate(self):
        """
        Return the most recent travel rate (velocity).
        """
        self._get_position()
        self._calculate_velocity()
        return self.travel_rate_dq[-1]
    
    def get_encoder_max(self):
        return self._encoder_max
    
    def get_encoder_trust(self):
        return self._trust_encoder
    
    def log_debug_values(self):
        """
        Log current encoder state.
        """
        self.logger.debug(f"Encoder position: {self.encoder_position_dq[-1]}")
        self.logger.debug(f"Encoder rate: {self.travel_rate_dq[-1]}")
        self.logger.debug(f"Encoder trust state: {self._trust_encoder}")
        
class Encoder_v5:
    def __init__(self):
        """
        Class initialization for USB CDC connection to Pico-based encoder.
        """
        # === Constants === #
        self._encoder_cpr = 32
        self._encoder_rev_to_max = 66.5
        self._encoder_max = self._encoder_cpr * self._encoder_rev_to_max
        
        # Serial connection variables
        self.ser_port = None
        self.timeout = 0.25  # Serial read timeout (s)
        self.connect_timeout = 3.0  # Max second to attempt connecting
        self.ser = None
        self._trust_encoder = None
        # === #
        
        # === Deques === #
        self.encoder_position_dq = deque([0 for _ in range(7)], maxlen=7)
        self.position_timestamps_dq = deque(
            [time.perf_counter_ns() for _ in range(7)], maxlen=7
        )
        self.travel_rate_dq = deque([0 for _ in range(7)], maxlen=7)
        # === #
        
        # === Logger Config === #
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.info("E5 with Pico (USB) encoder class initialized")
        # === #
        
        # === Connect to Encoder === #
        self.connect_encoder()
        # === #
        
    # === Connection Management === #
    def _find_pico_usb_port(self):
        """
        Attempts to find a connect Pico USB CDC port (e.g. /dev/ttyACM0).
        Returns the first matching device path, or None if not found.
        """
        devices = glob.glob("/dev/ttyACM")
        if not devices:
            return None
        # Return first valid device
        return devices[0]
    
    def connect_encoder(self):
        """
        Attempt to (re)connect to the Pico over USB serial (CDC).
        Non-blocking: Times out after connect_timeout seconds.
        """
        start_time = time.time()
        while time.time() - start_time < self.connect_timeout:
            try:
                port = "/dev/ttyACM0"
                if not port:
                    time.sleep(0.5)
                    continue
                
                self.ser = serial.Serial(
                    port = port,
                    baudrate=115200,  # Ignored for USB CDC, but required arg
                    timeout=self.timeout,
                    write_timeout=self.timeout,
                )
                
                time.sleep(0.5)  # Allow link to settle
                self.ser_port = port
                self._trust_encoder = True
                self.logger.info(f"USB encoder connected on {port} at {time.time():.3f}")
                return
            
            except (serial.SerialException, OSError):
                time.sleep(0.5)
                
        self._trust_encoder = False
        self.logger.warning("Failed to connect to USB encoder (timeout)")
        
    def disconnect_encoder(self):
        """
        Safely close the USB serial connection.
        """
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except serial.SerialException:
                pass
            
        self.ser = None
        self._trust_encoder = False
        self.logger.info(f"USB encoder disconnected at {time.time():.3f}")
        
    def reconnect_encoder(self):
        """
        Attempt to reconnect to the Pico only if the port is truly closed.
        """
        if not self._trust_encoder:
            self.logger.info(f"Attempting USB reconnection at {time.time():.3f}")
            self.connect_encoder()
            
        return self._trust_encoder
    # === #
    
    # === Command / Response Handling === #
    def _send_command(self, cmd):
        """
        Send a numeric command to the Pico and capture the response (USB CDC).
        """
        if not self._trust_encoder:
            self.response = None
            return
        
        try:
            # Clear stale data
            garbage = self.ser.read_all()
            if garbage:
                pass
            
            # Send command (match Pico's expectations)
            time.sleep(0.05)
            self.ser.write(f"{cmd}\r\n".encode("utf-8"))
            self.ser.flush()
            time.sleep(0.02)
            
            # Read back response
            self.response = self.ser.readline().decode("utf-8", errors="ignore").strip()
            
        except serial.SerialException:
            self.logger.warning("USB serial exception during write - disconnecting")
            self._trust_encoder = False
            self.disconnect_encoder()
            self.response = None
    # === #
    
    # === Encoder Commands === #
    def _reset_pico_position(self):
        """
        Send command 2: reset encoder position to zero.
        """
        if not self._trust_encoder:
            return None
        
        self._send_command(2)
        return self.response.strip() if self.response else None
    
    def _get_position(self):
        """
        Send command 1: read current position.
        """
        if not self._trust_encoder:
            return
        
        self._send_command(1)
        if not self.response:
            self.logger.debug("No response from Pico this cycle, skipping")
            return
        
        try:
            pos = int(self.response)
            ts = time.perf_counter_ns()
            self.encoder_position_dq.append(pos)
            self.position_timestamps_dq.append(ts)
        except ValueError:
            self.logger.warning(f"Invalid response: {self.response!r}")
    # === #
    
    # === Derived Calculations === #
    def _calculate_velocity(self):
        """
        Compute average velocity (counts/s).
        """
        if len(self.encoder_position_dq) < 2:
            return
        
        vels = [
            (self.encoder_position_dq[i] - self.encoder_position_dq[i-1])
            / ((self.position_timestamps_dq[i] - self.position_timestamps_dq[i-1])
               / 1_000_000_000)
            for i in range(1, len(self.encoder_position_dq))
        ]
        self.travel_rate_dq.append(sum(vels) / len(vels))
    # === #
    
    # === Public Interface === #
    def handle_zero_button_tripped(self, value):
        """
        Zero position when triggered by Tower class.
        """
        if value:
            self.position = 0
            resp = self._reset_pico_position()
            if resp:
                self.logger.info("Encoder position reset (via USB)")
            else:
                self.logger.warning("Attempted position reset but Pico did not respond")
                
    def get_position(self):
        """
        Returns the current tower position.
        """
        self._get_position()
        return self.encoder_position_dq[-1]
    
    def get_travel_rate(self):
        """
        Returns the current travel rate.
        """
        self._get_position()
        self._calculate_velocity()
        return self.travel_rate_dq[-1]
    
    def get_travel_rate(self):
        """
        Returns the max encoder value.
        """
        return self._encoder_max
    
    def get_encoder_trust(self):
        """
        Returns a boolean indicating if the encoder position can be trusted.
        """
        return self._trust_encoder
    # === #
    
    # === Logging === #
    def log_debug_values(self):
        """
        Logs all debug values.
        """
        self.logger.debug(f"Encoder position: {self.encoder_position_dq[-1]}")
        self.logger.debug(f"Encoder rate: {self.travel_rate_dq[-1]}")
        self.logger.debug(f"Encoder trust state: {self._trust_encoder}")